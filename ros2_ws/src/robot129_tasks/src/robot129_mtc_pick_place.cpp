// Robot 129 grasp/motion S2+S3-wiring: a real MTC (MoveIt Task Constructor) pick-and-place
// task, planned against the LIVE robot state (this node's CurrentState stage reads real
// /robot129_sim/joint_states -- no fake joint_states publisher, unlike the earlier
// robot129_mtc_plan.cpp demo). It does NOT execute the plan itself (the MTC
// ExecuteTaskSolution move_group capability is not built in this workspace, see
// docs/progress/grasp_motion_s0_inventory.md); instead it exports the solution's
// ordered sub-trajectories to JSON so a Python executor can play them through the
// robot129_sim_execution FollowJointTrajectory adapter, exactly like
// tools/verify_grasp_motion_manual_waypoints.py already did with hand-solved poses.
//
// Multi-candidate wiring (ZeroDex Sec 3.4's "try each grasp candidate g in G, keep the
// first one f_motion succeeds on" loop): if a --candidates_path ROS param is given, this
// node reads the grasp_candidates.py / grasp_contract.py JSON export (accepted candidates
// only, tcp_pose = pinch-frame pose in world coordinates -- exactly the frame this node's
// ComputeIK targets via pinch_offset), sorts by score ascending (lower is better in
// grasp_candidates.py's convention -- most closing margin first), and plans a FRESH task
// per candidate (a candidate's planning failure must not corrupt a later candidate's scene
// state) until one succeeds or --max_candidates is exhausted. Every attempt (planned or
// not, with its failure reasons) is recorded in the report JSON's "candidate_attempts",
// not just the winner -- this is the trace a future visualization (e.g. ghosting rejected
// candidate poses) would replay.
//
// If --candidates_path is omitted (or the file has zero accepted candidates), this falls
// back to the original S2 behavior: a single hardcoded, numerically-solved grasp pose
// (see docs/progress/grasp_motion_progress_report.md for its derivation). This keeps the
// existing S2 regression path unchanged when nothing new is supplied.
//
// The place side is candidate-driven too: for each grasp candidate, a small fixed
// nominal-plus-4-neighbor xy offset grid (--place_offset_step_m, default 1.5cm) is tried
// around the same numerically-solved nominal place pose, nominal first. This is a
// deliberately simple stand-in for ZeroDex Sec 3.4 eq. 12's collision-aware
// nearest-valid-position search -- see the PlaceOffsetCandidate comment for what it does
// NOT do (no real occupancy search, no adaptive radius). Nested loop: outer over grasp
// candidates, inner over place offsets; first (grasp, place) pair whose full task solves
// wins. Every (grasp, place) attempt -- not just the winner -- is recorded in the report
// JSON's "candidate_attempts" with separate grasp_candidate_id/place_candidate_id fields.
//
// Planning scene collision objects (support floor, target cube) mirror the real
// pick_place scene in sim/scripts/run_robot129_ros_webrtc.py: floor top at z=0, cube
// 0.035m starting at (0.32, 0, 0.0175).

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/fixed_cartesian_poses.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace mtc = moveit::task_constructor;
using moveit_msgs::msg::CollisionObject;
using shape_msgs::msg::SolidPrimitive;
using geometry_msgs::msg::PoseStamped;

namespace {

CollisionObject makeBox(const std::string& id, const std::string& frame, double sx, double sy, double sz,
                        double x, double y, double z, double qx = 0, double qy = 0, double qz = 0, double qw = 1) {
  CollisionObject obj;
  obj.header.frame_id = frame;
  obj.id = id;
  SolidPrimitive prim;
  prim.type = SolidPrimitive::BOX;
  prim.dimensions = { sx, sy, sz };
  geometry_msgs::msg::Pose pose;
  pose.position.x = x; pose.position.y = y; pose.position.z = z;
  pose.orientation.x = qx; pose.orientation.y = qy; pose.orientation.z = qz; pose.orientation.w = qw;
  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(pose);
  obj.operation = CollisionObject::ADD;
  return obj;
}

PoseStamped makePose(const std::string& frame, double x, double y, double z, double qx, double qy, double qz,
                     double qw) {
  PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x; p.pose.position.y = y; p.pose.position.z = z;
  p.pose.orientation.x = qx; p.pose.orientation.y = qy; p.pose.orientation.z = qz; p.pose.orientation.w = qw;
  return p;
}

// Very small hand-rolled JSON writer -- avoids adding a new JSON library dependency
// for a single export function. MTC error/comment strings (e.g. InitStageException
// detail, task.init() property errors) routinely contain embedded newlines, so all
// control characters are escaped, not just quote/backslash.
std::string jsonEscape(const std::string& s) {
  std::ostringstream out;
  for (unsigned char c : s) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out << buf;
        } else {
          out << static_cast<char>(c);
        }
    }
  }
  return out.str();
}

// ---------------------------------------------------------------------------------------
// Minimal recursive-descent JSON reader, scoped to what's needed to read back
// grasp_contract.py's write_candidates_json() output (a JSON object, arrays of objects,
// strings/numbers/bools). Not a general-purpose library on purpose -- same reasoning as
// jsonEscape() above: this is a single-caller need, not worth a new package dependency.
namespace tinyjson {

struct Value {
  enum class Type { Null, Bool, Number, String, Array, Object } type = Type::Null;
  bool b = false;
  double num = 0.0;
  std::string str;
  std::vector<Value> arr;
  std::map<std::string, Value> obj;

  bool isArray() const { return type == Type::Array; }

  const Value& at(const std::string& key) const {
    static const Value kNull{};
    auto it = obj.find(key);
    return it == obj.end() ? kNull : it->second;
  }
};

class ParseError : public std::runtime_error {
 public:
  explicit ParseError(const std::string& msg) : std::runtime_error("JSON parse error: " + msg) {}
};

class Parser {
 public:
  explicit Parser(const std::string& text) : s_(text) {}

  Value parse() {
    Value v = parseValue();
    skipWs();
    return v;
  }

 private:
  const std::string& s_;
  size_t i_ = 0;

  void skipWs() {
    while (i_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[i_]))) ++i_;
  }
  char peek() {
    skipWs();
    if (i_ >= s_.size()) throw ParseError("unexpected end of input");
    return s_[i_];
  }
  char get() {
    if (i_ >= s_.size()) throw ParseError("unexpected end of input");
    return s_[i_++];
  }
  void expect(char c) {
    if (get() != c) throw ParseError(std::string("expected '") + c + "'");
  }

  Value parseValue() {
    char c = peek();
    if (c == '{') return parseObject();
    if (c == '[') return parseArray();
    if (c == '"') {
      Value v;
      v.type = Value::Type::String;
      v.str = parseString();
      return v;
    }
    if (c == 't') { expectLiteral("true"); Value v; v.type = Value::Type::Bool; v.b = true; return v; }
    if (c == 'f') { expectLiteral("false"); Value v; v.type = Value::Type::Bool; v.b = false; return v; }
    if (c == 'n') { expectLiteral("null"); return Value{}; }
    return parseNumber();
  }

  Value parseObject() {
    Value v;
    v.type = Value::Type::Object;
    expect('{');
    if (peek() == '}') { get(); return v; }
    while (true) {
      std::string key = parseString();
      skipWs();
      expect(':');
      v.obj.emplace(std::move(key), parseValue());
      skipWs();
      char c = get();
      if (c == ',') continue;
      if (c == '}') break;
      throw ParseError("expected ',' or '}' in object");
    }
    return v;
  }

  Value parseArray() {
    Value v;
    v.type = Value::Type::Array;
    expect('[');
    if (peek() == ']') { get(); return v; }
    while (true) {
      v.arr.push_back(parseValue());
      skipWs();
      char c = get();
      if (c == ',') continue;
      if (c == ']') break;
      throw ParseError("expected ',' or ']' in array");
    }
    return v;
  }

  std::string parseString() {
    skipWs();
    expect('"');
    std::string out;
    while (true) {
      char c = get();
      if (c == '"') break;
      if (c == '\\') {
        char e = get();
        switch (e) {
          case '"': out += '"'; break;
          case '\\': out += '\\'; break;
          case '/': out += '/'; break;
          case 'n': out += '\n'; break;
          case 't': out += '\t'; break;
          case 'r': out += '\r'; break;
          case 'b': out += '\b'; break;
          case 'f': out += '\f'; break;
          case 'u':
            // grasp_contract.py never emits \u escapes for the fields we read (ids and
            // scores are ASCII); if one shows up anyway, don't mis-decode it silently.
            for (int k = 0; k < 4; ++k) get();
            out += '?';
            break;
          default: throw ParseError("unsupported escape sequence");
        }
      } else {
        out += c;
      }
    }
    return out;
  }

  void expectLiteral(const std::string& lit) {
    if (s_.compare(i_, lit.size(), lit) != 0) throw ParseError("expected literal '" + lit + "'");
    i_ += lit.size();
  }

  Value parseNumber() {
    size_t start = i_;
    if (i_ < s_.size() && s_[i_] == '-') ++i_;
    while (i_ < s_.size() &&
           (std::isdigit(static_cast<unsigned char>(s_[i_])) || s_[i_] == '.' || s_[i_] == 'e' || s_[i_] == 'E' ||
            s_[i_] == '+' || s_[i_] == '-')) {
      ++i_;
    }
    if (start == i_) throw ParseError("expected a number");
    Value v;
    v.type = Value::Type::Number;
    v.num = std::stod(s_.substr(start, i_ - start));
    return v;
  }
};

}  // namespace tinyjson

// One accepted grasp candidate, reduced to what this node needs: the pinch-frame
// (world-frame, same convention as grasp_contract.py's "tcp_pose") target pose.
struct CandidateRow {
  std::string id;
  double score = 0.0;
  double px = 0, py = 0, pz = 0;
  double qx = 0, qy = 0, qz = 0, qw = 1;
};

// Reads grasp_contract.py's write_candidates_json() output, keeps only accepted rows,
// and sorts by score ASCENDING -- grasp_candidates.py's own convention is "lower score
// is better" (least fraction of the gripper's max opening used, i.e. most closing
// margin left; see its comment above the `score = opening_width / gripper_max_opening_m`
// line), so the most-margin candidate is tried first. Throws std::runtime_error with a
// human-readable reason on any I/O or shape problem; the caller treats that the same as
// "zero candidates" (falls back to the hardcoded pose) but records the reason in the
// report JSON so a bad --candidates_path is visible.
std::vector<CandidateRow> loadAcceptedCandidatesSortedByScore(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open candidates_path: " + path);
  std::ostringstream buf;
  buf << in.rdbuf();
  const std::string text = buf.str();

  tinyjson::Parser parser(text);
  tinyjson::Value root = parser.parse();
  const tinyjson::Value& candidates = root.at("candidates");
  if (!candidates.isArray()) throw std::runtime_error("candidates JSON missing a 'candidates' array");

  std::vector<CandidateRow> rows;
  for (const auto& c : candidates.arr) {
    const auto& accepted = c.at("accepted");
    if (accepted.type != tinyjson::Value::Type::Bool || !accepted.b) continue;

    CandidateRow row;
    row.id = c.at("candidate_id").str;
    row.score = c.at("score").num;

    const auto& tcp = c.at("tcp_pose");
    const auto& pos = tcp.at("position_m");
    const auto& quat = tcp.at("quaternion_xyzw");
    if (pos.arr.size() != 3 || quat.arr.size() != 4) {
      throw std::runtime_error("candidate '" + row.id + "' has a malformed tcp_pose");
    }
    row.px = pos.arr[0].num; row.py = pos.arr[1].num; row.pz = pos.arr[2].num;
    row.qx = quat.arr[0].num; row.qy = quat.arr[1].num; row.qz = quat.arr[2].num; row.qw = quat.arr[3].num;
    rows.push_back(std::move(row));
  }
  std::sort(rows.begin(), rows.end(),
            [](const CandidateRow& a, const CandidateRow& b) { return a.score < b.score; });
  return rows;
}

// A place-side position offset (xy only, in world frame) around the nominal
// numerically-solved place pose. This is the S2->S3 place-side counterpart of the
// grasp candidate loop, and the deliberately simple stand-in for ZeroDex's eq. 12
// "collision-aware nearest-valid-position search": instead of a real occupancy-aware
// search, it tries a small fixed 4-neighbor-plus-nominal grid and uses the SAME
// mechanism the grasp loop already uses to judge feasibility -- a real MTC task
// solve (which includes real collision checking against the planning scene) -- rather
// than a standalone collision-check utility, which does not exist in this codebase and
// is out of scope to add here. See the "honest limits" note in the progress report for
// what this is NOT: it does not search density around a task-affordance region, does
// not grow the search radius adaptively, and does not distinguish "no room to place"
// from "IK failed for an unrelated reason" -- it only asks "did the whole task solve".
struct PlaceOffsetCandidate {
  std::string id;
  double dx_m = 0.0;
  double dy_m = 0.0;
};

// Nominal first (so behavior is unchanged from the old fixed-place-pose S2 code when
// the nominal pose already works -- this is only ever a fallback path), then the four
// axis-aligned neighbors at +/- step_m.
std::vector<PlaceOffsetCandidate> makePlaceOffsetCandidates(double step_m) {
  return {
    { "place_nominal", 0.0, 0.0 },
    { "place_x+", step_m, 0.0 },
    { "place_x-", -step_m, 0.0 },
    { "place_y+", 0.0, step_m },
    { "place_y-", 0.0, -step_m },
  };
}

// Clamp every bounded joint variable INTO its declared range with a small inward
// margin (not exactly onto the boundary -- clamping exactly to the boundary was
// observed to still occasionally fail a later, independently-evaluated bounds check,
// presumably from float representation differences between where the bound is read
// vs. re-checked). Inserted at several points in the task graph, not just once after
// CurrentState: this is needed again after "open gripper" (SRDF group_state "open" is
// close to the limit even after the 0.0345 margin fix) since it's easy for a fresh
// float discrepancy to reappear at any later state-producing stage.
std::unique_ptr<mtc::stages::ModifyPlanningScene> makeClampToBoundsStage(const std::string& name) {
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
  stage->setCallback([name](const planning_scene::PlanningScenePtr& scene, const mtc::PropertyMap&) {
    constexpr double kMargin = 1e-3;
    auto& state = scene->getCurrentStateNonConst();
    const auto& model = state.getRobotModel();
    bool changed = false;
    RCLCPP_DEBUG(rclcpp::get_logger("clamp_debug"), "[%s] before: joint7=%.6f joint8=%.6f", name.c_str(),
                 state.getVariablePosition("joint7"), state.getVariablePosition("joint8"));
    for (const std::string& jname : { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7",
                                      "joint8" }) {
      const auto* jm = model->hasJointModel(jname) ? model->getJointModel(jname) : nullptr;
      if (jm == nullptr) continue;
      // Mimic joints (joint8 = -joint7) are recomputed by state.update() below from
      // their mimic source, so clamping them directly is a no-op that gets silently
      // overwritten. Clamp only the independently-driven source joint; because
      // joint7's own bound [0, 0.035] maps exactly onto joint8's bound [-0.035, 0]
      // under the multiplier=-1 relationship, clamping joint7 alone keeps the
      // derived joint8 in bounds too (this was the actual bug: clamping joint8
      // in-place here had no effect once update() re-derived it from joint7).
      if (jm->getMimic() != nullptr) continue;
      const auto& b = model->getVariableBounds(jname);
      if (!b.position_bounded_) continue;
      double v = state.getVariablePosition(jname);
      double clamped = std::min(std::max(v, b.min_position_ + kMargin), b.max_position_ - kMargin);
      if (clamped != v) {
        state.setVariablePosition(jname, clamped);
        changed = true;
      }
    }
    if (changed) state.update();
    RCLCPP_DEBUG(rclcpp::get_logger("clamp_debug"), "[%s] after:  joint7=%.6f joint8=%.6f changed=%d", name.c_str(),
                 state.getVariablePosition("joint7"), state.getVariablePosition("joint8"), changed);
  });
  return stage;
}

// Outcome of one (candidate, task) planning attempt.
struct AttemptResult {
  bool initialized = false;
  bool planned = false;
  std::string error;
  size_t num_solutions = 0;
  std::vector<std::string> failures;
};

// Builds the full pick-place stage graph into `task` for the given grasp/place target
// poses, then calls task.init() + task.plan(1). Each candidate gets its OWN fresh Task
// (see loop in main): MTC tasks are not designed to be re-planned in place after a
// failure, and reusing one across candidates risks stale planning-scene state leaking
// from a failed attempt into the next one.
// `extra_obstacle`, if non-null, is added to the scene alongside the floor/cube. This
// exists to let a test inject a synthetic collision object (e.g. to verify the
// place-offset fallthrough loop in main() actually falls through, not just "looks like
// it would" -- see docs/progress/grasp_motion_progress_report.md for that test run);
// it is nullptr in normal operation and adds nothing to the scene.
AttemptResult buildAndPlan(mtc::Task& task, const rclcpp::Node::SharedPtr& node, const std::string& world_frame,
                           double cube_x, double cube_y, double cube_z, double cube_size,
                           const PoseStamped& grasp_pose, const PoseStamped& place_pose,
                           const CollisionObject* extra_obstacle = nullptr) {
  AttemptResult result;

  task.stages()->setName("robot129 pick_place");
  task.loadRobotModel(node);
  task.setProperty("group", std::string("arm"));
  task.setProperty("eef", std::string("robot129_gripper"));
  task.setProperty("ik_frame", std::string("gripper_base"));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl", "RRTConnect");
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.005);

  const std::string arm_group = "arm";
  const std::string gripper_group = "gripper";
  const std::string hand_frame = "gripper_base";
  const Eigen::Translation3d pinch_offset(0.0, 0.0, 0.125);

  try {
    mtc::Stage* current_state_stage = nullptr;
    {
      auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state_stage = stage.get();
      task.add(std::move(stage));
    }
    (void)current_state_stage;

    // Observed against the live Isaac state (2026-09-18): joint8 (URDF mimic of
    // joint7, multiplier -1, range [-0.035, 0]) is driven by its OWN independent PD
    // actuator in sim/scripts/run_robot129_ros_webrtc.py, not derived from joint7 in
    // PhysX -- real tracking error lets its measured position drift past its declared
    // bound under gravity/contact, which CheckStartStateBounds then rejects with zero
    // tolerance. This is a genuine sim fidelity gap (the URDF NewtonMimicAPI is not
    // perfectly enforced by PhysX, matching the S0 UNVERIFIED note on that), not a
    // planning bug, so live-read states are clamped into bounds with a safety margin
    // rather than silently loosening the URDF limits or disabling the safety check.
    task.add(makeClampToBoundsStage("clamp current state to bounds"));

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("add scene objects");
      stage->addObject(makeBox("support_floor", world_frame, 4.0, 4.0, 0.05, 0.0, 0.0, -0.025));
      stage->addObject(makeBox("target_cube", world_frame, cube_size, cube_size, cube_size, cube_x, cube_y, cube_z));
      stage->allowCollisions("target_cube", std::vector<std::string>{ "link7", "link8", "gripper_base" }, true);
      // The real floor top and base_link's bottom sit at the same z=0 plane (matching
      // the live pick_place scene exactly), so they register as touching/colliding at
      // the boundary; this is expected geometry, not a real collision to avoid. Same
      // reasoning for the cube resting on the floor before it's picked up. This is a
      // task-wide simplification appropriate for a single known candidate (S2); S3's
      // collision-aware placement refinement should toggle this only around the
      // moments contact is actually expected instead of disabling it globally.
      stage->allowCollisions("support_floor", std::vector<std::string>{ "base_link", "target_cube" }, true);
      if (extra_obstacle != nullptr) stage->addObject(*extra_obstacle);
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", sampling_planner);
      stage->setGroup(gripper_group);
      stage->setGoal("open");
      task.add(std::move(stage));
    }
    mtc::Stage* open_gripper_clamp_stage = nullptr;
    {
      auto clamp = makeClampToBoundsStage("clamp after open gripper");
      open_gripper_clamp_stage = clamp.get();
      task.add(std::move(clamp));
    }

    {
      auto stage = std::make_unique<mtc::stages::Connect>(
          "connect to pregrasp", mtc::stages::Connect::GroupPlannerVector{ { arm_group, sampling_planner } });
      stage->setTimeout(10.0);
      task.add(std::move(stage));
    }

    {
      auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
      grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
        stage->properties().set("link", hand_frame);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.04, 0.06);
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = world_frame;
        vec.vector.z = -1.0;  // descend toward the grasp pose
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }

      {
        auto poses = std::make_unique<mtc::stages::FixedCartesianPoses>("grasp pose candidate");
        poses->addPose(grasp_pose);
        // Monitor the clamped post-"open gripper" state, not raw current_state_stage:
        // FixedCartesianPoses::compute() diffs its scene from the MONITORED stage's
        // own end scene, not from whatever sibling stage happens to run immediately
        // before it in the task. Monitoring current_state_stage here was silently
        // discarding every clamp fix downstream (observed: the joint8-out-of-bounds
        // error kept recurring at "close gripper" because this candidate re-based the
        // whole pick branch off current_state's raw, unclamped gripper values).
        poses->setMonitoredStage(open_gripper_clamp_stage);
        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(poses));
        wrapper->setGroup(arm_group);
        wrapper->setEndEffector("robot129_gripper");
        wrapper->setIKFrame(pinch_offset, hand_frame);
        wrapper->setMaxIKSolutions(4);
        wrapper->setMinSolutionDistance(0.02);
        wrapper->setIgnoreCollisions(false);
        // target_pose is set dynamically by FixedCartesianPoses::compute() on its
        // spawned InterfaceState, not statically inherited from the parent -- must be
        // pulled from the child/interface explicitly or task.init() rejects it as
        // "declared, but undefined" before any stage ever computes anything.
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        grasp->insert(std::move(wrapper));
      }
      task.add(std::move(grasp));
    }
    task.add(makeClampToBoundsStage("clamp before close gripper"));

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", sampling_planner);
      stage->setGroup(gripper_group);
      stage->setGoal("closed");
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("target_cube", hand_frame);
      task.add(std::move(stage));
    }

    mtc::Stage* lift_stage_ptr = nullptr;
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().set("link", hand_frame);
      stage->setGroup(arm_group);
      stage->setMinMaxDistance(0.04, 0.06);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      lift_stage_ptr = stage.get();
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::Connect>(
          "connect to place", mtc::stages::Connect::GroupPlannerVector{ { arm_group, sampling_planner } });
      stage->setTimeout(10.0);
      task.add(std::move(stage));
    }

    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
      place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("lower to place", cartesian_planner);
        stage->properties().set("link", hand_frame);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.04, 0.06);
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = world_frame;
        vec.vector.z = -1.0;
        stage->setDirection(vec);
        place->insert(std::move(stage));
      }

      {
        auto poses = std::make_unique<mtc::stages::FixedCartesianPoses>("place pose candidate");
        poses->addPose(place_pose);
        // Monitor the post-lift state (gripper closed, object attached), not
        // current_state_stage -- see the matching comment on the grasp candidate above.
        poses->setMonitoredStage(lift_stage_ptr);
        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(poses));
        wrapper->setGroup(arm_group);
        wrapper->setEndEffector("robot129_gripper");
        wrapper->setIKFrame(pinch_offset, hand_frame);
        wrapper->setMaxIKSolutions(4);
        wrapper->setMinSolutionDistance(0.02);
        wrapper->setIgnoreCollisions(false);
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        place->insert(std::move(wrapper));
      }
      task.add(std::move(place));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper release", sampling_planner);
      stage->setGroup(gripper_group);
      stage->setGoal("open");
      task.add(std::move(stage));
    }
    task.add(makeClampToBoundsStage("clamp after open gripper release"));

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("target_cube", hand_frame);
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().set("link", hand_frame);
      stage->setGroup(arm_group);
      stage->setMinMaxDistance(0.03, 0.06);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      task.add(std::move(stage));
    }

    task.init();
    result.planned = static_cast<bool>(task.plan(1));
    result.initialized = true;
  } catch (const mtc::InitStageException& e) {
    // InitStageException::what() is a short generic message; operator<< carries the
    // actual per-stage detail we need to debug property propagation / IK setup.
    result.initialized = false;
    std::ostringstream detail;
    detail << e;
    result.error = detail.str();
  } catch (const std::exception& e) {
    result.initialized = false;
    result.error = e.what();
  }

  result.num_solutions = task.solutions().size();
  for (const auto& f : task.failures()) result.failures.push_back(f->comment());
  return result;
}

using LabeledPose = std::tuple<std::string, double, geometry_msgs::msg::PoseStamped, bool, std::string>;

// Appends one ARROW + TEXT_VIEW_FACING marker pair per labeled pose into `array`,
// under the given namespace (and `ns + "_labels"` for the text). `id` is threaded
// through by reference so grasp-side and place-side groups never collide on marker id.
void appendCandidateMarkerGroup(visualization_msgs::msg::MarkerArray& array, const std::string& frame_id,
                                const rclcpp::Time& stamp, const std::string& ns,
                                const std::vector<LabeledPose>& labeled_attempts, int& id) {
  for (const auto& [candidate_id, score, pose, chosen, label_suffix] : labeled_attempts) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = frame_id;
    arrow.header.stamp = stamp;
    arrow.ns = ns;
    arrow.id = id++;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose = pose.pose;
    arrow.scale.x = 0.06;   // arrow length
    arrow.scale.y = 0.010;  // head diameter
    arrow.scale.z = 0.010;  // head length
    arrow.color.a = 0.9;
    arrow.color.r = chosen ? 0.10 : 0.85;
    arrow.color.g = chosen ? 0.75 : 0.15;
    arrow.color.b = 0.15;
    array.markers.push_back(arrow);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = frame_id;
    text.header.stamp = stamp;
    text.ns = ns + "_labels";
    text.id = id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose = pose.pose;
    text.pose.position.z += 0.035;
    text.scale.z = 0.014;  // text height
    text.color = arrow.color;
    text.color.a = 1.0;
    std::ostringstream label;
    label << candidate_id << " score=" << std::fixed << std::setprecision(3) << score << "\n"
          << (chosen ? "CHOSEN" : "REJECTED");
    if (!label_suffix.empty()) label << "\n" << label_suffix;
    text.text = label.str();
    array.markers.push_back(text);
  }
}

// One RViz marker pair (ARROW + TEXT_VIEW_FACING label) per attempted (grasp, place)
// pair, on BOTH the grasp pose and the place pose, so "why did it pick this grasp and
// this place point" is something you can literally look at in RViz instead of only
// reading the report JSON's candidate_attempts array. Green = the candidate that was
// actually chosen (task.plan() succeeded); red = every candidate tried and rejected
// before it, with the first MTC failure reason (if any) as the label text. The ARROW
// uses the candidate's own orientation (pose-based Arrow marker points along its local
// +X) -- for grasp poses that's the axis generate_grasp_candidates() sweeps over yaw
// candidates around, so a cluster of different-colored arrows fanned out around the
// same point is a direct picture of "these are the yaws that were tried, this is the
// one that worked".
visualization_msgs::msg::MarkerArray buildCandidateMarkers(const std::string& frame_id, const rclcpp::Time& stamp,
                                                            const std::vector<LabeledPose>& grasp_labeled,
                                                            const std::vector<LabeledPose>& place_labeled) {
  visualization_msgs::msg::MarkerArray array;

  // One DELETEALL clears everything this publisher previously sent, regardless of
  // namespace -- otherwise stale candidates from an earlier invocation linger in RViz.
  visualization_msgs::msg::Marker clear_all;
  clear_all.header.frame_id = frame_id;
  clear_all.header.stamp = stamp;
  clear_all.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear_all);

  int id = 0;
  appendCandidateMarkerGroup(array, frame_id, stamp, "grasp_candidates", grasp_labeled, id);
  appendCandidateMarkerGroup(array, frame_id, stamp, "place_candidates", place_labeled, id);
  return array;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("robot129_mtc_pick_place", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor] { executor.spin(); });

  std::string report_path;
  node->get_parameter_or<std::string>("report_path", report_path, "/tmp/robot129_mtc_pick_place.json");
  std::string candidates_path;
  node->get_parameter_or<std::string>("candidates_path", candidates_path, "");
  int max_candidates = 8;
  node->get_parameter_or<int>("max_candidates", max_candidates, 8);
  double place_offset_step_m = 0.015;
  node->get_parameter_or<double>("place_offset_step_m", place_offset_step_m, 0.015);
  std::string world_frame = "world";

  // S0-chosen scene layout (docs/progress/grasp_motion_s0_inventory.md section 4),
  // now loaded from research/configs/scene_geometry.json -- the single source of truth
  // introduced 2026-09-20 after an audit found this exact cube geometry independently
  // hardcoded in FOUR places (this file, sim/scripts/run_robot129_ros_webrtc.py,
  // tools/generate_s2_cube_candidates.py, research/scripts/s5_pilot_hammer.py) with no
  // shared source; the cube numbers happened to still agree everywhere, but the hammer
  // copy had already drifted (see that file's provenance note). The hardcoded values
  // below are kept ONLY as a fallback if the file can't be read, so this node still
  // starts if the path is wrong or missing, logged loudly rather than silently.
  double cube_x = 0.32, cube_y = 0.0, cube_z = 0.0175, cube_size = 0.035;
  double place_x = 0.27, place_y = -0.12, place_z = 0.0175;
  std::string scene_geometry_path;
  node->get_parameter_or<std::string>(
      "scene_geometry_path", scene_geometry_path,
      "/mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913/research/configs/scene_geometry.json");
  try {
    std::ifstream geometry_in(scene_geometry_path);
    if (!geometry_in) throw std::runtime_error("cannot open scene_geometry_path: " + scene_geometry_path);
    std::ostringstream geometry_buf;
    geometry_buf << geometry_in.rdbuf();
    const std::string geometry_text = geometry_buf.str();
    tinyjson::Parser geometry_parser(geometry_text);
    tinyjson::Value geometry_root = geometry_parser.parse();
    const auto& cube_geom = geometry_root.at("cube");
    cube_x = cube_geom.at("xy_default_m").arr.at(0).num;
    cube_y = cube_geom.at("xy_default_m").arr.at(1).num;
    cube_z = cube_geom.at("rest_z_m").num;
    cube_size = cube_geom.at("size_m").num;
    const auto& place_geom = geometry_root.at("place");
    place_x = place_geom.at("xy_default_m").arr.at(0).num;
    place_y = place_geom.at("xy_default_m").arr.at(1).num;
    place_z = cube_z;  // place surface height mirrors the cube's own rest height, as before
  } catch (const std::exception& e) {
    RCLCPP_WARN(node->get_logger(),
                "failed to load scene_geometry_path=%s (%s); falling back to hardcoded S0 cube layout",
                scene_geometry_path.c_str(), e.what());
  }

  // Nominal, numerically-solved pinch-frame place pose (see file header). The place
  // SIDE is now candidate-driven too -- see PlaceOffsetCandidate above -- but every
  // offset is applied around this same nominal xy/z/orientation, not a different base.
  PoseStamped nominal_place_pose =
      makePose(world_frame, place_x, place_y, place_z, 0.207588, 0.978216, -0.000013, 0.000043);
  const std::vector<PlaceOffsetCandidate> place_offsets = makePlaceOffsetCandidates(place_offset_step_m);

  // Test-only hook (default off): inject a small collision box exactly at the nominal
  // place xy so the place_nominal offset is forced to fail and the loop below must
  // fall through to a neighbor -- proves the fallthrough logic actually runs, the same
  // way the earlier G_UNREACHABLE candidate test proved the grasp-side loop does.
  // Sized well under place_offset_step_m's default 1.5cm so real neighbor offsets are
  // unaffected. See docs/progress/grasp_motion_progress_report.md for that test run.
  bool debug_block_place_nominal = false;
  node->get_parameter_or<bool>("debug_block_place_nominal", debug_block_place_nominal, false);
  CollisionObject debug_obstacle;
  const CollisionObject* debug_obstacle_ptr = nullptr;
  if (debug_block_place_nominal) {
    // Short and table-height only (NOT a tall pillar): a tall obstacle intersects
    // upper-arm links (e.g. link3) during the swept "connect to place" motion for
    // every offset, not just at the final TCP position, which made every offset fail
    // identically on first attempt at this hook (observed 2026-09-19) -- keeping it
    // short and low means only the final placement pose is actually contested.
    debug_obstacle = makeBox("debug_place_blocker", world_frame, 0.02, 0.02, 0.05, place_x, place_y, place_z + 0.02);
    debug_obstacle_ptr = &debug_obstacle;
  }

  std::vector<CandidateRow> candidates;
  std::string candidates_load_error;
  if (!candidates_path.empty()) {
    try {
      candidates = loadAcceptedCandidatesSortedByScore(candidates_path);
    } catch (const std::exception& e) {
      candidates_load_error = e.what();
    }
  }

  struct NamedAttempt {
    std::string candidate_id;  // composite: "<grasp_id>+<place_id>"
    std::string grasp_candidate_id;
    std::string place_candidate_id;
    double score;  // the grasp candidate's score; place offsets don't carry their own
    AttemptResult result;
    PoseStamped grasp_pose;  // for RViz candidate markers -- see buildCandidateMarkers()
    PoseStamped place_pose;
  };
  std::vector<NamedAttempt> attempts;
  std::unique_ptr<mtc::Task> chosen_task;
  std::string chosen_candidate_id, chosen_grasp_candidate_id, chosen_place_candidate_id;

  // Grasp candidates to try, in order: either the S3-generated list (best-margin
  // first) or a single hardcoded fallback pose (original S2 behavior, used when no
  // --candidates_path is given, the file failed to load, or it had zero accepted
  // rows -- candidates_load_error, if any, is still reported in the JSON).
  std::vector<CandidateRow> grasp_attempts_list = candidates;
  if (grasp_attempts_list.empty()) {
    CandidateRow fallback;
    fallback.id = "hardcoded_single";
    fallback.score = 0.0;
    fallback.px = cube_x; fallback.py = cube_y; fallback.pz = cube_z;
    fallback.qx = 0.000002; fallback.qy = 1.0; fallback.qz = 0.0; fallback.qw = -0.00011;
    grasp_attempts_list.push_back(fallback);
  }
  const size_t n_grasp =
      std::min(grasp_attempts_list.size(), static_cast<size_t>(std::max(max_candidates, 0)));

  // ZeroDex Sec 3.4's (T_robot, eta) loop, extended to both sides: for each grasp
  // candidate g (best margin first), for each place offset o (nominal first), run the
  // full motion generator (a real MTC task solve, collision checking included); take
  // the first (g, o) pair that fully succeeds. Every attempt is still recorded, not
  // just the winner, in `attempts` / the report JSON's candidate_attempts.
  for (size_t gi = 0; gi < n_grasp && !chosen_task; ++gi) {
    const CandidateRow& c = grasp_attempts_list[gi];
    PoseStamped grasp_pose = makePose(world_frame, c.px, c.py, c.pz, c.qx, c.qy, c.qz, c.qw);
    for (const auto& off : place_offsets) {
      PoseStamped place_try = nominal_place_pose;
      place_try.pose.position.x += off.dx_m;
      place_try.pose.position.y += off.dy_m;

      auto task = std::make_unique<mtc::Task>();
      auto result = buildAndPlan(*task, node, world_frame, cube_x, cube_y, cube_z, cube_size, grasp_pose,
                                  place_try, debug_obstacle_ptr);
      const std::string composite_id = c.id + "+" + off.id;
      attempts.push_back({ composite_id, c.id, off.id, c.score, result, grasp_pose, place_try });
      RCLCPP_INFO(node->get_logger(), "candidate %s (grasp_score=%.4f): %s", composite_id.c_str(), c.score,
                  result.planned ? "PASS" : "FAIL");
      if (result.planned) {
        chosen_task = std::move(task);
        chosen_candidate_id = composite_id;
        chosen_grasp_candidate_id = c.id;
        chosen_place_candidate_id = off.id;
        break;  // first (grasp, place) pair that fully solves wins.
      }
    }
  }

  const bool planned = chosen_task != nullptr;
  const AttemptResult& last = attempts.back().result;  // chosen attempt if planned, else the final failure

  std::ostringstream json;
  json << "{\n";
  json << "  \"status\": \"" << (planned ? "PASS" : "FAIL") << "\",\n";
  json << "  \"initialized\": " << (last.initialized ? "true" : "false") << ",\n";
  json << "  \"planned_solutions\": " << last.num_solutions << ",\n";
  json << "  \"error\": \"" << jsonEscape(last.error) << "\",\n";
  json << "  \"candidates_path\": \"" << jsonEscape(candidates_path) << "\",\n";
  json << "  \"candidates_load_error\": \"" << jsonEscape(candidates_load_error) << "\",\n";
  json << "  \"candidates_considered\": " << candidates.size() << ",\n";
  json << "  \"place_offset_step_m\": " << place_offset_step_m << ",\n";
  json << "  \"place_offsets_per_grasp\": " << place_offsets.size() << ",\n";
  json << "  \"chosen_candidate_id\": \"" << jsonEscape(chosen_candidate_id) << "\",\n";
  json << "  \"chosen_grasp_candidate_id\": \"" << jsonEscape(chosen_grasp_candidate_id) << "\",\n";
  json << "  \"chosen_place_candidate_id\": \"" << jsonEscape(chosen_place_candidate_id) << "\",\n";

  json << "  \"candidate_attempts\": [\n";
  for (size_t i = 0; i < attempts.size(); ++i) {
    const auto& a = attempts[i];
    if (i) json << ",\n";
    json << "    {\n";
    json << "      \"candidate_id\": \"" << jsonEscape(a.candidate_id) << "\",\n";
    json << "      \"grasp_candidate_id\": \"" << jsonEscape(a.grasp_candidate_id) << "\",\n";
    json << "      \"place_candidate_id\": \"" << jsonEscape(a.place_candidate_id) << "\",\n";
    json << "      \"score\": " << a.score << ",\n";
    json << "      \"initialized\": " << (a.result.initialized ? "true" : "false") << ",\n";
    json << "      \"planned\": " << (a.result.planned ? "true" : "false") << ",\n";
    json << "      \"planned_solutions\": " << a.result.num_solutions << ",\n";
    json << "      \"error\": \"" << jsonEscape(a.result.error) << "\",\n";
    json << "      \"failures\": [";
    for (size_t k = 0; k < a.result.failures.size(); ++k) {
      if (k) json << ",";
      json << "\"" << jsonEscape(a.result.failures[k]) << "\"";
    }
    json << "]\n";
    json << "    }";
  }
  json << "\n  ],\n";

  json << "  \"failures\": [";
  for (size_t k = 0; k < last.failures.size(); ++k) {
    if (k) json << ",";
    json << "\"" << jsonEscape(last.failures[k]) << "\"";
  }
  json << "],\n";

  json << "  \"sub_trajectories\": [\n";
  if (planned && !chosen_task->solutions().empty()) {
    moveit_task_constructor_msgs::msg::Solution solution_msg;
    (*chosen_task->solutions().begin())->toMsg(solution_msg, nullptr);
    bool first = true;
    for (const auto& sub : solution_msg.sub_trajectory) {
      if (!first) json << ",\n";
      first = false;
      json << "    {\n";
      json << "      \"comment\": \"" << jsonEscape(sub.info.comment) << "\",\n";
      json << "      \"joint_names\": [";
      const auto& jt = sub.trajectory.joint_trajectory;
      for (size_t i = 0; i < jt.joint_names.size(); ++i) {
        if (i) json << ",";
        json << "\"" << jsonEscape(jt.joint_names[i]) << "\"";
      }
      json << "],\n";
      json << "      \"points\": [\n";
      for (size_t i = 0; i < jt.points.size(); ++i) {
        if (i) json << ",\n";
        const auto& pt = jt.points[i];
        json << "        {\"positions\": [";
        for (size_t k = 0; k < pt.positions.size(); ++k) {
          if (k) json << ",";
          json << pt.positions[k];
        }
        json << "], \"time_from_start_s\": "
             << (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9) << "}";
      }
      json << "\n      ]\n";
      json << "    }";
    }
  }
  json << "\n  ]\n";
  json << "}\n";

  std::ofstream out(report_path);
  out << json.str();
  out.close();

  RCLCPP_INFO(node->get_logger(),
              "MTC pick_place status=%s chosen_candidate=%s attempts=%zu/%zu solutions=%zu report=%s",
              planned ? "PASS" : "FAIL", chosen_candidate_id.c_str(), attempts.size(), candidates.size(),
              last.num_solutions, report_path.c_str());

  // Candidate decision markers for RViz (MoveIt's "Motion Planning Tasks" or a bare
  // MarkerArray display both work): one ARROW+label pair per attempted (grasp, place)
  // pair, on BOTH the grasp pose and the place pose, so the "why this grasp and this
  // place point, not another" trace in candidate_attempts is something you can
  // actually look at, not just read as JSON. transient_local durability means a
  // MarkerArray display added to RViz AFTER this node has already published (this node
  // exits right after planning, it doesn't stay up) still receives the last message.
  {
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "grasp_candidate_markers", qos);
    std::vector<LabeledPose> grasp_labeled, place_labeled;
    for (const auto& a : attempts) {
      const bool chosen = a.candidate_id == chosen_candidate_id && planned;
      std::string suffix;
      if (!chosen && !a.result.failures.empty()) suffix = a.result.failures.front();
      else if (!chosen && !a.result.error.empty()) suffix = a.result.error;
      grasp_labeled.emplace_back(a.grasp_candidate_id, a.score, a.grasp_pose, chosen, suffix);
      place_labeled.emplace_back(a.place_candidate_id, a.score, a.place_pose, chosen, suffix);
    }
    auto markers = buildCandidateMarkers(world_frame, node->now(), grasp_labeled, place_labeled);
    marker_pub->publish(markers);
    RCLCPP_INFO(node->get_logger(), "published %zu grasp + %zu place candidate markers on %s (frame=%s)",
                grasp_labeled.size(), place_labeled.size(), marker_pub->get_topic_name(), world_frame.c_str());
    // Give the transient_local publish time to actually reach DDS before this node
    // (which does not otherwise stay alive) shuts down.
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return planned ? 0 : 1;
}
