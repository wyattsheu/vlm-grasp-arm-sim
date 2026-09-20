#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <fstream>
#include <map>
#include <thread>
using namespace std::chrono_literals;
namespace mtc = moveit::task_constructor;

static std::map<std::string,double> arm(std::initializer_list<double> q) {
 std::map<std::string,double> out; int i=1; for(double v:q) out["joint"+std::to_string(i++)]=v; return out;
}
static std::map<std::string,double> grip(double a,double b) { return {{"joint7",a},{"joint8",b}}; }
int main(int argc,char** argv) {
 rclcpp::init(argc,argv);
 rclcpp::NodeOptions options; options.automatically_declare_parameters_from_overrides(true);
 auto node=std::make_shared<rclcpp::Node>("robot129_mtc_plan",options);
 auto pub=node->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
 rclcpp::executors::MultiThreadedExecutor executor; executor.add_node(node);
 std::thread spin([&]{executor.spin();});
 sensor_msgs::msg::JointState state; for(int i=1;i<=8;i++) state.name.push_back("joint"+std::to_string(i));
 state.position={0,1.20,-1.25,0,0.15,0,0.035,-0.035};
 for(int i=0;i<10;i++){state.header.stamp=node->now();pub->publish(state);std::this_thread::sleep_for(20ms);}
 auto timer=node->create_wall_timer(20ms,[&]{state.header.stamp=node->now();pub->publish(state);});
 mtc::Task task; task.stages()->setName("Robot 129 simulation pick-place stages"); task.loadRobotModel(node);
 auto planner=std::make_shared<mtc::solvers::JointInterpolationPlanner>();
 task.add(std::make_unique<mtc::stages::CurrentState>("CURRENT_STATE"));
 auto add=[&](const std::string& name,const std::string& group,const std::map<std::string,double>& goal){
  auto stage=std::make_unique<mtc::stages::MoveTo>(name,planner); stage->setGroup(group); stage->setGoal(goal); task.add(std::move(stage));};
 add("APPROACH","arm",arm({0.05,1.22,-1.30,0.02,0.15,0}));
 add("GRASP","gripper",grip(0.004,-0.004));
 add("LIFT","arm",arm({0.05,1.35,-1.55,0.02,0.15,0}));
 add("TRANSIT","arm",arm({0.25,1.35,-1.55,0.10,0.20,0}));
 add("PREPLACE","arm",arm({0.20,1.28,-1.45,0.08,0.18,0}));
 add("RELEASE","gripper",grip(0.035,-0.035));
 add("RETREAT","arm",arm({0,1.20,-1.25,0,0.15,0}));
 bool initialized=true, planned=false; std::string error;
 try { task.init(); planned=static_cast<bool>(task.plan(1)); }
 catch(const std::exception& e){ initialized=false; error=e.what(); }
 std::string path; node->get_parameter_or<std::string>("report_path", path, "/tmp/robot129_mtc_plan.json");
 std::ofstream out(path); out << "{\n  \"status\": \""<<(planned?"PASS":"FAIL")<<"\",\n  \"simulation_only\": true,\n  \"initialized\": "<<(initialized?"true":"false")<<",\n  \"planned_solutions\": "<<task.solutions().size()<<",\n  \"stages\": [\"CURRENT_STATE\",\"APPROACH\",\"GRASP\",\"LIFT\",\"TRANSIT\",\"PREPLACE\",\"RELEASE\",\"RETREAT\"],\n  \"hardware_commands\": 0,\n  \"error\": \""<<error<<"\"\n}\n"; out.close();
 RCLCPP_INFO(node->get_logger(),"MTC status=%s solutions=%zu report=%s",planned?"PASS":"FAIL",task.solutions().size(),path.c_str());
 executor.cancel(); spin.join(); rclcpp::shutdown(); return planned?0:1;
}
