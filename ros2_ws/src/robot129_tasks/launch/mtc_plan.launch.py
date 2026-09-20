from launch.actions import TimerAction, RegisterEventHandler, EmitEvent
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
 config=(MoveItConfigsBuilder('robot129',package_name='robot129_moveit_config').planning_pipelines(default_planning_pipeline='ompl',pipelines=['ompl']).to_moveit_configs())
 launch=generate_move_group_launch(config)
 mtc=Node(package='robot129_tasks',executable='robot129_mtc_plan',output='screen',parameters=[config.to_dict(),{'report_path':'/mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913/out/lesson_09/mtc_plan.json'}])
 launch.add_action(TimerAction(period=6.0,actions=[mtc]))
 launch.add_action(RegisterEventHandler(OnProcessExit(target_action=mtc,on_exit=[EmitEvent(event=Shutdown(reason='MTC acceptance complete'))])))
 return launch
