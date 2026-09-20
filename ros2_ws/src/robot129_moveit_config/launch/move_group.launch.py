from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    config = (MoveItConfigsBuilder('robot129', package_name='robot129_moveit_config')
              .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
              .to_moveit_configs())
    return generate_move_group_launch(config)
