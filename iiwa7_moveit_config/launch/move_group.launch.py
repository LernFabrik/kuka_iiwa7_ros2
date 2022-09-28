from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("iiwa", package_name="iiwa7_moveit_config").joint_limits(file_path='config/joint_limits.yaml').to_moveit_configs()
    return generate_move_group_launch(moveit_config)