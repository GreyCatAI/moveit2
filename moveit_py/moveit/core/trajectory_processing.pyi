from robot_trajectory import RobotTrajectory

def limit_max_cartesian_speed(
    trajectory: RobotTrajectory, speed: float, link_name: str = ""
) -> bool: ...
