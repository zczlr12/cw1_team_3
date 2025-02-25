import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cw1_team_3',
            executable='cw1_team_3',
            name='obstacle_follower'),
    ])
