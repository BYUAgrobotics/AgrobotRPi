##########################################################
# Launch: base (bash param in 'start.sh')
# Created by Nelson Durrant, Sep 2024
##########################################################

import launch
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():

    config_file = "/home/agrobot/config/robot_config.yaml"
    
    return launch.LaunchDescription([
        
        # Start the PID control node
        launch_ros.actions.Node(
            package='agrobot_cpp',
            executable='pid_control',
            parameters=[config_file]
        ),
    ])