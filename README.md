This repo contains ROS 2 packages, launch files, and testing scripts used by the **BYU Agricultural Robotics Team**. 
If you're doing code development, feel free to clone this repo directly, make a new branch, and start coding and pushing some changes.
Otherwise (if you're looking to set up a new robot from scratch), see this repo for instructions on how to get our custom Docker image running instead: https://github.com/BYUAgrobotics/AgrobotSetup

A quick high-level overview of the repo:
- **microros_tools/** - helpful scripts for launching the microROS agent ("agent.sh"), syncing custom message changes ("msg_sync.sh"), and testing Teensy board connections ("test.sh").
A description of what each script does is included as a header comment in the file.
- **src/** - contains the ROS 2 packages we use to run the robot.
Of note are "src/agrobot_cpp/" and "src/agrobot_py/" (miscellaneous custom nodes running PID controls, odometry conversions, etc), as well as "src/agrobot_interfaces/" (contains custom ROS message and service declarations).
The ROS launch files for different start configurations are included in "src/agrobot_py/launch/".
