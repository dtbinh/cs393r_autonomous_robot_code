**CS393R Autonomous Robot**

This respository is a fork of the UT Austin Villa robot soccer team's repo that has been modified to use a Kinematics-based Adaptively-parameterized Closed-loop Kicking (KACK) behavior in place of the old joint-keyframe-based kick, which served as the final project of the course cs393r. Run the kick behavior as you normally would and KACK will be executed instead. This project was finished by Yan Pei and Joshua James from UT Austin. KACK has strong potential to enjoy further application to the Austin Villa Robocup Team's kick motion.

Relevant files:

**core/motion/KickModule.cpp:**
Contains the Cartesian trajectory generation code.

**core/motion/KACK.hpp:**
Contains the KACK controller with balancing and whole-body optimization routines.

**core/motion/matec/*:**
Contains a ROS-free port of the MATEC control suite, written by Joshua James (https://bitbucket.org/Jraipxg/matec_control). This code is used by the KACK controller, primarily to load the robot model and compute the kinematics.

**core/vision/ImageProcessor.cpp:**
Contains code for detecting the position of the goal and ball.

**core/memory/RobotStateBlock.h:**
Modified to pass information about the goal and ball positions to the motion module.
