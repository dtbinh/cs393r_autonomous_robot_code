This respository is a fork of the UT Austin Villa robot soccer team's repo that has been modified to use a Kinematics-based Adaptively-parameterized Closed-loop Kicking (KACK) behavior in place of the old joint-keyframe-based kick. Run the kick behavior as you normally would and KACK will be executed instead.

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