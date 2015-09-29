import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *
import cfgpose, cfgstiff
from memory import sensors, walk_request, walk_response, kick_request, joint_commands, behavior_mem, joint_angles, vision_frame_info
from kick_trajectory import trajectory, names
import numpy 

#Core: ['ALWalkParamBlock', 'ALWalkParamBlock_swigregister', 'ARM_JOINT_FIRST', 'ARM_JOINT_LAST', 'AudioProcessingBlock', 'AudioProcessingBlock_fromName_AudioState', 'AudioProcessingBlock_getName', 'AudioProcessingBlock_swigregister', 'BALL_RADIUS', 'BHUMAN2011_WALK', 'BHUMAN2013_WALK', 'BHWalkParameters', 'BHWalkParameters_swigregister', 'BODY_JOINT_OFFSET', 'BOTTOM_CAM', 'BehaviorBlock', 'BehaviorBlock_swigregister', 'BehaviorModule', 'BehaviorModuleLog', 'BehaviorModule_swigregister', 'BehaviorParamBlock', 'BehaviorParamBlock_swigregister', 'BodyFrame', 'BodyFrame_swigregister', 'BodyModelBlock', 'BodyModelBlock_swigregister', 'BodyPart', 'BodyPart_swigregister', 'CARTESIAN', 'CAUTIOUS_DEFENDER', 'CENTI', 'CHASER', 'CIRCLE_DIAMETER', 'CIRCLE_HEX_LENGTH', 'CIRCLE_RADIUS', 'CONFIG_ID', 'CORE_INIT', 'CORE_ROBOT', 'CORE_SIM', 'CORE_TOOL', 'CORE_TOOLSIM', 'CORE_TOOL_NO_VISION', 'CROSS_OFFSET', 'Camera', 'CameraBlock', 'CameraBlock_swigregister', 'CameraParams', 'CameraParams_swigregister', 'Camera_swigregister', 'ChestBlue', 'ChestGreen', 'ChestRed', 'Circle', 'Circle_swigregister', 'Cluster', 'ClusterKickStrategy', 'ClusterKickStrategy_getValue', 'ClusterKickStrategy_swigregister', 'Cluster_swigregister', 'CoachPacket', 'CoachPacket_swigregister', 'CornerKickStrategy', 'CornerKickStrategy_swigregister', 'DEFENDER', 'DEG_T_RAD', 'Deg2Rad', 'Dive', 'Dive_swigregister', 'Dribble', 'EPSILON', 'EarLeft0', 'EarLeft108', 'EarLeft144', 'EarLeft180', 'EarLeft216', 'EarLeft252', 'EarLeft288', 'EarLeft324', 'EarLeft36', 'EarLeft72', 'EarRight0', 'EarRight108', 'EarRight144', 'EarRight180', 'EarRight216', 'EarRight252', 'EarRight288', 'EarRight324', 'EarRight36', 'EarRight72', 'EnumName', 'EnumName_swigregister', 'FALLING', 'FIELD', 'FIELD_CENTER_X', 'FIELD_CENTER_Y', 'FIELD_X', 'FIELD_Y', 'FINISHED', 'FORWARD', 'FOVx', 'FOVy', 'FaceBlueLeft0', 'FaceBlueLeft135', 'FaceBlueLeft180', 'FaceBlueLeft225', 'FaceBlueLeft270', 'FaceBlueLeft315', 'FaceBlueLeft45', 'FaceBlueLeft90', 'FaceBlueRight0', 'FaceBlueRight135', 'FaceBlueRight180', 'FaceBlueRight225', 'FaceBlueRight270', 'FaceBlueRight315', 'FaceBlueRight45', 'FaceBlueRight90', 'FaceGreenLeft0', 'FaceGreenLeft135', 'FaceGreenLeft180', 'FaceGreenLeft225', 'FaceGreenLeft270', 'FaceGreenLeft315', 'FaceGreenLeft45', 'FaceGreenLeft90', 'FaceGreenRight0', 'FaceGreenRight135', 'FaceGreenRight180', 'FaceGreenRight225', 'FaceGreenRight270', 'FaceGreenRight315', 'FaceGreenRight45', 'FaceGreenRight90', 'FaceRedLeft0', 'FaceRedLeft135', 'FaceRedLeft180', 'FaceRedLeft225', 'FaceRedLeft270', 'FaceRedLeft315', 'FaceRedLeft45', 'FaceRedLeft90', 'FaceRedRight0', 'FaceRedRight135', 'FaceRedRight180', 'FaceRedRight225', 'FaceRedRight270', 'FaceRedRight315', 'FaceRedRight45', 'FaceRedRight90', 'Fall', 'Fall_swigregister', 'FieldAreaRoleConfig', 'FieldAreaRoleConfig_swigregister', 'Floor', 'FrameInfoBlock', 'FrameInfoBlock_swigregister', 'FwdLongLargeGapKick', 'FwdLongSmallGapKick', 'FwdLongStraightKick', 'FwdMediumStraightKick', 'FwdPass2Kick', 'FwdPass3Kick', 'FwdPass4Kick', 'FwdPass5Kick', 'FwdShortStraightKick', 'FwdSuperStraightKick', 'GOAL_HEIGHT', 'GOAL_POST_WIDTH', 'GOAL_WIDTH', 'GOAL_X', 'GOAL_Y', 'GRASS', 'GRASS_X', 'GRASS_Y', 'GameStateBlock', 'GameStateBlock_swigregister', 'Getup', 'Getup_swigregister', 'HALF_FIELD_X', 'HALF_FIELD_Y', 'HALF_GOAL_Y', 'HALF_GRASS_X', 'HALF_GRASS_Y', 'HIPOFFSETZ', 'HeadPan', 'HeadPitch', 'HeadTilt', 'HeadYaw', 'IGNORE_COACH_DELAYS', 'IMAGE_CAPTURE', 'INITIAL', 'INTERFACE', 'INTERSECTION_OFFSET', 'INVALID_WALK', 'ImageBlock', 'ImageBlock_swigregister', 'ImageParams', 'ImageParams_swigregister', 'InterpreterModule', 'InterpreterModule_swigregister', 'InverseKinematics', 'InverseKinematics_calcArmJoints', 'InverseKinematics_calcElbowPosition', 'InverseKinematics_calcJointsForElbowPos', 'InverseKinematics_calcLegJoints', 'InverseKinematics_swigregister', 'JointBlock', 'JointBlock_swigregister', 'JointCommandBlock', 'JointCommandBlock_swigregister', 'JointRequest', 'JointRequest_swigregister', 'KEEPER', 'KICK_REGION_SIZE_X', 'KICK_REGION_SIZE_Y', 'Kick', 'KickEvaluation', 'KickEvaluation_swigregister', 'KickParamBlock', 'KickParamBlock_swigregister', 'KickParameters', 'KickParameters_swigregister', 'KickRequestBlock', 'KickRequestBlock_swigregister', 'KickState', 'KickStateInfo', 'KickStateInfo_swigregister', 'KickState_getName', 'KickState_swigregister', 'KickStrategy', 'KickStrategy_swigregister', 'Kick_swigregister', 'KinematicsModuleLog', 'LANDMARK_OFFSET', 'LAnklePitch', 'LAnkleRoll', 'LEDBlock', 'LEDBlock_swigregister', 'LEDModule', 'LEDModule_swigregister', 'LEDS_PER_EYE', 'LEFTLEG', 'LEG_JOINT_FIRST', 'LEG_JOINT_LAST', 'LElbowRoll', 'LElbowYaw', 'LFootBlue', 'LFootGreen', 'LFootRed', 'LHead0', 'LHead1', 'LHead2', 'LHead3', 'LHead4', 'LHead5', 'LHipPitch', 'LHipRoll', 'LHipYawPitch', 'LINE_OFFSET', 'LINE_WIDTH', 'LKneePitch', 'LShoulderPitch', 'LShoulderRoll', 'LUT_BIT', 'LUT_SIZE', 'L_INT_OFFSET', 'Line2D', 'Line2D_makeLineFromPositionAndAngle', 'Line2D_makeLineFromTwoPoints', 'Line2D_swigregister', 'LineSegment', 'LineSegment_swigregister', 'LocalizationBlock', 'LocalizationBlock_swigregister', 'LocalizationMethod', 'LocalizationMethod_swigregister', 'LocalizationModule', 'LocalizationModuleLog', 'LocalizationModule_swigregister', 'LocalizationParams', 'LocalizationParams_swigregister', 'MANUAL_CONTROL', 'MAX_MODELS_IN_MEM', 'MAX_NUM_FIELD_AREAS', 'MAX_OPP_MODELS_IN_MEM', 'MAX_SPLINE_POINTS', 'MAX_VISION_OPPS', 'MEMORY_ROBOT', 'MEMORY_SIM', 'MICRO', 'MIDFIELD', 'MILLI', 'MOTION', 'Memory', 'MemoryBlock', 'MemoryBlockHeader', 'MemoryBlockHeader_swigregister', 'MemoryBlock_swigregister', 'MemoryHeader', 'MemoryHeader_swigregister', 'Memory_swigregister', 'Module', 'Module_swigregister', 'NECKOFFSETZ', 'NONE', 'NUM_AL_WALK_PARAMS', 'NUM_ARM_JOINTS', 'NUM_BODY_JOINTS', 'NUM_CHANNELS', 'NUM_CROSSES', 'NUM_CoreTypes', 'NUM_FIELD_PLAYERS', 'NUM_HEAD_JOINTS', 'NUM_INTERSECTIONS', 'NUM_JOINTS', 'NUM_KICKS', 'NUM_KICK_REGIONS_X', 'NUM_KICK_REGIONS_Y', 'NUM_LANDMARKS', 'NUM_LEDS', 'NUM_LEG_JOINTS', 'NUM_LINES', 'NUM_L_INTS', 'NUM_MODULE_TYPES', 'NUM_PENALTY_POSES', 'NUM_PLAYERS', 'NUM_ROBOTS_ON_TEAM', 'NUM_ROLES', 'NUM_SENSORS', 'NUM_SONAR_VALS', 'NUM_TEAM_POSES', 'NUM_T_INTS', 'NUM_UNKNOWN_L', 'NUM_UNKNOWN_LINES', 'NUM_UNKNOWN_POSTS', 'NUM_UNKNOWN_T', 'NUM_WORLD_OBJS', 'NUM_WalkTypes', 'NUM_WorldObjectTypes', 'OdometryBlock', 'OdometryBlock_swigregister', 'OppModule', 'OppModuleLog', 'OppModule_swigregister', 'OpponentBlock', 'OpponentBlock_swigregister', 'OpponentModel', 'OpponentModel_swigregister', 'PENALISED', 'PENALTY_CROSS_X', 'PENALTY_MARK_SIZE', 'PENALTY_X', 'PENALTY_Y', 'PLAYING', 'POLAR', 'PassInfo', 'PassInfo_swigregister', 'PassStrategy', 'PassStrategy_swigregister', 'Point2D', 'Point2D_getPointFromPolar', 'Point2D_swigregister', 'Point3D', 'Point3D_swigregister', 'Pose2D', 'Pose2D_random', 'Pose2D_swigregister', 'Pose3D', 'Pose3D_swigregister', 'Poses', 'Poses_fromName_Stance', 'Poses_getName', 'Poses_swigregister', 'ProcessedSonarBlock', 'ProcessedSonarBlock_swigregister', 'PythonInterface', 'PythonInterface_swigregister', 'PythonModule', 'PythonModule_swigregister', 'RAD_T_DEG', 'RANDOM_SEED', 'RAnklePitch', 'RAnkleRoll', 'READY', 'RElbowRoll', 'RElbowYaw', 'RFootBlue', 'RFootGreen', 'RFootRed', 'RHead0', 'RHead1', 'RHead2', 'RHead3', 'RHead4', 'RHead5', 'RHipPitch', 'RHipRoll', 'RHipYawPitch', 'RIGHTLEG', 'RKneePitch', 'RShoulderPitch', 'RShoulderRoll', 'RUNSWIFT2014_WALK', 'Rad2Deg', 'Random', 'Random_inst', 'Random_swigregister', 'Rectangle', 'Rectangle_swigregister', 'RobotDimensions', 'RobotDimensions_swigregister', 'RobotInfoBlock', 'RobotInfoBlock_swigregister', 'RobotPositions', 'RobotPositions_swigregister', 'RobotStateBlock', 'RobotStateBlock_swigregister', 'RobotVisionBlock', 'RobotVisionBlock_swigregister', 'RolePositionConfig', 'RolePositionConfig_swigregister', 'RoleStrategy', 'RoleStrategy_swigregister', 'RotationMatrix', 'RotationMatrix_fromRotationX', 'RotationMatrix_fromRotationY', 'RotationMatrix_fromRotationZ', 'RotationMatrix_swigregister', 'Round', 'SAMPLE_COUNT', 'SAMPLE_RATE', 'SET', 'SET_PLAY_RECEIVER', 'SHARED', 'SIGMA', 'SIM_IMAGE_SIZE', 'SIM_IMAGE_X', 'SIM_IMAGE_Y', 'SPEECH_TEXT_SIZE', 'STATE_SIZE', 'SUPPORTER', 'SYNC', 'SensorBlock', 'SensorBlock_swigregister', 'SensorsModuleLog', 'SetPlay', 'SetPlayInfo', 'SetPlayInfo_swigregister', 'SetPlayStrategy', 'SetPlayStrategy_swigregister', 'SetPlay_fromName_Type', 'SetPlay_getName', 'SetPlay_swigregister', 'Sides', 'Sides_swigregister', 'Sign', 'SimTruthDataBlock', 'SimTruthDataBlock_swigregister', 'Spawns', 'Spawns_fromName_SpawnType', 'Spawns_getName', 'Spawns_swigregister', 'SpeechBlock', 'SpeechBlock_swigregister', 'SwigPyIterator', 'SwigPyIterator_swigregister', 'TEAM_BLUE', 'TEAM_RED', 'TESTING', 'TEST_ODOMETRY', 'TOOL', 'TOOL_MEM', 'TOP_CAM', 'T_INT_OFFSET', 'TeamPacket', 'TeamPacket_swigregister', 'TeamPacketsBlock', 'TeamPacketsBlock_swigregister', 'TextLogger', 'TextLogger_swigregister', 'TiltRoll', 'TiltRoll_swigregister', 'TimeList', 'TimeList_swigregister', 'Timer', 'Timer_swigregister', 'UNDEFINED', 'UNDEFINED_STATE', 'UNKNOWN', 'USE_AL_MOTION', 'VISION', 'VisionCore', 'VisionCore_swigregister', 'VisionModule', 'VisionModuleLog', 'VisionModule_swigregister', 'WO_BALL', 'WO_BEACON_BLUE_PINK', 'WO_BEACON_BLUE_YELLOW', 'WO_BEACON_PINK_BLUE', 'WO_BEACON_PINK_YELLOW', 'WO_BEACON_YELLOW_BLUE', 'WO_BEACON_YELLOW_PINK', 'WO_BOTTOM_SIDE_LINE', 'WO_CENTER_BOTTOM_T', 'WO_CENTER_CIRCLE', 'WO_CENTER_LINE', 'WO_CENTER_TOP_T', 'WO_GOAL_FIRST', 'WO_GOAL_LAST', 'WO_INVALID', 'WO_OPPONENT1', 'WO_OPPONENT2', 'WO_OPPONENT3', 'WO_OPPONENT4', 'WO_OPPONENT5', 'WO_OPPONENT_COACH', 'WO_OPPONENT_FIRST', 'WO_OPPONENT_LAST', 'WO_OPP_BACK_LEFT_GOAL_L', 'WO_OPP_BACK_RIGHT_GOAL_L', 'WO_OPP_BOTTOM_GOALBAR', 'WO_OPP_FIELD_LEFT_L', 'WO_OPP_FIELD_RIGHT_L', 'WO_OPP_FRONT_LEFT_GOAL_T', 'WO_OPP_FRONT_RIGHT_GOAL_T', 'WO_OPP_GOAL', 'WO_OPP_GOAL_LINE', 'WO_OPP_LEFT_GOALBAR', 'WO_OPP_LEFT_GOALPOST', 'WO_OPP_PENALTY', 'WO_OPP_PENALTY_CROSS', 'WO_OPP_PEN_LEFT_L', 'WO_OPP_PEN_LEFT_T', 'WO_OPP_PEN_RIGHT_L', 'WO_OPP_PEN_RIGHT_T', 'WO_OPP_RIGHT_GOALBAR', 'WO_OPP_RIGHT_GOALPOST', 'WO_OWN_BACK_LEFT_GOAL_L', 'WO_OWN_BACK_RIGHT_GOAL_L', 'WO_OWN_BOTTOM_GOALBAR', 'WO_OWN_FIELD_LEFT_L', 'WO_OWN_FIELD_RIGHT_L', 'WO_OWN_FRONT_LEFT_GOAL_T', 'WO_OWN_FRONT_RIGHT_GOAL_T', 'WO_OWN_GOAL', 'WO_OWN_GOAL_LINE', 'WO_OWN_LEFT_GOALBAR', 'WO_OWN_LEFT_GOALPOST', 'WO_OWN_PENALTY', 'WO_OWN_PENALTY_CROSS', 'WO_OWN_PEN_LEFT_L', 'WO_OWN_PEN_LEFT_T', 'WO_OWN_PEN_RIGHT_L', 'WO_OWN_PEN_RIGHT_T', 'WO_OWN_RIGHT_GOALBAR', 'WO_OWN_RIGHT_GOALPOST', 'WO_PENALTY_BOTTOM_OPP', 'WO_PENALTY_BOTTOM_OWN', 'WO_PENALTY_TOP_OPP', 'WO_PENALTY_TOP_OWN', 'WO_PLAYERS_FIRST', 'WO_PLAYERS_LAST', 'WO_ROBOTS_LAST', 'WO_ROBOT_CLUSTER', 'WO_TEAM1', 'WO_TEAM2', 'WO_TEAM3', 'WO_TEAM4', 'WO_TEAM5', 'WO_TEAM_COACH', 'WO_TEAM_FIELD_FIRST', 'WO_TEAM_FIRST', 'WO_TEAM_LAST', 'WO_TEAM_LISTENER', 'WO_TOP_SIDE_LINE', 'WO_UNKNOWN_FIELD_LINE_1', 'WO_UNKNOWN_FIELD_LINE_2', 'WO_UNKNOWN_FIELD_LINE_3', 'WO_UNKNOWN_FIELD_LINE_4', 'WO_UNKNOWN_GOAL', 'WO_UNKNOWN_GOALPOST', 'WO_UNKNOWN_LEFT_GOALPOST', 'WO_UNKNOWN_L_1', 'WO_UNKNOWN_L_2', 'WO_UNKNOWN_PENALTY_CROSS', 'WO_UNKNOWN_RIGHT_GOALPOST', 'WO_UNKNOWN_T_1', 'WO_UNKNOWN_T_2', 'WalkEngineParameters', 'WalkEngineParameters_swigregister', 'WalkInfoBlock', 'WalkInfoBlock_swigregister', 'WalkKickFront', 'WalkKickLeftward', 'WalkKickLeftwardSide', 'WalkKickRightward', 'WalkKickRightwardSide', 'WalkMode', 'WalkMode_getName', 'WalkMode_swigregister', 'WalkParamBlock', 'WalkParamBlock_swigregister', 'WalkRequestBlock', 'WalkRequestBlock_fromName_Motion', 'WalkRequestBlock_getName', 'WalkRequestBlock_swigregister', 'WalkResponseBlock', 'WalkResponseBlock_swigregister', 'WorldObject', 'WorldObjectBlock', 'WorldObjectBlock_swigregister', 'WorldObject_swigregister', '__builtins__', '__doc__', '__file__', '__name__', '__package__', '_newclass', '_object', '_pythonswig_module', '_swig_getattr', '_swig_property', '_swig_repr', '_swig_setattr', '_swig_setattr_nondynamic', 'accelX', 'accelY', 'accelZ', 'angleX', 'angleY', 'angleZ', 'battery', 'behaviorC', 'bumperLL', 'bumperLR', 'bumperRL', 'bumperRR', 'centerButton', 'circleLocation', 'clockavg', 'crop', 'cvar', 'fromName_CoreType', 'fromName_WalkType', 'fromName_WorldObjectType', 'fsrLFL', 'fsrLFR', 'fsrLRL', 'fsrLRR', 'fsrRFL', 'fsrRFR', 'fsrRRL', 'fsrRRR', 'getBisectorTwoAngles', 'getJointName', 'getLEDString', 'getName', 'getSensorString', 'getSystemTime', 'getpass', 'gyroX', 'gyroY', 'gyroZ', 'headFront', 'headMiddle', 'headRear', 'init', 'instance', 'intersectionLocation', 'isAngInInterval', 'joint_stiffness', 'joint_values', 'kickNames', 'kickTypeNames', 'landmarkLocation', 'ledsC', 'legNames', 'lineLocationEnds', 'lineLocationStarts', 'localizationC', 'math', 'max', 'maxJointLimits', 'min', 'minJointLimits', 'minmax', 'normalizeAngle', 'normalizeAngleAroundVal', 'normalizeAngleNinety', 'normalizeAngleNinetyAroundVal', 'notANumber', 'oppCrossLocation', 'opponentsC', 'ownCrossLocation', 'printtime', 'pythonC', 'pythonswig_module', 'robot_joint_signs', 'roleAbbrevs', 'roleNames', 'sensor_values', 'spark_joint_signs', 'square', 'stateNames', 'swig', 'sys', 'text_logger', 'tic', 'ticavg', 'toc', 'tocavg', 'vector2_float', 'vector2_float_swigregister', 'vector3_float', 'vector3_float_swigregister', 'vector_float', 'vector_float_swigregister', 'visionC', 'walkParamNames']

start_frame = -1
droll_off = 0.
dpitch_off = 0.
dlarmroll_off = 0.
dlarmpitch_off = 0.
drarmroll_off = 0.
drarmpitch_off = 0.
class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 3.0:
        self.finish()

  class Kick(Node):
    def run(self):
      global start_frame, droll_off, dpitch_off, dlarmroll_off, dlarmpitch_off, drarmroll_off, drarmpitch_off
      if(start_frame == -1):
        start_frame = vision_frame_info.frame_id

      traj_idx = vision_frame_info.frame_id - start_frame
      if(traj_idx >= len(trajectory)):
        start_frame = -1
        self.finish()
        print "done!"
        return
        # traj_idx = len(trajectory) - 1

      # print "Kick is " + str(float(traj_idx)/float(len(trajectory)-1.)*100.) + " percent complete"

      inverted_joints = [1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1, 1, 1, -1, -1, -1, -1]
      # inverted_joints[core.LShoulderPitch] = -1.
      # inverted_joints[core.RShoulderPitch] = -1.
      # inverted_joints[core.RShoulderRoll] = -1.
      # inverted_joints[core.RElbowRoll] = -1.
  
      pose = trajectory[traj_idx]
      for i in range(2, core.NUM_JOINTS):
        joint_commands.setJointCommand(i, pose[i]*inverted_joints[i])

      # tweak left ankle commands to handle cop dynamics
      fl = sensors.getValue(core.fsrLFL)
      fr = sensors.getValue(core.fsrLFR)
      rl = sensors.getValue(core.fsrLRL)
      rr = sensors.getValue(core.fsrLRR)
      dx = (fl + fr)/2. - (rl + rr)/2. + 0.
      dy = (fl + rl)/2. - (fr + rr)/2. - 0.05
      # print "dx: " + str(dx)
      # print "dy: " + str(dy)

      clamp = 0.2
      # droll_off += -dy * 0.001
      # dpitch_off += -dx * 0.0 
      dlarmroll_off = numpy.clip(dlarmroll_off - dy * 0.0025, -clamp, clamp)
      # dlarmpitch_off += dx * 0.0 
      drarmroll_off = numpy.clip(drarmroll_off - dy * 0.0025, -clamp, clamp)
      # drarmpitch_off += dx * 0.0 

      droll = dy * 0.01 + droll_off
      dpitch = -dx * 0.0  + dpitch_off
      dlarmroll = -dy * 0.6 + dlarmroll_off
      dlarmpitch = dx * 0.0  + dlarmpitch_off
      dlhiproll = -dy * 0.025
      drarmroll = -dy * 0.6 + drarmroll_off
      drarmpitch = dx * 0.0  + drarmpitch_off



      # print "droll: " + str(droll)
      # print "dpitch: " + str(dpitch)
      # print "dlarmroll: " + str(dlarmroll)
      # print "dlarmpitch: " + str(dlarmpitch)
      # print "drarmroll: " + str(drarmroll)
      # print "drarmpitch: " + str(drarmpitch)

      # joint_commands.setJointCommand(core.LAnklePitch, (pose[core.LAnklePitch] + dpitch)*inverted_joints[core.LAnklePitch])
      # joint_commands.setJointCommand(core.LAnkleRoll, (pose[core.LAnkleRoll] + droll)*inverted_joints[core.LAnkleRoll])
      # joint_commands.setJointCommand(core.LShoulderPitch, (pose[core.LShoulderPitch] + dlarmpitch)*inverted_joints[core.LShoulderPitch])
      # joint_commands.setJointCommand(core.LShoulderRoll, numpy.clip(pose[core.LShoulderRoll] + dlarmroll, 0.3, 10)*inverted_joints[core.LShoulderRoll])
      # joint_commands.setJointCommand(core.LHipRoll, (pose[core.LHipRoll] + dlhiproll)*inverted_joints[core.LHipRoll])
      # joint_commands.setJointCommand(core.RShoulderPitch, (pose[core.RShoulderPitch] + drarmpitch)*inverted_joints[core.RShoulderPitch])
      # joint_commands.setJointCommand(core.RShoulderRoll, numpy.clip(pose[core.RShoulderRoll] + drarmroll, -0.3, -10)*inverted_joints[core.RShoulderRoll])

      #send commands and prevent other controllers from operating
      joint_commands.send_body_angles_ = True
      joint_commands.body_angle_time = 1# 1.0/30.0 * 1000.0#todo: figure out what units this is
      walk_request.noWalk()
      kick_request.setNoKick()

  class Walk(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0)

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        self.finish()

  def setup(self):
    self.trans(self.Stand(), C, self.Kick(), C, self.Stand(), C, pose.Sit(), C, self.Off())
    # self.trans(self.Kick())
