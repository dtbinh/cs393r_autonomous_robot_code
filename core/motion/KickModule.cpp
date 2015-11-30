#include <motion/KickModule.h>
#include <common/Keyframe.h>
#include <memory/FrameInfoBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/BodyModelBlock.h>
#include <memory/JointBlock.h>
#include <memory/SensorBlock.h>
#include <memory/KickRequestBlock.h>
#include <memory/Memory.h>
#include <stdio.h>
#include <time.h> 
#include "kack.hpp"

#define JOINT_EPSILON (3.f * DEG_T_RAD)
#define DEBUG false

KickModule::KickModule() : state_(Finished), sequence_(NULL) 
{
  KACK::Point area_center;
  double area_long_radius, area_short_radius, area_cut_radius;

  kick_state_ = FINISHED;
  ball_direction_ = RIGHTBALL;
  goal_direction_ = MIDDLEGOAL;
  kick_foot_ = STANDFOOT;

  coordinate_shift.update(0,0,0);
  current_ball_location.update(0,0,0);
  current_goal_location.update(0,0,0);
  current_pose.update(0,0,0,0,0,0);
  desired_next_pose.update(0,0,0,0,0,0);
  desired_next_com.update(0,0,0);
  
  PoseOffset.update(0.005 , 0.005 , 0.005 , 0.05 , 0.05 , 0.2);
  ComOffset.update(0.0025 , 0.0025 , 0.0025);

  z_index  = 0.05;
  NUMBER_JOINTS = 26;

  initializing_counter = -1;
  tracking_counter = -1;
  executing_counter = -1;
  putting_back_counter = -1;

  CurrentJoints.resize(NUMBER_JOINTS);
  CurrentCommand.resize(NUMBER_JOINTS);

  IF_KICK_THRESHOLD = 300;
  if_kick_counter = 0;
  IF_RETRACK_THRESHOLD = 25;
  if_retrack_counter = 0;
  OldBallPosition.update( 0 , 0 , 0 );
  CurBallPosition.update( 0 , 0 , 0 );
  OldGoalPosition.update( 0 , 0 , 0 );
  CurGoalPosition.update( 0 , 0 , 0 );

  QueueHead = 0;
  QueueRear = 0;

  area_center.update(0, -100 , 50);
  area_long_radius = 100;
  area_short_radius = 80;
  area_cut_radius = 40;

  REACHAREA.center = area_center;
  REACHAREA.long_radius = area_long_radius;
  REACHAREA.short_radius = area_short_radius;
  REACHAREA.cut_radius = area_cut_radius;

  kack = new KACK::Kack("/home/nao/nao.urdf");

}

KickModule::~KickModule()
{
  delete kack;
}

void KickModule::processFrame() {
  if(cache_.kick_request->kick_type_ == Kick::STRAIGHT) {
    if(kick_state_ == FINISHED) 
    {
      kick_state_ = INITIALIZING;
      Initializing();
    }
  }
  
  
  if(kick_state_ == INITIALIZING) 
  {
    cache_.kick_request->kick_running_ = true;
    if(!Initializing()) kick_state_ = INITIALIZING;
    else kick_state_ =  INITIALIZING;
      //kick_state_ = TRACKING;
  }
  else if(kick_state_ == TRACKING)
  {
    // if(!TRACKING())
    //   kick_state_ = TRACKING;
    // else
    QueueHead = 0;
    QueueRear = 1;
    kick_state_ = EXECUTING;
  }
  else if(kick_state_ == EXECUTING)
  {
    QueueHead = 0;
    QueueRear = 1;
    kick_state_ = PUTTING_BACK;
  }
  else if(kick_state_ == PUTTING_BACK)
  {
    kick_state_ = FINISHED;
    Finishing();
  }
  
}


bool KickModule::Initializing()
{
  // printf("I'm running this part , NUM_JOINTS = %d\n" , NUM_JOINTS);

  KACK::CartesianKeyframe frame_moving_com, frame_first_tracking, frame_second_tracking ;
  KACK::Pose min_pose, max_pose;
  KACK::Point min_com, max_com;

  double desired_initial_y = 0.1 ;
  double period0 = 0, period1 = 0, period2 = 0;

  if(initializing_counter == -1)
  {
    //KickKeyFrames.clear();
    current_ball_location = get_ball_location(coordinate_shift);
    current_goal_location = get_goal_location(coordinate_shift);

    if(current_ball_location.y > 15 || (abs(current_ball_location.y) <= 15 && current_goal_location.y < 0)) //left foot
    {
      coordinate_shift.update(0 , z_index , -0.2);
      ball_direction_ = LEFTBALL;
      kick_foot_ = LEFTFOOT;
      REACHAREA.center.y = -REACHAREA.center.y;
      current_ball_location = get_ball_location(coordinate_shift);
      current_goal_location = get_goal_location(coordinate_shift);
      OldBallPosition = current_ball_location;
      OldGoalPosition = current_goal_location;
      current_pose.update( 0 , desired_initial_y , 0  , 0 , 0 , 0);
      desired_next_pose.update( 0 , desired_initial_y , 0 , 0 , 0 , 0);
      desired_next_com.update(  0 , 0 , 0);

    }
    else if(current_ball_location.y < -15 || (abs(current_ball_location.y) <= 15 && current_goal_location.y >= 0)) //right foot
    {
      coordinate_shift.update(0 , -z_index , -0.2);
      ball_direction_ = RIGHTBALL;
      kick_foot_ = RIGHTFOOT;
      current_ball_location = get_ball_location(coordinate_shift);
      current_goal_location = get_goal_location(coordinate_shift);
      OldBallPosition = current_ball_location;
      OldGoalPosition = current_goal_location;
      current_pose.update( 0 , -desired_initial_y , 0  , 0 , 0 , 0);
      desired_next_pose.update( 0 , -desired_initial_y , 0 , 0 , 0 , 0);
      desired_next_com.update(  0 , 0 , 0);
    }

    AddPoseTolerance(desired_next_pose , min_pose , max_pose , PoseOffset);
    AddComTolerance(desired_next_com , min_com , max_com , ComOffset);
    period0 = 2;
    frame_moving_com.update( period0 , min_pose, max_pose, min_com, max_com );
    QueuePush( frame_moving_com , period0*100 + 50 );
    printf("min_pose = (%f,%f,%f,%f,%f,%f) \n" , min_pose.x , min_pose.y, min_pose.z , min_pose.R, min_pose.P, min_pose.Y);
    printf("max_pose = (%f,%f,%f,%f,%f,%f) \n" , max_pose.x , max_pose.y, max_pose.z , max_pose.R, max_pose.P, max_pose.Y);
    printf("min_com = (%f,%f,%f)\n", min_com.x, min_com.y, min_com.z);
    printf("max_com = (%f,%f,%f)\n", max_com.x, max_com.y, max_com.z);

    desired_next_pose = get_desired_foot_position(current_ball_location, current_goal_location, REACHAREA , true);
    printf("desired_next_pose calculated: x = %f , y = %f\n" , desired_next_pose.x , desired_next_pose.y);
    desired_next_pose.update(-0.0, -desired_initial_y, 0.04 , -0.1 , 0 , 0);
    desired_next_com.update(0 , 0 , 0);
    AddPoseTolerance(desired_next_pose , min_pose , max_pose , PoseOffset);
    AddComTolerance(desired_next_com , min_com , max_com , ComOffset);
    period1 = 1;
    frame_first_tracking.update( period1 , min_pose, max_pose, min_com, max_com );
    QueuePush( frame_first_tracking , period1*100 + 50 );
    printf("period1 = %f\n", period1 );
    printf("min_pose = (%f,%f,%f,%f,%f,%f) \n" , min_pose.x , min_pose.y, min_pose.z , min_pose.R, min_pose.P, min_pose.Y);
    printf("max_pose = (%f,%f,%f,%f,%f,%f) \n" , max_pose.x , max_pose.y, max_pose.z , max_pose.R, max_pose.P, max_pose.Y);
    printf("min_com = (%f,%f,%f)\n", min_com.x, min_com.y, min_com.z);
    printf("max_com = (%f,%f,%f)\n", max_com.x, max_com.y, max_com.z);

    current_pose = desired_next_pose;
    desired_next_pose.update(-0.08 , -0.06 , 0.04 , 0 , 0 , 0);
    desired_next_com.update(0 , 0 , 0);
    if(desired_next_pose.y != 0)
    {
      AddPoseTolerance(desired_next_pose , min_pose , max_pose , PoseOffset);
      AddComTolerance(desired_next_com , min_com , max_com , ComOffset);
      period2 = 1;
      frame_second_tracking.update( period2 , min_pose, max_pose, min_com, max_com );
      QueuePush( frame_second_tracking , period2*100 + 50 );
      current_pose = desired_next_pose;
    }
    printf("period2 = %f\n", period2 );
    printf("min_pose = (%f,%f,%f,%f,%f,%f) \n" , min_pose.x , min_pose.y, min_pose.z , min_pose.R, min_pose.P, min_pose.Y);
    printf("max_pose = (%f,%f,%f,%f,%f,%f) \n" , max_pose.x , max_pose.y, max_pose.z , max_pose.R, max_pose.P, max_pose.Y);
    printf("min_com = (%f,%f,%f)\n", min_com.x, min_com.y, min_com.z);
    printf("max_com = (%f,%f,%f)\n", max_com.x, max_com.y, max_com.z);

    initializing_counter = (period0 + period1 + period2)*100 + 150;
  }

  KACK::FootSensor left_foot_force_sensor, right_foot_force_sensor;

  left_foot_force_sensor  = get_left_foot_sensor();
  right_foot_force_sensor = get_right_foot_sensor();
  CurrentJoints = getCurrentJoints();
  CurrentTime = getCurrentTime();

  int index = QueueHead%16;
  if(index == 3) index = 2;

  double kp_x = 0.001;
  double ki_x = 0.0005;
  double kd_x = 0.0;
  double kmax_x = 0.001;

  double kp_y = 0.3;
  double kmax_y = 0.3;
  double ki_y = 0.0005;
  double kd_y = 0.00005;
  kack->moveFoot(100, kick_foot_ == RIGHTFOOT, KickKeyFramesQueue[index], CurrentJoints, left_foot_force_sensor, right_foot_force_sensor, CurrentCommand, 0.2, kp_x, ki_x, kmax_x, kd_x, kp_y, ki_y, kmax_y, kd_y, 0.1); //alpha 0.3

  // printf("---------------------------------------------------------------------------------------------------------------------------------------\n");
  // printf("CounterQueue[%d] = %d , QueueRear = %d\n" , QueueHead , CounterQueue[QueueHead%16] , QueueRear);
  // printf("min_pose = (%f,%f,%f,%f,%f,%f) \n" , KickKeyFramesQueue[QueueHead%16].min_pose.x , KickKeyFramesQueue[QueueHead%16].min_pose.y, KickKeyFramesQueue[QueueHead%16].min_pose.z 
  //                    , KickKeyFramesQueue[QueueHead%16].min_pose.R, KickKeyFramesQueue[QueueHead%16].min_pose.P, KickKeyFramesQueue[QueueHead%16].min_pose.Y);
  // printf("max_pose = (%f,%f,%f,%f,%f,%f) \n" , KickKeyFramesQueue[QueueHead%16].max_pose.x , KickKeyFramesQueue[QueueHead%16].max_pose.y, KickKeyFramesQueue[QueueHead%16].max_pose.z 
  //                    , KickKeyFramesQueue[QueueHead%16].max_pose.R, KickKeyFramesQueue[QueueHead%16].max_pose.P, KickKeyFramesQueue[QueueHead%16].max_pose.Y);
  // printf("min_com = (%f,%f,%f)\n", KickKeyFramesQueue[QueueHead%16].min_com.x, KickKeyFramesQueue[QueueHead%16].min_com.y, KickKeyFramesQueue[QueueHead%16].min_com.z);
  // printf("max_com = (%f,%f,%f)\n", KickKeyFramesQueue[QueueHead%16].max_com.x, KickKeyFramesQueue[QueueHead%16].max_com.y, KickKeyFramesQueue[QueueHead%16].max_com.z);
  // printf("CounterQueue[%d] = %d , QueueRear = %d\n" , QueueHead , CounterQueue[QueueHead%16] , QueueRear);

  SendingFrame();

  if(QueueHead != QueueRear)
  {
    if(CounterQueue[QueueHead%16] != 0) CounterQueue[QueueHead%16]--;
    else QueuePop();
    return false;
  }
  else return true;
}

bool KickModule::Tracking()
{
  double RunningTime = 0;
  KACK::Pose min_pose, max_pose;
  KACK::Pose tmp_next_pose;
  KACK::Point min_com, max_com;
  KACK::CartesianKeyframe TrackingFrame;
  KACK::FootSensor left_foot_force_sensor, right_foot_force_sensor;

  CurBallPosition = get_ball_location(coordinate_shift);
  CurGoalPosition = get_goal_location(coordinate_shift);
  tmp_next_pose = get_desired_foot_position(CurBallPosition, CurGoalPosition, REACHAREA , true);
  RunningTime = getRunningTime(current_pose , tmp_next_pose);

  if(RunningTime > 0.2) if_retrack_counter++;
  else if_retrack_counter = 0;

  if(if_retrack_counter > IF_RETRACK_THRESHOLD)
  {
    current_pose = desired_next_pose;
    desired_next_pose = tmp_next_pose;
    CounterQueue[QueueHead%16] = RunningTime*100 + 20;
    AddPoseTolerance(desired_next_pose , min_pose , max_pose , PoseOffset);
    AddComTolerance(desired_next_com , min_com , max_com , ComOffset);
    TrackingFrame.update(RunningTime , min_pose, max_pose, min_com, max_com);
    KickKeyFramesQueue[QueueHead%16] = TrackingFrame;
  }

  left_foot_force_sensor  = get_left_foot_sensor();
  right_foot_force_sensor = get_right_foot_sensor();
  CurrentJoints = getCurrentJoints();
  CurrentTime = getCurrentTime();
  double kp_x = 0.001;
  double ki_x = 0.0005;
  double kd_x = 0.0;
  double kmax_x = 0.001;

  double kp_y = 0.3;
  double kmax_y = 0.3;
  double ki_y = 0.0005;
  double kd_y = 0.00005;
  kack->moveFoot(100, kick_foot_ == RIGHTFOOT, KickKeyFramesQueue[QueueHead%16], CurrentJoints, left_foot_force_sensor, right_foot_force_sensor, CurrentCommand, 0.2, kp_x, ki_x, kmax_x, kd_x, kp_y, ki_y, kmax_y, kd_y, 0.1); //alpha 0.3

  SendingFrame();

  if(if_kick_counter < IF_KICK_THRESHOLD)
  {
    if(CounterQueue[QueueHead%16] != 0)
    {
      CounterQueue[QueueHead%16]--;
      if_kick_counter = 0;
    }
    else if( if_retrack_counter != 0) if_kick_counter = 0;
    else if_kick_counter++;

    return false;
  } 
  else return true;
}

bool KickModule::Executing()
{

}

bool KickModule::Putting_back()
{

}

bool KickModule::Finishing()
{
  printf("Finishing kick\n");
  
  cache_.kick_request->kick_running_ = false;
  cache_.kick_request->kick_type_ == Kick::NO_KICK;
  return true;
}

KACK::Point KickModule::get_ball_location(KACK::Point shift)
{
  KACK::Point location;

  location.x = cache_.robot_state->ball_visionDistance * cos(cache_.robot_state->ball_visionBearing) + shift.x;
  location.y = cache_.robot_state->ball_visionDistance * sin(cache_.robot_state->ball_visionBearing) + shift.y;
  location.z = z_index;

  return location;
}

KACK::Point KickModule::get_goal_location(KACK::Point shift)
{
  KACK::Point location;

  location.update(1000, 0 , 100);
  return location;
}

KACK::FootSensor KickModule::get_left_foot_sensor()
{
  KACK::FootSensor foot_force;
  
  foot_force.fl = cache_.sensor->values_[fsrLFL];
  foot_force.fr = cache_.sensor->values_[fsrLFR];
  foot_force.rl = cache_.sensor->values_[fsrLRL];
  foot_force.rr = cache_.sensor->values_[fsrLRR];

  return foot_force;
}

KACK::FootSensor KickModule::get_right_foot_sensor()
{
  KACK::FootSensor foot_force;

  foot_force.fl = cache_.sensor->values_[fsrRFL];
  foot_force.fr = cache_.sensor->values_[fsrRFR];
  foot_force.rl = cache_.sensor->values_[fsrRRL];
  foot_force.rr = cache_.sensor->values_[fsrRRR];

  return foot_force;
}


double KickModule::getCurrentTime()    
{    
  struct timeval tv;    
  gettimeofday(&tv,NULL);    
  return tv.tv_sec*1.0 + tv.tv_usec / 1000000.0;    
}

std::vector<double> KickModule::getCurrentJoints()
{
  std::vector<double> currentjoints(NUMBER_JOINTS);
  for(int i = 0 ; i < NUM_JOINTS ; i++)
    currentjoints[i] = cache_.joint->values_[i];
  for(int i = NUM_JOINTS ; i < NUMBER_JOINTS ; i++)
    currentjoints[i] = 0;

  return currentjoints;
}


KACK::Pose KickModule::get_desired_foot_position(KACK::Point ball, KACK::Point goal, ReachableArea area , bool back)
{
  KACK::Pose foot;

  double slope = (ball.y - goal.y)/(ball.x - goal.x);
  double interception = ball.y - slope * ball.x;

  double a = slope;
  double b = interception;
  double e = area.center.y;
  double c = area.long_radius;
  double d = area.short_radius;

  double delta = c*c*d*d*(a*a+d*d-(a*e+b)*(a*e+b));
  double x0 = 0, x1 = 0 , y0 = 0 , y1 = 0 ;
  double x = 0, y = 0 ;


  printf("---------------------------------------------------------------------\n");
  printf("ball( %f , %f ), goal( %f , %f )\n",ball.x , ball.y , goal.x , goal.y );
  printf("slope = %f , interception = %f\n" , a , b);
  printf("center.y = %f , long_radius = %f , short_radius = %f\n" , e , c , d);

  if( delta < 0)
  {
    foot.update(0,0,0,0,0,0);
    printf("Not reachable ************ delta = %f\n" , delta);
    return foot;
  }

  x0 = (e*d*d-a*b*c*c+sqrt(delta))/(a*a*c*c+d*d);
  x1 = (e*d*d-a*b*c*c-sqrt(delta))/(a*a*c*c+d*d);

  if( e < 0 && x1 < -e - area.cut_radius) x1 = -e - area.cut_radius;
  if( e > 0 && x0 > -e + area.cut_radius) x0 = -e + area.cut_radius;

  y0 = a*x0 + b;
  y1 = a*x1 + b;

  if(back) x = (y0 > y1)? y1 : y0;
  else x = (y0 < y1)? y1 : y0;
  y = (x - b)/a;
  
  foot.update(x/1000,y/1000,z_index/1000 , 0 , 0 , 0);
  printf("baaaaaaaaaaaaall = (%f,%f,%f)\n", ball.x, ball.y, ball.z);
  printf("foooooooooooooot = (%f,%f,%f)\n", x , y , z_index );
  return foot;
}

void KickModule::AddPoseTolerance(KACK::Pose DesirePose, KACK::Pose &min_pose, KACK::Pose &max_pose, KACK::Pose offset )
{
  min_pose.update(DesirePose.x - offset.x , DesirePose.y - offset.y , DesirePose.z - offset.z ,
                  DesirePose.R - offset.R , DesirePose.P - offset.P , DesirePose.Y - offset.Y);
  max_pose.update(DesirePose.x + offset.x , DesirePose.y + offset.y , DesirePose.z + offset.z ,
                  DesirePose.R + offset.R , DesirePose.P + offset.P , DesirePose.Y + offset.Y);
}

void KickModule::AddComTolerance(KACK::Point DesireCom , KACK::Point &min_com, KACK::Point &max_com, KACK::Point offset)
{
  min_com.update(DesireCom.x - offset.x , DesireCom.y - offset.y , 0.24);
  max_com.update(DesireCom.x + offset.x , DesireCom.y + offset.y , 0.28);
}

double KickModule::getRunningTime(KACK::Pose current , KACK::Pose next)
{
  double delta_x = current.x - next.x;
  double delta_y = current.y - next.y;
  return 10*sqrt( delta_x*delta_x + delta_y*delta_y );
}

void KickModule::SendingFrame()
{
  std::array<float, NUM_JOINTS> JointsCommand;
  for( int i = 0 ; i < NUM_JOINTS ; i++) JointsCommand[i]= CurrentCommand[i];
  JointsCommand[RHipYawPitch] = - CurrentCommand[LHipYawPitch];
  //JointsCommand[RHipPitch] = 3*JointsCommand[RHipPitch] + 0.82;
  //JointsCommand[LHipPitch] += 0;

  //printf("sensed: lefthippitch = %f , leftknee = %f , righthippitch = %f, rightknee = %f \n", CurrentJoints[LHipPitch]*a , CurrentJoints[LKneePitch]*a , CurrentJoints[RHipPitch]*a , CurrentJoints[RKneePitch]*a);
  //printf("commanded: lefthippitch = %f , leftknee = %f , righthippitch = %f , rightknee = %f \n", JointsCommand[LHipPitch]*a , JointsCommand[LKneePitch]*a , JointsCommand[RHipPitch]*a , JointsCommand[RKneePitch]*a);
  // printf("HeadYaw: %f\n", JointsCommand[0]*180/3.1415926);
  // printf("HeadPitch: %f\n", JointsCommand[1]*180/3.1415926);
  // printf("LHipYawPitch: %f\n", JointsCommand[2]*180/3.1415926);
  // printf("LHipRoll: %f\n", JointsCommand[3]*180/3.1415926);
  // printf("LHipPitch: %f\n", JointsCommand[4]*180/3.1415926);
  // printf("LKneePitch: %f\n", JointsCommand[5]*180/3.1415926);
  // printf("LAnklePitch: %f\n", JointsCommand[6]*180/3.1415926);
  // printf("LAnkleRoll: %f\n", JointsCommand[7]*180/3.1415926);
  // printf("RHipYawPitch: %f\n", JointsCommand[8]*180/3.1415926);
  // printf("RHipRoll: %f\n", JointsCommand[9]*180/3.1415926);
  // printf("RHipPitch: %f\n", JointsCommand[10]*180/3.1415926);
  // printf("RKneePitch: %f\n", JointsCommand[11]*180/3.1415926);
  // printf("RAnklePitch: %f\n", JointsCommand[12]*180/3.1415926);
  // printf("RAnkleRoll: %f\n", JointsCommand[13]*180/3.1415926);
  // printf("LShoulderPitch: %f\n", JointsCommand[14]*180/3.1415926);
  // printf("LShoulderRoll: %f\n", JointsCommand[15]*180/3.1415926);
  // printf("LElbowYaw: %f\n", JointsCommand[16]*180/3.1415926);
  // printf("LElbowRoll: %f\n", JointsCommand[17]*180/3.1415926);
  // printf("RShoulderPitch: %f\n", JointsCommand[18]*180/3.1415926);
  // printf("RShoulderRoll: %f\n", JointsCommand[19]*180/3.1415926);
  // printf("RElbowYaw: %f\n", JointsCommand[20]*180/3.1415926);
  // printf("RElbowRoll: %f\n", JointsCommand[21]*180/3.1415926);

  cache_.joint_command->setSendAllAngles(true,10.0);
  cache_.joint_command->setPoseRad(JointsCommand.data());
}



//-------------------------------------old keyframe based kicking version-------------------------------------------- 
void KickModule::initSpecificModule() {
  auto file = cache_.memory->data_path_ + "kicks/default.yaml";
  sequence_ = new KeyframeSequence();
  printf("Loading kick sequence from '%s'...", file.c_str());
  fflush(stdout);
  if(sequence_->load(file))
    printf("success!\n");
  else {
    printf("failed!\n");
    sequence_ = NULL;
  }
  initial_ = NULL;
}

void KickModule::start() {
  printf("Starting kick sequence\n");
  state_ = Initial;
  cache_.kick_request->kick_running_ = true;
  keyframe_ = 0;
  frames_ = 0;
  initial_ = new Keyframe(cache_.joint->values_, 0);
}

void KickModule::finish() {
  printf("Finishing kick sequence\n");
  state_ = Finished;
  cache_.kick_request->kick_running_ = false;
  cache_.kick_request->kick_type_ == Kick::NO_KICK;
  if(initial_) delete initial_;
  initial_ = NULL;
}

bool KickModule::finished() {
  return state_ == Finished;
}

void KickModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("frame_info");
  requiresMemoryBlock("walk_request");
  requiresMemoryBlock("processed_joint_angles");
  requiresMemoryBlock("processed_joint_commands");
  requiresMemoryBlock("odometry");
  requiresMemoryBlock("processed_sensors");
  requiresMemoryBlock("body_model");
  requiresMemoryBlock("kick_request");
  requiresMemoryBlock("robot_state");
}

void KickModule::specifyMemoryBlocks() {
  cache_.memory = memory_;
  // cache_.memory->getBlockByName(cache_.world_object,"world_objects",MemoryOwner::VISION);
  getMemoryBlock(cache_.frame_info,"frame_info");
  getMemoryBlock(cache_.walk_request,"walk_request");
  getMemoryBlock(cache_.joint,"processed_joint_angles");
  getMemoryBlock(cache_.joint_command,"processed_joint_commands");
  getMemoryBlock(cache_.odometry,"odometry");
  getMemoryBlock(cache_.sensor,"processed_sensors");
  getMemoryBlock(cache_.body_model,"body_model");
  getMemoryBlock(cache_.kick_request,"kick_request");
  getMemoryBlock(cache_.robot_state,"robot_state");
}


void KickModule::performKick() {
  if(DEBUG) printf("performKick, state: %s, keyframe: %i, frames: %i\n", getName(state_), keyframe_, frames_);
  if(state_ == Finished) return;
  if(sequence_ == NULL) return;
  if(keyframe_ >= sequence_->keyframes.size()) {
    finish();
    return;
  }
  auto& keyframe = sequence_->keyframes[keyframe_];
  if(state_ == Initial) {
    if(frames_ >= keyframe.frames) {
      state_ = Running;
      frames_ = 0;
    } else {
      moveToInitial(keyframe, frames_);
    }
  }
  if(state_ == Running) {
    if(keyframe_ == sequence_->keyframes.size() - 1) {
      finish();
      return;
    }
    auto& next = sequence_->keyframes[keyframe_ + 1];
    if(frames_ >= next.frames) {
      keyframe_++;
      frames_ = 0;
      performKick();
      return;
    }
    moveBetweenKeyframes(keyframe, next, frames_);
  }
  frames_++;
}

bool KickModule::reachedKeyframe(const Keyframe& keyframe) {
  for(int i = 0; i < NUM_JOINTS; i++) {
    if(fabs(cache_.joint->values_[i] - keyframe.joints[i]) > JOINT_EPSILON) {
      return false;
    }
  }
  return true;
}

void KickModule::moveToInitial(const Keyframe& keyframe, int cframe) {
  if(initial_ == NULL) return;
  moveBetweenKeyframes(*initial_, keyframe, cframe);
}

void KickModule::moveBetweenKeyframes(const Keyframe& start, const Keyframe& finish, int cframe) {
  if(cframe == 0) {
    if(DEBUG) printf("moving between keyframes, time: %i, joints:\n", finish.frames * 10);
    for(int i = 0; i < finish.joints.size(); i++)
      if(DEBUG) printf("j[%i]:%2.2f,", i, finish.joints[i] * RAD_T_DEG);
    if(DEBUG) printf("\n");
    printf("Left foot has side %g front %g\n", cache_.sensor->fsr_left_side_, cache_.sensor->fsr_left_front_);
    double kp = 0.0 / 360.0 * M_PI;

    std::array<float, NUM_JOINTS> joints = finish.joints;
    joints[15] -= kp * cache_.sensor->fsr_left_side_;
    printf("Adding offset of %g => %g\n", kp * cache_.sensor->fsr_left_side_, joints[15]);
    cache_.joint_command->setSendAllAngles(true, finish.frames * 10);
    printf("finish.frames = %d\n", finish.frames);
    cache_.joint_command->setPoseRad(joints.data());
  }
}
