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
  double area_long_radius, area_short_radius, area_close_cut_radius, area_far_cut_radius;

  kick_state_ = FINISHED;
  ball_direction_ = RIGHTBALL;
  goal_direction_ = MIDDLEGOAL;
  kick_foot_ = STANDFOOT;

  coordinate_shift.update(0,0,0);
  current_ball_location.update(0,0,0);
  current_goal_location.update(0,0,0);
  current_pose.update(0,0,0,0,0,0);
  current_com.update(0,0,0);
  desired_next_pose.update(0,0,0,0,0,0);
  desired_next_com.update(0,0,0);
  
  PoseOffset.update(1e-6 , 1e-6 , 1e-6 , 1e-6, 1e-6, 1e-6);
  ComOffset.update(1e-6 , 1e-6 , 1e-6);

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
  IF_RETRACK_THRESHOLD = 250;
  if_retrack_counter = 0;
  CurBallPosition.update( 0 , 0 , 0 );
  CurGoalPosition.update( 0 , 0 , 0 );

  QueueHead = 0;
  QueueRear = 0;

  ball_seen_threshold = 15;
  ball_seen_counter = 0;
  ball_seen_array = new int[ball_seen_threshold];
  for(int i = 0 ; i < ball_seen_threshold ; ++i) ball_seen_array[i] = 0;

  area_center.update(10, -100 , 45);
  area_long_radius = 100;
  area_short_radius = 70;
  area_close_cut_radius = 40;
  area_far_cut_radius = 75; 

  REACHAREA.center = area_center;
  REACHAREA.long_radius = area_long_radius;
  REACHAREA.short_radius = area_short_radius;
  REACHAREA.close_cut_radius = area_close_cut_radius;
  REACHAREA.far_cut_radius = area_far_cut_radius;

  kack = new KACK::Kack("/home/nao/nao.urdf", 100, true);

}

KickModule::~KickModule()
{
  delete kack;
  delete []ball_seen_array;
}

void KickModule::processFrame() {
  if(cache_.kick_request->kick_type_ == Kick::STRAIGHT) {
    if(kick_state_ == FINISHED) 
    {
      cache_.kick_request->kick_running_ = true;
      kick_state_ = INITIALIZING;
      Initializing();
    }
  }
  
  
  if(kick_state_ == INITIALIZING) 
  {
    cache_.kick_request->kick_running_ = true;
    if(!Initializing()) kick_state_ = INITIALIZING;
    else kick_state_ = TRACKING;
    // else kick_state_ = INITIALIZING;
  }
  else if(kick_state_ == TRACKING)
  {
    cache_.kick_request->kick_running_ = true;
    // if(!Tracking()) kick_state_ = TRACKING;
    // else kick_state_ = EXECUTING;
    // else kick_state_ = TRACKING;
    kick_state_ = EXECUTING;
  }
  else if(kick_state_ == EXECUTING)
  {
    cache_.kick_request->kick_running_ = true;
    if(!Executing()) kick_state_ = EXECUTING;
    else kick_state_ = PUTTING_BACK;
    // else kick_state_ = EXECUTING;
  }
  else if(kick_state_ == PUTTING_BACK)
  {
    if(!Putting_back())
    {
      cache_.kick_request->kick_running_ = true;
      kick_state_ = PUTTING_BACK;
    } 
    else 
    {
      kick_state_ = FINISHED;
      Finishing();
    }
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

  KACK::FootSensor left_foot_force_sensor, right_foot_force_sensor;
  double kp_x = 0.01;
  double kmax_x = 0.01;
  double ki_x = 0.0005;
  double kd_x = 0.000001;

  double kp_y = 0.21; //0.5
  double kmax_y = 0.21;//0.5
  double ki_y = 0.0005;//0.0005
  double kd_y = 0.0000001;//0.00001

  current_ball_location = get_ball_location(coordinate_shift);
  current_goal_location = get_goal_location(coordinate_shift);

  if(initializing_counter == -1 && if_ball_seen())
  {
    //KickKeyFrames.clear();
    if(current_ball_location.y > 15 || (abs(current_ball_location.y) <= 15 && current_goal_location.y < 0)) //left foot
    {
      coordinate_shift.update(0 , 50, -50);
      ball_direction_ = LEFTBALL;
      kick_foot_ = LEFTFOOT;
      sole = "r_sole";
      REACHAREA.center.y = -REACHAREA.center.y;
      current_ball_location = get_ball_location(coordinate_shift);
      current_goal_location = get_goal_location(coordinate_shift);
      // CurBallPosition = current_ball_location;
      // CurGoalPosition = current_goal_location;
      current_pose.update( 0 , desired_initial_y , 0  , 0 , 0 , 0);
      current_com.update(0 , 0.5*desired_initial_y , 0);
      desired_next_pose.update( 0 , desired_initial_y , 0 , 0 , 0 , 0);
      desired_next_com.update(  0 , 0 , 0);

    }
    else if(current_ball_location.y < -15 || (abs(current_ball_location.y) <= 15 && current_goal_location.y >= 0)) //right foot
    {
      coordinate_shift.update(0 , -50 , -50);
      ball_direction_ = RIGHTBALL;
      kick_foot_ = RIGHTFOOT;
      sole = "l_sole";
      current_ball_location = get_ball_location(coordinate_shift);
      current_goal_location = get_goal_location(coordinate_shift);
      // CurBallPosition = current_ball_location;
      // CurGoalPosition = current_goal_location;
      current_pose.update( 0 , -desired_initial_y , 0  , 0 , 0 , 0);
      current_com.update(0 , -0.5*desired_initial_y , 0);
      desired_next_pose.update( 0 , -desired_initial_y , 0 , 0 , 0 , 0);
      desired_next_com.update(  0 , 0 , 0);
    }

    period0 = 1.5;
    getInterpolatedFrame(period0*100 , period0*100 , current_pose , desired_next_pose , current_com , desired_next_com);
    current_pose = desired_next_pose;
    current_com = desired_next_com;
    getInterpolatedFrame(50, 50 , current_pose , desired_next_pose , current_com , desired_next_com );
    
    double raising_y = (kick_foot_ == RIGHTFOOT)?(-desired_initial_y):desired_initial_y;
    desired_next_pose.update(0.00, raising_y, 0.045 , 0 , 0 , 0);
    desired_next_com.update(0 , 0 , 0);
    period1 = 2;
    getInterpolatedFrame(period1*100 , period1*100 , current_pose , desired_next_pose , current_com , desired_next_com);
    current_pose = desired_next_pose;
    current_com = desired_next_com;
    getInterpolatedFrame(50, 50 , current_pose , desired_next_pose , current_com , desired_next_com );

//---------------------------------------------------------------------------------------------------------------------------
    
    desired_next_pose = get_desired_foot_position(current_ball_location, current_goal_location, REACHAREA , true);
    printf("current_ball_location = (%f,%f)\n", current_ball_location.x , current_ball_location.y);
    printf("desired_next_pose = (%f,%f,%f,%f,%f,%f)\n", desired_next_pose.x ,desired_next_pose.y ,desired_next_pose.z , desired_next_pose.R, desired_next_pose.P , desired_next_pose.Y );
    //desired_next_pose.update(-0.03 , -0.175 , 0.055 , -0.45 , 0.1 , 0);
    
    desired_next_com = get_desired_CoM(desired_next_pose , REACHAREA);
    printf("desire_CoM = (%f,%f)\n" , desired_next_com.x , desired_next_com.y);
    //desired_next_com.update(0 , -0.01 , 0);
    
    period2 = getRunningTime(current_pose , desired_next_pose);
    printf("period2 = %f\n", period2);
    getInterpolatedFrame(period2*100 , period2*100 , current_pose , desired_next_pose , current_com , desired_next_com);
    current_pose = desired_next_pose;
    current_com = desired_next_com;
    getInterpolatedFrame(200, 200 , current_pose , desired_next_pose , current_com , desired_next_com );


    initializing_counter = 1;
  }

  if(initializing_counter == 1)
  {
    left_foot_force_sensor  = get_left_foot_sensor();
    right_foot_force_sensor = get_right_foot_sensor();
    CurrentJoints = getCurrentJoints();
    CurrentTime = getCurrentTime();

    int index = QueueHead%1024;
    if(QueueHead == QueueRear) index = QueueHead - 1;

    kack->moveFoot(100, kick_foot_ == RIGHTFOOT, KickKeyFramesQueue[index], CurrentJoints, left_foot_force_sensor, right_foot_force_sensor, CurrentCommand, 0.15, kp_x, ki_x, kmax_x, kd_x, kp_y, ki_y, kmax_y, kd_y, 0.1); //alpha 0.3
    SendingFrame();

    printf("!!-----------------------------------------------------------------------------------------------------\n");
    printf("CounterQueue[%d] = %d , QueueRear = %d\n" , QueueHead , CounterQueue[index] , QueueRear);
    printf("min_pose = (%f,%f,%f,%f,%f,%f) \n" , KickKeyFramesQueue[index].min_pose.x , KickKeyFramesQueue[index].min_pose.y, KickKeyFramesQueue[index].min_pose.z 
                       , KickKeyFramesQueue[index].min_pose.R, KickKeyFramesQueue[index].min_pose.P, KickKeyFramesQueue[index].min_pose.Y);
    printf("max_pose = (%f,%f,%f,%f,%f,%f) \n" , KickKeyFramesQueue[index].max_pose.x , KickKeyFramesQueue[index].max_pose.y, KickKeyFramesQueue[index].max_pose.z 
                       , KickKeyFramesQueue[index].max_pose.R, KickKeyFramesQueue[index].max_pose.P, KickKeyFramesQueue[index].max_pose.Y);
    printf("min_com = (%f,%f,%f)\n", KickKeyFramesQueue[index].min_com.x, KickKeyFramesQueue[index].min_com.y, KickKeyFramesQueue[index].min_com.z);
    printf("max_com = (%f,%f,%f)\n", KickKeyFramesQueue[index].max_com.x, KickKeyFramesQueue[index].max_com.y, KickKeyFramesQueue[index].max_com.z);

    coordinate_shift.y = getYOffset(CurrentJoints, sole, "torso", kick_foot_ == RIGHTFOOT );
    CurBallPosition = get_ball_location(coordinate_shift);
    
    
  }
  else
  {
    SendingStandingFrame();
    return false;
  } 
  


  if(QueueHead != QueueRear)
  {
    //printf("if_ball_seen() = %d\n", if_ball_seen() );
    if(CounterQueue[QueueHead%1024] != 1) CounterQueue[QueueHead%1024]--;
    else QueuePop();
    return false;
  }
  else 
  {
    // QueueHead = QueueRear - 1;
    printf("!!-----------------------------------------------------------------------------------------------------\n");
    printf("!!-----------------------------------------------------------------------------------------------------\n");
    printf("!!-----------------------------------------------------------------------------------------------------\n");
    if(initializing_counter == 1) return true;
    else return false;
  }
}

bool KickModule::Tracking()
{
  double RunningTime = 0;
  KACK::Pose min_pose, max_pose;
  KACK::Pose tmp_next_pose;
  KACK::Point min_com, max_com;
  KACK::CartesianKeyframe TrackingFrame;
  KACK::FootSensor left_foot_force_sensor, right_foot_force_sensor;

  coordinate_shift.y = getYOffset(CurrentJoints, sole, "torso", kick_foot_ == RIGHTFOOT );
  CurBallPosition = get_ball_location(coordinate_shift);
  CurGoalPosition = get_goal_location(coordinate_shift);
  tmp_next_pose = get_desired_foot_position(CurBallPosition, CurGoalPosition, REACHAREA , true);
  RunningTime = getRunningTime(current_pose , tmp_next_pose);

  // printf("----------------------------------------------------------------------\n");
  // printf("CurBallPosition = ( %f , %f )\n", CurBallPosition.x , CurBallPosition.y );
  // printf("tmp_next_pose = (%f , %f , %f , %f , %f)\n" , tmp_next_pose.x , tmp_next_pose.y, tmp_next_pose.z, tmp_next_pose.R, tmp_next_pose.P);
  // printf("current_pose = (%f , %f , %f , %f , %f)\n" , current_pose.x , current_pose.y, current_pose.z, current_pose.R, current_pose.P);
  // printf("RunningTime = %f\n" , RunningTime);
  if(if_ball_seen())
  {
    if(QueueHead == QueueRear)
    {
      if(RunningTime > 0.3) if_retrack_counter++;
      else if_retrack_counter = 0;
    }
    else
      if_retrack_counter = 0;

    if(if_retrack_counter > IF_RETRACK_THRESHOLD)
    {
      printf("if_retrack_counter = %d\n" ,if_retrack_counter);
      if_retrack_counter = 0;
      desired_next_pose = tmp_next_pose;
      getInterpolatedFrame(RunningTime*100 , RunningTime*100 , current_pose , desired_next_pose , current_com , desired_next_com);
      printf("*****current_pose = (%f , %f , %f , %f , %f)\n" , current_pose.x , current_pose.y, current_pose.z, current_pose.R, current_pose.P);
      printf("*****desired_next_pose = (%f,%f,%f,%f,%f)\n", desired_next_pose.x ,desired_next_pose.y ,desired_next_pose.z , desired_next_pose.R, desired_next_pose.P  );
      printf("*****RunningTime = %f\n" , RunningTime);
      current_pose = desired_next_pose;
      current_com = desired_next_com;
      getInterpolatedFrame(300 , 300 , current_pose , desired_next_pose , current_com , desired_next_com);
    }
  }
  else
  {
    if_retrack_counter = 0;
    if_kick_counter = 0;
  }

  left_foot_force_sensor  = get_left_foot_sensor();
  right_foot_force_sensor = get_right_foot_sensor();
  CurrentJoints = getCurrentJoints();
  CurrentTime = getCurrentTime();
  double kp_x = 0.001;
  double ki_x = 0.0005;
  double kd_x = 0.0;
  double kmax_x = 0.001;

  double kp_y = 0.21;
  double kmax_y = 0.21;
  double ki_y = 0.0005;
  double kd_y = 0.000001;
  int index = QueueHead%1024;
  if(QueueHead == QueueRear) index = QueueHead - 1;

  kack->moveFoot(100, kick_foot_ == RIGHTFOOT, KickKeyFramesQueue[index], CurrentJoints, left_foot_force_sensor, right_foot_force_sensor, CurrentCommand, 0.2, kp_x, ki_x, kmax_x, kd_x, kp_y, ki_y, kmax_y, kd_y, 0.1); //alpha 0.3
  // printf("!!-----------------------------------------------------------------------------------------------------\n");
  // printf("CounterQueue[%d] = %d , QueueRear = %d\n" , QueueHead , CounterQueue[index] , QueueRear);
  // printf("min_pose = (%f,%f,%f,%f,%f,%f) \n" , KickKeyFramesQueue[index].min_pose.x , KickKeyFramesQueue[index].min_pose.y, KickKeyFramesQueue[index].min_pose.z 
  //                    , KickKeyFramesQueue[index].min_pose.R, KickKeyFramesQueue[index].min_pose.P, KickKeyFramesQueue[index].min_pose.Y);
  // printf("max_pose = (%f,%f,%f,%f,%f,%f) \n" , KickKeyFramesQueue[index].max_pose.x , KickKeyFramesQueue[index].max_pose.y, KickKeyFramesQueue[index].max_pose.z 
  //                    , KickKeyFramesQueue[index].max_pose.R, KickKeyFramesQueue[index].max_pose.P, KickKeyFramesQueue[index].max_pose.Y);
  // printf("min_com = (%f,%f,%f)\n", KickKeyFramesQueue[index].min_com.x, KickKeyFramesQueue[index].min_com.y, KickKeyFramesQueue[index].min_com.z);
  // printf("max_com = (%f,%f,%f)\n", KickKeyFramesQueue[index].max_com.x, KickKeyFramesQueue[index].max_com.y, KickKeyFramesQueue[index].max_com.z);

  SendingFrame();

  if(if_kick_counter < IF_KICK_THRESHOLD)
  {
    if(QueueHead != QueueRear)
    {
      if(CounterQueue[QueueHead%1024] != 1) 
      {
        CounterQueue[QueueHead%1024]--;
        if_kick_counter = 0;
      }
      else QueuePop();
      return false;
    }

    if( if_retrack_counter != 0) if_kick_counter = 0;
    else if_kick_counter++;

    return false;
  } 
  else return true;
}

bool KickModule::Executing()
{
  double RunningTime = 0.15;
  KACK::Pose min_pose, max_pose;
  KACK::Point min_com, max_com;
  KACK::CartesianKeyframe KickingFrame;
  KACK::FootSensor left_foot_force_sensor, right_foot_force_sensor;
  KACK::Pose DesiredMidPose;
  KACK::Point DesiredMidCom;

  coordinate_shift.y = getYOffset(CurrentJoints, sole, "torso", kick_foot_ == RIGHTFOOT );
  CurBallPosition = get_ball_location(coordinate_shift);
  CurGoalPosition = get_goal_location(coordinate_shift);

  if(executing_counter == -1)
  {
    QueueHead = 0; QueueRear = 0;
    executing_counter = 0;

    desired_next_pose = get_desired_foot_position(CurBallPosition, CurGoalPosition, REACHAREA , false);
    desired_next_pose.x += 0.015;
    //desired_next_pose.update(0.065 , -0.1 , 0.045 , 0 , -0.1 , 0);
    desired_next_com = get_desired_CoM(desired_next_pose , REACHAREA);
    //desired_next_com.update(0 , 0 , 0);

    DesiredMidPose.update((desired_next_pose.x+current_pose.x)/2.0 , (desired_next_pose.y+current_pose.y)/2.0 , 0.02 , (desired_next_pose.R+current_pose.R)/2.0 , 0 , 0);
    printf("Midpose.y = %f\n" , (desired_next_pose.y+current_pose.y)/2.0 );
    DesiredMidCom.update(0 , (desired_next_com.y + current_com.y)/2.0 , 0);
    printf("MidpCoM.y = %f\n" , (desired_next_com.y + current_com.y)/2.0 );

    getInterpolatedFrame(4 , 4 , current_pose , DesiredMidPose , current_com , DesiredMidCom);
    getInterpolatedFrame(4 , 4 , DesiredMidPose , desired_next_pose , DesiredMidCom , desired_next_com);
    current_pose = desired_next_pose;
    current_com = desired_next_com;
    getInterpolatedFrame(200 , 200 , current_pose , desired_next_pose , current_com , desired_next_com);
  }
  
  double kp_x = 0.3;
  double kmax_x = 0.3;
  double ki_x = 0.0005;
  double kd_x = 0.0;

  double kp_y = 0.21; //0.4
  double kmax_y = 0.21; //0.4
  double ki_y = 0.00005; //0.0005
  double kd_y = 0.00004; //0.00005

  left_foot_force_sensor  = get_left_foot_sensor();
  right_foot_force_sensor = get_right_foot_sensor();
  CurrentJoints = getCurrentJoints();
  CurrentTime = getCurrentTime();

  int index = QueueHead%1024;
  if(QueueHead == QueueRear) index = QueueHead - 1;

  kack->moveFoot(100, kick_foot_ == RIGHTFOOT, KickKeyFramesQueue[index], CurrentJoints, left_foot_force_sensor, right_foot_force_sensor, CurrentCommand, 0.1, kp_x, ki_x, kmax_x, kd_x, kp_y, ki_y, kmax_y, kd_y, 0.1); //alpha 0.3
  
  SendingFrame();

  if(QueueHead != QueueRear)
  {
    if(CounterQueue[QueueHead%1024] != 1) CounterQueue[QueueHead%1024]--;
    else QueuePop();
    return false;
  }
  else return true;
}

bool KickModule::Putting_back()
{
  double RunningTime0 = 2.0;
  double RunningTime1 = 1.0;
  KACK::Pose min_pose, max_pose;
  KACK::Point min_com, max_com;
  KACK::CartesianKeyframe PuttingBackFrame;
  KACK::FootSensor left_foot_force_sensor, right_foot_force_sensor;

  double putting_back_pose_y = (kick_foot_ == RIGHTFOOT)?(-0.1 ):0.1;
  double putting_back_com_y  = (kick_foot_ == RIGHTFOOT)?(-0.05):0.05;

  if(putting_back_counter == -1)
  {
    putting_back_counter = 0;

    //current_pose = desired_next_pose;
    desired_next_pose.update( 0 , putting_back_pose_y , 0 , 0 , 0 , 0 );
    desired_next_com.update( 0 , 0 , 0 );
    getInterpolatedFrame(RunningTime0*100 , RunningTime0*100 , current_pose , desired_next_pose , current_com , desired_next_com);
    current_pose = desired_next_pose;
    current_com  = desired_next_com;
    getInterpolatedFrame(100 , 100 , current_pose , desired_next_pose , current_com , desired_next_com);
    desired_next_pose.update( 0 , putting_back_pose_y , 0 , 0 , 0 , 0 );
    desired_next_com.update( 0 , putting_back_com_y , 0);
    getInterpolatedFrame(RunningTime1*100 , RunningTime1*100 , current_pose , desired_next_pose , current_com , desired_next_com);
    current_pose = desired_next_pose;
    current_com  = desired_next_com;
  }

  double kp_x = 0.3;
  double kmax_x = 0.3;
  double ki_x = 0.0005;
  double kd_x = 0.0;

  double kp_y = 0.21; //0.4
  double kmax_y = 0.21; //0.4
  double ki_y = 0.00005; //0.0005
  double kd_y = 0.00004; //0.00005

  left_foot_force_sensor  = get_left_foot_sensor();
  right_foot_force_sensor = get_right_foot_sensor();
  CurrentJoints = getCurrentJoints();
  CurrentTime = getCurrentTime();

  int index = QueueHead%1024;
  if(QueueHead == QueueRear) index = QueueHead - 1;

  kack->moveFoot(100, kick_foot_ == RIGHTFOOT, KickKeyFramesQueue[index], CurrentJoints, left_foot_force_sensor, right_foot_force_sensor, CurrentCommand, 0.1, kp_x, ki_x, kmax_x, kd_x, kp_y, ki_y, kmax_y, kd_y, 0.1); //alpha 0.3
  
  SendingFrame();

  if(QueueHead != QueueRear)
  {
    if(CounterQueue[QueueHead%1024] != 1) CounterQueue[QueueHead%1024]--;
    else QueuePop();
    return false;
  }
  else return true;
}

bool KickModule::Finishing()
{
  printf("Finishing kick\n");
  
  cache_.kick_request->kick_running_ = false;
  cache_.kick_request->kick_type_ == Kick::NO_KICK;
  return true;
}

double KickModule::random( double length ){ return (rand()*1.0/RAND_MAX)*length - length*0.5; }

KACK::Point KickModule::get_ball_location(KACK::Point shift)
{
  KACK::Point location;

  if(cache_.robot_state->ball_seen)
  {
    location.x = cache_.robot_state->ball_visionDistance * cos(cache_.robot_state->ball_visionBearing) + shift.x;
    location.y = cache_.robot_state->ball_visionDistance * sin(cache_.robot_state->ball_visionBearing) + shift.y;
    location.z = 55;

    ball_seen_array[ball_seen_counter] = 1;
    ball_seen_counter = (ball_seen_counter+1)%ball_seen_threshold;

    if(!if_ball_seen()) location.update(0,0,0);
  }
  else 
  {
    location.update(0,0,0);
    ball_seen_array[ball_seen_counter] = 0;
    ball_seen_counter = (ball_seen_counter+1)%ball_seen_threshold;

    if(if_ball_seen()) location = CurBallPosition;
  }

  return location;
}

KACK::Point KickModule::get_goal_location(KACK::Point shift)
{
  KACK::Point location;

  location.x = cache_.robot_state->goal_visionDistance * cos(cache_.robot_state->goal_visionBearing) + shift.x;
  location.y = cache_.robot_state->goal_visionDistance * sin(cache_.robot_state->goal_visionBearing) + shift.y;
  location.z = z_index;

  location.update(1500, 0 , 100);
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

bool KickModule::if_ball_seen()
{
  int sum = 0;
  for(int i = 0 ; i < ball_seen_threshold ; i++ ) sum += ball_seen_array[i];

  printf("sum = %d\n" , sum);

  if(sum > ball_seen_threshold/2) return true;
  else return false;
}


KACK::Point KickModule::get_desired_CoM(KACK::Pose foot , ReachableArea area)
{
  KACK::Point CoM(0 , 0 , 0);

  if(area.center.y > 0)
  {
    if(foot.y > (area.center.y/1000.0)) CoM.y = (foot.y - (area.center.y/1000.0))/10.0;
    else CoM.y = 0;
  }
  else if(area.center.y <= 0)
  {
    if(foot.y < (area.center.y/1000.0)) CoM.y = (foot.y - (area.center.y/1000.0))/10.0;
    else CoM.y = 0;
  }

  return CoM;
}


KACK::Pose KickModule::get_desired_foot_position(KACK::Point ball, KACK::Point goal, ReachableArea area , bool back)
{
  KACK::Pose foot;

  if(ball.x == goal.x || ball.z == 0) 
  {
    foot = current_pose;
    printf("divde zero or z = 0\n");
    return foot;
  }
  double slope = (ball.y - goal.y)/(ball.x - goal.x);
  double interception = ball.y - slope * ball.x;

  double a = slope;
  double b = interception/1000.0;
  double e = area.center.y/1000.0;
  double f = area.center.x/1000.0;
  double c = area.short_radius/1000.0;
  double d = area.long_radius/1000.0;

  double delta = c*c*d*d*(a*a*c*c-(b-e)*(b-e)+d*d-a*f*a*f-2*a*f*(b-e));
  double x0 = 0, x1 = 0 , y0 = 0 , y1 = 0 ;
  double x = 0, y = 0 , z = 0.045 ;


  // printf("---------------------------------------------------------------------\n");
  // printf("ball( %f , %f ), goal( %f , %f )\n",ball.x , ball.y , goal.x , goal.y );
  // printf("slope = %f , interception = %f\n" , a , b);
  // printf("center.y = %f , long_radius = %f , short_radius = %f\n" , e , c , d);

  if( delta <= 0)
  {
    foot = current_pose;
    printf("Not reachable ************ delta = %f\n" , delta);
    return foot;
  }

  x0 = (a*c*c*(e-b)+f*d*d+sqrt(delta))/(a*a*c*c+d*d);
  x1 = (a*c*c*(e-b)+f*d*d-sqrt(delta))/(a*a*c*c+d*d);

  x = (back)?x1:x0;
  y = a*x+b;

  // printf("calculated********************* x0 = %f, x1 = %f, y = %f\n" , x0 , x1 , y);

  if( e < 0 && y > e + (area.close_cut_radius/1000.0)) y = e + (area.close_cut_radius/1000.0);
  else if( e > 0 && y < e - (area.close_cut_radius/1000.0)) y = e - (area.close_cut_radius/1000.0);
  else if( e < 0 && y < e - (area.far_cut_radius/1000.0)) y = e - (area.far_cut_radius/1000.0);
  else if( e > 0 && y > e + (area.far_cut_radius/1000.0)) y = e + (area.far_cut_radius/1000.0);

  if( e > 0 && y > e) z += 0.13333*(y-e);
  else if( e < 0 && y < e) z += 0.13333*(e-y);

  double roll = getFootRoll( y , area );
  double pitch = (back)?0.1:(-0.1);
  
  foot.update(x , y , z , roll , pitch , 0);
  // printf("baaaaaaaaaaaaall = (%f,%f,%f)\n", ball.x, ball.y, ball.z);
  // printf("foooooooooooooot = (%f,%f,%f)\n", x , y , z_index );
  // printf("---------------------------------------------------------------------\n");
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
  min_com.update(DesireCom.x - offset.x , DesireCom.y - offset.y , 0.25);
  max_com.update(DesireCom.x + offset.x , DesireCom.y + offset.y , 0.32);
}

double KickModule::getRunningTime(KACK::Pose current , KACK::Pose next)
{
  double delta_x = next.x - current.x;
  double delta_y = next.y - current.y;
  return 25*sqrt( delta_x*delta_x + delta_y*delta_y )+0.05;
}

double KickModule::getFootRoll( double y , ReachableArea area )
{
  double roll;
  double roll_ratio = 6;

  if( area.center.y > 0 && y > (area.center.y/1000.0)) roll = roll_ratio*( y - (area.center.y/1000.0));
  else if(area.center.y < 0 && y < (area.center.y/1000.0)) roll = roll_ratio*( y - (area.center.y/1000.0));
  else roll = 0;

  // if( roll > 0.785 ) roll = 0.785;
  // else if( roll < -0.785 ) roll = -0.785;
  
  return roll;
}

void KickModule::getInterpolatedFrame(int num_frames , int num_interpolate , KACK::Pose cur_pose, KACK::Pose next_pose , KACK::Point cur_com, KACK::Point next_com )
{
  KACK::Pose min_pose, max_pose;
  KACK::Point min_com, max_com;
  KACK::CartesianKeyframe tmp_frame;
  KACK::Pose tmp_pose;
  KACK::Point tmp_com;

  double interval_x = (next_pose.x - cur_pose.x)/num_interpolate;
  double interval_y = (next_pose.y - cur_pose.y)/num_interpolate;
  double interval_z = (next_pose.z - cur_pose.z)/num_interpolate;
  double interval_R = (next_pose.R - cur_pose.R)/num_interpolate;
  double interval_P = (next_pose.P - cur_pose.P)/num_interpolate;
  double interval_Y = (next_pose.Y - cur_pose.Y)/num_interpolate;

  double interval_com_x = (next_com.x - cur_com.x)/num_interpolate;
  double interval_com_y = (next_com.y - cur_com.y)/num_interpolate;
  
  double RunningTime = 0;

  for( int i = 0 ; i <= num_interpolate ; i++)
  {
    tmp_pose.update(cur_pose.x + i*interval_x , cur_pose.y + i*interval_y , cur_pose.z + i*interval_z , cur_pose.R + i*interval_R , cur_pose.P + i*interval_P , cur_pose.Y + i*interval_Y);
    tmp_com.update(cur_com.x + i*interval_com_x, cur_com.y + i*interval_com_y, 0);
    RunningTime = ((int)num_frames/num_interpolate) / 100.0;
    AddPoseTolerance(tmp_pose, min_pose , max_pose , PoseOffset);
    AddComTolerance(tmp_com , min_com , max_com , ComOffset);
    tmp_frame.update( RunningTime , min_pose, max_pose, min_com, max_com);
    QueuePush( tmp_frame , (int)num_frames/num_interpolate );
  }
}

double KickModule::getYOffset(std::vector<double> current_joint_positions, std::string support_frame, std::string lookup_frame, bool left_foot_supporting)
{ 
  dynamics_tree::Matrix4 m_rootTworld = dynamics_tree::Matrix4::Identity();
  kack->tree(left_foot_supporting).kinematics(current_joint_positions,  m_rootTworld);

  dynamics_tree::Matrix4 lookupTsupport;
  kack->tree(left_foot_supporting).lookupTransform(lookup_frame, support_frame, lookupTsupport);
  return lookupTsupport(1, 3);
}

bool KickModule::if_pose_valid(KACK::Pose cur , ReachableArea area)
{
  if(kick_foot_ == RIGHTFOOT)
  {

  }
  else
  {

  }
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

void KickModule::SendingStandingFrame()
{
  std::array<float, NUM_JOINTS> JointsCommand;
  double a = 180.0/3.1415926;

  JointsCommand[HeadYaw] = 0.0854866476518796/a;
  JointsCommand[HeadPitch] = -21.9705328993966/a;
  JointsCommand[LHipYawPitch] = 0.441858597786465/a;
  JointsCommand[LHipRoll] = 0.261268396131328/a;
  JointsCommand[LHipPitch] = -24.1678184157681/a;
  JointsCommand[LKneePitch] = 51.5021569269836/a;
  JointsCommand[LAnklePitch] = -24.7878766490002/a;
  JointsCommand[LAnkleRoll] = 0.261268396131328/a;
  JointsCommand[RHipYawPitch] = 0.441858597786465/a;
  JointsCommand[RHipRoll] = 0.705531220505637/a;
  JointsCommand[RHipPitch] = -23.6452816235054/a;
  JointsCommand[RKneePitch] =  48.8702084171163/a;
  JointsCommand[RAnklePitch] =  -24.607286447345/a;
  JointsCommand[RAnkleRoll] =  -0.261268396131328/a;
  JointsCommand[LShoulderPitch] =  -95.6237958519591/a;
  JointsCommand[LShoulderRoll] =  6.58947964328316/a;
  JointsCommand[LElbowYaw] =  -0.353967723546741/a;
  JointsCommand[LElbowRoll] =  -1.93120866706443/a;
  JointsCommand[RShoulderPitch] =  -94.8375864369772/a;
  JointsCommand[RShoulderRoll] =  7.56108771309582/a;
  JointsCommand[RElbowYaw] =  -0.173377535231817/a;
  JointsCommand[RElbowRoll] =  -2.63915777453625/a;

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
