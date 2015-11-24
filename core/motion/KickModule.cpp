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
#include <stdio.h>
#include <time.h> 
#include "kack.hpp"

#define JOINT_EPSILON (3.f * DEG_T_RAD)
#define DEBUG false

KickModule::KickModule() : state_(Finished), sequence_(NULL) 
{
  KACK::Point area_center;
  double area_radius, area_short_radius;

  kick_state_ = FINISHED;
  ball_direction_ = RIGHTBALL;
  goal_direction_ = MIDDLEGOAL;
  kick_foot_ = STANDFOOT;

  coordinate_shift.update(0,0,0);
  current_ball_location.update(0,0,0);
  current_goal_location.update(0,0,0);
  current_foot_pose.update(0,0,0,0,0,0);
  desired_foot_pose.update(0,0,0,0,0,0);

  t_d_pose = 0.005;
  t_a_pose = 0.04;
  t_d_com  = 0.0025;
  z_index  = 0.05;
  NUMBER_JOINTS = 26;

  if_planned = 0;
  if_ready_for_initializing = 0;
  running_counter = 0;
  executing_counter = 0;

  CurrentJoints.resize(NUMBER_JOINTS);
  CurrentCommand.resize(NUMBER_JOINTS);

  area_center.update(0, 0.150 , z_index);
  area_radius = 0.15;
  area_short_radius = 0.1;

  ReachableArea REACHAREA(area_center , area_radius , area_short_radius);

  kack = new KACK::Kack("/home/nao/nao.urdf");

}

KickModule::~KickModule()
{
  delete kack;
}

void KickModule::processFrame() {
  if(cache_.kick_request->kick_type_ == Kick::STRAIGHT) {
    if(kick_state_ == FINISHED || kick_state_ == INITIALIZING) Initializing();
  }
  
  if(kick_state_ == INITIALIZING) {
    cache_.kick_request->kick_running_ = true;
    Initializing();
  }
}



void KickModule::Initializing()
{
  KACK::CartesianKeyframe frame_moving_com, frame_first_tracking ;
  KACK::Pose min_pose, max_pose;
  KACK::Point min_com, max_com;
  KACK::Point point_first_tracking ;

  double desired_initial_y = 0.1 ;
  KickKeyFrames.clear();

  printf("I'm running this part , NUM_JOINTS = %d\n" , NUM_JOINTS);

  cache_.kick_request->kick_running_ = true;
  printf("I'm running this part 2\n");
  //auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  kick_state_ = INITIALIZING;
  printf("I'm running this part 3\n");
  current_ball_location = get_ball_location(coordinate_shift);
  printf("I'm running this part 4\n");
  current_goal_location = get_goal_location(coordinate_shift);

  printf("I'm running this part 5\n");

  if(current_ball_location.y > 15 || (abs(current_ball_location.y) < 15 && current_goal_location.y < 0)) //left foot
  {
    coordinate_shift.update(0 , -z_index , -200);
    ball_direction_ = LEFTBALL;
    kick_foot_ = LEFTFOOT;
    current_ball_location = get_ball_location(coordinate_shift);
    current_goal_location = get_goal_location(coordinate_shift);
    current_foot_pose.update( 0    , -desired_initial_y  , 0  , 0 , 0 , 0);
    desired_foot_pose.update( 0    , -desired_initial_y  , 0  , 0 , 0 , 0);

    min_pose.update( -t_d_pose , desired_initial_y - t_d_pose , -t_d_pose, -t_a_pose , -t_a_pose , -t_a_pose );
    max_pose.update(  t_d_pose , desired_initial_y + t_d_pose ,  t_d_pose,  t_a_pose ,  t_a_pose ,  t_a_pose );
    min_com.update(-t_d_com , -t_d_com , 0);
    max_com.update( t_d_com ,  t_d_com , 1);

    frame_moving_com.update( 0.5 , min_pose, max_pose, min_com, max_com );
  }
  else if(current_ball_location.y < -15 || (abs(current_ball_location.y) < 15 && current_goal_location.y > 0)) //right foot
  {
    coordinate_shift.update(0 , z_index , -200);
    ball_direction_ = RIGHTBALL;
    kick_foot_ = RIGHTFOOT;
    current_ball_location = get_ball_location(coordinate_shift);
    current_goal_location = get_goal_location(coordinate_shift);
    current_foot_pose.update( 0    ,  desired_initial_y  , 0  , 0 , 0 , 0);
    desired_foot_pose.update( 0    ,  desired_initial_y  , 0  , 0 , 0 , 0);

    min_pose.update( -t_d_pose , -desired_initial_y - t_d_pose , -t_d_pose, -t_a_pose , -t_a_pose , -t_a_pose );
    max_pose.update(  t_d_pose , -desired_initial_y + t_d_pose ,  t_d_pose,  t_a_pose ,  t_a_pose ,  t_a_pose );
    min_com.update(-t_d_com , -t_d_com , 0);
    max_com.update( t_d_com ,  t_d_com , 1);

    frame_moving_com.update( 0.5 , min_pose, max_pose, min_com, max_com );
  }

  printf("I'm running this part 6\n");

  //point_first_tracking = get_desired_foot_position(current_ball_location, current_goal_location, REACHAREA);
  printf("min_pose = (%f,%f,%f,%f,%f,%f) \n" , min_pose.x , min_pose.y, min_pose.z , min_pose.R, min_pose.P, min_pose.Y);
  printf("max_pose = (%f,%f,%f,%f,%f,%f) \n" , max_pose.x , max_pose.y, max_pose.z , max_pose.R, max_pose.P, max_pose.Y);
  printf("min_com = (%f,%f,%f)\n", min_com.x, min_com.y, min_com.z);
  printf("max_com = (%f,%f,%f)\n", max_com.x, max_com.y, max_com.z);
  KickKeyFrames.push_back(frame_moving_com);
  //KickKeyFrames.push_back(frame_first_tracking);
  printf("I'm running this part 7\n");

  getCurrentJoints();
  if(if_planned == 0 && kack->plan(CurrentJoints, KickKeyFrames , kick_foot_ == RIGHTFOOT ))
    if_planned = 1;

  printf("I'm running this part 8\n");

  KACK::FootSensor left_foot_force_sensor, right_foot_force_sensor;

  left_foot_force_sensor  = get_left_foot_sensor();
  right_foot_force_sensor = get_right_foot_sensor();
  getCurrentTime();
  getCurrentJoints();
  printf("I'm running this part 9\n");
  kack->execute( CurrentTime, CurrentJoints, left_foot_force_sensor, right_foot_force_sensor, CurrentCommand);
  printf("I'm running this part 10\n");

  std::array<float, NUM_JOINTS> JointsCommand;
  for( int i = 0 ; i < NUM_JOINTS ; i++) 
  {
    JointsCommand[i]= CurrentCommand[i];
    printf("JointsCommand[%d] = %f\n", i , JointsCommand[i] );
  }

  cache_.joint_command->setSendAllAngles(true, 1 * 10);
  cache_.joint_command->setPoseRad(JointsCommand.data());

 
}

bool KickModule::Tracking()
{

}

void KickModule::Executing()
{

}

void KickModule::Putting_back()
{

}

bool KickModule::Finishing()
{
  return true;
}

KACK::Point KickModule::get_ball_location(KACK::Point shift)
{
  KACK::Point location;
  // auto& ball = cache_.world_object->objects_[WO_BALL];

  // location.x = ball.visionDistance * cos(ball.visionBearing) + shift.x;
  // location.y = ball.visionDistance * sin(ball.visionBearing) + shift.y;
  // location.z = z_index;

  location.update(50, -100 , 50);
  return location;
}

KACK::Point KickModule::get_goal_location(KACK::Point shift)
{
  KACK::Point location;
  // auto& goal  = cache_.world_object->objects_[WO_OPP_GOAL];
  // auto& enemy = cache_.world_object->objects_[WO_OPPONENT1];

  // if(!goal.seen) location.update(0,0,0);
  // else if(!enemy.seen)
  // {
  //   location.x = goal.visionDistance * cos(goal.visionBearing) + shift.x;
  //   location.y = goal.visionDistance * sin(goal.visionBearing) + shift.y;
  //   location.z = z_index;
  // }
  // else
  // {
  //   location.x = goal.visionDistance * cos(goal.visionBearing) + shift.x;
  //   location.y = goal.visionDistance * sin(goal.visionBearing) + shift.y;
  //   location.z = z_index;
  //   // double gx = goal.visionDistance  * cos(goal.visionBearing);
  //   // double gy = goal.visionDistance  * sin(goal.visionBearing);
  //   // double ex = enemy.visionDistance * cos(enemy.visionBearing);
  //   // double ey = enemy.visionDistance * sin(enemy.visionBearing);
  //   // double center_threshold = goal.radius*0.1;
  // }
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


void KickModule::getCurrentTime()    
{    
  struct timeval tv;    
  gettimeofday(&tv,NULL);    
  CurrentTime = tv.tv_sec*1.0 + tv.tv_usec / 1000000.0;    
}

void KickModule::getCurrentJoints()
{
  for(int i = 0 ; i < NUM_JOINTS ; i++)
    CurrentJoints[i] = cache_.joint->values_[i];
  for(int i = NUM_JOINTS ; i < NUMBER_JOINTS ; i++)
    CurrentJoints[i] = 0;
}


KACK::Point KickModule::get_desired_foot_position(KACK::Point ball, KACK::Point goal, ReachableArea area)
{
  KACK::Point foot;

  double slope = (ball.y - goal.y)/(ball.x - goal.x);
  double interception = ball.y - slope * ball.x;

  double a = slope;
  double b = interception;
  double c = area.center.y;
  double r = area.radius;

  double delta = (a*a+1)*r*r - (b-c)*(b-c);
  double x = 0, y = 0;

  if( delta > 0 ) x = (a*(c-b)-sqrt(delta))/(a*a+1);
  else
  {
    foot.update(0,0,0);
    return foot;
  }

  y = a*x + b;
  if(y > c + area.short_radius && c < 0)
  {
    y = c + area.short_radius;
    x = (y - b)/a;
  }
  else if( y < c - area.short_radius && c > 0)
  {
    y = c - area.short_radius;
    x = (y - b)/a;
  }

  foot.update(x,y,z_index);
  return foot;
}

void KickModule::SendingFrame()
{
  KACK::FootSensor left_foot_force_sensor, right_foot_force_sensor;

  left_foot_force_sensor  = get_left_foot_sensor();
  right_foot_force_sensor = get_right_foot_sensor();
  getCurrentTime();
  getCurrentJoints();
  kack->execute( CurrentTime, CurrentJoints, left_foot_force_sensor, right_foot_force_sensor, CurrentCommand);

  std::array<float, NUM_JOINTS> JointsCommand;
  for( int i = 0 ; i < NUM_JOINTS ; i++) JointsCommand[i]= CurrentCommand[i];

  cache_.joint_command->setSendAllAngles(true, 1 * 10);
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
  requiresMemoryBlock("frame_info");
  requiresMemoryBlock("walk_request");
  requiresMemoryBlock("processed_joint_angles");
  requiresMemoryBlock("processed_joint_commands");
  requiresMemoryBlock("odometry");
  requiresMemoryBlock("processed_sensors");
  requiresMemoryBlock("body_model");
  requiresMemoryBlock("kick_request");
}

void KickModule::specifyMemoryBlocks() {
  cache_.memory = memory_;
  getMemoryBlock(cache_.frame_info,"frame_info");
  getMemoryBlock(cache_.walk_request,"walk_request");
  getMemoryBlock(cache_.joint,"processed_joint_angles");
  getMemoryBlock(cache_.joint_command,"processed_joint_commands");
  getMemoryBlock(cache_.odometry,"odometry");
  getMemoryBlock(cache_.sensor,"processed_sensors");
  getMemoryBlock(cache_.body_model,"body_model");
  getMemoryBlock(cache_.kick_request,"kick_request");
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
