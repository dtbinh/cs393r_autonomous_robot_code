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
#include <kack.hpp>

#define JOINT_EPSILON (3.f * DEG_T_RAD)
#define DEBUG false

KickModule::KickModule() : state_(Finished), sequence_(NULL) 
{
  Point area_center;
  double area_radius, area_short_radius;

  kick_state_ = Finishing;
  ball_direction_ = RIGHT;
  goal_direction_ = MIDDLE;
  kick_foot_ = STAND;

  coordinate_shift.update(0,0,0);
  current_ball_location.update(0,0,0);
  current_goal_location.update(0,0,0);
  current_foot_pose.update(0,0,0,0,0,0,0);
  desired_foot_pose.update(0,0,0,0,0,0,0);

  area_center.update(0, 150.0 , 40.0);
  area_radius = 150.0;
  area_short_radius = 100.0;

  ReachableArea REACHAREA(area_center , area_radius , area_short_radius);
}

void KickModule::processFrame() {
  if(cache_.kick_request->kick_type_ == Kick::STRAIGHT) {
    if(state_ == Finished) Initializing();
  }
  if(state_ == Initial || state_ == Running) {
    cache_.kick_request->kick_running_ = true;
    performKick();
  }
}

void KickModule::Initializing()
{
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  kick_state_ = Initialing;

  current_ball_location = get_ball_location(coordinate_shift);
  current_goal_location = get_goal_location(coordinate_shift);

  if(current_ball_location.y > 30 || (abs(current_ball_location) < 30 && current_goal_location.y < 0)) //left foot
  {
    coordinate_shift.update(0 , -40 , -200);
    ball_direction_ = LEFT;
    current_ball_location = get_ball_location(coordinate_shift);
    current_foot_pose.update( 0    , -100  , 0  , 0 , 0 , 0);
    desired_foot_pose.update( 0    , -100  , 0  , 0 , 0 , 0);
  }
  else if(current_ball_location.y < -30 || (abs(current_ball_location) < 30 && current_goal_location.y > 0)) //right foot
  {
    coordinate_shift.update(0 , 40 , -200);
    ball_direction_ = RIGHT;
    current_ball_location = get_ball_location(coordinate_shift);
    current_foot_pose.update( 0    ,  100  , 0  , 0 , 0 , 0);
    desired_foot_pose.update( 0    ,  100  , 0  , 0 , 0 , 0);
  }


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

Point KickModule::get_ball_location(Point shift)
{
  Point location;
  auto& ball = cache_.world_object->objects_[WO_BALL];

  location.x = ball.visionDistance * cos(ball.visionBearing) + shift.x;
  location.y = ball.visionDistance * sin(ball.visionBearing) + shift.y;
  location.z = 40.0;

  return location;
}

Point KickModule::get_goal_location(Point shift)
{
  Point location;
  auto& goal = cache_.world_object->objects_[WO_OPP_GOAL];

  location.x = goal.visionDistance * cos(goal.visionBearing) + shift.x;
  location.y = goal.visionDistance * sin(goal.visionBearing) + shift.y;
  location.z = 40.0;

  return location;
}

FootSensor KickModule::get_foot_sensor(KickFoot state)
{
  FootSensor foot_force;
  if(state = LEFT)
  {
    foot_force.fl = cache_.sensor->values_[fsrRFL];
    foot_force.fr = cache_.sensor->values_[fsrRFR];
    foot_force.rl = cache_.sensor->values_[fsrRRL];
    foot_force.rr = cache_.sensor->values_[fsrRRR];
  }
  else(state = RIGHT)
  {
    foot_force.fl = cache_.sensor->values_[fsrLFL];
    foot_force.fr = cache_.sensor->values_[fsrLFR];
    foot_force.rl = cache_.sensor->values_[fsrLRL];
    foot_force.rr = cache_.sensor->values_[fsrLRR];
  }

  return foot_force;
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
    cache_.joint_command->setPoseRad(joints.data());
  }
}
