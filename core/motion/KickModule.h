#pragma once

#include <Module.h>
#include <common/RobotInfo.h>
#include <memory/MemoryCache.h>
#include <memory/WorldObjectBlock.h>
#include "kack.hpp"
#include <stdio.h>    
#include <sys/time.h>      

class Keyframe;
class KeyframeSequence;

class KickModule : public Module {
  public:
    KickModule();
    ~KickModule();
    void initSpecificModule();
    void specifyMemoryDependency();
    void specifyMemoryBlocks();

    void processFrame();

  protected:
    //------------------------------------------------dynamic kick-------------------------------------------------------
    ENUM(NewKickState,
      INITIALIZING,
      TRACKING,
      EXECUTING,
      PUTTING_BACK,
      FINISHED
    );

    ENUM(BallDirection,
      LEFTBALL,
      MIDDLEBALL,
      RIGHTBALL
    );

    ENUM(GoalDirection,
      LEFTGOAL,
      MIDDLEGOAL,
      RIGHTGOAL
    );

    ENUM(KickFoot,
      STANDFOOT,
      LEFTFOOT,
      RIGHTFOOT
    );

    void Initializing();
    bool Tracking();
    void Executing();
    void Putting_back();
    bool Finishing();

    KACK::Point get_ball_location(KACK::Point shift);
    KACK::Point get_goal_location(KACK::Point shift);
    KACK::FootSensor get_left_foot_sensor();
    KACK::FootSensor get_right_foot_sensor();
    void getCurrentTime();    
    void getCurrentJoints();
    //-------------------------------------old keyframe based kicking version-------------------------------------------- 
    ENUM(KickState,
      Initial,
      Running,
      Finished
    );
    void start();
    void finish();
    bool finished();
    void moveBetweenKeyframes(const Keyframe& start, const Keyframe& finish, int cframe);
    void performKick();
    void moveToInitial(const Keyframe& keyframe, int cframe);
    bool reachedKeyframe(const Keyframe& keyframe);
    //------------------------------------------------------------------------------------------------------------------- 
  private:
    NewKickState kick_state_;
    BallDirection ball_direction_;
    GoalDirection goal_direction_;
    KickFoot kick_foot_;

    KACK::Point current_ball_location;
    KACK::Point current_goal_location;
    KACK::Point coordinate_shift;
    KACK::Pose  current_foot_pose;
    KACK::Pose  desired_foot_pose;

    KACK::Kack* kack;
    std::vector<KACK::CartesianKeyframe> KickKeyFrames ;
    std::vector<double> CurrentJoints;
    std::vector<double> CurrentCommand;
    double CurrentTime;

    double t_d_pose;
    double t_a_pose;
    double t_d_com;
    double z_index;

    //Pose  desired_initial_left_foot_pose;
    //Pose  desired_initial_right_foot_pose;

    struct ReachableArea
    {
      KACK::Point center;
      double radius;
      double short_radius;

      ReachableArea( KACK::Point c, double r, double sr):center(c), radius(r), short_radius(sr){}
    };

    //-------------------------------------old keyframe based kicking version--------------------------------------------
    KickState state_;
    //-------------------------------------------------------------------------------------------------------------------

    MemoryCache cache_;
    KeyframeSequence* sequence_;
    Keyframe* initial_;
    int frames_;
    int keyframe_;
};
