#pragma once

#include <Module.h>
#include <common/RobotInfo.h>
#include <memory/MemoryCache.h>
#include <memory/RobotStateBlock.h>
#include <memory/Memory.h>
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

    struct ReachableArea
    {
      KACK::Point center;
      double radius;
      double short_radius;

      ReachableArea( KACK::Point c, double r, double sr):center(c), radius(r), short_radius(sr){}
    };

    bool Initializing();
    bool Tracking();
    bool Executing();
    bool Putting_back();
    bool Finishing();

    KACK::Point get_ball_location(KACK::Point shift);
    KACK::Point get_goal_location(KACK::Point shift);
    KACK::Point get_desired_foot_position(KACK::Point ball, KACK::Point goal, ReachableArea area);
    KACK::FootSensor get_left_foot_sensor();
    KACK::FootSensor get_right_foot_sensor();
    std::vector<double> getCurrentJoints();
    void AddPoseTolerance(KACK::Pose DesirePose, KACK::Pose &min_pose, KACK::Pose &max_pose, KACK::Pose offset );
    void AddComTolerance(KACK::Point DesireCom , KACK::Point &min_com, KACK::Point &max_com, KACK::Point offset);
    double getCurrentTime();
    double getRunningTime(KACK::Pose current , KACK::Pose next);    
    
    void SendingFrame();
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
    KACK::Point desired_next_com;
    KACK::Pose  desired_next_pose;
    KACK::Pose  current_pose;
    

    KACK::Kack* kack;
    std::vector<KACK::CartesianKeyframe> KickKeyFrames ;
    std::vector<double> CurrentJoints;
    std::vector<double> CurrentCommand;
    double CurrentTime;
    double z_index;

    KACK::Pose PoseOffset;
    KACK::Point ComOffset;
    int NUMBER_JOINTS;
    
    int initializing_counter;
    int tracking_counter;
    int executing_counter;
    int putting_back_counter;

    //Pose  desired_initial_left_foot_pose;
    //Pose  desired_initial_right_foot_pose;

    //-------------------------------------old keyframe based kicking version--------------------------------------------
    KickState state_;
    //-------------------------------------------------------------------------------------------------------------------

    MemoryCache cache_;
    KeyframeSequence* sequence_;
    Keyframe* initial_;
    int frames_;
    int keyframe_;
};
