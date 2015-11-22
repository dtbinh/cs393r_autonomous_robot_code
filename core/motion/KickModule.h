#pragma once

#include <Module.h>
#include <common/RobotInfo.h>
#include <memory/MemoryCache.h>
#include <kack.hpp>

class Keyframe;
class KeyframeSequence;

class KickModule : public Module {
  public:
    KickModule();
    void initSpecificModule();
    void specifyMemoryDependency();
    void specifyMemoryBlocks();

    void processFrame();

  protected:
    //------------------------------------------------dynamic kick-------------------------------------------------------
    ENUM(NewKickState,
      Initializing,
      Tracking,
      Executing,
      Putting_back,
      Finished
    );

    ENUM(BallDirection,
      LEFT,
      MIDDLE,
      RIGHT
    );

    ENUM(GoalDirection,
      LEFT,
      MIDDEL,
      RIGHT
    );

    ENUM(KickFoot,
      LEFT,
      RIGHT
    );

    bool Initializing();
    bool Tracking();
    bool Executing();
    bool Putting_back();
    bool Finished();



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
    BallPosition ball_direction_;
    GoalPosition goal_direction_;
    KickFoot kick_foot_

    Point current_ball_location;
    Pose  current_foot_pose;
    Pose  desired_initial_left_foot_pose;
    Pose  desired_initial_right_foot_pose;
    Pose  desired_current_left_foot_pose;
    Pose  desired_current_right_foot_pose;

    struct ReachableArea
    {
      Point center;
      double radius;
      double ratio;
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
