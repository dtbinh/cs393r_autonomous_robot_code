#pragma once

#include <Module.h>
#include <common/RobotInfo.h>
#include <memory/MemoryCache.h>
#include <kack.h>

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
    NewKickState kick_state_
    
    //-------------------------------------old keyframe based kicking version--------------------------------------------
    KickState state_;
    MemoryCache cache_;
    KeyframeSequence* sequence_;
    Keyframe* initial_;
    int frames_;
    int keyframe_;
};
