#pragma once

#include <Module.h>
#include <memory/MemoryCache.h>
#include <localization/LocalizationParams.h>
#include <../kalman_filters/include/kalman_filters/linear_kalman_filter.hpp>

class LocalizationModule : public Module {
  public:
    LocalizationModule();
    void specifyMemoryDependency();
    void specifyMemoryBlocks();
    void initSpecificModule();
    void initFromMemory();
    void initFromWorld();
    void reInit();
    void processFrame();

    void loadParams(LocalizationParams params);

    //Kalman filter
    typedef LinearKalmanFilter<4, 4, 1> KF;
    KF *ball_filter;

  protected:
    MemoryCache cache_;
    TextLogger*& tlogger_;
    LocalizationParams params_;
};
