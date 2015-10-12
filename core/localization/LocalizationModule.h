#pragma once

#include <Module.h>
#include <memory/MemoryCache.h>
#include <localization/LocalizationParams.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
// #include <../kalman_filters/include/kalman_filters/linear_kalman_filter.hpp>
#include <../kalman_filters/include/kalman_filters/extended_kalman_filter.hpp>

typedef ExtendedKalmanFilter<4, 4, 1> KF;

class ParticleFilter;
class Point2D;

class LocalizationModule : public Module {
  public:
    LocalizationModule();
    ~LocalizationModule();
    void specifyMemoryDependency();
    void specifyMemoryBlocks();
    void initSpecificModule();
    void initFromMemory();
    void initFromWorld();
    void reInit();
    void processFrame();

    void loadParams(LocalizationParams params);
    
    //Kalman filter
    bool first;
    unsigned int unseen_count;
    KF *ball_filter;
    //KF :: StateVector mu_past_0;
    //KF :: StateVector mu_past_1;

    Eigen::Matrix<double, 4, 4> A;
    Eigen::Matrix<double, 4, 1> B;
    Eigen::Matrix<double, 4, 4> C;


    KF::StateVector g(KF::StateVector x, KF::ControlVector u);
    KF::MeasurementVector h(KF::StateVector x);
    KF::StateJacobianMatrix G(KF::StateVector x, KF::ControlVector u);
    KF::MeasurementJacobianMatrix H(KF::StateVector x);

    void moveBall(const Point2D& position);
    void movePlayer(const Point2D& position, float orientation);
  protected:
    MemoryCache cache_;
    TextLogger*& tlogger_;
    LocalizationParams params_;
    ParticleFilter* pfilter_;
};
