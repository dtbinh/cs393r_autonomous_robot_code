#pragma once

#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/WalkInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <Module.h>
#include <memory/MemoryCache.h>
#include <localization/LocalizationParams.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <../kalman_filters/include/kalman_filters/extended_kalman_filter.hpp>
#include <../particle_filters/RMCL_particle_filter.hpp>

typedef ExtendedKalmanFilter<4, 4, 1> KF;
typedef RMCLParticleFilter<(25*12), 3, 12, 3> RPF;

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
    void createPF();

    void loadParams(LocalizationParams params);
    double getdistance( double x , double y , double bx , double by );
    double gettheta( double x , double y , double ori , double bx , double by );
    
    //Kalman filter
    bool first;
    unsigned int unseen_count;
    KF *ball_filter;

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
    RPF* pfilter_;

    RPF::ParticleVector NAO_LOCATION;
    //KF :: StateVector mu_past_0;
    //KF :: StateVector mu_past_1;

    Eigen::Matrix<double, 4, 4> A;
    Eigen::Matrix<double, 4, 1> B;
    Eigen::Matrix<double, 4, 4> C;
};
