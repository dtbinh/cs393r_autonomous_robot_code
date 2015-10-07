#include <localization/LocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
//#include <Eigen/Dense>

// Boilerplate
LocalizationModule::LocalizationModule() : tlogger_(textlogger) {
  KF::StateTransitionMatrix A_matrix = KF::StateTransitionMatrix::Identity();
  KF::InputMatrix B_matrix = KF::InputMatrix::Zero();
  KF::OutputMatrix C_matrix = KF::OutputMatrix::Identity();
  KF::StateCovarianceMatrix R_matrix = KF::StateCovarianceMatrix::Zero();
  KF::MeasurementCovarianceMatrix Q_matrix = KF::MeasurementCovarianceMatrix::Identity();

  double dt = 1.0/30.0;
  double xvf = 0.99;
  double yvf = 0.99;
  A_matrix << 1.0 , dt  , 0   , 0   ,
              0   , xvf , 0   , 0   ,
              0   , 0   , 1.0 , dt  ,
              0   , 0   , 0   , yvf ;

  //B_matrix << 0;
  C_matrix << 1 , 0 , 0 , 0 ,
              0 , 1 , 0 , 0 ,
              0 , 0 , 1 , 0 ,
              0 , 0 , 0 , 1 ;

  double x_var = 0.5 ;
  double xv_var = 2;
  double yv_var = 2;
  double y_var = 0.5 ;

  R_matrix <<   x_var ,   0    ,   0    ,   0    ,
                  0   , xv_var ,   0    ,   0    ,
                  0   ,   0    ,  y_var ,   0    ,
                  0   ,   0    ,   0    , yv_var ;

  double zx_var = 1;
  double zy_var = 1;
  double zxv_var = 2;
  double zyv_var = 2;
  Q_matrix <<   zx_var  ,   0     ,   0    ,   0     ,
                  0     , zxv_var ,   0    ,   0     ,
                  0     ,   0     , zy_var ,   0     ,
                  0     ,   0     ,   0    , zyv_var ;

  ball_filter = new KF(A_matrix, B_matrix, C_matrix, R_matrix, Q_matrix);
}

// Boilerplate
void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
}


// Load params that are defined in cfglocalization.py
void LocalizationModule::loadParams(LocalizationParams params) {
  params_ = params;
  printf("Loaded localization params for %s\n", params_.behavior.c_str());
}

// Perform startup initialization such as allocating memory
void LocalizationModule::initSpecificModule() {
  reInit();
}

// Initialize the localization module based on data from the LocalizationBlock
void LocalizationModule::initFromMemory() {
  reInit();
}

// Initialize the localization module based on data from the WorldObjectBlock
void LocalizationModule::initFromWorld() {
  reInit();
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  cache_.localization_mem->player = self.loc;
}

// Reinitialize from scratch
void LocalizationModule::reInit() {
  cache_.localization_mem->player = Point2D(-750,0);
  cache_.localization_mem->state = decltype(cache_.localization_mem->state)::Zero();
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity();
}

void LocalizationModule::processFrame() {
  auto& ball = cache_.world_object->objects_[WO_BALL];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  // Retrieve the robot's current location from localization memory
  // and store it back into world objects
  auto sloc = cache_.localization_mem->player;
  self.loc = sloc;
    
  //TODO: modify this block to use your Kalman filter implementation

  KF::StateVector estimated_state;
  KF::MeasurementVector measurement;
  KF::ControlVector control;
  control << 0.0;

  if(ball.seen) {
    // Compute the relative position of the ball from vision readings

    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);
    // Compute the global position of the ball based on our assumed position and orientation
    auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

    // Update the ball in the WorldObject block so that it can be accessed in python
    ball.loc = globalBall;
    ball.distance = ball.visionDistance;
    ball.bearing = ball.visionBearing;

    printf("bearing = %f \n" , ball.bearing);

    //double relative_x = ball.distance * cos( ball.bearing );
    //double relative_y = ball.distance * sin( ball.bearing );
    KF :: StateVector mu = ball_filter->get_mu();
    double measured_vx = (ball.loc.x - mu(0)) * 30.0;
    double measured_vy = (ball.loc.y - mu(2)) * 30.0;
    measurement << ball.loc.x , measured_vx , ball.loc.y , measured_vy ;
    estimated_state = ball_filter -> process(measurement, control);

    Eigen:: Matrix< float , 2 , 2 > cov ;
    cov(0)= ball_filter -> get_sigma_value(0,0);
    cov(1)= ball_filter -> get_sigma_value(0,2);
    cov(2)= ball_filter -> get_sigma_value(2,0);
    cov(3)= ball_filter -> get_sigma_value(2,2);

    //ball.absVel = fill this in
    // Update the localization memory objects with localization calculations
    // so that they are drawn in the World window()
    cache_.localization_mem->state[0] = estimated_state(0) ;
    cache_.localization_mem->state[1] = estimated_state(2) ;
    cache_.localization_mem->state[2] = estimated_state(1) ;
    cache_.localization_mem->state[3] = estimated_state(3) ;
    cache_.localization_mem->covariance = cov * 10000;
  } 
  //TODO: How do we handle not seeing the ball?
  else {
    //ball.distance = 10000.0f;
    //ball.bearing = 0.0f;
    KF :: StateVector mu = ball_filter->get_mu();

    ball_filter->update_mu(0 , mu(0) + mu(1)/30.0 );
    ball_filter->update_mu(1 , mu(1) * 0.98);
    ball_filter->update_mu(2 , mu(2) + mu(3)/30.0 );
    ball_filter->update_mu(3 , mu(3) * 0.98);


    Eigen:: Matrix< float , 2 , 2 > cov ;
    cov(0)= ball_filter -> get_sigma_value(0,0);
    cov(1)= ball_filter -> get_sigma_value(0,2);
    cov(2)= ball_filter -> get_sigma_value(2,0);
    cov(3)= ball_filter -> get_sigma_value(2,2);
   
    cache_.localization_mem->state[0] = mu(0) + mu(1)/30.0 ;
    cache_.localization_mem->state[1] = mu(2) + mu(3)/30.0 ;
    cache_.localization_mem->state[2] = mu(1) * 0.98 ;
    cache_.localization_mem->state[3] = mu(3) * 0.98 ;
    cache_.localization_mem->covariance = cov * 10000;
  }
}
