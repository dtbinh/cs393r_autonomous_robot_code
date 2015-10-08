#include <localization/LocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
//#include <Eigen/Dense>

KF::StateVector LocalizationModule::g(KF::StateVector x, KF::ControlVector u)
{
  return A * x + B * u;
}

KF::MeasurementVector LocalizationModule::h(KF::StateVector x)
{
  return C * x;
}

KF::StateJacobianMatrix LocalizationModule::G(KF::StateVector x, KF::ControlVector u)
{
  return A;
}

KF::MeasurementJacobianMatrix LocalizationModule::H(KF::StateVector x)
{
  return C;
}

// Boilerplate
LocalizationModule::LocalizationModule() : tlogger_(textlogger) {
  // KF::StateTransitionMatrix A_matrix = KF::StateTransitionMatrix::Identity();
  // KF::InputMatrix B_matrix = KF::InputMatrix::Zero();
  // KF::OutputMatrix C_matrix = KF::OutputMatrix::Identity();
  KF::StateCovarianceMatrix R_matrix = KF::StateCovarianceMatrix::Zero();
  KF::MeasurementCovarianceMatrix Q_matrix = KF::MeasurementCovarianceMatrix::Identity();

  //mu_past_0 << 0 , 0 , 0 , 0;
  //mu_past_1 << 0 , 0 , 0 , 0;

  double dt = 1.0/30.0;
  double xvf = 0.99;
  double yvf = 0.99;
  A << 1.0 , dt  , 0   , 0   ,
              0   , 1.0 , 0  ,  dt  ,
              0   , 0   , xvf , 0   ,
              0   , 0   , 0   , yvf ;

  B << 0, 0;
  C << 1 , 0 , 0 , 0 ,
              0 , 1 , 0 , 0 ,
              0 , 0 , 1 , 0 ,
              0 , 0 , 0 , 1 ;

  double x_var = 0.5 ;
  double xv_var = 1;
  double yv_var = 1;
  double y_var = 0.5 ;

  R_matrix <<   x_var ,   0    ,   0    ,   0    ,
                  0   ,  y_var ,   0    ,   0    ,
                  0   ,   0    , xv_var ,   0    ,
                  0   ,   0    ,   0    , yv_var ;

  double zx_var = 0.5;
  double zy_var = 0.5;
  double zxv_var = 20;
  double zyv_var = 20;
  Q_matrix <<   zx_var  ,   0     ,   0    ,   0     ,
                  0     , zy_var  ,   0    ,   0     ,
                  0     ,   0     , zxv_var,   0     ,
                  0     ,   0     ,   0    , zyv_var ;

  // ball_filter = new KF(A_matrix, B_matrix, C_matrix, R_matrix, Q_matrix);
  ball_filter = new KF(boost::bind(&LocalizationModule::g, this, _1, _2), boost::bind(&LocalizationModule::h, this, _1), boost::bind(&LocalizationModule::G, this, _1, _2), boost::bind(&LocalizationModule::H, this, _1), R_matrix, Q_matrix);
  first = true;
  unseen_count = 0;
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

  KF::StateVector estimated_state;
  KF::MeasurementVector measurement;
  KF::ControlVector control;
  control << 0.0;

  //TODO: modify this block to use your Kalman filter implementation
  if(ball.seen) {
    // Compute the relative position of the ball from vision readings
    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

    // Compute the global position of the ball based on our assumed position and orientation
    auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

    // Update the ball in the WorldObject block so that it can be accessed in python

    if(first || unseen_count > 30)
    {
      ball_filter->update_mu(0 , globalBall.x );
      ball_filter->update_mu(1 , globalBall.y );
      ball_filter->update_mu(2 , 0 );
      ball_filter->update_mu(3 , 0 );
      first = false;
    }

    unseen_count = 0;
    
    KF :: StateVector mu = ball_filter->get_mu();
    double measured_vx = (globalBall.x - mu(0)) * 30.0;
    double measured_vy = (globalBall.y - mu(1)) * 30.0;

    // double vel_thresh = 300.0;
    // if(fabs(measured_vy) > vel_thresh || fabs(measured_vx) > vel_thresh) //bad ball!
    // {
    //   std::cerr << "BAD BALL vel x:" << measured_vx << " y: " << measured_vy << std::endl;
    //   measurement = mu;
    // }
    // else
    // {
    measurement << globalBall.x , globalBall.y , measured_vx , measured_vy ;
    // }

    estimated_state = ball_filter -> process(measurement, control);

    ball.loc.x = estimated_state(0);
    ball.loc.y = estimated_state(1);
    ball.distance = ball.loc.getDistanceTo(self.loc);
    ball.bearing = self.loc.getBearingTo(ball.loc,self.orientation);
    ball.absVel.x = estimated_state(2);
    ball.absVel.y = estimated_state(3);

    //printf("x = %f\t, xv = %f\t, y = %f\t, yv = %f , distance = %f \n" , ball.loc.x , measured_vx , ball.loc.y , measured_vy , ball.distance);

    Eigen:: Matrix< float , 4 , 4 > cov ;
    for(int i = 0 ; i < 4*4 ; ++i) cov(i) = ball_filter -> get_sigma_value(i);

    cache_.localization_mem->state[0] = estimated_state(0);
    cache_.localization_mem->state[1] = estimated_state(1);
    cache_.localization_mem->state[2] = estimated_state(2);
    cache_.localization_mem->state[3] = estimated_state(3);
    cache_.localization_mem->covariance = cov * 10000;
  } 
  //TODO: How do we handle not seeing the ball?
  else {
    unseen_count++;
    //estimated_state = ball_filter -> process(ball_filter->get_mu(), control);
    // ball.distance = 10000.0f;

    //ball.distance = 10000.0f;
    //ball.bearing = 0.0f;
    KF :: StateVector mu = ball_filter->get_mu();
    ball_filter->update_mu(0 , mu(0));
    ball_filter->update_mu(1 , mu(1));
    ball_filter->update_mu(2 , 0 );
    ball_filter->update_mu(3 , 0 );

    KF :: StateVector mu2 = ball_filter->get_mu();

    ball.loc.x = mu2(0);
    ball.loc.y = mu2(1);
    ball.distance = ball.loc.getDistanceTo(self.loc);
    ball.bearing = self.loc.getBearingTo(ball.loc,self.orientation);
    ball.absVel.x = mu2(2);
    ball.absVel.y = mu2(3);

    Eigen:: Matrix< float , 4 , 4 > cov ;
    for(int i = 0 ; i < 4*4 ; ++i) cov(i) = ball_filter -> get_sigma_value(i);

    cache_.localization_mem->state[0] = mu2(0);
    cache_.localization_mem->state[1] = mu2(1);
    cache_.localization_mem->state[2] = mu2(2);
    cache_.localization_mem->state[3] = mu2(3);
    cache_.localization_mem->covariance = cov * 10000;
    


    // ball.bearing = 0.0f;

    // ball.loc.x = estimated_state(0);
    // ball.loc.y = estimated_state(1);
    // ball.distance = ball.loc.getDistanceTo(self.loc);
    // //ball.bearing = self.loc.getBearingTo(ball.loc,self.orientation);
    // ball.absVel.x = 0;
    // ball.absVel.y = 0;

    // Eigen:: Matrix< float , 4 , 4 > cov ;
    // for(int i = 0 ; i < 4*4 ; ++i) cov(i) = ball_filter -> get_sigma_value(i);

    // cache_.localization_mem->state[0] = estimated_state(0);
    // cache_.localization_mem->state[1] = estimated_state(1);
    // cache_.localization_mem->state[2] = 0;
    // cache_.localization_mem->state[3] = 0;
    // cache_.localization_mem->covariance = cov * 10000;
  }
}

// void LocalizationModule::processFrame() {
//   auto& ball = cache_.world_object->objects_[WO_BALL];
//   auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

//   // Retrieve the robot's current location from localization memory
//   // and store it back into world objects
//   auto sloc = cache_.localization_mem->player;
//   self.loc = sloc;
    
//   //TODO: modify this block to use your Kalman filter implementation

//   KF::StateVector estimated_state;
//   KF::MeasurementVector measurement;
//   KF::ControlVector control;
//   control << 0.0;

//   if(ball.seen) {
//     auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);
//     auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

//     ball.loc = globalBall;
//     ball.distance = ball.visionDistance;
//     ball.bearing = ball.visionBearing;

//     //printf("bearing = %f \n" , ball.bearing);

//     //double relative_x = ball.distance * cos( ball.bearing );
//     //double relative_y = ball.distance * sin( ball.bearing );
//     KF :: StateVector mu = ball_filter->get_mu();
//     double measured_vx = (ball.loc.x - mu(0)) * 30.0;
//     double measured_vy = (ball.loc.y - mu(1)) * 30.0;


//     //mu_past_1 = mu_past_0;
//     //mu_past_0 = mu;

//     measurement << ball.loc.x , ball.loc.y , measured_vx , measured_vy ;
//     estimated_state = ball_filter -> process(measurement, control);

//     printf("x = %f , xv = %f , y = %f , yv = %f  \n" , ball.loc.x , measured_vx , ball.loc.y , measured_vy);

//     Eigen:: Matrix< float , 4 , 4 > cov ;
//     for(int i = 0 ; i < 4*4 ; ++i) cov(i) = ball_filter -> get_sigma_value(i);

//     ball.absVel.x = abs(mu(2));
//     ball.absVel.y = abs(mu(3));

//     // Update the localization memory objects with localization calculations
//     // so that they are drawn in the World window()
//     cache_.localization_mem->state[0] = estimated_state(0) ;
//     cache_.localization_mem->state[1] = estimated_state(1) ;
//     cache_.localization_mem->state[2] = estimated_state(2) ;
//     cache_.localization_mem->state[3] = estimated_state(3) ;
//     cache_.localization_mem->covariance = cov * 10000;
//   } 
//   //TODO: How do we handle not seeing the ball?
//   else {
//     //ball.distance = 10000.0f;
//     //ball.bearing = 0.0f;
//     KF :: StateVector mu = ball_filter->get_mu();

//     ball_filter->update_mu(0 , mu(0) + mu(2)/30.0 );
//     ball_filter->update_mu(1 , mu(1) + mu(3)/30.0 );
//     ball_filter->update_mu(2 , mu(2) * 0.98 );
//     ball_filter->update_mu(3 , mu(3) * 0.98 );

//     Eigen:: Matrix< float , 4 , 4 > cov ;
//     for(int i = 0 ; i < 4*4 ; ++i) cov(i) = ball_filter -> get_sigma_value(i);

//     cache_.localization_mem->state[0] = mu(0) + mu(2)/30.0 ;
//     cache_.localization_mem->state[1] = mu(1) + mu(3)/30.0 ;
//     cache_.localization_mem->state[2] = mu(2) * 0.98 ;
//     cache_.localization_mem->state[3] = mu(3) * 0.98 ;
//     cache_.localization_mem->covariance = cov * 10000;
//   }
// }
