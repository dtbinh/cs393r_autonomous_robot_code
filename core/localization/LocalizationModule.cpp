#include <localization/LocalizationModule.h>
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

  double x_var = 1 ;
  double xv_var = 5;
  double yv_var = 5;
  double y_var = 1 ;

  R_matrix <<   x_var ,   0    ,   0    ,   0    ,
                  0   ,  y_var ,   0    ,   0    ,
                  0   ,   0    , xv_var ,   0    ,
                  0   ,   0    ,   0    , yv_var ;

  double zx_var = 0.5;
  double zy_var = 0.5;
  double zxv_var = 1000;
  double zyv_var = 1000;
  Q_matrix <<   zx_var  ,   0     ,   0    ,   0     ,
                  0     , zy_var  ,   0    ,   0     ,
                  0     ,   0     , zxv_var,   0     ,
                  0     ,   0     ,   0    , zyv_var ;

  // ball_filter = new KF(A_matrix, B_matrix, C_matrix, R_matrix, Q_matrix);
  ball_filter = new KF(boost::bind(&LocalizationModule::g, this, _1, _2), boost::bind(&LocalizationModule::h, this, _1), boost::bind(&LocalizationModule::G, this, _1, _2), boost::bind(&LocalizationModule::H, this, _1), R_matrix, Q_matrix);
  first = true;
  unseen_count = 0;

  createPF();
}

// Boilerplate
LocalizationModule::~LocalizationModule() {
  delete pfilter_;
}

void LocalizationModule::createPF()
{  
  srand(time(NULL));

  RPF::StateTransitionMatrix PF_A;
  RPF::ControlMatrix PF_B;
  RPF::MeasurementCovarianceMatrix PF_Q;
  RPF::WhiteCovarianceVector PF_N;

  RPF::ParticleVector PF_p;


  PF_A << 1 , 0 , 0 ,
          0 , 1 , 0 ,
          0 , 0 , 1 ;

  PF_B << 1 , 0 , 0 , //Should be tuned by the real speed;
          0 , 1 , 0 ,
          0 , 0 , 1 ;

  PF_Q <<   10000 , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
            0     , 0.001 , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
            0     , 0     , 10000 , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
            0     , 0     , 0     , 0.001 , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
            0     , 0     , 0     , 0     , 10000 , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
            0     , 0     , 0     , 0     , 0     , 0.001 , 0     , 0     , 0     , 0     , 0     , 0     ,
            0     , 0     , 0     , 0     , 0     , 0     , 10000 , 0     , 0     , 0     , 0     , 0     ,
            0     , 0     , 0     , 0     , 0     , 0     , 0     , 0.001 , 0     , 0     , 0     , 0     ,
            0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 10000 , 0     , 0     , 0     ,
            0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0.001 , 0     , 0     ,
            0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 10000 , 0     ,
            0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0.001;

  // PF_N << 40     , 0     ,  0    ,
  //         0      , 40    ,  0    ,
  //         0      , 0     , 0.08 ;

  PF_N << 10     , 0      ,   0    ,
          0      , 10     ,   0    ,
          0      , 0      ,  0.02 ;

  NAO_LOCATION << 0,0,0;

  pfilter_ = new RPF(PF_A,PF_B,PF_Q,PF_N);
}

void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_odometry");
  // std::cerr << "SMD cache address is: " << cache_.localization_mem << std::endl;
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.odometry,"vision_odometry");
  // std::cerr << "SMB cache address is: " << cache_.localization_mem << std::endl;
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
  pfilter_->init(self.loc, self.orientation);
}

// Reinitialize from scratch
void LocalizationModule::reInit() {
  // std::cerr << "reInit cache address is: " << cache_.localization_mem << std::endl;
  pfilter_->init(Point2D(-750,0), 0.0f);
  cache_.localization_mem->state = decltype(cache_.localization_mem->state)::Zero();
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity();
  // std::cerr << "reInit end cache address is: " << cache_.localization_mem << std::endl;
}

void LocalizationModule::moveBall(const Point2D& position) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::movePlayer(const Point2D& position, float orientation) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

double LocalizationModule::getdistance( double x , double y , double bx , double by )
{ 
    return sqrt( (x - bx)*(x - bx) + (y - by)*(y - by) );
}

double LocalizationModule::gettheta( double x , double y , double ori , double bx , double by )
{
    double delta_x = bx - x;
    double delta_y = by - y;
    double beacon_theta = 0 ;

    if(      delta_x > 0 && delta_y >= 0) beacon_theta = atan(delta_y/delta_x);
    else if( delta_x < 0 && delta_y >= 0) beacon_theta = PI + atan(delta_y/delta_x);
    else if( delta_x < 0 && delta_y <  0) beacon_theta = PI + atan(delta_y/delta_x);
    else if( delta_x > 0 && delta_y <  0) beacon_theta = 2*PI + atan(delta_y/delta_x);
    else if( delta_x ==0 && delta_y >  0) beacon_theta = 0.5*PI;
    else if( delta_x ==0 && delta_y <  0) beacon_theta = 1.5*PI;

    double theta = beacon_theta - ori;
    if( delta_x > 0 && delta_y > 0 && ori > PI ) theta += 2*PI;
    else if( delta_x > 0 && delta_y < 0 && ori < PI) theta -= 2*PI;

    return theta;
}

void LocalizationModule::processFrame() {
  // std::cerr << "Initial cache address is: " << cache_.localization_mem << std::endl;
  auto& ball = cache_.world_object->objects_[WO_BALL];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  // Retrieve the robot's current location from localization memory
  // and store it back into world objects
  // auto sloc = cache_.localization_mem->player;
  // self.loc = sloc;

  KF::StateVector estimated_state;
  KF::MeasurementVector measurement;
  KF::ControlVector control;
  control << 0.0;

  // Process the current frame and retrieve our location/orientation estimate
  // from the particle filter

  //std::cerr << "=========================================" << endl;

  RPF::MeasurementVector pf_z;
  RPF::ControlVector pf_u;

  // printf("1.=============================================================\n");

  bool any_beacon_seen = false;
  for(unsigned int i = WO_BEACON_BLUE_YELLOW; i <=WO_BEACON_YELLOW_PINK; i++)
  {
    auto& beacon = cache_.world_object->objects_[i];
    if(beacon.seen)
    {
      any_beacon_seen = true;
      pf_z(2*(i-WO_BEACON_BLUE_YELLOW)) = beacon.visionDistance ;
      pf_z(2*(i-WO_BEACON_BLUE_YELLOW) + 1) = beacon.visionBearing ;

      printf("Saw beacon %d at (x,y)=(%g,%g) || distance = %f , bearing = %f \n", (int) i, beacon.loc.x , beacon.loc.y, beacon.visionDistance, beacon.visionBearing);
      printf("Self(x,y,ori) = (%f,%f,%f)\n" ,  NAO_LOCATION(0) , NAO_LOCATION(1) , NAO_LOCATION(2) );
    }
    else
    {
      pf_z(2*(i-WO_BEACON_BLUE_YELLOW)) = -1;
      pf_z(2*(i-WO_BEACON_BLUE_YELLOW) + 1) = 0;
    }
  } 

  

  const auto& disp = cache_.odometry->displacement;
  pf_u << disp.translation.x, disp.translation.y, disp.rotation;

  //std::cerr << "Control is: " << pf_u.transpose() << std::endl;
  //cout << "Measurement is " << pf_z.transpose() << std::endl;
  // printf("2.=============================================================\n" );
  pfilter_->process(pf_z, pf_u);
  // printf("3.=============================================================\n" );
  NAO_LOCATION = pfilter_->getNAO_LOCATION();
  // printf("4.=============================================================\n" );
  // if(cache_.localization_mem != NULL && any_beacon_seen)
  // {
  //   std::cerr << "Cache address is: " << cache_.localization_mem << std::endl;
  cache_.localization_mem->particles = pfilter_->getParticles();
  // }
  // printf("5.=============================================================\n" );
  self.loc.x = NAO_LOCATION(0);
  self.loc.y = NAO_LOCATION(1);
  self.orientation = NAO_LOCATION(2);
  // self.loc = pfilter_->pose().translation;
  // self.orientation = pfilter_->pose().rotation;

  // printf("Robot is at (x,y,theta)=(%g,%g,%g)\n", NAO_LOCATION(0), NAO_LOCATION(1), NAO_LOCATION(2));

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

    measurement << globalBall.x , globalBall.y , measured_vx , measured_vy ;

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

    // cache_.localization_mem->state[0] = estimated_state(0);
    // cache_.localization_mem->state[1] = estimated_state(1);
    // cache_.localization_mem->state[2] = estimated_state(2);
    // cache_.localization_mem->state[3] = estimated_state(3);
    // cache_.localization_mem->covariance = cov * 10000;
  } 
  //TODO: How do we handle not seeing the ball?
  else {
    unseen_count++;

    //ball.distance = 10000.0f;
    //ball.bearing = 0.0f;
    KF :: StateVector mu = ball_filter->get_mu();

    if( unseen_count > 30 )
    {
      ball_filter->update_mu(0 , mu(0));
      ball_filter->update_mu(1 , mu(1));
      ball_filter->update_mu(2 , 0 );
      ball_filter->update_mu(3 , 0 );
    }
    else
    { 
      ball_filter->update_mu(0 , mu(0) + (mu(2)/30) );
      ball_filter->update_mu(1 , mu(1) + (mu(3)/30) );
      ball_filter->update_mu(2 , mu(2)*0.98);
      ball_filter->update_mu(3 , mu(3)*0.98);
    }

    KF :: StateVector mu2 = ball_filter->get_mu();

    ball.loc.x = mu2(0);
    ball.loc.y = mu2(1);
    ball.distance = ball.loc.getDistanceTo(self.loc);
    ball.bearing = self.loc.getBearingTo(ball.loc,self.orientation);
    ball.absVel.x = mu2(2);
    ball.absVel.y = mu2(3);

    Eigen:: Matrix< float , 4 , 4 > cov ;
    for(int i = 0 ; i < 4*4 ; ++i) cov(i) = ball_filter -> get_sigma_value(i);

    // cache_.localization_mem->state[0] = mu2(0);
    // cache_.localization_mem->state[1] = mu2(1);
    // cache_.localization_mem->state[2] = mu2(2);
    // cache_.localization_mem->state[3] = mu2(3);
    // cache_.localization_mem->covariance = cov * 10000;
    
  }
}
