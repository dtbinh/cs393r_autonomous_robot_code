#include <vision/ImageProcessor.h>
#include <iostream>

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
    vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera), calibration_(NULL)
{
  enableCalibration_ = false;
  classifier_ = new Classifier(vblocks_, vparams_, iparams_, camera_);
  beacon_detector_ = new BeaconDetector(DETECTOR_PASS_ARGS);
}

void ImageProcessor::init(TextLogger* tl)
{
  textlogger = tl;
  vparams_.init();
  classifier_->init(tl);
  beacon_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg()
{
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

unsigned char* ImageProcessor::getSegImg()
{
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable()
{
  return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix()
{
  return cmatrix_;
}

void ImageProcessor::updateTransform()
{
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;

  Pose3D pcamera;
  if(enableCalibration_)
  {
    float joints[NUM_JOINTS], sensors[NUM_SENSORS], dimensions[RobotDimensions::NUM_DIMENSIONS];
    memcpy(joints, vblocks_.joint->values_, NUM_JOINTS * sizeof(float));
    memcpy(sensors, vblocks_.sensor->values_, NUM_SENSORS * sizeof(float));
    memcpy(dimensions, vblocks_.robot_info->dimensions_.values_, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
    Pose3D *rel_parts = vblocks_.body_model->rel_parts_, *abs_parts = vblocks_.body_model->abs_parts_;
    calibration_->applyJoints(joints);
    calibration_->applySensors(sensors);
    calibration_->applyDimensions(dimensions);
    ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
    Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
    ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
    ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
    cmatrix_.setCalibration(*calibration_);
    pcamera = abs_parts[camera];
  }
  else
    pcamera = vblocks_.body_model->abs_parts_[camera];

  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH)
  {
    auto self = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF];
    pcamera.translation.z += self.height;
  }

  cmatrix_.updateCameraPose(pcamera);
}

bool ImageProcessor::isRawImageLoaded()
{
  if(camera_ == Camera::TOP)
    return vblocks_.image->img_top_;
  return vblocks_.image->img_bottom_;
}

int ImageProcessor::getImageHeight()
{
  return iparams_.height;
}

int ImageProcessor::getImageWidth()
{
  return iparams_.width;
}

double ImageProcessor::getCurrentTime()
{
  return vblocks_.frame_info->seconds_since_start;
}

void ImageProcessor::setCalibration(RobotCalibration calibration)
{
  if(calibration_)
    delete calibration_;
  calibration_ = new RobotCalibration(calibration);
}

void ImageProcessor::processFrame()
{
  int orange_blob_counter = 0;

  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM) return;
  visionLog(30, "Process Frame camera %i", camera_);

  updateTransform();
  
  // Horizon calculation
  visionLog(30, "Calculating horizon line");
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 30000);
  vblocks_.robot_vision->horizon = horizon;
  visionLog(30, "Classifying Image", camera_);
  if(!classifier_->classifyImage(color_table_))
    return;

  mergeblob = new MergeBlob(getSegImg(), 320, 240, 4, 2, 4);

  //detectBall(getSegImg(), mergeblob);
  if(camera_ == Camera::TOP)
  {
    beacon_detector_->findBeacons(getSegImg(), mergeblob);
    // detectGoal(getSegImg(), mergeblob);
    detectGoal(getSegImg(), mergeblob);
  }

  // printf("Found %d blobs\n", mergeblob.get_blob_number());

  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];
  if(!ball->seen)
  {
    for(int i = 0; i < mergeblob->get_blob_number(); i++)
    {
      MergeBlob::Blob* blob = &mergeblob->blob[i];
      if(blob->color == c_ORANGE)
      {
        // mergeblob.DisplayBlob(i);
        // findBall(blob);
        findBall(blob);
        orange_blob_counter++;
      }
      if(orange_blob_counter == 0) vblocks_.robot_state->ball_seen = false;
    }
  }

  delete mergeblob;

  WorldObject* goal = &vblocks_.world_object->objects_[WO_OPP_GOAL];
  if(camera_ == Camera::TOP && goal->seen)
  {
    //recalc blobs after we isolate the goal in the segmented image
    mergeblob = new MergeBlob(getSegImg(), 320, 240, 4, 2, 4);
    detectEnemy(getSegImg(), mergeblob);
    delete mergeblob;
  }
}

void ImageProcessor::detectEnemy(unsigned char* img, MergeBlob* mb)
{
  WorldObject* goalie = &vblocks_.world_object->objects_[WO_OPPONENT1];
  
  //pad
  int padding = 3;
  goal_x_min -= padding;
  goal_x_max += padding;
  goal_y_min -= padding;
  goal_y_max += padding;

  int min_enemy_blob_size = 1500;
  for(int i = 0; i < mb->get_blob_number(); i++)
  {
    int size = mb->blob[i].boundingbox_length * mb->blob[i].boundingbox_height;
    double ar = (double) mb->blob[i].boundingbox_length / (double) mb->blob[i].boundingbox_height;
    if(size > min_enemy_blob_size && mb->blob[i].color == c_WHITE)// && ar < 1.2)// && ar > 1.2 && ar < 4)
    {
      int bx_min = mb->blob[i].boundingbox_vertex_x;
      int bx_max = mb->blob[i].boundingbox_vertex_x + mb->blob[i].boundingbox_length;
      int by_min = mb->blob[i].boundingbox_vertex_y;
      int by_max = mb->blob[i].boundingbox_vertex_y + mb->blob[i].boundingbox_height;

      bool x_min_in = goal_x_min < bx_min && bx_min < goal_x_max;
      bool x_max_in = goal_x_min < bx_max && bx_max < goal_x_max;
      bool y_min_in = goal_y_min < by_min && by_min < goal_y_max;
      bool y_max_in = goal_y_min < by_max && by_max < goal_y_max;
      int num_edges_in = (x_min_in?1:0)+(x_max_in?1:0)+(y_min_in?1:0)+(y_max_in?1:0);
      
      bool y_min_above = by_min < goal_y_min;
      bool y_max_below = by_max > goal_y_max;

      if(num_edges_in > 3 || ((x_max_in || x_min_in) && y_min_above && y_max_below)) //surrounded by blue goal pixels
      {
        int area = (bx_max - bx_min)*(by_max - by_min);
        if(goalie->seen && goalie->radius > area)
        {
          continue;
        }

        int x = (bx_max + bx_min) / 2.0;
        int y = (by_max + by_min) / 2.0;
        goalie->imageCenterX = x;
        goalie->imageCenterY = y;
        float centroid_height = 210.0;
        Position p = cmatrix_.getWorldPosition(x, y, centroid_height);
        goalie->visionBearing = cmatrix_.bearing(p);
        goalie->visionElevation = cmatrix_.elevation(p);
        goalie->fromTopCamera = true;
        goalie->visionDistance = cmatrix_.groundDistance(p);
        goalie->radius = area; //actually area
        goalie->seen = true;

        drawLine(img, bx_min, by_min, bx_max, by_min, c_PINK);
        drawLine(img, bx_min, by_max, bx_max, by_max, c_PINK);
        drawLine(img, bx_min, by_min, bx_min, by_max, c_PINK);
        drawLine(img, bx_max, by_min, bx_max, by_max, c_PINK);
        drawLine(img, bx_min, by_min, bx_max, by_max, c_PINK);
        drawLine(img, bx_min, by_max, bx_max, by_min, c_PINK);
      }
      // else
      // {
        // drawLine(img, bx_min, by_min, bx_max, by_min, c_UNDEFINED);
        // drawLine(img, bx_min, by_max, bx_max, by_max, c_UNDEFINED);
        // drawLine(img, bx_min, by_min, bx_min, by_max, c_UNDEFINED);
        // drawLine(img, bx_max, by_min, bx_max, by_max, c_UNDEFINED);
        // drawLine(img, bx_min, by_min, bx_max, by_max, c_UNDEFINED);
        // drawLine(img, bx_min, by_max, bx_max, by_min, c_UNDEFINED);
      // }
    }
  }


}

void ImageProcessor::detectGoal(unsigned char* img, MergeBlob* mb)
{
  WorldObject* goal = &vblocks_.world_object->objects_[WO_OPP_GOAL];
  std::vector<MergeBlob::Blob*> blue_blobs;
  int min_goal_blob_size = 1000;
  for(int i = 0; i < mb->get_blob_number(); i++)
  {
    int size = mb->blob[i].boundingbox_length * mb->blob[i].boundingbox_height;
    if(size > min_goal_blob_size && mb->blob[i].color == c_BLUE)
    {
      blue_blobs.push_back(&mb->blob[i]);

      int bx_min = mb->blob[i].boundingbox_vertex_x;
      int bx_max = mb->blob[i].boundingbox_vertex_x + mb->blob[i].boundingbox_length;
      int by_min = mb->blob[i].boundingbox_vertex_y;
      int by_max = mb->blob[i].boundingbox_vertex_y + mb->blob[i].boundingbox_height;
    }
  }

  if(blue_blobs.size() == 0)
  {
    // std::cerr << "Nothing blue!" << std::endl;
    return;
  }

  goal_x_min = std::numeric_limits<int>::max();
  goal_x_max = -std::numeric_limits<int>::max();
  goal_y_min = std::numeric_limits<int>::max();
  goal_y_max = -std::numeric_limits<int>::max();
  for(auto b : blue_blobs)
  {
    goal_x_min = std::min(goal_x_min, b->boundingbox_vertex_x);
    goal_x_max = std::max(goal_x_max, b->boundingbox_vertex_x + b->boundingbox_length);
    goal_y_min = std::min(goal_y_min, b->boundingbox_vertex_y);
    goal_y_max = std::max(goal_y_max, b->boundingbox_vertex_y + b->boundingbox_height);
  }

  double min_goal_size = 4500;
  double goal_size = (goal_y_max-goal_y_min)*(goal_x_max-goal_x_min);
  double goal_ar = (double) (goal_x_max - goal_x_min) / (double) (goal_y_max - goal_y_min);
  if(goal_size <= min_goal_size || goal_ar <= 1.2 || goal_ar >= 4.0) //not a valid goal
  {
    // std::cerr << "Goal rejected!" << std::endl;
    return;
  }

  int x = (goal_x_max + goal_x_min)/2.0;
  int y = (goal_y_max + goal_y_min)/2.0;
  goal->imageCenterX = x;
  goal->imageCenterY = y;
  float centroid_height = 210.0;
  Position p = cmatrix_.getWorldPosition(x, y, centroid_height);
  goal->visionBearing = cmatrix_.bearing(p);
  goal->visionElevation = cmatrix_.elevation(p);
  goal->fromTopCamera = true;
  goal->visionDistance = cmatrix_.groundDistance(p);
  goal->radius = (goal_x_max - goal_x_min);
  goal->seen = true;

  vblocks_.robot_state->goal_seen = goal->seen;
  vblocks_.robot_state->goal_visionDistance = goal->visionDistance;
  vblocks_.robot_state->goal_visionBearing = goal->visionBearing;


  for(unsigned int i = 0; i < 5; i++) //border thickness
  {
    drawLine(img, goal_x_min-i, goal_y_min-i, goal_x_max+i, goal_y_min-i, c_ROBOT_WHITE);
    drawLine(img, goal_x_min-i, goal_y_max+i, goal_x_max+i, goal_y_max+i, c_ROBOT_WHITE);
    drawLine(img, goal_x_min-i, goal_y_min-i, goal_x_min-i, goal_y_max+i, c_ROBOT_WHITE);
    drawLine(img, goal_x_max+i, goal_y_min-i, goal_x_max+i, goal_y_max+i, c_ROBOT_WHITE);
  }
}


bool ImageProcessor::findBall(MergeBlob::Blob* blob)
{
  int box_height = blob->boundingbox_height;
  int box_length = blob->boundingbox_length;
  int long_side = (box_length > box_height)? box_length : box_height;
  int short_side = (box_length <= box_height)? box_length : box_height;
  int ball_pixels = blob->pixel_index_x[0];

  

  int real_centroid_x;
  int real_centroid_y;
  int real_radius;

  double box_ratio = (double)long_side / (double)short_side;
  double pixel_density = (double)ball_pixels / ((box_height + 3)*(box_length + 1) / 8.0);

  //printf("ball_pixels = %d , box_length = %d , box_height = %d , pixel_density = %f \n", ball_pixels , box_length , box_height ,  pixel_density );

  if(camera_ == Camera::TOP)
  {
    if( box_height > 40 || box_length > 40)
    {
      //printf("TOP camera: too big!");
      return false;
    }
    else if( box_height < 3 || box_length < 3)
    {
      return false;
    }
    else if( blob->centroid_y < 45)
    {
      //printf("Ball to high\n");
      return false;
    }
    vblocks_.robot_state->ball_seen = false;
  }
  else if(camera_ == Camera::BOTTOM)
  {
    if( box_height > 65 || box_length > 65)
    {
      //printf("BOTTOM camera: too big!");
      vblocks_.robot_state->ball_seen = false;
      return false;
    }
    if( box_height < 10 || box_length < 10)
    {
      vblocks_.robot_state->ball_seen = false;
      return false;
    }
  }


  if( pixel_density < 0.62 || pixel_density > 0.88) 
  {
    //printf("not a ball  pixel_density = %f\n" , pixel_density);
    vblocks_.robot_state->ball_seen = false;
    return false;
  }
  else if( box_ratio >  1.5 )
  {
    //printf("a rectangle\n");
    vblocks_.robot_state->ball_seen = false;
    return false;
  }

  for( int j = 1 ; j <= blob->pixel_index_x[0] ; ++j) getSegImg()[320*blob->pixel_index_y[j] + blob->pixel_index_x[j]] = c_UNDEFINED;

  //Position p = cmatrix_.getWorldPosition(blob->centroid_x, blob->centroid_y);
  //double distance = cmatrix_.groundDistance(p);

  //int theoretical_long_side_pixels = (10000/distance);
  //if(long_side < 0.67*theoretical_long_side_pixels || long_side > 1.33*theoretical_long_side_pixels) return false;

  if(abs(box_length - box_height) < 10)
  {
      real_centroid_x = blob->boundingbox_vertex_x + (box_length/2);
      real_centroid_y = blob->boundingbox_vertex_y + (box_height/2);
      real_radius = (box_length + box_height) / 4;

      if(real_radius < 4) 
      {
        vblocks_.robot_state->ball_seen = false;
        return false;
      }
      getSegImg()[320*real_centroid_y + real_centroid_x] = c_UNDEFINED;
      //printf("A ball. real_centroid_x = %d , real_centroid_y = %d \n", real_centroid_x , real_centroid_y );
  }
  else
  {
    // int top_pixel_count = 0;
    // int bottom_pixel_count = 0;
    // int left_pixel_count = 0;
    // int bottom_pixel_count = 0;

    // double top_pixel_x_average = 0;
    // double bottom_pixel_x_average = 0;
    // double left_pixel_y_average = 0;
    // double right_pixel_y_average = 0;
    // int side_pixel_count[4] = {0};
    // int side_pixel_index_average[4] = {0};

    // for( int i = 1 ; i <= blob->pixel_index_x[0] ; ++i)
    // {
    //     if( blob->pixel_index_x[i] == blob->boundingbox_vertex_x) //left
    //     {
    //         ++side_pixel_count[0];
    //         side_pixel_index_average[0] += blob->pixel_index_y[i];
    //     }
    //     else if( blob->pixel_index_x[i] == blob->boundingbox_vertex_x + blob->boundingbox_length - 1)  //right
    //     {
    //         ++side_pixel_count[1];
    //         side_pixel_index_average[1] += blob->pixel_index_y[i];
    //     }

    //     if( blob->pixel_index_y[i] == blob->boundingbox_vertex_y) //top
    //     {
    //         ++side_pixel_count[2];
    //         side_pixel_index_average[2] += blob->pixel_index_x[i];
    //     }
    //     else if( blob->pixel_index_y[i] == blob->boundingbox_vertex_y + blob->boundingbox_height - 1) //bottom
    //     {
    //         ++side_pixel_count[3];
    //         side_pixel_index_average[3] += blob->pixel_index_x[i];
    //     }  
    // }

    // int circle_pixel_index[4][2];
    // circle_pixel_index[0][0] = blob->boundingbox_vertex_x;
    // circle_pixel_index[0][1] = side_pixel_index_average[0]/++side_pixel_count[0];
    // circle_pixel_index[1][0] = blob->boundingbox_vertex_x + blob->boundingbox_length - 1;
    // circle_pixel_index[1][1] = side_pixel_index_average[1]/++side_pixel_count[1];
    // circle_pixel_index[2][0] = side_pixel_index_average[2]/++side_pixel_count[2];;
    // circle_pixel_index[2][1] = blob->boundingbox_vertex_y;
    // circle_pixel_index[3][0] = side_pixel_index_average[3]/++side_pixel_count[3];;
    // circle_pixel_index[3][1] = blob->boundingbox_vertex_y + blob->boundingbox_height - 1;

    // for( int i = 0 ; i < 3 ; ++i)
    // {
    //    if( side_pixel_count[i] > side_pixel_count[i+1])
    //    {
    //       int tmp0;
    //       int tmp1;

    //       tmp0 = circle_pixel_index[i][0];
    //       tmp1 = circle_pixel_index[i][1];
    //       circle_pixel_index[i][0] = circle_pixel_index[i+1][0];
    //       circle_pixel_index[i][1] = circle_pixel_index[i+1][1];
    //       circle_pixel_index[i+1][0] = tmp0;
    //       circle_pixel_index[i+1][1] = tmp1;
    //    }
    // }

    // double x1 = circle_pixel_index[0][0];
    // double y1 = circle_pixel_index[0][1];
    // double x2 = circle_pixel_index[1][0];
    // double y2 = circle_pixel_index[1][1];
    // double x3 = circle_pixel_index[2][0];
    // double y3 = circle_pixel_index[3][1];

    // double ml1 = (y2-y1)/(x2-x1);
    // double ml2 = (y3-y2)/(x3-x2);

    // real_centroid_x = (ml1*ml2*(y1-y3) + (ml2-ml1)*x2 + (ml2*x1-ml1*x3))/(2*(ml2-ml1));
    // real_centroid_y = ((x1-x3) + (ml1-ml2)*y2 + (ml2*y1 - ml1*y3))/(2*(ml2-ml1));
    // real_radius = sqrt((real_centroid_x-x1)*(real_centroid_x-x1) + (real_centroid_y-y1)*(real_centroid_y-y1));
  }

  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];
  int x = real_centroid_x;
  int y = real_centroid_y;
  ball->imageCenterX = x;
  ball->imageCenterY = y;
  Position p = cmatrix_.getWorldPosition(x, y, 20);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->fromTopCamera = (camera_ == Camera::TOP);
  //ball->fromBottomCamera = (camera == Camera :: BOTTOM);
  ball->visionDistance = cmatrix_.groundDistance(p);
  ball->seen = true;
  ball->radius = real_radius;

  vblocks_.robot_state->ball_seen = ball->seen;
  vblocks_.robot_state->ball_visionDistance = ball->visionDistance;
  vblocks_.robot_state->ball_visionBearing = ball->visionBearing;

  return true;

}

int ImageProcessor::getTeamColor()
{
  return vblocks_.robot_state->team_;
}

void ImageProcessor::SetColorTable(unsigned char* table)
{
  color_table_ = table;
}

float ImageProcessor::getHeadChange() const
{
  if(vblocks_.joint == NULL)
    return 0;
  return vblocks_.joint->getJointDelta(HeadPan);
}

std::vector<BallCandidate*> ImageProcessor::getBallCandidates()
{
  return std::vector<BallCandidate*>();
}

BallCandidate* ImageProcessor::getBestBallCandidate()
{
  return NULL;
}

void ImageProcessor::enableCalibration(bool value)
{
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded()
{
  return vblocks_.image->loaded_;
}


void ImageProcessor::detectBall(unsigned char* img, MergeBlob* mb)
{
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  for(int i = 0; i < mb->get_blob_number(); i++)
  {
    if(mb->blob[i].color != c_ORANGE)
    {
      continue;
    }

    //make sure blob doesn't contain other blobs
    bool contains_other_blobs = false;
    for(int j = 0; j < mb->get_blob_number(); j++)
    {
      if(mb->blob[i].color == c_PINK)
      {
        //pink sometimes looks like orange
        continue;
      }

      unsigned int orange_y_min = mb->blob[i].boundingbox_vertex_y;
      unsigned int orange_y_max = orange_y_min + mb->blob[i].boundingbox_height;
      unsigned int orange_x_min = mb->blob[i].boundingbox_vertex_x;
      unsigned int orange_x_max = orange_x_min + mb->blob[i].boundingbox_length;
      unsigned int other_y_min = mb->blob[j].boundingbox_vertex_y;
      unsigned int other_y_max = other_y_min + mb->blob[j].boundingbox_height;
      unsigned int other_x_min = mb->blob[j].boundingbox_vertex_x;
      unsigned int other_x_max = other_x_min + mb->blob[j].boundingbox_length;
      if(orange_y_min < other_y_min && orange_y_max > other_y_max && orange_x_min < other_x_min && orange_x_max > other_x_max)
      {
        //other blob is contained inside the orange blob
        contains_other_blobs = true;
        break;
      }
    }
    if(contains_other_blobs)
    {
      continue;
    }

    float centroid_height = 20.0;
    unsigned int size = mb->blob[i].boundingbox_length * mb->blob[i].boundingbox_height;
    double ar = (double) mb->blob[i].boundingbox_length / (double) mb->blob[i].boundingbox_height;
    double dist = cmatrix_.groundDistance(cmatrix_.getWorldPosition(mb->blob[i].centroid_x, mb->blob[i].centroid_y, centroid_height));

    float d1 = 688.865;
    float d2 = 791.81;
    float pixels_at_d1 = 525;
    float pixels_at_d2 = 323;
    unsigned int pixels_at_dist = ((pixels_at_d2-pixels_at_d1) / (d2-d1)) * (dist-d1) + pixels_at_d1;
    unsigned int min_blob_size = 100; //0.5 * pixels_at_dist;
    unsigned int max_blob_size = 400; //2.5 * pixels_at_dist;
    printf("min=%d,max=%d,dist=%g,size=%d,ar=%g\n", min_blob_size, max_blob_size,dist,size,ar);

    if(size > min_blob_size && size < max_blob_size && ar > 0.7 && ar < 1.2)
    {
      unsigned int bx_min = mb->blob[i].boundingbox_vertex_x;
      unsigned int bx_max = mb->blob[i].boundingbox_vertex_x + mb->blob[i].boundingbox_length;
      unsigned int by_min = mb->blob[i].boundingbox_vertex_y;
      unsigned int by_max = mb->blob[i].boundingbox_vertex_y + mb->blob[i].boundingbox_height;

      drawLine(img, bx_min, by_min, bx_max, by_min, c_UNDEFINED);
      drawLine(img, bx_min, by_max, bx_max, by_max, c_UNDEFINED);
      drawLine(img, bx_min, by_min, bx_min, by_max, c_UNDEFINED);
      drawLine(img, bx_max, by_min, bx_max, by_max, c_UNDEFINED);

      unsigned int x = mb->blob[i].centroid_x;
      unsigned int y = mb->blob[i].centroid_y;
      ball->imageCenterX = x;
      ball->imageCenterY = y;
      Position p = cmatrix_.getWorldPosition(x, y, centroid_height);
      ball->visionBearing = cmatrix_.bearing(p);
      ball->visionElevation = cmatrix_.elevation(p);
      ball->fromTopCamera = (camera_ == Camera::TOP);
      ball->visionDistance = cmatrix_.groundDistance(p);
      ball->seen = true;
      //printf("found at goal %d,%d", x, y);
    }
  }
}
