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

  mergeblob = new MergeBlob(getSegImg(), 320, 240, 4, 2, 3);

  detectBall(getSegImg(), mergeblob);
  if(camera_ == Camera::TOP)
  {
    beacon_detector_->findBeacons(getSegImg(), mergeblob);
    detectGoal(getSegImg(), mergeblob);
  }

  // WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  // // printf("Found %d blobs\n", mergeblob.get_blob_number());
  // for(int i = 0; i < mergeblob->get_blob_number(); i++)
  // {
  //   MergeBlob::Blob* blob = &mergeblob->blob[i];
  //   if(blob->color == c_ORANGE)
  //   {
  //     // printf("Found orange blob\n");
  //     // mergeblob.DisplayBlob(i);
  //     findBall(blob);

  //     // for( int j = 1 ; j <= blob->pixel_index_x[0] ; ++j)
  //     // {
  //     //   getSegImg()[320*blob->pixel_index_y[j] + blob->pixel_index_x[j]] = c_UNDEFINED;
  //     // }

  //     // Position p = cmatrix_.getWorldPosition(blob->centroid_x, blob->centroid_y);
  //     // ball->visionBearing = cmatrix_.bearing(p);
  //     // ball->visionElevation = cmatrix_.elevation(p);
  //     // ball->visionDistance = cmatrix_.groundDistance(p);
  //     // // printf("Found orange blob at img_x=%d,img_y=%d, raw_dist=%g , num_pixel=%d\n", blob->centroid_x, blob->centroid_y, ball->visionDistance, blob->pixel_index_x[0]);

  //     // getSegImg()[320*blob->centroid_y + blob->centroid_x] = c_FIELD_GREEN;
  //     //  getSegImg()[320*blob->boundingbox_vertex_y + blob->boundingbox_vertex_x] = c_BLUE;
  //     //  getSegImg()[320*blob->boundingbox_vertex_y + blob->boundingbox_vertex_x + blob->boundingbox_length - 1] = c_YELLOW;
  //     //  getSegImg()[320*(blob->boundingbox_vertex_y + blob->boundingbox_height -1) + blob->boundingbox_vertex_x] = c_BLUE;
  //     //  getSegImg()[320*(blob->boundingbox_vertex_y + blob->boundingbox_height -1) + blob->boundingbox_vertex_x + blob->boundingbox_length - 1] = c_BLUE;
  //   }
  // }

  delete mergeblob;
}

void ImageProcessor::detectGoal(unsigned char* img, MergeBlob* mb)
{
  WorldObject* goal = &vblocks_.world_object->objects_[WO_OPP_GOAL];

  unsigned int min_blob_size = 8000;
  for(int i = 0; i < mb->get_blob_number(); i++)
  {
    unsigned int size = mb->blob[i].boundingbox_length * mb->blob[i].boundingbox_height;
    double ar = (double) mb->blob[i].boundingbox_length / (double) mb->blob[i].boundingbox_height;
    if(size > min_blob_size && mb->blob[i].color == c_BLUE && ar > 1.25 && ar < 4.0)
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
      goal->imageCenterX = x;
      goal->imageCenterY = y;
      float centroid_height = 200.0;
      Position p = cmatrix_.getWorldPosition(x, y, centroid_height);
      goal->visionBearing = cmatrix_.bearing(p);
      goal->visionElevation = cmatrix_.elevation(p);
      goal->fromTopCamera = true;
      goal->visionDistance = cmatrix_.groundDistance(p);
      goal->seen = true;
    }
  }
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
      // printf("found at %d,%d", x, y);
    }
  }
}

bool ImageProcessor::findBall(MergeBlob::Blob* blob)
//bool ImageProcessor::findBall(Point2d& point)
{
  //int number_data_sampled = 20;
  //int *distance_sampled = new int[number_data_sampled];
  //int *long_side_sampled = new int[number_data_sampled];

  int box_height = blob->boundingbox_height;
  int box_length = blob->boundingbox_length;
  int long_side = (box_length > box_height)? box_length : box_height;
  int ball_pixels = blob->pixel_index_x[0];

  

  int real_centroid_x;
  int real_centroid_y;
  int real_radius;

  double box_ratio = (double)box_length / (double)box_height;
  double pixel_density = ball_pixels / ((box_height + 3)*(box_length + 1) / 8.0);

  printf("box_length = %d , box_height = %d , pixel_density = %f \n", box_length , box_height ,  pixel_density );

  //for(int i = 0 ; i < box_length; ++i)

  //drawLine(img, blob->boundingbox_vertex_x, blob->boundingbox_vertex_y, blob->boundingbox_vertex_x + box_length, blob->boundingbox_vertex_y, c_UNDEFINED);
      //drawLine(img, bx_min, by_max, bx_max, by_max, c_UNDEFINED);
      //drawLine(img, bx_min, by_min, bx_min, by_max, c_UNDEFINED);
      //drawLine(img, bx_max, by_min, bx_max, by_max, c_UNDEFINED);



  if( pixel_density < 0.65 ) 
  {
    printf("not a ball  box_ratio = %f\n" , box_ratio);
    return false;
  }
  else if( box_ratio >  1.5 )
  {
    printf("a rectangle\n");
    return false;
  }

   

  Position p = cmatrix_.getWorldPosition(blob->centroid_x, blob->centroid_y);
  double distance = cmatrix_.groundDistance(p);

  //int theoretical_long_side_pixels = (10000/distance);
  //if(long_side < 0.67*theoretical_long_side_pixels || long_side > 1.33*theoretical_long_side_pixels) return false;

  if(abs(box_length - box_height) < 6)
  {
      real_centroid_x = blob->boundingbox_vertex_x + (box_length/2);
      real_centroid_y = blob->boundingbox_vertex_y + (box_height/2);
      real_radius = (box_length + box_height) / 4;

      printf("A ball real_centroid_x=%d,real_centroid_y=%d, real_radius=%d \n", real_centroid_x, real_centroid_y, real_radius);
      getSegImg()[320*real_centroid_y + real_centroid_x] = c_UNDEFINED;
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



  // double ball_cx = 0.0;
  // double ball_cy = 0.0;
  // double num_ball_hits = 0.0;
  // for(unsigned int x = 0; x < 320; x++)
  // {
  //   for(unsigned int y = 0; y < 240; y++)
  //   {
  //     unsigned int idx = 320 * y + x;
  //     unsigned char color = getSegImg()[idx];

  //     if(color == c_ORANGE) //is ball colored
  //     {
  //       ball_cx += x;
  //       ball_cy += y;
  //       num_ball_hits += 1.0;
  //     }
  //   }
  // }

  // if(num_ball_hits == 0.0)
  // {
  //   point.x = point.y = 0;
  //   //printf("no ball hits\n");
  //   return false;
  // }
  // else
  // {
  //   point.x = (int) ball_cx / num_ball_hits;
  //   point.y = (int) ball_cy / num_ball_hits;
  //   //printf("ball at x=%d, y=%d\n", point.x, point.y);
  //   return true;
  // }
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
