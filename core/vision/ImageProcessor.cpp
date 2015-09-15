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

  //beacon_detector_->findBeacons(getSegImg());

  mergeblob = new MergeBlob(getSegImg(), 320, 240, 4, 2, 10);

  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  // printf("Found %d blobs\n", mergeblob.get_blob_number());
  for(int i = 0; i < mergeblob->get_blob_number(); i++)
  {
    MergeBlob::Blob* blob = &mergeblob->blob[i];
    if(blob->color == c_ORANGE)
    {
      // printf("Found orange blob\n");
      // mergeblob.DisplayBlob(i);

      Position p = cmatrix_.getWorldPosition(blob->centroid_x, blob->centroid_y);
      ball->visionBearing = cmatrix_.bearing(p);
      ball->visionElevation = cmatrix_.elevation(p);
      ball->visionDistance = cmatrix_.groundDistance(p);
      printf("Found orange blob at img_x=%d,img_y=%d, raw_dist=%g , num_pixel=%d\n", blob->centroid_x, blob->centroid_y, ball->visionDistance, blob->pixel_index_x[0]);

      getSegImg()[320*blob->centroid_y + blob->centroid_x] = c_FIELD_GREEN;
      getSegImg()[320*blob->boundingbox_vertex_y + blob->boundingbox_vertex_x] = c_BLUE;
      getSegImg()[320*blob->boundingbox_vertex_y + blob->boundingbox_vertex_x + blob->boundingbox_length - 1] = c_YELLOW;
      getSegImg()[320*(blob->boundingbox_vertex_y + blob->boundingbox_height -1) + blob->boundingbox_vertex_x] = c_BLUE;
      getSegImg()[320*(blob->boundingbox_vertex_y + blob->boundingbox_height -1) + blob->boundingbox_vertex_x + blob->boundingbox_length - 1] = c_BLUE;
    }
  }

  // detectBall();
  // detectGoal();
  // findBeacons();

  delete mergeblob;
}

void ImageProcessor::detectGoal()
{
  Point2d point;
  if(!findGoal(point))
    return; // function defined elsewhere that fills in point.x, point.y by reference
  WorldObject* goal = &vblocks_.world_object->objects_[WO_OPP_GOAL];

  goal->imageCenterX = point.x;
  goal->imageCenterY = point.y;

  Position p = cmatrix_.getWorldPosition(point.x, point.y);
  goal->visionBearing = cmatrix_.bearing(p);
  goal->visionElevation = cmatrix_.elevation(p);
  goal->visionDistance = cmatrix_.groundDistance(p);

  goal->seen = true;
}

bool ImageProcessor::findGoal(Point2d& point)
{
  double ball_cx = 0.0;
  double ball_cy = 0.0;
  double num_ball_hits = 0.0;
  double min_goal_size = 500;
  for(unsigned int x = 0; x < 320; x++)
  {
    for(unsigned int y = 0; y < 240; y++)
    {
      unsigned int idx = 320 * y + x;
      unsigned char color = getSegImg()[idx];

      if(color == c_BLUE) //is goal colored
      {
        ball_cx += x;
        ball_cy += y;
        num_ball_hits += 1.0;
      }
    }
  }

  if(num_ball_hits < min_goal_size)
  {
    point.x = point.y = 0;
    //printf("no goal hits\n");
    return false;
  }
  else
  {
    point.x = (int) ball_cx / num_ball_hits;
    point.y = (int) ball_cy / num_ball_hits;
    //printf("goal at x=%d, y=%d\n", point.x, point.y);
    return true;
  }
}

void ImageProcessor::detectBall()
{
  Point2d point;
  if(!findBall(point))
    return; // function defined elsewhere that fills in point.x, point.y by reference
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  ball->imageCenterX = point.x;
  ball->imageCenterY = point.y;

  Position p = cmatrix_.getWorldPosition(point.x, point.y);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  ball->seen = true;
}

bool ImageProcessor::findBall(Point2d& point)
{
  double ball_cx = 0.0;
  double ball_cy = 0.0;
  double num_ball_hits = 0.0;
  for(unsigned int x = 0; x < 320; x++)
  {
    for(unsigned int y = 0; y < 240; y++)
    {
      unsigned int idx = 320 * y + x;
      unsigned char color = getSegImg()[idx];

      if(color == c_ORANGE) //is ball colored
      {
        ball_cx += x;
        ball_cy += y;
        num_ball_hits += 1.0;
      }
    }
  }

  if(num_ball_hits == 0.0)
  {
    point.x = point.y = 0;
    //printf("no ball hits\n");
    return false;
  }
  else
  {
    point.x = (int) ball_cx / num_ball_hits;
    point.y = (int) ball_cy / num_ball_hits;
    //printf("ball at x=%d, y=%d\n", point.x, point.y);
    return true;
  }
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
