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

  mergeblob = new MergeBlob(getSegImg(), 320, 240, 4, 2, 10);

  detectBall(getSegImg(), mergeblob);
  if(camera_ == Camera::TOP)
  {
    beacon_detector_->findBeacons(getSegImg(), mergeblob);
    detectGoal(getSegImg(), mergeblob);
  }

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
      // printf("Found orange blob at img_x=%d,img_y=%d, raw_dist=%g , num_pixel=%d\n", blob->centroid_x, blob->centroid_y, ball->visionDistance, blob->pixel_index_x[0]);

      getSegImg()[320*blob->centroid_y + blob->centroid_x] = c_FIELD_GREEN;
      getSegImg()[320*blob->boundingbox_vertex_y + blob->boundingbox_vertex_x] = c_BLUE;
      getSegImg()[320*blob->boundingbox_vertex_y + blob->boundingbox_vertex_x + blob->boundingbox_length - 1] = c_YELLOW;
      getSegImg()[320*(blob->boundingbox_vertex_y + blob->boundingbox_height -1) + blob->boundingbox_vertex_x] = c_BLUE;
      getSegImg()[320*(blob->boundingbox_vertex_y + blob->boundingbox_height -1) + blob->boundingbox_vertex_x + blob->boundingbox_length - 1] = c_BLUE;
    }
  }

  delete mergeblob;
}

void ImageProcessor::detectGoal(unsigned char* img, MergeBlob* mb)
{
  WorldObject* goal = &vblocks_.world_object->objects_[WO_OPP_GOAL];

  unsigned int min_blob_size = 800;
  for(int i = 0; i < mb->get_blob_number(); i++)
  {
    unsigned int size = mb->blob[i].boundingbox_length * mb->blob[i].boundingbox_height;
    double ar = (double) mb->blob[i].boundingbox_length / (double) mb->blob[i].boundingbox_height;
    if(size > min_blob_size && mb->blob[i].color == c_BLUE && ar > 1.6 && ar < 3.0)
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

    float d1 = 137.679;
    float d2 = 300;
    float pixels_at_d1 = 378;
    float pixels_at_d2 = 50;
    unsigned int pixels_at_dist = (pixels_at_d2-pixels_at_d1) / (d2-d1) * (dist-d1) + pixels_at_d1;
    unsigned int min_blob_size = 0.9 * pixels_at_dist;
    unsigned int max_blob_size = 1.1 * pixels_at_dist;
    printf("min=%d,max=%d,dist=%g,size=%d,ar=%g\n", min_blob_size, max_blob_size,dist,size,ar);

    if(size > min_blob_size && size < max_blob_size && ar > 0.8 && ar < 1.2)
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
      printf("found at %d,%d", x, y);
    }
  }
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
