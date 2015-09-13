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

  // MergeBlob mergeblob(getSegImg(), 320, 240, 4, 2, 10);

  // printf("Found %d blobs\n", mergeblob.get_blob_number());
  // for(int i = 0; i < mergeblob.get_blob_number(); i++)
  // {
  //   printf("pre-crash\n");
  //   MergeBlob::Blob blob = mergeblob.blob[i];
  //   printf("post-crash\n");
  //   if(blob.color == c_ORANGE)
  //   {
  //     printf("Found orange blob\n");
  //     // printf("Found orange blob at x=%d,y=%g\n", blob.centroid_x, blob.centroid_y);
  //     // mergeblob.DisplayBlob(i);
  //   }
  // }

  // detectBall();
  // detectGoal();
  // findBeacons();
    // printf("destructing\n");
}

inline unsigned int idx(unsigned int x, unsigned int y)
{
  return 320 * y + x;
}

inline unsigned char applyMatrix3(unsigned char* img, unsigned int cx, unsigned int cy, unsigned char* mat)
{
  unsigned char v;
  for(unsigned int x = -1; x <= 1; x++)
  {
    for(unsigned int y = -1; y <= 1; y++)
    {
      unsigned int mat_idx = 320 * y + x;
      unsigned int img_idx = 320 * (cy + y) + (cx + x);
      v += mat[mat_idx] * img[img_idx];
    }
  }
  return v;
}

//img (input), gx, gy, sobel (outputs) must be preallocated 320x240
bool ImageProcessor::sobel(unsigned char* img, unsigned char* gx, unsigned char* gy, unsigned char* sobel, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
//  static unsigned char sobel_x[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
//  static unsigned char sobel_y[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

  for(unsigned int cx = min_x; cx < max_x; cx++)
  {
    for(unsigned int cy = min_y; cy < max_y; cy++)
    {
      unsigned int img_idx = idx(cx, cy);
      unsigned char color = img[img_idx];
//      gx[idx] = applyMatrix3(img, x, y, sobel_x);
//      gy[idx] = applyMatrix3(img, x, y, sobel_y);

      if(cx == 0)
      {
        if(cy == 0)
        {
          gx[img_idx] = 2 * img[idx(cx + 1, cy)] + img[idx(cx + 1, cy + 1)];
          gy[img_idx] = img[idx(cx + 1, cy + 1)] + 2 * img[idx(cx, cy + 1)];
        }
        else if(cy == 239)
        {
          gx[img_idx] = img[idx(cx + 1, cy - 1)] + 2 * img[idx(cx + 1, cy)];
          gy[img_idx] = -(img[idx(cx + 1, cy - 1)] + 2 * img[idx(cx, cy - 1)]);
        }
        else
        {
          gx[img_idx] = img[idx(cx + 1, cy - 1)] + 2 * img[idx(cx + 1, cy)] + img[idx(cx + 1, cy + 1)];
          gy[img_idx] = img[idx(cx + 1, cy + 1)] + 2 * img[idx(cx, cy + 1)] - (img[idx(cx + 1, cy - 1)] + 2 * img[idx(cx, cy - 1)]);
        }
      }
      else if(cx == 239)
      {
        if(cy == 0)
        {
          gx[img_idx] = -(2 * img[idx(cx - 1, cy)] + img[idx(cx - 1, cy + 1)]);
          gy[img_idx] = 2 * img[idx(cx, cy + 1)] + img[idx(cx - 1, cy + 1)];
        }
        else if(cy == 239)
        {
          gx[img_idx] = -(img[idx(cx - 1, cy - 1)] + 2 * img[idx(cx - 1, cy)]);
          gy[img_idx] = -(2 * img[idx(cx, cy - 1)] + img[idx(cx - 1, cy - 1)]);
        }
        else
        {
          gx[img_idx] = -(img[idx(cx - 1, cy - 1)] + 2 * img[idx(cx - 1, cy)] + img[idx(cx - 1, cy + 1)]);
          gy[img_idx] = 2 * img[idx(cx, cy + 1)] + img[idx(cx - 1, cy + 1)] - (2 * img[idx(cx, cy - 1)] + img[idx(cx - 1, cy - 1)]);
        }
      }
      else
      {
        if(cy == 0)
        {
          gx[img_idx] = 2 * img[idx(cx + 1, cy)] + img[idx(cx + 1, cy + 1)] - (2 * img[idx(cx - 1, cy)] + img[idx(cx - 1, cy + 1)]);
          gy[img_idx] = img[idx(cx + 1, cy + 1)] + 2 * img[idx(cx, cy + 1)] + img[idx(cx - 1, cy + 1)];
        }
        else if(cy == 239)
        {
          gx[img_idx] = img[idx(cx + 1, cy - 1)] + 2 * img[idx(cx + 1, cy)] - (img[idx(cx - 1, cy - 1)] + 2 * img[idx(cx - 1, cy)]);
          gy[img_idx] = -(img[idx(cx + 1, cy - 1)] + 2 * img[idx(cx, cy - 1)] + img[idx(cx - 1, cy - 1)]);
        }
        else
        {
          //general case
          gx[img_idx] = img[idx(cx + 1, cy - 1)] + 2 * img[idx(cx + 1, cy)] + img[idx(cx + 1, cy + 1)] - (img[idx(cx - 1, cy - 1)] + 2 * img[idx(cx - 1, cy)] + img[idx(cx - 1, cy + 1)]);
          gy[img_idx] = img[idx(cx + 1, cy + 1)] + 2 * img[idx(cx, cy + 1)] + img[idx(cx - 1, cy + 1)] - (img[idx(cx + 1, cy - 1)] + 2 * img[idx(cx, cy - 1)] + img[idx(cx - 1, cy - 1)]);
        }
      }

      sobel[img_idx] = (unsigned int) sqrt(gx[img_idx] * gx[img_idx] + gy[img_idx] * gy[img_idx]);
//      double theta = atan2(gy,gx);
    }
  }

  return true;
}

//img (input) and thresholded (output) must be preallocated 320x240
void ImageProcessor::grayThreshold(unsigned char threshold, unsigned char* img, unsigned char* thresholded, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
  for(unsigned int x = min_x; x < max_x; x++)
  {
    for(unsigned int y = min_y; y < max_y; y++)
    {
      unsigned int img_idx = idx(x, y);
      if(img[img_idx] > threshold)
      {
        thresholded[img_idx] = 1;
      }
      else
      {
        thresholded[img_idx] = 0;
      }
    }
  }
}

bool columnBeaconType(unsigned char* seg_img, unsigned int x, WorldObjectType& beacon_type, unsigned int& lowest_y)
{ 
  unsigned int pink_count = 0;
  unsigned int pink_min = 0;
  unsigned int pink_max = 0;
  unsigned int blue_count = 0;
  unsigned int blue_min = 0;
  unsigned int blue_max = 0;
  unsigned int yellow_count = 0;
  unsigned int yellow_min = 0;
  unsigned int yellow_max = 0;
  std::vector<unsigned char> column_colors;

  //gather beacon colors
  for(unsigned int y = 0; y < 240; y++) //scan the rows from top to bottom
  {
    unsigned int c = seg_img[idx(x,y)];
    if(c == c_PINK)
    {
      if(pink_count == 0)
      {
        pink_min = y;
      }
      pink_max = y;
      pink_count++;
      if(column_colors.size() == 0 || column_colors[column_colors.size()-1] != c_PINK)
        column_colors.push_back(c);
    }
    else if(c == c_BLUE)
    {
      if(blue_count == 0)
      {
        blue_min = y;
      }
      blue_max = y;
      blue_count++;
      if(column_colors.size() == 0 || column_colors[column_colors.size()-1] != c_BLUE)
        column_colors.push_back(c);
    }
    else if(c == c_YELLOW)
    {
      if(yellow_count == 0)
      {
        yellow_min = y;
      }
      yellow_max = y;
      yellow_count++;
      if(column_colors.size() == 0 || column_colors[column_colors.size()-1] != c_YELLOW)
        column_colors.push_back(c);
    }
  }

  //filter noise
  unsigned int count_threshold = 4;
  unsigned int valid_color_count = 3;
  if(pink_count < count_threshold)
  {
    //todo: also rip these out of the column colors
    pink_count = 0;
    valid_color_count--;
  }
  if(blue_count < count_threshold)
  {
    blue_count = 0;
    valid_color_count--;
  }
  if(yellow_count < count_threshold)
  {
    yellow_count = 0;
    valid_color_count--;
  }

  if(valid_color_count != 2 || column_colors.size() == 0) //something weird happened or nothing was there
  {
    return false;
  }

  if(pink_count < blue_count && pink_count < yellow_count) //beacon is blue and yellow
  {
    unsigned int avg_cylinder_height = (blue_max - blue_min + yellow_max - yellow_min) / 2;
    if(column_colors[0] == c_YELLOW)
    {
      lowest_y = blue_max + avg_cylinder_height * 2;
      beacon_type = WO_BEACON_YELLOW_BLUE;
      return true;
    }
    else
    {
      lowest_y = yellow_max + avg_cylinder_height * 2;
      beacon_type = WO_BEACON_BLUE_YELLOW;
      return true;
    }
  }

  if(yellow_count < blue_count && yellow_count < pink_count) //beacon is blue and pink
  {
    unsigned int avg_cylinder_height = (blue_max - blue_min + pink_max - pink_min) / 2;
    if(column_colors[0] == c_PINK)
    {
      lowest_y = blue_max + avg_cylinder_height;
      beacon_type = WO_BEACON_PINK_BLUE;
      return true;
    }
    else
    {
      lowest_y = pink_max + avg_cylinder_height;
      beacon_type = WO_BEACON_BLUE_PINK;
      return true;
    }
  }

  if(blue_count < yellow_count && blue_count < pink_count) //beacon is yellow and pink
  {
    unsigned int avg_cylinder_height = (yellow_max - yellow_min + pink_max - pink_min) / 2;
    if(column_colors[0] == c_PINK)
    {
      lowest_y = yellow_max + avg_cylinder_height;
      beacon_type = WO_BEACON_PINK_YELLOW;
      return true;
    }
    else
    {
      lowest_y = pink_max + avg_cylinder_height;
      beacon_type = WO_BEACON_YELLOW_PINK;
      return true;
    }
  }

  return false;
}

void drawBeaconMarker(unsigned char* img, unsigned int x, unsigned int y)
{
  unsigned int marker_size = 10;
  for(unsigned int dx = (x-marker_size/2); dx < (x+marker_size/2); dx++)
  {
    for(unsigned int dy = (y-marker_size/2); dy < (y+marker_size/2); dy++)
    {
      unsigned int Y_idx = 3*idx(dx,dy) + 0;
      unsigned int U_idx = 3*idx(dx,dy) + 1;
      unsigned int V_idx = 3*idx(dx,dy) + 2;
      img[Y_idx] = 76;
      img[U_idx] = 84;
      img[V_idx] = 255;
    }
  }
}

bool ImageProcessor::findBeacons()
{
  unsigned char* seg_img = getSegImg();
  std::map<WorldObjectType, double> column_count_map;
  column_count_map[WO_BEACON_BLUE_YELLOW] = 0.0;
  column_count_map[WO_BEACON_YELLOW_BLUE] = 0.0;
  column_count_map[WO_BEACON_BLUE_PINK] = 0.0;
  column_count_map[WO_BEACON_PINK_BLUE] = 0.0;
  column_count_map[WO_BEACON_PINK_YELLOW] = 0.0;
  column_count_map[WO_BEACON_YELLOW_PINK] = 0.0;

  for(unsigned int x = 0; x < 320; x++) //for every column
  {
    WorldObjectType beacon_type;
    unsigned int lowest_y;
    if(columnBeaconType(seg_img, x, beacon_type, lowest_y)) //throw away empty columns
    {
      (&vblocks_.world_object->objects_[beacon_type])->imageCenterX += x;
      (&vblocks_.world_object->objects_[beacon_type])->imageCenterY += lowest_y;
      column_count_map[beacon_type] += 1.0;
    }
  }

  unsigned char* img = getImg();
  for(std::map<WorldObjectType, double>::iterator iter = column_count_map.begin(); iter != column_count_map.end(); iter++)
  {
    if(iter->second != 0.0)
    {
      (&vblocks_.world_object->objects_[iter->first])->imageCenterX /= iter->second;
      (&vblocks_.world_object->objects_[iter->first])->imageCenterY /= iter->second;
      (&vblocks_.world_object->objects_[iter->first])->seen = true;

      std::cerr << "Found beacon " << iter->first << " at x=" <<(&vblocks_.world_object->objects_[iter->first])->imageCenterX << ", y="<<(&vblocks_.world_object->objects_[iter->first])->imageCenterY << std::endl;

      //draw points on image
      drawBeaconMarker(img,(&vblocks_.world_object->objects_[iter->first])->imageCenterX, (&vblocks_.world_object->objects_[iter->first])->imageCenterY);
    }
  }
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
    printf("goal at x=%d, y=%d\n", point.x, point.y);
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
    printf("ball at x=%d, y=%d\n", point.x, point.y);
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
