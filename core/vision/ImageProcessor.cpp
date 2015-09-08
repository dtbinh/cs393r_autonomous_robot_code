#include <vision/ImageProcessor.h>
#include <vision/BeaconDetector.h>
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
  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM)
    return;visionLog((30, "Process Frame camera %i", camera_));

  updateTransform();
  
  // Horizon calculation
  visionLog((30, "Calculating horizon line"));
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 30000);
  vblocks_.robot_vision->horizon = horizon;
  visionLog((30, "Classifying Image", camera_));
  if(!classifier_->classifyImage(color_table_))
    return;
  detectBall();
  detectGoal();
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
bool ImageProcessor::sobel(unsigned char* img, unsigned char* gx, unsigned char* gy, unsigned char* sobel, unsigned int min_x = 0, unsigned int min_y = 0, unsigned int max_x = 320, unsigned int max_y = 240)
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
void ImageProcessor::grayThreshold(unsigned char threshold, unsigned char* img, unsigned char* thresholded, unsigned int min_x = 0, unsigned int min_y = 0, unsigned int max_x = 320, unsigned int max_y = 240)
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

//struct hough_cmp_gt
//{
//    hough_cmp_gt(const int* _aux) : aux(_aux) {}
//    bool operator()(int l1, int l2) const
//    {
//        return aux[l1] > aux[l2] || (aux[l1] == aux[l2] && l1 < l2);
//    }
//    const int* aux;
//};
//
//CvSeq*
//detectCircles( CvArr* src_image, void* circle_storage,
//                int method, double dp, double min_dist,
//                double param1, double param2,
//                int min_radius, int max_radius )
//{
//    CvSeq* result = 0;
//
//    CvMat stub, *img = (CvMat*)src_image;
//    CvMat* mat = 0;
//    CvSeq* circles = 0;
//    CvSeq circles_header;
//    CvSeqBlock circles_block;
//    int circles_max = INT_MAX;
//    int canny_threshold = cvRound(param1);
//    int acc_threshold = cvRound(param2);
//
//    img = cvGetMat( img, &stub );
//
//    if( !CV_IS_MASK_ARR(img))
//        CV_Error( CV_StsBadArg, "The source image must be 8-bit, single-channel" );
//
//    if( !circle_storage )
//        CV_Error( CV_StsNullPtr, "NULL destination" );
//
//    if( dp <= 0 || min_dist <= 0 || canny_threshold <= 0 || acc_threshold <= 0 )
//        CV_Error( CV_StsOutOfRange, "dp, min_dist, canny_threshold and acc_threshold must be all positive numbers" );
//
//    min_radius = MAX( min_radius, 0 );
//    if( max_radius <= 0 )
//        max_radius = MAX( img->rows, img->cols );
//    else if( max_radius <= min_radius )
//        max_radius = min_radius + 2;
//
//    if( CV_IS_STORAGE( circle_storage ))
//    {
//        circles = cvCreateSeq( CV_32FC3, sizeof(CvSeq),
//            sizeof(float)*3, (CvMemStorage*)circle_storage );
//    }
//    else if( CV_IS_MAT( circle_storage ))
//    {
//        mat = (CvMat*)circle_storage;
//
//        if( !CV_IS_MAT_CONT( mat->type ) || (mat->rows != 1 && mat->cols != 1) ||
//            CV_MAT_TYPE(mat->type) != CV_32FC3 )
//            CV_Error( CV_StsBadArg,
//            "The destination matrix should be continuous and have a single row or a single column" );
//
//        circles = cvMakeSeqHeaderForArray( CV_32FC3, sizeof(CvSeq), sizeof(float)*3,
//                mat->data.ptr, mat->rows + mat->cols - 1, &circles_header, &circles_block );
//        circles_max = circles->total;
//        cvClearSeq( circles );
//    }
//    else
//        CV_Error( CV_StsBadArg, "Destination is not CvMemStorage* nor CvMat*" );
//
//    const int SHIFT = 10, ONE = 1 << SHIFT;
//    cv::Ptr<CvMat> dx, dy;
//    cv::Ptr<CvMat> edges, accum, dist_buf;
//    std::vector<int> sort_buf;
//    cv::Ptr<CvMemStorage> storage;
//
//    int x, y, i, j, k, center_count, nz_count;
//    float min_radius2 = (float)min_radius*min_radius;
//    float max_radius2 = (float)max_radius*max_radius;
//    int rows, cols, arows, acols;
//    int astep, *adata;
//    float* ddata;
//    CvSeq *nz, *centers;
//    float idp, dr;
//    CvSeqReader reader;
//
//    edges.reset(cvCreateMat( img->rows, img->cols, CV_8UC1 ));
//
//    // Use the Canny Edge Detector to detect all the edges in the image.
//    cvCanny( img, edges, MAX(canny_threshold/2,1), canny_threshold, 3 );
//
//    dx.reset(cvCreateMat( img->rows, img->cols, CV_16SC1 ));
//    dy.reset(cvCreateMat( img->rows, img->cols, CV_16SC1 ));
//
//    /*Use the Sobel Derivative to compute the local gradient of all the non-zero pixels in the edge image.*/
//    cvSobel( img, dx, 1, 0, 3 );
//    cvSobel( img, dy, 0, 1, 3 );
//
//    if( dp < 1.f )
//        dp = 1.f;
//    idp = 1.f/dp;
//    accum.reset(cvCreateMat( cvCeil(img->rows*idp)+2, cvCeil(img->cols*idp)+2, CV_32SC1 ));
//    cvZero(accum);
//
//    storage.reset(cvCreateMemStorage());
//    /* Create sequences for the nonzero pixels in the edge image and the centers of circles
//    which could be detected.*/
//    nz = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );
//    centers = cvCreateSeq( CV_32SC1, sizeof(CvSeq), sizeof(int), storage );
//
//    rows = img->rows;
//    cols = img->cols;
//    arows = accum->rows - 2;
//    acols = accum->cols - 2;
//    adata = accum->data.i;
//    astep = accum->step/sizeof(adata[0]);
//    // Accumulate circle evidence for each edge pixel
//    for( y = 0; y < rows; y++ )
//    {
//        const uchar* edges_row = edges->data.ptr + y*edges->step;
//        const short* dx_row = (const short*)(dx->data.ptr + y*dx->step);
//        const short* dy_row = (const short*)(dy->data.ptr + y*dy->step);
//
//        for( x = 0; x < cols; x++ )
//        {
//            float vx, vy;
//            int sx, sy, x0, y0, x1, y1, r;
//            CvPoint pt;
//
//            vx = dx_row[x];
//            vy = dy_row[x];
//
//            if( !edges_row[x] || (vx == 0 && vy == 0) )
//                continue;
//
//            float mag = std::sqrt(vx*vx+vy*vy);
//            assert( mag >= 1 );
//            sx = cvRound((vx*idp)*ONE/mag);
//            sy = cvRound((vy*idp)*ONE/mag);
//
//            x0 = cvRound((x*idp)*ONE);
//            y0 = cvRound((y*idp)*ONE);
//            // Step from min_radius to max_radius in both directions of the gradient
//            for(int k1 = 0; k1 < 2; k1++ )
//            {
//                x1 = x0 + min_radius * sx;
//                y1 = y0 + min_radius * sy;
//
//                for( r = min_radius; r <= max_radius; x1 += sx, y1 += sy, r++ )
//                {
//                    int x2 = x1 >> SHIFT, y2 = y1 >> SHIFT;
//                    if( (unsigned)x2 >= (unsigned)acols ||
//                        (unsigned)y2 >= (unsigned)arows )
//                        break;
//                    adata[y2*astep + x2]++;
//                }
//
//                sx = -sx; sy = -sy;
//            }
//
//            pt.x = x; pt.y = y;
//            cvSeqPush( nz, &pt );
//        }
//    }
//
//    nz_count = nz->total;
//    if( !nz_count )
//      return result;
//    //Find possible circle centers
//    for( y = 1; y < arows - 1; y++ )
//    {
//        for( x = 1; x < acols - 1; x++ )
//        {
//            int base = y*(acols+2) + x;
//            if( adata[base] > acc_threshold &&
//                adata[base] > adata[base-1] && adata[base] > adata[base+1] &&
//                adata[base] > adata[base-acols-2] && adata[base] > adata[base+acols+2] )
//                cvSeqPush(centers, &base);
//        }
//    }
//
//    center_count = centers->total;
//    if( !center_count )
//        return result;
//
//    sort_buf.resize( MAX(center_count,nz_count) );
//    cvCvtSeqToArray( centers, &sort_buf[0] );
//    /*Sort candidate centers in descending order of their accumulator values, so that the centers
//    with the most supporting pixels appear first.*/
//    std::sort(sort_buf.begin(), sort_buf.begin() + center_count, hough_cmp_gt(adata));
//    cvClearSeq( centers );
//    cvSeqPushMulti( centers, &sort_buf[0], center_count );
//
//    dist_buf.reset(cvCreateMat( 1, nz_count, CV_32FC1 ));
//    ddata = dist_buf->data.fl;
//
//    dr = dp;
//    min_dist = MAX( min_dist, dp );
//    min_dist *= min_dist;
//    // For each found possible center
//    // Estimate radius and check support
//    for( i = 0; i < centers->total; i++ )
//    {
//        int ofs = *(int*)cvGetSeqElem( centers, i );
//        y = ofs/(acols+2);
//        x = ofs - (y)*(acols+2);
//        //Calculate circle's center in pixels
//        float cx = (float)((x + 0.5f)*dp), cy = (float)(( y + 0.5f )*dp);
//        float start_dist, dist_sum;
//        float r_best = 0;
//        int max_count = 0;
//        // Check distance with previously detected circles
//        for( j = 0; j < circles->total; j++ )
//        {
//            float* c = (float*)cvGetSeqElem( circles, j );
//            if( (c[0] - cx)*(c[0] - cx) + (c[1] - cy)*(c[1] - cy) < min_dist )
//                break;
//        }
//
//        if( j < circles->total )
//            continue;
//        // Estimate best radius
//        cvStartReadSeq( nz, &reader );
//        for( j = k = 0; j < nz_count; j++ )
//        {
//            CvPoint pt;
//            float _dx, _dy, _r2;
//            CV_READ_SEQ_ELEM( pt, reader );
//            _dx = cx - pt.x; _dy = cy - pt.y;
//            _r2 = _dx*_dx + _dy*_dy;
//            if(min_radius2 <= _r2 && _r2 <= max_radius2 )
//            {
//                ddata[k] = _r2;
//                sort_buf[k] = k;
//                k++;
//            }
//        }
//
//        int nz_count1 = k, start_idx = nz_count1 - 1;
//        if( nz_count1 == 0 )
//            continue;
//        dist_buf->cols = nz_count1;
//        cvPow( dist_buf, dist_buf, 0.5 );
//        // Sort non-zero pixels according to their distance from the center.
//        std::sort(sort_buf.begin(), sort_buf.begin() + nz_count1, hough_cmp_gt((int*)ddata));
//
//        dist_sum = start_dist = ddata[sort_buf[nz_count1-1]];
//        for( j = nz_count1 - 2; j >= 0; j-- )
//        {
//            float d = ddata[sort_buf[j]];
//
//            if( d > max_radius )
//                break;
//
//            if( d - start_dist > dr )
//            {
//                float r_cur = ddata[sort_buf[(j + start_idx)/2]];
//                if( (start_idx - j)*r_best >= max_count*r_cur ||
//                    (r_best < FLT_EPSILON && start_idx - j >= max_count) )
//                {
//                    r_best = r_cur;
//                    max_count = start_idx - j;
//                }
//                start_dist = d;
//                start_idx = j;
//                dist_sum = 0;
//            }
//            dist_sum += d;
//        }
//        // Check if the circle has enough support
//        if( max_count > acc_threshold )
//        {
//            float c[3];
//            c[0] = cx;
//            c[1] = cy;
//            c[2] = (float)r_best;
//            cvSeqPush( circles, c );
//            if( circles->total > circles_max )
//                return result;
//        }
//    }
//
//    if( mat )
//    {
//        if( mat->cols > mat->rows )
//            mat->cols = circles->total;
//        else
//            mat->rows = circles->total;
//    }
//    else
//        result = circles;
//
//    return result;
//}

void ImageProcessor::detectGoal()
{
  int imageX, imageY;
  if(!findGoal(imageX, imageY))
    return; // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* goal = &vblocks_.world_object->objects_[WO_OPP_GOAL];

  goal->imageCenterX = imageX;
  goal->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  goal->visionBearing = cmatrix_.bearing(p);
  goal->visionElevation = cmatrix_.elevation(p);
  goal->visionDistance = cmatrix_.groundDistance(p);

  goal->seen = true;
}

bool ImageProcessor::findGoal(int& imageX, int& imageY)
{
  double ball_cx = 0.0;
  double ball_cy = 0.0;
  double num_ball_hits = 0.0;
  double min_goal_size = 500;
  for(unsigned int x = 0; x < getImageWidth(); x++)
  {
    for(unsigned int y = 0; y < getImageHeight(); y++)
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
    imageX = imageY = 0;
    //printf("no goal hits\n");
    return false;
  }
  else
  {
    imageX = (int) ball_cx / num_ball_hits;
    imageY = (int) ball_cy / num_ball_hits;
    printf("goal at x=%d, y=%d\n", imageX, imageY);
    return true;
  }
}

void ImageProcessor::detectBall()
{
  int imageX, imageY;
  if(!findBall(imageX, imageY))
    return; // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  ball->imageCenterX = imageX;
  ball->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  ball->seen = true;
}

bool ImageProcessor::findBall(int& imageX, int& imageY)
{
  double ball_cx = 0.0;
  double ball_cy = 0.0;
  double num_ball_hits = 0.0;
  for(unsigned int x = 0; x < getImageWidth(); x++)
  {
    for(unsigned int y = 0; y < getImageHeight(); y++)
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
    imageX = imageY = 0;
    //printf("no ball hits\n");
    return false;
  }
  else
  {
    imageX = (int) ball_cx / num_ball_hits;
    imageY = (int) ball_cy / num_ball_hits;
    printf("ball at x=%d, y=%d\n", imageX, imageY);
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
