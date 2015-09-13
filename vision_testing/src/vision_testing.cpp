#include "vision_testing/vision_testing.hpp"

using namespace cv;
using namespace std;

unsigned long fileSize(const char* filename)
{
  ifstream is;
  is.open(filename, ios::binary);
  is.seekg(0, ios::end);
  return is.tellg();
}

bool loadYUV(const char* filename, Mat& bgr, int width = 320, int height = 240)
{
  Size img_size = Size(width, height);
  size_t npixels = width * height;
  std::cerr << "Expecting file of size " << (npixels + npixels / 4 + npixels / 4) << std::endl;

  unsigned long file_size = fileSize(filename);
  std::cerr << "Opening file of size " << file_size << std::endl;

  FILE *fin = fopen(filename, "rb");
  if(!fin)
  {
    printf("Unable to open file: %s\n", filename);
    return false;
  }

  bgr = Mat(img_size, CV_8U, 3);
  Mat ycrcb(img_size, CV_8U, 3);
  Mat y(img_size, CV_8U, 1);
  Mat cb(img_size, CV_8U, 1);
  Mat cr(img_size, CV_8U, 1);
  Mat cb_half(Size(width / 2, height / 2), CV_8U, 1);
  Mat cr_half(Size(width / 2, height / 2), CV_8U, 1);

  if(fread(y.data, sizeof(uint8_t), npixels, fin) != npixels || fread(cb_half.data, sizeof(uint8_t), npixels / 4, fin) != npixels / 4 || fread(cr_half.data, sizeof(uint8_t), npixels / 4, fin) != npixels / 4)
    return false;

  cv::resize(cb_half, cb, img_size);
  cv::resize(cr_half, cr, img_size);
  cv::merge(std::vector<Mat>{y, cr, cb}, ycrcb);

  cv::cvtColor(ycrcb, bgr, CV_YCrCb2BGR);
  return true;
}

bool loadWeirdNaoFormat(const char* filename, Mat& ycrcb, unsigned int width = 320, unsigned int height = 240)
{
  Size img_size = Size(width, height);
  FILE* fin = fopen(filename, "rb");
  if(!fin)
  {
    printf("Unable to open file: %s\n", filename);
    return false;
  }

  uint8_t buffer[4];
  std::vector<unsigned char> wtf_y, wtf_u, wtf_v;
  for(unsigned int py = 0; py < height; py++)
  {
    for(unsigned int px = 0; px < width; px += 2)
    {
      fread(buffer, sizeof(uint8_t), 4, fin);
      unsigned char y1 = buffer[0];
      unsigned char v = buffer[1];
      unsigned char y2 = buffer[2];
      unsigned char u = buffer[3];

      wtf_y.push_back(y1);
      wtf_u.push_back(u);
      wtf_v.push_back(v);
      wtf_y.push_back(y2);
      wtf_u.push_back(u);
      wtf_v.push_back(v);
    }
  }

  Mat y(img_size, CV_8U, 1);
  Mat cb(img_size, CV_8U, 1);
  Mat cr(img_size, CV_8U, 1);
  memcpy(y.data, &wtf_y.front(), wtf_y.size());
  memcpy(cr.data, &wtf_u.front(), wtf_u.size());
  memcpy(cb.data, &wtf_v.front(), wtf_v.size());
  cv::merge(std::vector<Mat>{y, cr, cb}, ycrcb);
  cv::cvtColor(ycrcb, ycrcb, CV_YCrCb2BGR);

  return true;
}

inline unsigned int idx(unsigned int x, unsigned int y)
{
  return 320 * y + x;
}

bool sobel(unsigned char* img, unsigned char* gx, unsigned char* gy, unsigned char* sobel, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
//  static unsigned char sobel_x[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
//  static unsigned char sobel_y[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

  for(unsigned int cx = min_x; cx < max_x; cx++)
  {
    for(unsigned int cy = min_y; cy < max_y; cy++)
    {
      unsigned int img_idx = idx(cx, cy);
//      unsigned char color = img[img_idx];
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
void grayThreshold(unsigned char min_val, unsigned char max_val, unsigned char* img, unsigned char* thresholded, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
  for(unsigned int x = min_x; x < max_x; x++)
  {
    for(unsigned int y = min_y; y < max_y; y++)
    {
      unsigned int img_idx = idx(x, y);
      if(img[img_idx] > min_val && img[img_idx] < max_val)
      {
        thresholded[img_idx] = 255;
      }
      else
      {
        thresholded[img_idx] = 0;
      }
    }
  }
}
enum Color
{
  c_UNDEFINED = 0,
  c_FIELD_GREEN = 1,
  c_WHITE = 2,
  c_ORANGE = 3,
  c_PINK = 4,
  c_BLUE = 5,
  c_YELLOW = 6,
  c_ROBOT_WHITE = 7,
  NUM_COLORS = 8
};

void generateClusterVis(unsigned char* clusters, unsigned char* vis_bgr, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
  for(unsigned int x = min_x; x < max_x; x++)
  {
    for(unsigned int y = min_y; y < max_y; y++)
    {
      switch(clusters[idx(x, y)])
      {
      case c_UNDEFINED:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      case c_FIELD_GREEN:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      case c_WHITE:
        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case c_ORANGE:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 255 / 2;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case c_PINK:
        vis_bgr[3 * idx(x, y) + 0] = 255 * 0.75;
        vis_bgr[3 * idx(x, y) + 1] = 255 * 0.5;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case c_BLUE:
        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      case c_YELLOW:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case c_ROBOT_WHITE:
        vis_bgr[3 * idx(x, y) + 0] = 255 * 0.75;
        vis_bgr[3 * idx(x, y) + 1] = 255 * 0.75;
        vis_bgr[3 * idx(x, y) + 2] = 255 * 0.75;
        break;
      default:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      }
    }
  }
}

enum FullColor
{
  RED,
  YELLOW_RED,
  YELLOW,
  GREEN_YELLOW,
  GREEN,
  CYAN_GREEN,
  CYAN,
  BLUE_CYAN,
  BLUE,
  MAGENTA_BLUE,
  MAGENTA,
  RED_MAGENTA,
  WHITE,
  LIGHT_GREY,
  DARK_GREY,
  BLACK,
  NUM_FULL_COLORS
};

void generateFullClusterVis(unsigned char* clusters, unsigned char* vis_bgr, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
  for(unsigned int x = min_x; x < max_x; x++)
  {
    for(unsigned int y = min_y; y < max_y; y++)
    {
      switch(clusters[idx(x, y)])
      {
      case RED:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case YELLOW_RED:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 255 / 2;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case YELLOW:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case GREEN_YELLOW:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 255 / 2;
        break;
      case GREEN:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      case CYAN_GREEN:
        vis_bgr[3 * idx(x, y) + 0] = 255 / 2;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      case CYAN:
        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      case BLUE_CYAN:
        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 255 / 2;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      case BLUE:
        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      case MAGENTA_BLUE:
        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 255 / 2;
        break;
      case MAGENTA:
        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case RED_MAGENTA:
        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 255 / 2;
        break;
      case WHITE:
        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case LIGHT_GREY:
        vis_bgr[3 * idx(x, y) + 0] = 255 * 0.75;
        vis_bgr[3 * idx(x, y) + 1] = 255 * 0.75;
        vis_bgr[3 * idx(x, y) + 2] = 255 * 0.75;
        break;
      case DARK_GREY:
        vis_bgr[3 * idx(x, y) + 0] = 255 * 0.375;
        vis_bgr[3 * idx(x, y) + 1] = 255 * 0.375;
        vis_bgr[3 * idx(x, y) + 2] = 255 * 0.375;
        break;
      case BLACK:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      default:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      }
    }
  }
}

class HSV
{
public:
  HSV()
  {
  }
  HSV(unsigned char H, unsigned char S, unsigned char V)
  {
    h = H;
    s = S;
    v = V;
  }
  unsigned char h;
  unsigned char s;
  unsigned char v;
};

class RGB
{
public:
  RGB()
  {
  }
  RGB(unsigned char R, unsigned char G, unsigned char B)
  {
    r = R;
    g = G;
    b = B;
  }
  unsigned char r;
  unsigned char g;
  unsigned char b;
};

class YCrCb
{
public:
  YCrCb()
  {
  }
  YCrCb(unsigned char Y, unsigned char Cr, unsigned char Cb)
  {
    y = Y;
    cr = Cr;
    cb = Cb;
  }
  unsigned char y;
  unsigned char cr;
  unsigned char cb;
};

RGB yuv2rgb(YCrCb in)
{
  return RGB(CLIP(in.y + (91881 * in.cr >> 16) - 179), CLIP(in.y - ((22544 * in.cb + 46793 * in.cr) >> 16) + 135), CLIP(in.y + (116129 * in.cb >> 16) - 226));
//  double delta = 128.0;
//  double R = (double) in.y + 1.403 * ((double) in.cr - delta);
//  double G = (double) in.y - 0.714 * ((double) in.cr - delta) - 0.344 * ((double) in.cb - delta);
//  double B = (double) in.y + 1.773 * ((double) in.cb - delta);
//  return RGB(R,G,B);
}

HSV rgb2hsv(RGB in)
{
  double R = (double) in.r / 255.0;
  double G = (double) in.g / 255.0;
  double B = (double) in.b / 255.0;
  double V = std::max(std::max(R, G), B);
  double delta = V - std::min(std::min(R, G), B);
  double S = (V == 0)? 0 : delta / V;
  double H = 0.0;
  if(V == R)
  {
    H = 60 * (G - B) / delta;
  }
  else if(V == G)
  {
    H = 120 + 60 * (B - R) / delta;
  }
  else //V==B
  {
    H = 240 + 60 * (R - G) / delta;
  }

  if(H < 0)
  {
    H += 360;
  }
  return HSV(H / 2, S * 255, V * 255);
}

void yuvImage2rgb(unsigned char* img_ycrcb, unsigned char* img_rgb, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
  static bool have_table = false;
  static RGB ycrcbrgb_table[256][256][256];
  if(!have_table)
  {
    have_table = true;
    for(unsigned int y = 0; y < 256; y++) //should stop at 220
      for(unsigned int cr = 0; cr < 256; cr++) //should stop at 235
        for(unsigned int cb = 0; cb < 256; cb++) //should stop at 235
        {
          RGB rgb = yuv2rgb(YCrCb(y, cr, cb));
          ycrcbrgb_table[y][cr][cb] = rgb;
        }
    std::cerr << "Table generated!" << std::endl;
  }

  for(unsigned int x = min_x; x < max_x; x++)
  {
    for(unsigned int y = min_y; y < max_y; y++)
    {
      unsigned char cy = img_ycrcb[3 * idx(x, y) + 0];
      unsigned char cr = img_ycrcb[3 * idx(x, y) + 1];
      unsigned char cb = img_ycrcb[3 * idx(x, y) + 2];

      RGB col = ycrcbrgb_table[cy][cr][cb];
      img_rgb[3 * idx(x, y) + 0] = col.r;
      img_rgb[3 * idx(x, y) + 1] = col.g;
      img_rgb[3 * idx(x, y) + 2] = col.b;
    }
  }
}

void rgbImage2hsv(unsigned char* img_rgb, unsigned char* img_hsv, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
  static bool have_table = false;
  static HSV rgbhsv_table[256][256][256];
  if(!have_table)
  {
    have_table = true;
    for(unsigned int r = 0; r < 256; r++)
      for(unsigned int g = 0; g < 256; g++)
        for(unsigned int b = 0; b < 256; b++)
        {
          HSV hsv = rgb2hsv(RGB(r, g, b));
          rgbhsv_table[r][g][b] = hsv;
        }
    std::cerr << "Table generated!" << std::endl;
  }

  for(unsigned int x = min_x; x < max_x; x++)
  {
    for(unsigned int y = min_y; y < max_y; y++)
    {
      unsigned char cr = img_rgb[3 * idx(x, y) + 0];
      unsigned char cg = img_rgb[3 * idx(x, y) + 1];
      unsigned char cb = img_rgb[3 * idx(x, y) + 2];

      HSV col = rgbhsv_table[cr][cg][cb];
      img_hsv[3 * idx(x, y) + 0] = col.h;
      img_hsv[3 * idx(x, y) + 1] = col.s;
      img_hsv[3 * idx(x, y) + 2] = col.v;
    }
  }
}

void yuvImage2hsv(unsigned char* img_yuv, unsigned char* img_hsv, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
  static bool have_table = false;
  static HSV ycrcbhsv_table[256][256][256];
  if(!have_table)
  {
    have_table = true;
    for(unsigned int y = 0; y < 256; y++) //should stop at 220
      for(unsigned int cr = 0; cr < 256; cr++) //should stop at 235
        for(unsigned int cb = 0; cb < 256; cb++) //should stop at 235
        {
          RGB rgb = yuv2rgb(YCrCb(y, cr, cb));
          ycrcbhsv_table[y][cr][cb] = rgb2hsv(rgb);
//          printf("yuv(%d,%d,%d) => rgb(%d,%d,%d)\n", y, u, v, rgb.r, rgb.g, rgb.b);
//          printf("rgb(%d,%d,%d) => hsv(%d,%d,%d)\n", rgb.r, rgb.g, rgb.b, yuv2hsv[y][u][v].h, yuv2hsv[y][u][v].s, yuv2hsv[y][u][v].v);
        }
    std::cerr << "Table generated!" << std::endl;
  }

  for(unsigned int x = min_x; x < max_x; x++)
  {
    for(unsigned int y = min_y; y < max_y; y++)
    {
      unsigned char cy = img_yuv[3 * idx(x, y) + 0];
      unsigned char cr = img_yuv[3 * idx(x, y) + 1];
      unsigned char cb = img_yuv[3 * idx(x, y) + 2];

      HSV col = ycrcbhsv_table[cy][cr][cb];
//      printf("yuv(%d,%d,%d) => hsv(%d,%d,%d)\n", cy, cu, cv, yuvhsv_table[cy][cu][cv].h, yuvhsv_table[cy][cu][cv].s, yuvhsv_table[cy][cu][cv].v);
      img_hsv[3 * idx(x, y) + 0] = col.h;
      img_hsv[3 * idx(x, y) + 1] = col.s;
      img_hsv[3 * idx(x, y) + 2] = col.v;
    }
  }
}

void hsvCluster(unsigned char* img_hsv, unsigned char* full_clusters, unsigned char* clusters, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
  for(unsigned int x = min_x; x < max_x; x++)
  {
    for(unsigned int y = min_y; y < max_y; y++)
    {
      HSV col;
      col.h = img_hsv[3 * idx(x, y) + 0];
      col.s = img_hsv[3 * idx(x, y) + 1];
      col.v = img_hsv[3 * idx(x, y) + 2];
      if(col.v < 30) //black
      {
        full_clusters[idx(x, y)] = BLACK;
        clusters[idx(x, y)] = c_UNDEFINED;
      }
      else if(col.s < 85) //not colorful
      {
        if(col.v < 3 * 255 / 8)
        {
          full_clusters[idx(x, y)] = DARK_GREY;
          clusters[idx(x, y)] = c_UNDEFINED;
        }
        else if(col.v < 160)
        {
          full_clusters[idx(x, y)] = LIGHT_GREY;
          clusters[idx(x, y)] = c_ROBOT_WHITE;
        }
        else
        {
          full_clusters[idx(x, y)] = WHITE;
          clusters[idx(x, y)] = c_WHITE;
        }
      }
      else //colored
      {
        if(col.h < 15 / 2 || col.h >= 345 / 2)
        {
          //red
          full_clusters[idx(x, y)] = RED;
          clusters[idx(x, y)] = c_PINK;
        }
        else if(col.h < 45 / 2)
        {
          //yellow-red
          full_clusters[idx(x, y)] = YELLOW_RED;
          clusters[idx(x, y)] = c_ORANGE;
        }
        else if(col.h < 75 / 2)
        {
          //yellow
          full_clusters[idx(x, y)] = YELLOW;
          clusters[idx(x, y)] = c_YELLOW;
        }
        else if(col.h < 105 / 2)
        {
          //green-yellow
          full_clusters[idx(x, y)] = GREEN_YELLOW;
          clusters[idx(x, y)] = c_YELLOW;
        }
        else if(col.h < 135 / 2)
        {
          //green
          full_clusters[idx(x, y)] = GREEN;
          clusters[idx(x, y)] = c_FIELD_GREEN;
        }
        else if(col.h < 165 / 2)
        {
          //cyan-green
          full_clusters[idx(x, y)] = CYAN_GREEN;
          clusters[idx(x, y)] = c_FIELD_GREEN;
        }
        else if(col.h < 195 / 2)
        {
          //cyan
          full_clusters[idx(x, y)] = CYAN;
          clusters[idx(x, y)] = c_BLUE;
        }
        else if(col.h < 225 / 2)
        {
          //blue-cyan
          full_clusters[idx(x, y)] = BLUE_CYAN;
          clusters[idx(x, y)] = c_BLUE;
        }
        else if(col.h < 255 / 2)
        {
          //blue
          full_clusters[idx(x, y)] = BLUE;
          clusters[idx(x, y)] = c_BLUE;
        }
        else if(col.h < 285 / 2)
        {
          //magenta-blue
          full_clusters[idx(x, y)] = MAGENTA_BLUE;
          clusters[idx(x, y)] = c_BLUE;
        }
        else if(col.h < 315 / 2)
        {
          //magenta
          full_clusters[idx(x, y)] = MAGENTA;
          clusters[idx(x, y)] = c_PINK;
        }
        else if(col.h < 345 / 2)
        {
          //red-magenta
          full_clusters[idx(x, y)] = RED_MAGENTA;
          clusters[idx(x, y)] = c_PINK;
        }
      }
    }
  }
}

std::pair<Color, unsigned int> mode(std::vector<Color> colors, unsigned int max)
{
  std::vector<int> histogram(max, 0);
  for(unsigned int i = 0; i < colors.size(); ++i)
    ++histogram[colors[i]];
  Color max_value = (Color) (std::max_element(histogram.begin(), histogram.end()) - histogram.begin());
  unsigned int count = histogram[max_value];
  return std::pair<Color, unsigned int>(max_value, count);
}

enum ColorTransition
{
  NO_TRANSITION,
  PINK_BLUE,
  PINK_YELLOW,
  PINK_WHITE,
  BLUE_PINK,
  BLUE_YELLOW,
  BLUE_WHITE,
  YELLOW_BLUE,
  YELLOW_PINK,
  YELLOW_WHITE,
  WHITE_GREEN
};

void detectTransitions(unsigned char* img, unsigned char* transitions, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
  //set no transition everywhere
  for(unsigned int x = min_x; x < max_x; x++)
  {
    for(unsigned int y = min_y; y < max_y; y++)
    {
      transitions[idx(x, y)] = NO_TRANSITION;
    }
  }

  unsigned int range = 3;
  for(unsigned int x = min_x; x < max_x; x++)
  {
    for(unsigned int y = min_y + range; y < max_y - range; y++)
    {
      std::pair<Color, unsigned int> upper_color;
      std::pair<Color, unsigned int> lower_color;
      std::vector<Color> upper;
      std::vector<Color> lower;
      for(unsigned int r = 1; r <= range; r++)
      {
        upper.push_back((Color) img[idx(x, y - r)]);
        lower.push_back((Color) img[idx(x, y + r)]);
      }
      upper_color = mode(upper, NUM_COLORS);
      lower_color = mode(lower, NUM_COLORS);

      if(upper_color.second < 2 || lower_color.second < 2 || abs(upper_color.second - lower_color.second) > 1) //make sure it's a good fit
      {
        continue;
      }

      //beacons
      if(upper_color.first == c_PINK && lower_color.first == c_BLUE)
      {
        transitions[idx(x, y)] = PINK_BLUE;
      }
      else if(upper_color.first == c_PINK && lower_color.first == c_YELLOW)
      {
        transitions[idx(x, y)] = PINK_YELLOW;
      }
      else if(upper_color.first == c_BLUE && lower_color.first == c_PINK)
      {
        transitions[idx(x, y)] = BLUE_PINK;
      }
      else if(upper_color.first == c_BLUE && lower_color.first == c_YELLOW)
      {
        transitions[idx(x, y)] = BLUE_YELLOW;
      }
      else if(upper_color.first == c_YELLOW && lower_color.first == c_BLUE)
      {
        transitions[idx(x, y)] = YELLOW_BLUE;
      }
      else if(upper_color.first == c_YELLOW && lower_color.first == c_PINK)
      {
        transitions[idx(x, y)] = YELLOW_PINK;
      }
      else if(upper_color.first == c_YELLOW && lower_color.first == c_WHITE)
      {
        transitions[idx(x, y)] = YELLOW_WHITE;
      }
      else if(upper_color.first == c_BLUE && lower_color.first == c_WHITE)
      {
        transitions[idx(x, y)] = BLUE_WHITE;
      }
      else if(upper_color.first == c_PINK && lower_color.first == c_WHITE)
      {
        transitions[idx(x, y)] = PINK_WHITE;
      }
      else if(upper_color.first == c_WHITE && lower_color.first == c_FIELD_GREEN)
      {
        transitions[idx(x, y)] = WHITE_GREEN;
      }
    }
  }
}

enum BeaconPossibilityState
{
  HAS_NOTHING,
  HAS_BOTTOM,
  FIRST_COLOR_PINK,
  FIRST_COLOR_BLUE,
  FIRST_COLOR_YELLOW,
  IS_BEACON
};

enum BeaconType
{
  NOT_A_BEACON,
  YELLOW_PINK_BEACON,
  YELLOW_BLUE_BEACON,
  BLUE_PINK_BEACON,
  BLUE_YELLOW_BEACON,
  PINK_BLUE_BEACON,
  PINK_YELLOW_BEACON,
  NUM_BEACON_TYPES
};

void drawPoint(unsigned char* img, int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
  img[3 * idx(x, y) + 0] = b;
  img[3 * idx(x, y) + 1] = g;
  img[3 * idx(x, y) + 2] = r;
}

void drawLine(unsigned char* img, int x1, int y1, int x2, int y2, unsigned char r, unsigned char g, unsigned char b)
{
  int delta_x(x2 - x1);
  // if x1 == x2, then it does not matter what we set here
  signed char const ix((delta_x > 0) - (delta_x < 0));
  delta_x = std::abs(delta_x) << 1;

  int delta_y(y2 - y1);
  // if y1 == y2, then it does not matter what we set here
  signed char const iy((delta_y > 0) - (delta_y < 0));
  delta_y = std::abs(delta_y) << 1;

  drawPoint(img, x1, y1, r, g, b);

  if(delta_x >= delta_y)
  {
    // error may go below zero
    int error(delta_y - (delta_x >> 1));

    while(x1 != x2)
    {
      if((error >= 0) && (error || (ix > 0)))
      {
        error -= delta_x;
        y1 += iy;
      }
      // else do nothing

      error += delta_y;
      x1 += ix;

      drawPoint(img, x1, y1, r, g, b);
    }
  }
  else
  {
    // error may go below zero
    int error(delta_x - (delta_y >> 1));

    while(y1 != y2)
    {
      if((error >= 0) && (error || (iy > 0)))
      {
        error -= delta_y;
        x1 += ix;
      }
      // else do nothing

      error += delta_x;
      y1 += iy;

      drawPoint(img, x1, y1, r, g, b);
    }
  }
}

void detectBeacons(unsigned char* transitions, unsigned char* beacons, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
  std::vector<std::pair<unsigned int, unsigned int> > beacon_points(NUM_BEACON_TYPES, std::pair<unsigned int, unsigned int>(0, 0));
  std::vector<unsigned int> beacon_counts(NUM_BEACON_TYPES, 0.0);
  for(unsigned int x = min_x; x < max_x; x++)
  {
    //loop through each column from top to bottom
    BeaconPossibilityState column_state = HAS_NOTHING;
    BeaconType type = NOT_A_BEACON;
    int beacon_bottom_idx = 0;
    for(int y = (int) max_y - 2; y >= (int) min_y + 1; --y)
    {
//      drawPoint(beacons, x, y, 0, 0, 0);
      switch(column_state)
      {
      case HAS_NOTHING:
        if(transitions[idx(x, y)] == WHITE_GREEN)
        {
          column_state = HAS_BOTTOM;
          beacon_bottom_idx = y;
        }
        break;
      case HAS_BOTTOM:
        if(transitions[idx(x, y)] == WHITE_GREEN)
        {
          column_state = HAS_BOTTOM;
          beacon_bottom_idx = y;
        }
        if(transitions[idx(x, y)] == PINK_WHITE)
        {
          column_state = FIRST_COLOR_PINK;
        }
        if(transitions[idx(x, y)] == BLUE_WHITE)
        {
          column_state = FIRST_COLOR_BLUE;
        }
        if(transitions[idx(x, y)] == YELLOW_WHITE)
        {
          column_state = FIRST_COLOR_YELLOW;
        }
        break;
      case FIRST_COLOR_PINK:
        if(transitions[idx(x, y)] == BLUE_PINK)
        {
          column_state = IS_BEACON;
          type = BLUE_PINK_BEACON;
        }
        if(transitions[idx(x, y)] == YELLOW_PINK)
        {
          column_state = IS_BEACON;
          type = YELLOW_PINK_BEACON;
        }
        break;
      case FIRST_COLOR_BLUE:
        if(transitions[idx(x, y)] == PINK_BLUE)
        {
          column_state = IS_BEACON;
          type = PINK_BLUE_BEACON;
        }
        if(transitions[idx(x, y)] == YELLOW_BLUE)
        {
          column_state = IS_BEACON;
          type = YELLOW_BLUE_BEACON;
        }
        break;
      case FIRST_COLOR_YELLOW:
        if(transitions[idx(x, y)] == BLUE_YELLOW)
        {
          column_state = IS_BEACON;
          type = BLUE_YELLOW_BEACON;
        }
        if(transitions[idx(x, y)] == PINK_YELLOW)
        {
          column_state = IS_BEACON;
          type = PINK_YELLOW_BEACON;
        }
        break;
      case IS_BEACON:
//        if(transitions[idx(x, y)] != NO_TRANSITION)
//        {
//          //TODO: better specify which transitions are bad
//          //decoy beacon
//          column_state = HAS_NOTHING;
//          type = NOT_A_BEACON;
//          beacon_bottom_idx = 0;
//        }
        break;
      default:
        break;
      }
    }

    beacon_points[type].first += x;
    beacon_points[type].second += beacon_bottom_idx;
    beacon_counts[type]++;
  }

  for(unsigned int type = 1; type < (unsigned int) NUM_BEACON_TYPES; type++)
  {
    if(beacon_counts[type] == 0)
    {
      continue;
    }

    unsigned int x = beacon_points[type].first / beacon_counts[type];
    unsigned int y = beacon_points[type].second / beacon_counts[type];
    //todo: check bounds
    switch(type)
    {
    //todo: add colors
    case PINK_YELLOW_BEACON:
      drawLine(beacons, x, y, x, y - 5, 255, 0, 0);
      drawLine(beacons, x, y - 5, x + 2, y - 3, 255, 255, 0);
      drawLine(beacons, x, y - 5, x - 2, y - 3, 255, 255 * 0.5, 255 * 0.75);
      break;
    case PINK_BLUE_BEACON:
      drawLine(beacons, x, y, x, y - 5, 255, 0, 0);
      drawLine(beacons, x, y - 5, x + 2, y - 3, 0, 0, 255);
      drawLine(beacons, x, y - 5, x - 2, y - 3, 255, 255 * 0.5, 255 * 0.75);
      break;
    case YELLOW_BLUE_BEACON:
      drawLine(beacons, x, y, x, y - 5, 255, 0, 0);
      drawLine(beacons, x, y - 5, x + 2, y - 3, 0, 0, 255);
      drawLine(beacons, x, y - 5, x - 2, y - 3, 255, 255, 0);
      break;
    case YELLOW_PINK_BEACON:
      drawLine(beacons, x, y, x, y - 5, 255, 0, 0);
      drawLine(beacons, x, y - 5, x + 2, y - 3, 255, 255 * 0.5, 255 * 0.75);
      drawLine(beacons, x, y - 5, x - 2, y - 3, 255, 255, 0);
      break;
    case BLUE_YELLOW_BEACON:
      drawLine(beacons, x, y, x, y - 5, 255, 0, 0);
      drawLine(beacons, x, y - 5, x + 2, y - 3, 255, 255, 0);
      drawLine(beacons, x, y - 5, x - 2, y - 3, 0, 0, 255);
      break;
    case BLUE_PINK_BEACON:
      drawLine(beacons, x, y, x, y - 5, 255, 0, 0);
      drawLine(beacons, x, y - 5, x + 2, y - 3, 255, 255 * 0.5, 255 * 0.75);
      drawLine(beacons, x, y - 5, x - 2, y - 3, 0, 0, 255);
      break;
    default:
      break;
    }
  }
}

void generateTransitionVis(unsigned char* transitions, unsigned char* vis_bgr, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
  for(unsigned int x = min_x; x < max_x; x++) //todo: allow full bounds
  {
    for(unsigned int y = min_y + 1; y < max_y - 1; y++)
    {
      switch(transitions[idx(x, y)])
      {
      case PINK_BLUE:
        vis_bgr[3 * idx(x, y - 1) + 0] = 255 * 0.75;
        vis_bgr[3 * idx(x, y - 1) + 1] = 255 * 0.5;
        vis_bgr[3 * idx(x, y - 1) + 2] = 255;

        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      case PINK_YELLOW:
        vis_bgr[3 * idx(x, y - 1) + 0] = 255 * 0.75;
        vis_bgr[3 * idx(x, y - 1) + 1] = 255 * 0.5;
        vis_bgr[3 * idx(x, y - 1) + 2] = 255;

        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case PINK_WHITE:
        vis_bgr[3 * idx(x, y - 1) + 0] = 255 * 0.75;
        vis_bgr[3 * idx(x, y - 1) + 1] = 255 * 0.5;
        vis_bgr[3 * idx(x, y - 1) + 2] = 255;

        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case BLUE_PINK:
        vis_bgr[3 * idx(x, y - 1) + 0] = 255;
        vis_bgr[3 * idx(x, y - 1) + 1] = 255;
        vis_bgr[3 * idx(x, y - 1) + 2] = 255;

        vis_bgr[3 * idx(x, y) + 0] = 255 * 0.75;
        vis_bgr[3 * idx(x, y) + 1] = 255 * 0.5;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case BLUE_YELLOW:
        vis_bgr[3 * idx(x, y - 1) + 0] = 255;
        vis_bgr[3 * idx(x, y - 1) + 1] = 0;
        vis_bgr[3 * idx(x, y - 1) + 2] = 0;

        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case BLUE_WHITE:
        vis_bgr[3 * idx(x, y - 1) + 0] = 255;
        vis_bgr[3 * idx(x, y - 1) + 1] = 0;
        vis_bgr[3 * idx(x, y - 1) + 2] = 0;

        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case YELLOW_BLUE:
        vis_bgr[3 * idx(x, y - 1) + 0] = 0;
        vis_bgr[3 * idx(x, y - 1) + 1] = 255;
        vis_bgr[3 * idx(x, y - 1) + 2] = 255;

        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      case YELLOW_PINK:
        vis_bgr[3 * idx(x, y - 1) + 0] = 0;
        vis_bgr[3 * idx(x, y - 1) + 1] = 255;
        vis_bgr[3 * idx(x, y - 1) + 2] = 255;

        vis_bgr[3 * idx(x, y) + 0] = 255 * 0.75;
        vis_bgr[3 * idx(x, y) + 1] = 255 * 0.5;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case YELLOW_WHITE:
        vis_bgr[3 * idx(x, y - 1) + 0] = 0;
        vis_bgr[3 * idx(x, y - 1) + 1] = 255;
        vis_bgr[3 * idx(x, y - 1) + 2] = 255;

        vis_bgr[3 * idx(x, y) + 0] = 255;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 255;
        break;
      case WHITE_GREEN:
        vis_bgr[3 * idx(x, y - 1) + 0] = 255;
        vis_bgr[3 * idx(x, y - 1) + 1] = 255;
        vis_bgr[3 * idx(x, y - 1) + 2] = 255;

        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 255;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      default:
        vis_bgr[3 * idx(x, y) + 0] = 0;
        vis_bgr[3 * idx(x, y) + 1] = 0;
        vis_bgr[3 * idx(x, y) + 2] = 0;
        break;
      }
    }
  }
}

//assumes single channel img, smoothed already allocated
void smooth(unsigned char* img, unsigned char* smoothed, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
{
  for(unsigned int x = min_x + 1; x < max_x - 1; x++) //todo: allow full bounds
  {
    for(unsigned int y = min_y + 1; y < max_y - 1; y++)
    {
      smoothed[idx(x, y)] = ((int) img[idx(x - 1, y - 1)] + (int) img[idx(x - 1, y)] + (int) img[idx(x - 1, y + 1)] + (int) img[idx(x, y - 1)] + (int) img[idx(x, y)] + (int) img[idx(x, y + 1)] + (int) img[idx(x + 1, y - 1)] + (int) img[idx(x + 1, y)] + (int) img[idx(x + 1, y + 1)]) / 9;
    }
  }
}

int main(int argc, char **argv)
{
  if(argc != 2)
  {
    cout << "provide filename" << endl;
    return -1;
  }

  string filename = argv[1];
  string extension = filename.substr(filename.rfind("."));

  Mat bgr;
  if(extension == ".yuv")
  {
    if(!loadWeirdNaoFormat(argv[1], bgr, 320, 240))
    {
      std::cerr << "Failed to load image!" << endl;
      return 0;
    }
  }
  else
  {
    bgr = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    if(!bgr.data)
    {
      std::cerr << "Failed to load image!" << endl;
      return -1;
    }
  }

  cv::resize(bgr, bgr, Size(320, 240));

  namedWindow("raw", WINDOW_NORMAL);
  imshow("raw", bgr);

  Mat yuv, clusters, full_clusters, transitions, vis, full_vis, transition_vis, beacon_vis;
  cv::cvtColor(bgr, yuv, CV_BGR2YCrCb);

  Mat rgb, custom_rgb;
  cv::cvtColor(yuv, rgb, CV_YCrCb2RGB);
  custom_rgb = yuv.clone();
  yuvImage2rgb(yuv.data, custom_rgb.data, 0, 0, bgr.cols, bgr.rows);
  cv::cvtColor(rgb, rgb, CV_RGB2BGR);
  cv::cvtColor(custom_rgb, custom_rgb, CV_RGB2BGR);
  namedWindow("rgb", WINDOW_NORMAL);
  imshow("rgb", rgb);
  namedWindow("custom_rgb", WINDOW_NORMAL);
  imshow("custom_rgb", custom_rgb);

  Mat hsv, hsv_custom;
  hsv_custom = yuv.clone();
  cv::cvtColor(rgb, rgb, CV_BGR2RGB);
  yuvImage2hsv(yuv.data, hsv_custom.data, 0, 0, bgr.cols, bgr.rows);
  cv::cvtColor(rgb, hsv, CV_RGB2HSV);
  clusters = hsv_custom.clone();
  full_clusters = hsv_custom.clone();
  vis = hsv_custom.clone();
  full_vis = hsv_custom.clone();
  transitions = hsv_custom.clone();
  transition_vis = hsv_custom.clone();
  beacon_vis = bgr.clone();
  hsvCluster(hsv_custom.data, full_clusters.data, clusters.data, 0, 0, hsv_custom.cols, hsv_custom.rows);
  detectTransitions(clusters.data, transitions.data, 0, 0, hsv_custom.cols, hsv_custom.rows);
  detectBeacons(transitions.data, beacon_vis.data, 0, 0, hsv_custom.cols, hsv_custom.rows);

  generateFullClusterVis(full_clusters.data, full_vis.data, 0, 0, hsv_custom.cols, hsv_custom.rows);
  generateClusterVis(clusters.data, vis.data, 0, 0, hsv_custom.cols, hsv_custom.rows);
  generateTransitionVis(transitions.data, transition_vis.data, 0, 0, hsv_custom.cols, hsv_custom.rows);
  cv::cvtColor(hsv, hsv, CV_HSV2BGR);
  cv::cvtColor(hsv_custom, hsv_custom, CV_HSV2BGR);

  namedWindow("hsv", WINDOW_NORMAL);
  imshow("hsv", hsv);
  namedWindow("hsv_custom", WINDOW_NORMAL);
  imshow("hsv_custom", hsv_custom);
  namedWindow("clustered", WINDOW_NORMAL);
  imshow("clustered", vis);
  namedWindow("transitions", WINDOW_NORMAL);
  imshow("transitions", transition_vis);
  namedWindow("beacon_vis", WINDOW_NORMAL);
  imshow("beacon_vis", beacon_vis);

  waitKey(0); // Wait for a keystroke in the window
  return 0;
}
