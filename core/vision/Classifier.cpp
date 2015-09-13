#include "Classifier.h"
#include <iostream>

using namespace cv;

Classifier::Classifier(const VisionBlocks& vblocks, const VisionParams& vparams, const ImageParams& iparams, const Camera::Type& camera) :
    vblocks_(vblocks), vparams_(vparams), iparams_(iparams), camera_(camera), initialized_(false) {
  segImg_ = new unsigned char[iparams.size];
  segImgLocal_ = segImg_;
  setImagePointers();
}

Classifier::~Classifier() {
  delete [] segImgLocal_;
}

bool Classifier::setImagePointers() {
  if(vblocks_.image == NULL) {
    printf("No image block loaded! Classification failed.\n");
    return false;
  }
  if(vblocks_.robot_vision == NULL) {
    printf("No vision block loaded! Classification failed.\n");
    return false;
  }
  bool imageLoaded = vblocks_.image->loaded_;
  if(camera_ == Camera::TOP) {
    #ifdef TOOL
    if(imageLoaded) {
    #endif
      vblocks_.robot_vision->setSegImgTop(segImg_);
      img_ = vblocks_.image->getImgTop();
      if(!img_) return false;
    #ifdef TOOL
    } else if(vblocks_.robot_vision->loaded_) {
      segImg_ = vblocks_.robot_vision->getSegImgTop();
    }
    #endif
  }
  else {
    #ifdef TOOL
    if(imageLoaded) {
    #endif
      vblocks_.robot_vision->setSegImgBottom(segImg_);
      img_ = vblocks_.image->getImgBottom();
      if(!img_) return false;
    #ifdef TOOL
    } else if(vblocks_.robot_vision->loaded_) {
      segImg_ = vblocks_.robot_vision->getSegImgBottom();
    }
    #endif
  }
  if(!initialized_) {
    #ifdef TOOL
    if(imageLoaded)
    #endif
    memset(segImg_, c_UNDEFINED, sizeof(unsigned char) * iparams_.size);
    initialized_ = true;
  }
  return true;
}

bool Classifier::classifyImage(unsigned char *colorTable) {
  if(!setImagePointers()) return false;
  FocusArea area(0, 0, iparams_.width - 1, iparams_.height - 1);
  classifyImage(area, colorTable);
  return true;
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

#define CLIP(X) ( (X) > 255 ? 255 : (X) < 0 ? 0 : X)
inline RGB yuv2rgb(YCrCb in)
{
  return RGB(CLIP(in.y + (91881 * in.cr >> 16) - 179), CLIP(in.y - ((22544 * in.cb + 46793 * in.cr) >> 16) + 135), CLIP(in.y + (116129 * in.cb >> 16) - 226));
}

inline HSV rgb2hsv(RGB in)
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

void Classifier::classifyImage(const FocusArea& area, unsigned char* colorTable){
  bool imageLoaded = vblocks_.image->loaded_;
  if(!imageLoaded) {
    visionLog(20, "Classifying with no raw image");
  }
  int vstep = 1 << 1;
  int hstep = 1 << 2;

  // static bool have_table = false;
  // static HSV ycrcbhsv_table[256][256][256];
  // if(!have_table)
  // {
  //   have_table = true;
  //   for(unsigned int y = 0; y < 256; y++) //should stop at 220
  //     for(unsigned int cr = 0; cr < 256; cr++) //should stop at 235
  //       for(unsigned int cb = 0; cb < 256; cb++) //should stop at 235
  //       {
  //         RGB rgb = yuv2rgb(YCrCb(y, cr, cb));
  //         ycrcbhsv_table[y][cr][cb] = rgb2hsv(rgb);
  //       }
  //   std::cerr << "Table generated!" << std::endl;
  // }
  for (int y = area.y1; y <= area.y2; y += vstep) 
  {
    for(int x = area.x1; x <= area.x2; x += hstep) 
    {
      if(x >= 320 || x < 0 || y >= 240 || y < 0)
      {
        printf("BAD ITERATION PIXEL\n");
        continue;
      }

      int y, u, v;
      printf("getting pixel x=%d,y=%d\n", x, y);
      ColorTableMethods::xy2yuv(img_, x, y, iparams_.width, y, u, v);
      printf("yuv=%d,%d,%d\n",y,u,v);
      HSV col = rgb2hsv(yuv2rgb(YCrCb(y, u, v)));
      printf("hsv=%d,%d,%d\n",col.h,col.s,col.v);

      // if(col.v < 30) //black
      // {
      //   segImg_[iparams_.width * y + x] = c_UNDEFINED;
      // }
      // else if(col.s < 85) //not colorful
      // {
      //   if(col.v < 3 * 255 / 8)
      //   {
      //     segImg_[iparams_.width * y + x] = c_UNDEFINED;
      //   }
      //   else if(col.v < 160)
      //   {
      //     segImg_[iparams_.width * y + x] = c_ROBOT_WHITE;
      //   }
      //   else
      //   {
      //     segImg_[iparams_.width * y + x] = c_WHITE;
      //   }
      // }
      // else //colored
      // {
      //   if(col.h < 15 / 2 || col.h >= 345 / 2)
      //   {
      //     //red
      //     segImg_[iparams_.width * y + x] = c_PINK;
      //   }
      //   else if(col.h < 45 / 2)
      //   {
      //     //yellow-red
      //     segImg_[iparams_.width * y + x] = c_ORANGE;
      //   }
      //   else if(col.h < 75 / 2)
      //   {
      //     //yellow
      //     segImg_[iparams_.width * y + x] = c_YELLOW;
      //   }
      //   else if(col.h < 105 / 2)
      //   {
      //     //green-yellow
      //     segImg_[iparams_.width * y + x] = c_YELLOW;
      //   }
      //   else if(col.h < 135 / 2)
      //   {
      //     //green
      //     segImg_[iparams_.width * y + x] = c_FIELD_GREEN;
      //   }
      //   else if(col.h < 165 / 2)
      //   {
      //     //cyan-green
      //     segImg_[iparams_.width * y + x] = c_FIELD_GREEN;
      //   }
      //   else if(col.h < 195 / 2)
      //   {
      //     //cyan
      //     segImg_[iparams_.width * y + x] = c_BLUE;
      //   }
      //   else if(col.h < 225 / 2)
      //   {
      //     //blue-cyan
      //     segImg_[iparams_.width * y + x] = c_BLUE;
      //   }
      //   else if(col.h < 255 / 2)
      //   {
      //     //blue
      //     segImg_[iparams_.width * y + x] = c_BLUE;
      //   }
      //   else if(col.h < 285 / 2)
      //   {
      //     //magenta-blue
      //     segImg_[iparams_.width * y + x] = c_BLUE;
      //   }
      //   else if(col.h < 315 / 2)
      //   {
      //     //magenta
      //     segImg_[iparams_.width * y + x] = c_PINK;
      //   }
      //   else if(col.h < 345 / 2)
      //   {
      //     //red-magenta
      //     segImg_[iparams_.width * y + x] = c_PINK;
      //   }
      // }
    }
  }

  // colorTable_ = colorTable;
  // for (int y = area.y1; y <= area.y2; y += vstep) {
  //   for(int x = area.x1; x <= area.x2; x += hstep) {
  //     auto c = ColorTableMethods::xy2color(img_, colorTable, x, y, iparams_.width);
  //     segImg_[iparams_.width * y + x] = c;
  //   }
  // }
}
