#ifndef UTILS_H
#define UTILS_H

inline unsigned int idx(unsigned int x, unsigned int y)
{
  return 320 * y + x;
}

inline void drawPoint(unsigned char* img, int x, int y, Color c)
{
  if(x < 0 || y < 0 || x > 320 || y > 240)
  {
    return;
  }
  img[idx(x, y) + 0] = c;
}

inline void drawLine(unsigned char* img, int x1, int y1, int x2, int y2, Color c)
{
  int delta_x(x2 - x1);
  // if x1 == x2, then it does not matter what we set here
  signed char const ix((delta_x > 0) - (delta_x < 0));
  delta_x = std::abs(delta_x) << 1;

  int delta_y(y2 - y1);
  // if y1 == y2, then it does not matter what we set here
  signed char const iy((delta_y > 0) - (delta_y < 0));
  delta_y = std::abs(delta_y) << 1;

  drawPoint(img, x1, y1, c);

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

      drawPoint(img, x1, y1, c);
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

      drawPoint(img, x1, y1, c);
    }
  }
}

inline std::pair<Color, unsigned int> mode(std::vector<Color> colors, unsigned int max)
{
  std::vector<int> histogram(max, 0);
  for(unsigned int i = 0; i < colors.size(); ++i)
    ++histogram[colors[i]];
  Color max_value = (Color) (std::max_element(histogram.begin(), histogram.end()) - histogram.begin());
  unsigned int count = histogram[max_value];
  return std::pair<Color, unsigned int>(max_value, count);
}

#endif //UTILS_H