#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE 
{
	transitions = new unsigned char[320*240];
}

void BeaconDetector::findBeacons(unsigned char* img) 
{
	detectTransitions(img);
	detectBeacons(img);
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

void BeaconDetector::detectTransitions(unsigned char* img, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
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

void drawPoint(unsigned char* img, int x, int y, Color c)
{
  img[idx(x, y) + 0] = c;
}

void drawLine(unsigned char* img, int x1, int y1, int x2, int y2, Color c)
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

void BeaconDetector::detectBeacons(unsigned char* img, unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y)
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


		WorldObject* beacon = &vblocks_.world_object->objects_[type + (WO_BEACON_BLUE_YELLOW - BLUE_YELLOW_BEACON)];
	  beacon->imageCenterX = x;
	  beacon->imageCenterY = y;
	  Position p = cmatrix_.getWorldPosition(x, y);
	  beacon->visionBearing = cmatrix_.bearing(p);
	  beacon->visionElevation = cmatrix_.elevation(p);
	  beacon->visionDistance = cmatrix_.groundDistance(p);
	  beacon->seen = true;

    //todo: check bounds
    unsigned char arrow_height = 5;
    unsigned char tip_size = 2;
    switch(type)
    {
    //todo: add colors
    case PINK_YELLOW_BEACON:
      drawLine(img, x, y, x, y - arrow_height, c_YELLOW);
      drawLine(img, x, y - arrow_height, x + tip_size, y - arrow_height + tip_size, c_PINK);
      drawLine(img, x, y - arrow_height, x - tip_size, y - arrow_height + tip_size, c_PINK);
      break;
    case PINK_BLUE_BEACON:
      drawLine(img, x, y, x, y - arrow_height, c_BLUE);
      drawLine(img, x, y - arrow_height, x + tip_size, y - arrow_height + tip_size, c_PINK);
      drawLine(img, x, y - arrow_height, x - tip_size, y - arrow_height + tip_size, c_PINK);
      break;
    case YELLOW_BLUE_BEACON:
      drawLine(img, x, y, x, y - arrow_height, c_BLUE);
      drawLine(img, x, y - arrow_height, x + tip_size, y - arrow_height + tip_size, c_YELLOW);
      drawLine(img, x, y - arrow_height, x - tip_size, y - arrow_height + tip_size, c_YELLOW);
      break;
    case YELLOW_PINK_BEACON:
      drawLine(img, x, y, x, y - arrow_height, c_PINK);
      drawLine(img, x, y - arrow_height, x + tip_size, y - arrow_height + tip_size, c_YELLOW);
      drawLine(img, x, y - arrow_height, x - tip_size, y - arrow_height + tip_size, c_YELLOW);
      break;
    case BLUE_YELLOW_BEACON:
      drawLine(img, x, y, x, y - arrow_height, c_YELLOW);
      drawLine(img, x, y - arrow_height, x + tip_size, y - arrow_height + tip_size, c_BLUE);
      drawLine(img, x, y - arrow_height, x - tip_size, y - arrow_height + tip_size, c_BLUE);
      break;
    case BLUE_PINK_BEACON:
      drawLine(img, x, y, x, y - arrow_height, c_PINK);
      drawLine(img, x, y - arrow_height, x + tip_size, y - arrow_height + tip_size, c_BLUE);
      drawLine(img, x, y - arrow_height, x - tip_size, y - arrow_height + tip_size, c_BLUE);
      break;
    default:
      break;
    }
  }
}

