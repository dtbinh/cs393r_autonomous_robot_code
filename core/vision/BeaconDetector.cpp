#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE 
{
	transitions = new unsigned char[320*240];

  //set no transition everywhere
  for(unsigned int x = 0; x < 320; x++)
  {
    for(unsigned int y = 0; y < 240; y++)
    {
      transitions[idx(x, y)] = NO_TRANSITION;
    }
  }
}

//returns true if b1 is stacked on top of and touching b2
inline bool stacked(MergeBlob::Blob* b1, MergeBlob::Blob* b2, unsigned int touch_threshold = 5)
{
  unsigned int b1_y_min = b1->boundingbox_vertex_y;
  unsigned int b1_y_max = b1_y_min + b1->boundingbox_height;
  unsigned int b1_x_min = b1->boundingbox_vertex_x;
  unsigned int b1_x_max = b1_x_min + b1->boundingbox_length;
  unsigned int b2_y_min = b2->boundingbox_vertex_y;
  unsigned int b2_y_max = b2_y_min + b1->boundingbox_height;
  unsigned int b2_x_min = b2->boundingbox_vertex_x;
  unsigned int b2_x_max = b2_x_min + b1->boundingbox_length;

  //test y touching
  if(b1_y_max < (b2_y_min - touch_threshold))
  {
    return false;
  }

  //test x touching
  if((b1_x_min > (b2_x_max + touch_threshold)) || (b2_x_min > (b1_x_max + touch_threshold)))
  { 
    return false;
  }
}

bool checkColorChain(MergeBlob::Blob* blob, Color last_color, WorldObjectType& beacon, std::vector<MergeBlob::Blob*>& blobs)
{
  if(blobs.size() == 0) 
  {
    //first in the chain must be white
    if(blob->color != c_WHITE)
    {
      return false;
    }
  }
  else if(blobs.size() == 1)
  {
    if(!(blob->color == c_BLUE || blob->color == c_YELLOW || blob->color == c_PINK))
    {
      return false;
    }
  }
  else if(blobs.size() == 2)
  {
    if(!(blob->color == c_BLUE || blob->color == c_YELLOW || blob->color == c_PINK))
    {
      return false;
    }

    //have two (and not three) valid colors stacked on top of white!
    if(blob->blobs_connected_to_top.size() == 0)
    {
      if(blob->color == c_PINK && last_color == c_BLUE)
      {
        beacon = WO_BEACON_PINK_BLUE;
      }
      else if(blob->color == c_PINK && last_color == c_YELLOW)
      {
        beacon = WO_BEACON_PINK_YELLOW;
      }
      else if(blob->color == c_BLUE && last_color == c_PINK)
      {
        beacon = WO_BEACON_BLUE_PINK;
      }
      else if(blob->color == c_BLUE && last_color == c_YELLOW)
      {
        beacon = WO_BEACON_BLUE_YELLOW;
      }
      else if(blob->color == c_YELLOW && last_color == c_BLUE)
      {
        beacon = WO_BEACON_YELLOW_BLUE;
      }
      else if(blob->color == c_YELLOW && last_color == c_PINK)
      {
        beacon = WO_BEACON_YELLOW_PINK;
      }
      return true;
    }
  }
  else
  {
    //too many colors 
    return false;
  }

  blobs.push_back(blob);
  last_color = (Color) blob->color;
  for(unsigned int i = 0; i < blob->blobs_connected_to_top.size(); i++)
  {
    if(checkColorChain(blob->blobs_connected_to_top[i], last_color, beacon, blobs))
    {
      return true;
    }
  }

  //no valid chains! clean up and try again.
  blobs.pop_back();
  return false;
}

void BeaconDetector::findBeacons(MergeBlob* mb)
{
  std::vector<MergeBlob::Blob*> relevant_blobs;
  unsigned int min_blob_size = 20;
  for(int i = 0; i < mb->get_blob_number(); i++)
  {
    unsigned int size = mb->blob[i].boundingbox_length * mb->blob[i].boundingbox_height;
    double ar = mb->blob[i].boundingbox_length / mb->blob[i].boundingbox_height;
    if(size > min_blob_size && ar > 0.9 && ar < 1.5 && (mb->blob[i].color == c_WHITE || mb->blob[i].color == c_YELLOW || mb->blob[i].color == c_BLUE || mb->blob[i].color == c_PINK))
    {
      relevant_blobs.push_back(&mb->blob[i]);
    }
  }

  //populate blob connections
  for(unsigned int i = 0; i < relevant_blobs.size(); i++)
  {
    for(unsigned int j = 0; j < relevant_blobs.size(); j++)
    {
      if(stacked(relevant_blobs[i], relevant_blobs[j]))
      {
        relevant_blobs[j]->blobs_connected_to_top.push_back(relevant_blobs[i]);
      }
    }
  }

  //find beacons
  for(unsigned int i = 0; i < relevant_blobs.size(); i++)
  {
    Color last_color;
    WorldObjectType beacon_type;
    std::vector<MergeBlob::Blob*> blobs;
    if(checkColorChain(relevant_blobs[i], last_color, beacon_type, blobs))
    {
      //get bottom of the beacon
      unsigned int x = (blobs[0]->centroid_x + blobs[1]->centroid_x + blobs[2]->centroid_x) / 3;
      unsigned int y = blobs[0]->boundingbox_vertex_y + blobs[0]->boundingbox_height;

      //populate beacon
      WorldObject* beacon = &vblocks_.world_object->objects_[beacon_type];
      beacon->imageCenterX = x;
      beacon->imageCenterY = y;
      Position p = cmatrix_.getWorldPosition(x, y);
      beacon->visionBearing = cmatrix_.bearing(p);
      beacon->visionElevation = cmatrix_.elevation(p);
      beacon->visionDistance = cmatrix_.groundDistance(p);
      beacon->seen = true;
    }
  }
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
      else if(upper_color.first == c_WHITE && lower_color.first == c_PINK)
      {
        transitions[idx(x, y)] = WHITE_PINK;
      }
      else if(upper_color.first == c_WHITE && lower_color.first == c_BLUE)
      {
        transitions[idx(x, y)] = WHITE_BLUE;
      }
      else if(upper_color.first == c_WHITE && lower_color.first == c_YELLOW)
      {
        transitions[idx(x, y)] = WHITE_YELLOW;
      }
      else if(upper_color.first == c_FIELD_GREEN && lower_color.first == c_PINK)
      {
        transitions[idx(x, y)] = GREEN_PINK;
      }
      else if(upper_color.first == c_FIELD_GREEN && lower_color.first == c_BLUE)
      {
        transitions[idx(x, y)] = GREEN_BLUE;
      }
      else if(upper_color.first == c_FIELD_GREEN && lower_color.first == c_YELLOW)
      {
        transitions[idx(x, y)] = GREEN_YELLOW;
      }
      else
      {
        transitions[idx(x, y)] = NO_TRANSITION;
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
          column_state = SECOND_COLOR_BLUE;
          type = BLUE_PINK_BEACON;
        }
        if(transitions[idx(x, y)] == YELLOW_PINK)
        {
          column_state = SECOND_COLOR_YELLOW;
          type = YELLOW_PINK_BEACON;
        }
        break;
      case FIRST_COLOR_BLUE:
        if(transitions[idx(x, y)] == PINK_BLUE)
        {
          column_state = SECOND_COLOR_PINK;
          type = PINK_BLUE_BEACON;
        }
        if(transitions[idx(x, y)] == YELLOW_BLUE)
        {
          column_state = SECOND_COLOR_YELLOW;
          type = YELLOW_BLUE_BEACON;
        }
        break;
      case FIRST_COLOR_YELLOW:
        if(transitions[idx(x, y)] == BLUE_YELLOW)
        {
          column_state = SECOND_COLOR_BLUE;
          type = BLUE_YELLOW_BEACON;
        }
        if(transitions[idx(x, y)] == PINK_YELLOW)
        {
          column_state = SECOND_COLOR_PINK;
          type = PINK_YELLOW_BEACON;
        }
        break;
      case SECOND_COLOR_PINK:
       if(transitions[idx(x, y)] == GREEN_PINK || transitions[idx(x, y)] == WHITE_PINK)
       {
          column_state = IS_BEACON;
       }
       else if(!(type==PINK_YELLOW_BEACON && transitions[idx(x, y)]==PINK_YELLOW) && !(type==PINK_BLUE_BEACON && transitions[idx(x, y)]==PINK_BLUE) && transitions[idx(x, y)] != NO_TRANSITION)
       {
         //decoy beacon
         column_state = HAS_NOTHING;
         type = NOT_A_BEACON;
         beacon_bottom_idx = 0;
       }
       break;
      case SECOND_COLOR_BLUE:
       if(transitions[idx(x, y)] == GREEN_BLUE || transitions[idx(x, y)] == WHITE_BLUE)
       {
          column_state = IS_BEACON;
       }
       else if(!(type==BLUE_YELLOW_BEACON && transitions[idx(x, y)]==BLUE_YELLOW) && !(type==BLUE_PINK_BEACON && transitions[idx(x, y)]==BLUE_PINK) && transitions[idx(x, y)] != NO_TRANSITION)
       {
         //decoy beacon
         column_state = HAS_NOTHING;
         type = NOT_A_BEACON;
         beacon_bottom_idx = 0;
       }
       break;
      case SECOND_COLOR_YELLOW:
       if(transitions[idx(x, y)] == GREEN_YELLOW || transitions[idx(x, y)] == WHITE_YELLOW)
       {
          column_state = IS_BEACON;
       }
       else if(!(type==YELLOW_PINK_BEACON && transitions[idx(x, y)]==YELLOW_PINK) && !(type==YELLOW_BLUE && transitions[idx(x, y)]==YELLOW_BLUE) && transitions[idx(x, y)] != NO_TRANSITION)
       {
         //decoy beacon
         column_state = HAS_NOTHING;
         type = NOT_A_BEACON;
         beacon_bottom_idx = 0;
       }
       break;
      case IS_BEACON:
        break;
      default:
        break;
      }
    }

    beacon_points[type].first += x;
    beacon_points[type].second += beacon_bottom_idx;
    beacon_counts[type]++;
  }

  unsigned int min_beacon_count = 4;
  for(unsigned int type = 1; type < (unsigned int) NUM_BEACON_TYPES; type++)
  {
    if(beacon_counts[type] < min_beacon_count)
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

