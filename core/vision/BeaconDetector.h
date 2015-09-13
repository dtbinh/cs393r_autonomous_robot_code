#pragma once

#include <vision/ObjectDetector.h>


inline unsigned int idx(unsigned int x, unsigned int y)
{
  return 320 * y + x;
}

class TextLogger;

/// @ingroup vision
class BeaconDetector : public ObjectDetector {

 public:
  BeaconDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void findBeacons(unsigned char* img);
 private:
  TextLogger* textlogger;

  void detectTransitions(unsigned char* img, unsigned int min_x=0, unsigned int min_y=0, unsigned int max_x=320, unsigned int max_y=240);
  void detectBeacons(unsigned char* img, unsigned int min_x=0, unsigned int min_y=0, unsigned int max_x=320, unsigned int max_y=240);

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
    WHITE_GREEN,
    WHITE_PINK,
    WHITE_BLUE,
    WHITE_YELLOW,
    GREEN_PINK,
    GREEN_BLUE,
    GREEN_YELLOW
  };

  enum BeaconPossibilityState
  {
    HAS_NOTHING,
    HAS_BOTTOM,
    FIRST_COLOR_PINK,
    FIRST_COLOR_BLUE,
    FIRST_COLOR_YELLOW,
    SECOND_COLOR_PINK,
    SECOND_COLOR_BLUE,
    SECOND_COLOR_YELLOW,
    IS_BEACON
  };

  enum BeaconType
  {
    NOT_A_BEACON,
    BLUE_YELLOW_BEACON,
    YELLOW_BLUE_BEACON,
    BLUE_PINK_BEACON,
    PINK_BLUE_BEACON,
    PINK_YELLOW_BEACON,
    YELLOW_PINK_BEACON,
    NUM_BEACON_TYPES
  };

  unsigned char* transitions;
};
