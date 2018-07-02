#pragma once

#include <deque>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm>

namespace corner_event_detector
{

class ToBeTracked 
{
public:
  ToBeTracked(void);
  virtual ~ToBeTracked(void);
  bool ClassifyEvent(int x, int y);
  bool ClassifyCorner(int x, int y);
  void Update(int x, int y);

private:
  Eigen::MatrixXd tracked_corners;
  // Declare some static variables
  static const int ncorners = 10;
  static const int center_x = 150;
  static const int center_y = 40;
  static const int radius = 10;
  static const int window_size = 5; //Actually half of the window
  static constexpr double pi = 3.1415;
  int corner_offset[ncorners];
  int object_center[2];
};

} // namespace
