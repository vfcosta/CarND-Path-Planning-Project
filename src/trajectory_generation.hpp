#include <vector>
#include "spline.h"

using namespace std;

class TrajectoryGeneration {

public:
  TrajectoryGeneration(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);
  vector<vector<double>> generate(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> previous_path_x, vector<double> previous_path_y);
  vector<double> JMT(vector<double> start, vector<double> end, double time);
  void jmtTrajectory(double s, double goal_s, double car_speed, double goal_speed, double d, double goal_d, double time, vector<double> &next_x_vals, vector<double> &next_y_vals);
  void keepLaneTrajectory(vector<double> frenet, int next_waypoint, double car_speed, vector<double> &next_x_vals, vector<double> &next_y_vals);
  void changeLaneTrajectory(vector<double> frenet, int next_waypoint, double car_speed, vector<double> &next_x_vals, vector<double> &next_y_vals);

private:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  tk::spline spline_waypoints_x;
  tk::spline spline_waypoints_y;
  tk::spline spline_waypoints_dx;
  tk::spline spline_waypoints_dy;
  double lane_width = 4;
};
