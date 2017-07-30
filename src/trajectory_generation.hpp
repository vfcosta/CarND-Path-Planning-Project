#include <vector>
#include "frenet.hpp"

using namespace std;

class TrajectoryGeneration {

public:
  TrajectoryGeneration(Frenet frenet);
  vector<vector<double>> generate(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> previous_path_x, vector<double> previous_path_y);
  vector<double> JMT(vector<double> start, vector<double> end, double time);
  void jmtTrajectory(double s, double goal_s, double car_speed, double goal_speed, double d, double goal_d, double time, vector<double> &next_x_vals, vector<double> &next_y_vals);
  void keepLaneTrajectory(vector<double> frenet, vector<double> nextSd, double car_speed, vector<double> &next_x_vals, vector<double> &next_y_vals);
  void changeLaneTrajectory(vector<double> frenet, vector<double> nextSd, double car_speed, vector<double> &next_x_vals, vector<double> &next_y_vals);

private:
  double lane_width = 4;
  Frenet frenet;
};
