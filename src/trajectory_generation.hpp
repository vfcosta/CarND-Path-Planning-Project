#include <vector>
#include "frenet.hpp"

using namespace std;

class TrajectoryGeneration {

public:
  TrajectoryGeneration(Frenet frenet);
  vector<vector<double>> generate(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double goal_d, double goal_speed, vector<double> previous_path_x, vector<double> previous_path_y);
  vector<double> JMT(vector<double> start, vector<double> end, double time);
  void jmtTrajectory(double s, double goal_s, double car_speed, double goal_speed, double d, double goal_d, double time, vector<double> &next_x_vals, vector<double> &next_y_vals);
  double trajectoryCost(vector<double> s_coeff, vector<double> d_coeff, double t);

private:
  double delay = 0.02;
  double lane_width = 4;
  Frenet frenet;
  vector<double> previous_path_s;
  vector<double> previous_path_d;

  double maxAccelCost(vector<double> s_coeff, vector<double> d_coeff, double t);
  double maxSpeedCost(vector<double> s_coeff, vector<double> d_coeff, double t);
  vector<vector<double>> selectBest(double s, double goal_s, double car_speed, double goal_speed, double d, double goal_d, double &time);
};
