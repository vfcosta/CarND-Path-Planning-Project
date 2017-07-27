#include "trajectory_generation.hpp"
#include "utils.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"

using Eigen::MatrixXd;
using Eigen::VectorXd;

TrajectoryGeneration::TrajectoryGeneration(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy) {
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
  this->map_waypoints_dx = map_waypoints_dx;
  this->map_waypoints_dy = map_waypoints_dy;
  this->spline_waypoints_x.set_points(map_waypoints_s, map_waypoints_x);
  this->spline_waypoints_y.set_points(map_waypoints_s, map_waypoints_y);
  this->spline_waypoints_dx.set_points(map_waypoints_s, map_waypoints_dx);
  this->spline_waypoints_dy.set_points(map_waypoints_s, map_waypoints_dy);
}

vector<double> TrajectoryGeneration::JMT(vector<double> start, vector<double> end, double T) {
  double alpha0 = start[0];
  double alpha1 = start[1];
  double alpha2 = start[2]/2.0;
    
  MatrixXd A(3,3);
  A << pow(T, 3),   pow(T, 4),    pow(T, 5),
        3*pow(T, 2), 4*pow(T, 3),  5*pow(T, 4),
        6*T,         12*pow(T, 2), 20*pow(T, 3);
  VectorXd b(3);
  b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T*T),
        end[1] - (start[1] + start[2]*T),
        end[2] - start[2];
  
  MatrixXd Ai = A.inverse();
	MatrixXd C = Ai*b;
  return {alpha0, alpha1, alpha2, C.data()[0], C.data()[1], C.data()[2]};
}

vector<vector<double>> TrajectoryGeneration::generate(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> previous_path_x, vector<double> previous_path_y) {
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double pos_x;
  double pos_y;
  double angle;
  int path_size = previous_path_x.size();
  // if (path_size > 50) path_size = 50;

  for(int i = 0; i < path_size; i++)
  {
     next_x_vals.push_back(previous_path_x[i]);
     next_y_vals.push_back(previous_path_y[i]);
  }

  if(path_size == 0)
  {
     pos_x = car_x;
     pos_y = car_y;
     angle = deg2rad(car_yaw);
  }
  else
  {
     pos_x = previous_path_x[path_size-1];
     pos_y = previous_path_y[path_size-1];

     double pos_x2 = previous_path_x[path_size-2];
     double pos_y2 = previous_path_y[path_size-2];
     angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
  }

  auto freenet = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
  int next_waypoint = NextWaypoint(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
  double dist_inc = this->map_waypoints_s[next_waypoint] - freenet[0];
  // for(int i = 0; i < 1-path_size; i++)
  int i = 0;
  if (path_size < 200)
  {
    double s = freenet[0]+0.01*car_speed;
    double goal_s = s+(dist_inc*(i+1));
    double d = freenet[1];
    cout << "s: " << s << " d: " << d << " speed: " << car_speed << " angle: " << angle << endl;
    int lane = 1;
    double lane_center = this->lane_width/2 + lane*this->lane_width;
    // TODO: consider X,Y as target and apply JMT to reach it
    // double x = this->spline_waypoints_x(s) + this->spline_waypoints_dx(s)*(lane_center);
    // double y = this->spline_waypoints_y(s) + this->spline_waypoints_dy(s)*(lane_center);

    cout << "current s: " << s << " goal s: " << goal_s << endl;
    double goal_speed = 15.0;
    double time = dist_inc/goal_speed;
    auto coeff = JMT({s, car_speed, 0}, {goal_s, goal_speed, 0}, time);
    double delay = 0.02;
    // for(int j=0; j<time/delay; j++) {
    for(int j=0; j<2000; j++) {
      if (s>goal_s) break;
      double t = delay * j;
      double total = 0;
      for(int k=0; k<coeff.size(); k++) {
        total += coeff[k]*pow(t, k);
      }
      s = total;
      double x = this->spline_waypoints_x(s) + this->spline_waypoints_dx(s)*(lane_center);
      double y = this->spline_waypoints_y(s) + this->spline_waypoints_dy(s)*(lane_center);
      cout << "JMT s: " << s << endl;
      // cout << "coeff: " << coeff[0] << ", " << coeff[1] << ", " << coeff[2] << ", " << coeff[3] << ", " << coeff[4] << ", " << coeff[5] << endl;
      // cout << "x: " << x << " y: " << y << endl;
      next_x_vals.push_back(x);
      next_y_vals.push_back(y);
    }
  }
  return {next_x_vals, next_y_vals};
}
