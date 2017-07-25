#include "trajectory_generation.hpp"
#include "utils.hpp"

TrajectoryGeneration::TrajectoryGeneration(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy) {
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
  this->map_waypoints_dx = map_waypoints_dx;
  this->map_waypoints_dy = map_waypoints_dy;
}

vector<vector<double>> TrajectoryGeneration::generate(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> previous_path_x, vector<double> previous_path_y) {
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double pos_x;
  double pos_y;
  double angle;
  int path_size = previous_path_x.size();

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

  auto freenet = getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
  double dist_inc = 0.5;
  for(int i = 0; i < 50-path_size; i++)
  {
  //    next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
  //    next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
  //    pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
  //    pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
  }
  return {next_x_vals, next_y_vals};
}
