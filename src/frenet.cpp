#include "frenet.hpp"
#include "utils.hpp"
#include "spline.h"

tk::spline spline_waypoints_x;
tk::spline spline_waypoints_y;
tk::spline spline_waypoints_dx;
tk::spline spline_waypoints_dy;

Frenet::Frenet() {

}

void Frenet::setup(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy) {
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
  this->map_waypoints_dx = map_waypoints_dx;
  this->map_waypoints_dy = map_waypoints_dy;
  this->max_s = this->map_waypoints_s[this->map_waypoints_s.size()-1];

  // add more points to the end to smooth lap transition
  double delta_s = map_waypoints_s[1] - map_waypoints_s[0];
  map_waypoints_s.push_back(this->max_s + delta_s);
  map_waypoints_x.push_back(map_waypoints_x[0]);
  map_waypoints_y.push_back(map_waypoints_y[0]);
  map_waypoints_dx.push_back(map_waypoints_dx[0]);
  map_waypoints_dy.push_back(map_waypoints_dy[0]);
  this->max_s += delta_s;

  spline_waypoints_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_waypoints_y.set_points(map_waypoints_s, map_waypoints_y);
  spline_waypoints_dx.set_points(map_waypoints_s, map_waypoints_dx);
  spline_waypoints_dy.set_points(map_waypoints_s, map_waypoints_dy);
}

vector<double> Frenet::fromFrenet(double s, double d) {
  s = fmod(s, max_s); // adjust s to handle multiple laps
  double x = spline_waypoints_x(s) + spline_waypoints_dx(s)*d;
  double y = spline_waypoints_y(s) + spline_waypoints_dy(s)*d;
  return {x, y};
}
