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
  spline_waypoints_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_waypoints_y.set_points(map_waypoints_s, map_waypoints_y);
  spline_waypoints_dx.set_points(map_waypoints_s, map_waypoints_dx);
  spline_waypoints_dy.set_points(map_waypoints_s, map_waypoints_dy);
}

vector<double> Frenet::fromXY(double x, double y, double angle) {
  return getFrenet(x, y, angle, map_waypoints_x, map_waypoints_y);
}

vector<double> Frenet::fromFrenet(double s, double d) {
    double x = spline_waypoints_x(s) + spline_waypoints_dx(s)*(d);
    double y = spline_waypoints_y(s) + spline_waypoints_dy(s)*(d);
    return {x, y};
}

vector<double> Frenet::nextFromXY(double x, double y, double angle) {
  int next_waypoint = NextWaypoint(x, y, angle, map_waypoints_x, map_waypoints_y);
  return {this->map_waypoints_s[next_waypoint], this->map_waypoints_dx[next_waypoint], this->map_waypoints_dy[next_waypoint]};
}