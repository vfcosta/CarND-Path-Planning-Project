#ifndef FRENET_H
#define FRENET_H

#include <vector>

using namespace std;

class Frenet {

public:
  Frenet();
  void setup(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);
  
  vector<double> fromXY(double x, double y, double angle);
  vector<double> fromFrenet(double s, double d);
  vector<double> nextFromXY(double x, double y, double angle);

private:
  double max_s;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
};

#endif
