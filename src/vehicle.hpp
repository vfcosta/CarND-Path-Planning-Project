#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {

private:
  double lane_width = 4;

public:

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 4;

  int preferred_buffer = 2*L; // impacts "keep lane" behavior.

  int lane = -1;

  double s;
  double d;
  double x;
  double y;

  double v;
  double vx;
  double vy;

  double a;

  double target_speed;

  double max_speed = 21;

  int max_acceleration;

  string state;

  /**
  * Constructor
  */
  Vehicle();

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_data(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double dt);

  void initialize(double car_x, double car_y, double vx, double vy, double s, double d);

  void update_state(vector<Vehicle> vehicles);

  string display();

  void increment(double dt);

  int current_lane();

  vector<double> state_at(double t);

  bool collides_with(Vehicle other, double at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(vector<Vehicle> vehicles);

  void realize_constant_speed();

  int _max_accel_for_lane(vector<Vehicle> vehicles, int lane, int s);

  void realize_keep_lane(vector<Vehicle> vehicles);

  void realize_lane_change(vector<Vehicle> vehicles, string direction);

  void realize_prep_lane_change(vector<Vehicle> vehicles, string direction);

  vector<vector<double> > generate_predictions(int horizon);

  vector<string> get_possible_successor_states();
  double safety_cost(vector < vector<double> > trajectory, vector<Vehicle> vehicles);
  double speed_cost(vector<Vehicle> vehicles);
  double lane_cost(vector<Vehicle> vehicles);
};

#endif