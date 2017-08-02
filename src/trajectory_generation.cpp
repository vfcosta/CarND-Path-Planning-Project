#include "trajectory_generation.hpp"
#include "utils.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"

using Eigen::MatrixXd;
using Eigen::VectorXd;

TrajectoryGeneration::TrajectoryGeneration(Frenet frenet) {
  this->frenet = frenet;
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

vector<vector<double>> TrajectoryGeneration::generate(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double goal_d, double goal_speed, vector<double> previous_path_x, vector<double> previous_path_y) {
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double pos_x;
  double pos_y;
  double angle;
  int path_size = previous_path_x.size();

  int size_consumed = previous_path_s.size() - previous_path_x.size();
  previous_path_s = vector<double>(previous_path_s.begin()+size_consumed, previous_path_s.end());
  previous_path_d = vector<double>(previous_path_d.begin()+size_consumed, previous_path_d.end());

  // copy previous path to next vals
  for(int i = 0; i < path_size; i++) {
     next_x_vals.push_back(previous_path_x[i]);
     next_y_vals.push_back(previous_path_y[i]);
  }
  // take cached vals into account when get current vehicle position
  if(path_size == 0) {
     pos_x = car_x;
     pos_y = car_y;
     angle = deg2rad(car_yaw);
  } else {
     pos_x = previous_path_x[path_size-1];
     pos_y = previous_path_y[path_size-1];

     double pos_x2 = previous_path_x[path_size-2];
     double pos_y2 = previous_path_y[path_size-2];
     angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

     car_s = previous_path_s[path_size-1];
     car_d = previous_path_d[path_size-1];
     double car_s2 = previous_path_s[path_size-2];
     double car_d2 = previous_path_d[path_size-2];
     car_speed = distance(car_s2, car_d, car_s, car_d2)/delay;
  }
  // cout << "speed: " << car_speed << " angle: " << angle << " path_size: " << path_size << " goal_d: " << goal_d << endl;
  if (path_size < 100) {
    double time = 1 + fabs(goal_d - car_d)/lane_width;
    goal_speed = max(car_speed - 5/time, min(goal_speed, car_speed + 5/time)); // limit goal_speed
    double dv = goal_speed - car_speed;
    double dist_s = car_speed*time + dv*time*0.5;
    // cout << "car_s: " << car_s << " dist_s " << dist_s << endl;
    // cout << "speed: " << car_speed << " goal_speed: " << goal_speed << " time: " << time << endl;
    jmtTrajectory(car_s, car_s + dist_s, car_speed, goal_speed, car_d, goal_d, time, next_x_vals, next_y_vals);
  }
  return {next_x_vals, next_y_vals};
}

void TrajectoryGeneration::jmtTrajectory(double s, double goal_s, double car_speed, double goal_speed, double d, double goal_d, double time, vector<double> &next_x_vals, vector<double> &next_y_vals) {
  auto solution = selectBest(s, goal_s, car_speed, goal_speed, d, goal_d, time);
  auto s_coeff = solution[0];
  auto d_coeff = solution[1];
  double t = 0;
  // cout << "time: " << time << " d: " << d << " goal_d: " << goal_d << endl;
  while(t < time - delay) {
    t += delay;
    // evaluate equation using JMT coefficients
    double s_proj = evaluate_coefficients(s_coeff, t);
    double d_proj = evaluate_coefficients(d_coeff, t);

    // transform from frenet to XY using fitted spline
    auto xy = frenet.fromFrenet(s_proj, d_proj);
    // cout << "JMT s: " << s_proj << " d: " << d_proj << endl;
    // cout << "s_coeff: " << s_coeff[0] << ", " << s_coeff[1] << ", " << s_coeff[2] << ", " << s_coeff[3] << ", " << s_coeff[4] << ", " << s_coeff[5] << endl;
    // cout << "x: " << x << " y: " << y << endl;
    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
    previous_path_s.push_back(s_proj);
    previous_path_d.push_back(d_proj);
  }
}

vector<vector<double>> TrajectoryGeneration::selectBest(double s, double goal_s, double car_speed, double goal_speed, double d, double goal_d, double &time) {
  double timestep = 0.5;
  double min_cost = 9999;
  vector<double> min_s_coeff;
  vector<double> min_d_coeff;
  double min_time = time;
  for (int i=0; i<10; i++) {
    double t = time + (i-1)*timestep;
    auto s_coeff = JMT({s, car_speed, 0}, {goal_s, goal_speed, 0}, t);
    auto d_coeff = JMT({d, 0, 0}, {goal_d, 0, 0}, t);
    double cost = trajectoryCost(s_coeff, d_coeff, t);
    if (cost < min_cost) {
      min_cost = cost;
      min_s_coeff = s_coeff;
      min_d_coeff = d_coeff;
      min_time = t;
    }
  }
  time = min_time;
  // cout << ">>>>>>>> SELECTED TIME " << time << endl;
  return {min_s_coeff, min_d_coeff};
}

double TrajectoryGeneration::trajectoryCost(vector<double> s_coeff, vector<double> d_coeff, double t) {
  double value = 0;
  value += 1*maxAccelCost(s_coeff, d_coeff, t);
  value += 1*maxSpeedCost(s_coeff, d_coeff, t);
  // cout << "VALUE " << value << " T " << t << endl;
  return value;
}

double TrajectoryGeneration::maxSpeedCost(vector<double> s_coeff, vector<double> d_coeff, double t) {
  auto s_dot = differentiate(s_coeff);
  auto d_dot = differentiate(d_coeff);
  double max = 0;
  double ti = 0;
  double dt = t/100.0;
  while(ti < t - delay) {
  // for (int i=0; i<100; i++) {
    double speed = fabs(evaluate_coefficients(s_dot, ti)) + fabs(evaluate_coefficients(d_dot, ti));
    // cout << "MAX " << speed << " " << ti << endl;
    if (speed > 22) {
      return 1;
    }
    ti += delay;
  }
  return 0;
}

double TrajectoryGeneration::maxAccelCost(vector<double> s_coeff, vector<double> d_coeff, double t) {
  auto s_d_dot = differentiate(differentiate(s_coeff));
  auto d_d_dot = differentiate(differentiate(d_coeff));
  double max = 0;
  double ti = 0;
  double dt = t/100.0;
  while(ti < t - delay) {
  // for (int i=0; i<100; i++) {
    double accel = fabs(evaluate_coefficients(s_d_dot, ti)) + fabs(evaluate_coefficients(d_d_dot, ti));
    // cout << "MAX " << accel << " " << ti << endl;
    if (accel > 10) {
      return 1;
    }
    ti += delay;
  }
  return 0;
}
