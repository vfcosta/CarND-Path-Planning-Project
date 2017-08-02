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
  // if (path_size > 10) path_size = 10;

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
     car_speed = sqrt(pow(pos_y-pos_y2,2) + pow(pos_x-pos_x2,2))/0.02;

     car_s = previous_path_s[path_size-1];
     car_d = previous_path_d[path_size-1];
  }
  // cout << "speed: " << car_speed << " angle: " << angle << " path_size: " << path_size << " goal_d: " << goal_d << endl;
  if (path_size < 50) {
    goal_speed = max(car_speed - 10, min(goal_speed, car_speed + 10)); // limit goal_speed
    double dv = fabs(goal_speed - car_speed);

    // 0.5*a*t*t + vi*t - dx = 0;
    // 5*t*t + vi*t - dx = 0;
    // double t2 = (-car_speed + sqrt(car_speed*car_speed + 20*dx))/10.0;
    // cout << "T2: " << t2 << endl;

    // dx = vi*t + 0.5*a*t*t;
    // dx = vi*t + 0.5*dv*t;
    // dx = t*(vi + 0.5*dv);
    // t = dx/(vi + 0.5*dv);
    double time = 1;//dx/(car_speed + 0.5*dv);
    double dist = car_speed + dv/2.0;
    cout << "dist " << dist << endl;
    cout << "speed: " << car_speed << " goal_speed: " << goal_speed << " time: " << time << endl;
    jmtTrajectory(car_s, car_s + dist, car_speed, goal_speed, car_d, goal_d, time, next_x_vals, next_y_vals);
  }
  return {next_x_vals, next_y_vals};
}

void TrajectoryGeneration::jmtTrajectory(double s, double goal_s, double car_speed, double goal_speed, double d, double goal_d, double time, vector<double> &next_x_vals, vector<double> &next_y_vals) {
    auto s_coeff = JMT({s, car_speed, 0}, {goal_s, goal_speed, 0}, time);
    auto d_coeff = JMT({d, 0, 0}, {goal_d, 0, 0}, time);
    double delay = 0.02;
    double t = 0;
    // cout << "d: " << d << " goal_d: " << goal_d << endl;
    while(t < time) {
      t += delay;
      // evaluate equation using JMT coefficients
      double s_proj = 0;
      for(int k=0; k<s_coeff.size(); k++) {
        s_proj += s_coeff[k]*pow(t, k);
      }
      double d_proj = 0;
      for(int k=0; k<d_coeff.size(); k++) {
        d_proj += d_coeff[k]*pow(t, k);
      }

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
