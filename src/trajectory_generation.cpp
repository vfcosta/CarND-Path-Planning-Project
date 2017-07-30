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

vector<vector<double>> TrajectoryGeneration::generate(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double goal_d, vector<double> previous_path_x, vector<double> previous_path_y) {
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double pos_x;
  double pos_y;
  double angle;
  int path_size = previous_path_x.size();
  // if (path_size > 10) path_size = 10;

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
  // cout << "speed: " << car_speed << " angle: " << angle << " path_size: " << path_size << " goal_d: " << goal_d << endl;
  if (path_size < 50)
  {
    // TODO use spline to calculate frenet given x,y
    auto sd = frenet.fromXY(pos_x, pos_y, angle);
    auto nextSd = frenet.nextFromXY(pos_x, pos_y, angle);
    // keepLaneTrajectory(sd, nextSd, car_speed, next_x_vals, next_y_vals);
    // changeLaneTrajectory(sd, nextSd, car_speed, next_x_vals, next_y_vals);
    double goal_speed = 20.0;
    double time = (nextSd[0] - sd[0])/goal_speed;
    jmtTrajectory(sd[0], nextSd[0], car_speed, goal_speed, sd[1], goal_d, time, next_x_vals, next_y_vals);  
  }
  return {next_x_vals, next_y_vals};
}

void TrajectoryGeneration::keepLaneTrajectory(vector<double> sd, vector<double> nextSd, double car_speed, vector<double> &next_x_vals, vector<double> &next_y_vals) {
    cout << "current s: " << sd[0] << " goal s: " << nextSd[0] << endl;
    double goal_speed = 20.0;
    double time = (nextSd[0] - sd[0])/goal_speed;
    jmtTrajectory(sd[0], nextSd[0], car_speed, goal_speed, sd[1], sd[1], time, next_x_vals, next_y_vals);  
}

void TrajectoryGeneration::changeLaneTrajectory(vector<double> sd, vector<double> nextSd, double car_speed, vector<double> &next_x_vals, vector<double> &next_y_vals) {
    // TODO: random change lane
    int lane_diff = (rand() % static_cast<int>(3)) - 1;
    double goal_d = max(2.0, min(sd[1] + lane_diff * this->lane_width, 10.0));

    cout << "current s: " << sd[0] << " goal s: " << nextSd[0] << endl;
    double goal_speed = 15.0;
    double time = (nextSd[0] - sd[0])/goal_speed;
    jmtTrajectory(sd[0], nextSd[0], car_speed, goal_speed, sd[1], goal_d, time, next_x_vals, next_y_vals);  
}

void TrajectoryGeneration::jmtTrajectory(double s, double goal_s, double car_speed, double goal_speed, double d, double goal_d, double time, vector<double> &next_x_vals, vector<double> &next_y_vals) {
    auto s_coeff = JMT({s, car_speed, 0}, {goal_s, goal_speed, 0}, time);
    auto d_coeff = JMT({d, 0, 0}, {goal_d, 0, 0}, time);
    double delay = 0.02;
    double t = 0;
    while(t < time) {
      t += delay;
      // evaluate equation using JMT coefficients
      double s_proj = 0;
      for(int k=0; k<s_coeff.size(); k++) {
        s_proj += s_coeff[k]*pow(t, k);
      }
      double d_proj = 0;
      for(int k=0; k<s_coeff.size(); k++) {
        d_proj += d_coeff[k]*pow(t, k);
      }

      // transform from frenet to XY using fitted spline
      auto xy = frenet.fromFrenet(s_proj, d_proj);
      // cout << "JMT s: " << s_proj << " d: " << d_proj << endl;
      // cout << "s_coeff: " << s_coeff[0] << ", " << s_coeff[1] << ", " << s_coeff[2] << ", " << s_coeff[3] << ", " << s_coeff[4] << ", " << s_coeff[5] << endl;
      // cout << "x: " << x << " y: " << y << endl;
      next_x_vals.push_back(xy[0]);
      next_y_vals.push_back(xy[1]);
    }
}
