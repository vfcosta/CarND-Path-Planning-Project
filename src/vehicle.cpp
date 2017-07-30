#include <iostream>
#include "vehicle.hpp"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(Frenet frenet) {
    state = "CS";
    max_acceleration = 10;
    this->frenet = frenet;
    this->target_speed = 20;
    this->lanes_available = 3;
}

Vehicle::~Vehicle() {}

void Vehicle::update_data(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double dt) {
  // double new_v = (sd[0] - this->s)/dt;
  this->vx = (car_x - this->x)/dt;
  this->vy = (car_y - this->y)/dt;
  double new_v = car_speed;
  this->s = car_s;
  this->d = car_d;
  this->a = (new_v - this->v)/dt;
  this->v = new_v;
  this->x = car_x;
  this->y = car_y;
  if (this->lane == -1) {
    this->lane = current_lane(); // initialize lane
  }
  // cout << "dt: " << dt << ", LANE: " << this->lane << ", a: " << a << ", v: " << v << ", s: " << s << ", d: " << sd[1] << endl;
}

// TODO - Implement this method.
void Vehicle::update_state(map<int,vector < vector<double> > > predictions) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
    
    vector<string> possible_states = get_possible_successor_states();
    double min_cost = 9999999;
    string min_state;
    
    for(auto const& possible_state: possible_states) {
        // cout << "verify " << possible_state << endl;
        auto vehicle = Vehicle(this->frenet);
        vehicle.lane = this->lane;
        vehicle.s = this->s;
        vehicle.v = this->v;
        vehicle.a = this->a;
        vehicle.state = possible_state;
        if (possible_state == "PLCL") vehicle.state = "LCL";
        if (possible_state == "PLCR") vehicle.state = "LCR";
        vehicle.realize_state(predictions);
        auto trajectory = vehicle.generate_predictions(10);
        double state_cost = 0;
        
        state_cost += 1*safety_cost(trajectory, predictions);

        if (state_cost < min_cost) {
            // cout << "new min " << possible_state << " " << state_cost << endl;
            min_cost = state_cost;
            min_state = possible_state;
        }
    }
    cout << min_state << endl;
    state = min_state;
}

double Vehicle::safety_cost(vector < vector<double> > trajectory, map<int,vector < vector<double> > > predictions) {
    double cost = 0;
    for (auto& prediction : predictions) {
        int i = 0;
        if (prediction.first <= 0) continue;
        for (auto& prediction_step : prediction.second) {
            auto const& step = trajectory[i++];
            if (step[0] == prediction_step[0] && abs(step[1]-prediction_step[1]) <= L) {
                cout << "BATEU: " << i << " " << step[0] << " " << prediction_step[0] << endl;
                cost = 1;
                break;
            }
        }
        if (cost == 1) break;
    }
    return cost;
}

vector<string> Vehicle::get_possible_successor_states() {
    vector<string> states;
    states.push_back("KL");
    if (this->state=="LCL" || this->state=="PLCL") states.push_back("LCL");
    if (this->state=="LCR" || this->state=="PLCR") states.push_back("LCR");
    if (this->state=="PLCL" || this->state=="KL")  states.push_back("PLCL");
    if (this->state=="PLCR" || this->state=="KL") states.push_back("PLCR");
    return states;
}

string Vehicle::display() {

	ostringstream oss;
	
	oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    
    return oss.str();
}

void Vehicle::increment(double dt = 1) {

	this->s += this->v * dt;
  this->v += this->a * dt;
}

vector<double> Vehicle::state_at(double t) {

	/*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    double s = this->s + this->v * t + this->a * t * t / 2;
    double v = this->v + this->a * t;
    return {(double)this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

	/*
    Simple collision detection.
    */
    vector<double> check1 = state_at(at_time);
    vector<double> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1; 

	for (int t = 0; t < timesteps+1; t++)
	{
      	if( collides_with(other, t) )
      	{
			collider_temp.collision = true;
			collider_temp.time = t; 
        	return collider_temp;
    	}
	}

	return collider_temp;
}

void Vehicle::realize_state(map<int,vector < vector<double> > > predictions) {
   
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
    	realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(predictions, "R");
    }

}

void Vehicle::realize_constant_speed() {
	a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<double> > > predictions, int lane, int s) {

	int delta_v_til_target = target_speed - v;
  int max_acc = min(max_acceleration, delta_v_til_target);

  map<int, vector<vector<double> > >::iterator it = predictions.begin();
  vector<vector<vector<double> > > in_front;
  while(it != predictions.end())
  {
      
    int v_id = it->first;
    
      vector<vector<double> > v = it->second;
      
      if((v[0][0] == lane) && (v[0][1] > s))
      {
        in_front.push_back(v);

      }
      it++;
  }
  
  if(in_front.size() > 0)
  {
    int min_s = 1000;
    vector<vector<double>> leading = {};
    for(int i = 0; i < in_front.size(); i++)
    {
      if((in_front[i][0][1]-s) < min_s)
      {
        min_s = (in_front[i][0][1]-s);
        leading = in_front[i];
      }
    }
    
    int next_pos = leading[1][1];
    int my_next = s + this->v;
    int separation_next = next_pos - my_next;
    int available_room = separation_next - preferred_buffer;
    max_acc = min(max_acc, available_room);
  }
  
  return max_acc;
}

void Vehicle::realize_keep_lane(map<int,vector< vector<double> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<double> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("R") == 0)
    {
    	delta = 1;
    }
    this->lane += delta;
    cout << "realize_lane_change: " << this->lane << endl;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<double> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("R") == 0)
    {
    	delta = 1;
    }
    int lane = this->lane + delta;

    map<int, vector<vector<double> > >::iterator it = predictions.begin();
    vector<vector<vector<double> > > at_behind;
    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<double> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] <= this->s))
        {
        	at_behind.push_back(v);

        }
        it++;
    }
    if(at_behind.size() > 0)
    {

    	int max_s = -1000;
    	vector<vector<double> > nearest_behind = {};
    	for(int i = 0; i < at_behind.size(); i++)
    	{
    		if((at_behind[i][0][1]) > max_s)
    		{
    			max_s = at_behind[i][0][1];
    			nearest_behind = at_behind[i];
    		}
    	}
    	int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    	int delta_v = this->v - target_vel;
    	int delta_s = this->s - nearest_behind[0][1];
    	if(delta_v != 0)
    	{

    		int time = -2 * delta_s/delta_v;
    		int a;
    		if (time == 0)
    		{
    			a = this->a;
    		}
    		else
    		{
    			a = delta_v/time;
    		}
    		if(a > this->max_acceleration)
    		{
    			a = this->max_acceleration;
    		}
    		if(a < -this->max_acceleration)
    		{
    			a = -this->max_acceleration;
    		}
    		this->a = a;
    	}
    	else
    	{
    		int my_min_acc = max(-this->max_acceleration,-delta_s);
    		this->a = my_min_acc;
    	}

    }

}

vector<vector<double> > Vehicle::generate_predictions(int horizon = 10) {

	vector<vector<double> > predictions;
    for( int i = 0; i < horizon; i++)
    {
      vector<double> check1 = state_at(i);
      vector<double> lane_s = {check1[0], check1[1]};
      predictions.push_back(lane_s);
  	}
    return predictions;

}

int Vehicle::current_lane() {
  return d/lane_width;
}