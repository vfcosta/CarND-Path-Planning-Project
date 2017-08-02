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
Vehicle::Vehicle() {
    state = "CS";
    max_acceleration = 10;
    this->target_speed = max_speed;
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

void Vehicle::initialize(double car_x, double car_y, double vx, double vy, double car_s, double car_d) {
  this->x = car_x;
  this->y = car_y;
  this->vx = vx;
  this->vy = vy;
  this->s = car_s;
  this->d = car_d;
  this->v = sqrt(vx*vx + vy*vy);
  if (this->lane == -1) {
    this->lane = current_lane(); // initialize lane
  }
}

void Vehicle::update_state(vector<Vehicle> vehicles) {
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
    */
    
    vector<string> possible_states = get_possible_successor_states();
    double min_cost = 9999999;
    string min_state;
    
    for(auto const& possible_state: possible_states) {
        // cout << "verify " << possible_state << endl;
        auto vehicle = Vehicle();
        vehicle.lane = this->lane;
        vehicle.s = this->s;
        vehicle.v = this->v;
        vehicle.a = this->a;
        vehicle.state = possible_state;
        if (possible_state == "PLCL") vehicle.state = "LCL";
        if (possible_state == "PLCR") vehicle.state = "LCR";
        vehicle.realize_state(vehicles);
        auto trajectory = vehicle.generate_predictions(10);
        double state_cost = 0;
        
        state_cost += 5*vehicle.lane_cost(vehicles);
        state_cost += 3*vehicle.safety_cost(trajectory, vehicles);
        state_cost += 1*vehicle.speed_cost(vehicles);

        cout << "POSSIBLE STATE: " << possible_state << " " << state_cost << endl;
        if (state_cost < min_cost) {
            // cout << "new min " << possible_state << " " << state_cost << endl;
            min_cost = state_cost;
            min_state = possible_state;
        }
    }
    cout << ">>>>>>>> SELECTED STATE: " << min_state << endl;
    state = min_state;
}

double Vehicle::lane_cost(vector<Vehicle> vehicles) {
  if (this->lane > 2 || this->lane < 0) return 1;
  return 0;
}

double Vehicle::speed_cost(vector<Vehicle> vehicles) {
  if (target_speed > max_speed) return 1;
  return 1 - target_speed/max_speed;
}

double Vehicle::safety_cost(vector < vector<double> > trajectory, vector<Vehicle> vehicles) {
  if (state.compare("KL") == 0) return 0; // consider that collision will always be avoided when state is KL
  double cost = 0;
  for (auto& vehicle : vehicles) {
      auto collider = will_collide_with(vehicle, 20);
      if (collider.collision) {
        cout << "HIT: " << state << "**********************************************************" << endl;
        cout << "C1] " << " lane: " << this->lane << " s: " << this->s << endl;
        cout << "C2] " << " lane: " << vehicle.lane << " s: " << vehicle.s << endl;
        cost = 1;
        break;
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
    // cout << "State at " << this->lane << " " << this->s << " " << this->v << endl;
    double s = this->s + this->v * t;// + this->a * t * t / 2;
    double v = this->v;// + this->a * t;
    return {(double)this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, double at_time) {
  	/*
      Simple collision detection.
    */
    vector<double> check1 = state_at(at_time);
    vector<double> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (fabs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {
	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1; 
	for (int t = 0; t < timesteps; t++) {
    if( collides_with(other, t*0.1) ) {
			collider_temp.collision = true;
			collider_temp.time = t; 
      return collider_temp;
    }
	}
	return collider_temp;
}

void Vehicle::realize_state(vector<Vehicle> vehicles) {
   
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
    	realize_keep_lane(vehicles);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(vehicles, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(vehicles, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
    	realize_prep_lane_change(vehicles, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(vehicles, "R");
    }

}

void Vehicle::realize_constant_speed() {
	a = 0;
}

int Vehicle::_max_accel_for_lane(vector<Vehicle> vehicles, int lane, int s) {
	int delta_v_til_target = target_speed - v;
  int max_acc = min(max_acceleration, delta_v_til_target);

  int leading_index = -1;
  int min_s = 1000;
  for (int i=0; i < vehicles.size(); i++) {
    Vehicle v = vehicles[i];
    if((v.lane == lane) && (v.s > s) && (v.s - s) < min_s) {
      min_s = v.s - s;
      leading_index = i;
    }
  }
  
  this->target_speed = max_speed;
  if(leading_index >= 0) {
    int next_pos = vehicles[leading_index].s;
    int my_next = s + this->v*1.5;
    int separation_next = next_pos - my_next;
    cout << "next_pos: " << next_pos << " my_next: " << my_next << endl;
    // cout << lane << " separation: " << separation_next << " v: " << vehicles[leading_index].v << endl;
    int available_room = separation_next - preferred_buffer;
    if (separation_next < preferred_buffer) {
      cout << "KEEP DISTANCE" << endl;
      this->target_speed = vehicles[leading_index].v;
      if (separation_next < preferred_buffer/2) {
        cout << "INCREASE DISTANCE" << endl;
        this->target_speed *= 0.9;
      }
      // reduce speed to increase separation
      // cout << "TARGET SPEED1 " << target_speed << endl;
      // this->target_speed *= min(1.0, (preferred_buffer - separation_next)/(double)preferred_buffer);
      cout << "TARGET SPEED " << target_speed << " " << separation_next << endl;
    }
  }
  // cout << "max_acc: " << max_acc << endl;  
  return max_acc;
}

void Vehicle::realize_keep_lane(vector<Vehicle> vehicles) {
	this->a = _max_accel_for_lane(vehicles, this->lane, this->s);
}

void Vehicle::realize_lane_change(vector<Vehicle> vehicles, string direction) {
	int delta = -1;
    if (direction.compare("R") == 0)
    {
    	delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(vehicles, lane, s);
}

void Vehicle::realize_prep_lane_change(vector<Vehicle> vehicles, string direction) {
	int delta = direction.compare("R") == 0 ? 1 : -1;
  int lane = this->lane + delta;

  int nearest_behind_index = -1;
  int max_s = -1000;
  for (int i=0; i < vehicles.size(); i++) {
    Vehicle v = vehicles[i];
    if((v.current_lane() == current_lane()) && (v.s <= this->s) && v.s > max_s) {
      max_s = v.s;
      nearest_behind_index = i;
    }
  }
  if(nearest_behind_index >= 0) {
    int target_vel = vehicles[nearest_behind_index].vx;
    int delta_v = this->v - target_vel;
    int delta_s = this->s - vehicles[nearest_behind_index].s;
    target_speed = min(max_speed, target_vel * 1.1);
    // if(delta_v != 0) {
    // }
    // else {
    //   int my_min_acc = max(-this->max_acceleration,-delta_s);
    //   this->a = my_min_acc;
    // }
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