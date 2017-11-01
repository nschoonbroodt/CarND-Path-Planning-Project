#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

/* Usefull Constants Section */
#define M_PER_MILES 1609.34
// Time between two points in the simulator
#define DT 0.02
// Lanes are 4m width (d=0 is center, positive direction = to the right
#define LANE_WIDTH 4.0
/* End of Usefull Constants Section */

/* Tuneable Parameters Section */
// MAYBE TODO: would be nice to add as a command line argument, if I have the time
// speed in mph
#define TARGET_SPEED 49.0
#define NB_POINTS 50
#define LONG_TERM_INC 30.0
#define NB_LONG_TERM 3
#define MIN_SAFE_DISTANCE 30.0
#define SPEED_MARGIN 5
/* End of Tuneable Parameters Section */

/* State machine mode */
#define NORMAL_MODE    0
#define CAR_IN_FRONT   1
#define OVERTAKE_LEFT  2
#define OVERTAKE_RIGHT 3

/* Sensor fusion table indexing */
#define SF_ID 0
#define SF_X  1
#define SF_Y  2
#define SF_DX 3
#define SF_DY 4
#define SF_S  5
#define SF_D  6

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  // current lane
  int cur_lane = 1;
  // current target speed in m/s
  double cur_tar_speed = TARGET_SPEED*M_PER_MILES/3600;
  // mode
  int mode = NORMAL_MODE;
  int prev_lane = cur_lane;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,
               &cur_lane, &cur_tar_speed, &mode, &prev_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            
            // Current state of the car (replaced by previous_path last data later if enough data available)
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            
            vector<double> pts_x, pts_y;
            
            // check if there are previous points available
            // the previous_path should be empty on start, and never empty later, if everything works as expected
            int prev_size = previous_path_y.size();
            
            double last_s = car_s;
            double last_d = car_d;
            if (prev_size > 0) {
                last_s = end_path_s;
                last_d = end_path_d;
            } else {
                cur_tar_speed = car_speed;
            }
            
            // Check other cars
            bool detected = false;
            double other_speed = 1e10; // just an initial value
            for (int i=0; i<sensor_fusion.size(); ++i) {
                double other_car_d = sensor_fusion[i][SF_D];
                if ((other_car_d-2<(0.5+cur_lane)*LANE_WIDTH) && (other_car_d+2>(0.5+cur_lane)*LANE_WIDTH)) {
                    // we share a lane
                    double vx = sensor_fusion[i][SF_DX];
                    double vy = sensor_fusion[i][SF_DY];
                    double v = sqrt(vx*vx+vy*vy);
                    double other_car_s = sensor_fusion[i][SF_S];
                    
                    // future position
                    other_car_s += prev_size*DT*v;
                    if ((other_car_s > last_s) && (other_car_s-last_s < MIN_SAFE_DISTANCE)) {
                        detected = true;
                        if (other_speed < v) {
                            other_speed = v;
                        }
                        if (mode == NORMAL_MODE) {
                            mode = CAR_IN_FRONT;
                        }
                    }
                }
            }
            
            // Clear the mode if no more car
            if (!detected && mode == CAR_IN_FRONT) {
                mode = NORMAL_MODE;
            }
            
            // If there is a car in front, check if we can take over
            if (mode == CAR_IN_FRONT && cur_lane > 0) { // check on the left
                bool can_change = true;
                int tested_lane = cur_lane-1;
                for (int i=0; i<sensor_fusion.size(); ++i) {
                    double other_car_d = sensor_fusion[i][SF_D];
                    if ((other_car_d-2<(0.5+tested_lane)*LANE_WIDTH) && (other_car_d+2>(0.5+tested_lane)*LANE_WIDTH)) {
                        // car is on the tested lane
                        double vx = sensor_fusion[i][SF_DX];
                        double vy = sensor_fusion[i][SF_DY];
                        double v = sqrt(vx*vx+vy*vy);
                        double other_car_s = sensor_fusion[i][SF_S];
                    
                        // future position
                        other_car_s += prev_size*DT*v;
                        if ((other_car_s > car_s-MIN_SAFE_DISTANCE) && (other_car_s-last_s < MIN_SAFE_DISTANCE)) {
                            // CANNOT CHANGE
                        }
                    }
                }
                if (can_change) {
                    prev_lane = cur_lane;
                    cur_lane = tested_lane;
                    mode = OVERTAKE_LEFT;
                }
            }
/*            if (mode == CAR_IN_FRONT && cur_lane < 2) { // check on the right
                int tested_lane = cur_lane+1;
                for (int i=0; i<sensor_fusion.size(); ++i) {
                    double other_car_d = sensor_fusion[i][SF_D];
                    if ((other_car_d-2<(0.5+tested_lane)*LANE_WIDTH) && (other_car_d+2>(0.5+tested_lane)*LANE_WIDTH)) {
                        // car is on the tested lane
                        double vx = sensor_fusion[i][SF_DX];
                        double vy = sensor_fusion[i][SF_DY];
                        double v = sqrt(vx*vx+vy*vy);
                        double other_car_s = sensor_fusion[i][SF_S];
                    
                        // future position
                        other_car_s += prev_size*DT*v;
                        if ((other_car_s > car_s-MIN_SAFE_DISTANCE) && (other_car_s-last_s < MIN_SAFE_DISTANCE)) {
                            // CANNOT CHANGE
                        } else {
                            prev_lane = cur_lane;
                            cur_lane = tested_lane;
                            mode = OVERTAKE_RIGHT;
                        }
                    }
                }
            }
  */          
            if ((mode != NORMAL_MODE) && (cur_tar_speed+SPEED_MARGIN > other_speed)) {
                cur_tar_speed -= 10*DT;
            } else if (cur_tar_speed <= TARGET_SPEED*M_PER_MILES/3600) {
                cur_tar_speed += 10*DT;
            }
            
            
            // Add points to the next path
            if (prev_size < 2) {
                // fake previous point, just for direction
                pts_x.push_back(car_x - cos(car_yaw));
                pts_y.push_back(car_y - sin(car_yaw));
                
                // current point
                pts_x.push_back(car_x);
                pts_y.push_back(car_y);
            } else {
                // second to last point in queue
                double prev_point_x = previous_path_x[prev_size-2];
                double prev_point_y = previous_path_y[prev_size-2];
                
                // last point in queue
                ref_x = previous_path_x[prev_size-1];
                ref_y = previous_path_y[prev_size-1];
                
                // last angle in queue
                ref_yaw = atan2(ref_y-prev_point_y, ref_x-prev_point_x);
                
                pts_x.push_back(prev_point_x);
                pts_y.push_back(prev_point_y);
                pts_x.push_back(ref_x);
                pts_y.push_back(ref_y);
            }
            
            // Long term points
            for (int i=1; i<=NB_LONG_TERM; ++i) {
                vector<double> long_term = getXY(car_s+i*LONG_TERM_INC, LANE_WIDTH*(0.5+cur_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                pts_x.push_back(long_term[0]);
                pts_y.push_back(long_term[1]);
            }
            
            // transform to local car coordinates
            for (int i=0; i<pts_x.size(); ++i) {
                double dx = pts_x[i]-ref_x;
                double dy = pts_y[i]-ref_y;
                
                pts_x[i] = dx*cos(-ref_yaw)-dy*sin(-ref_yaw);
                pts_y[i] = dx*sin(-ref_yaw)+dy*cos(-ref_yaw);
            }
            
            // Create and populate the spline with our reference and target points
            tk::spline s;
            s.set_points(pts_x, pts_y);
            
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
            
            // push back the points from previous cycle (for a smooth planning)
            for (int i=0; i<prev_size; ++i) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            
            // Now use the spline to add new points
            // Step 1: compute the desired x point for target speed (we're in local coordinate, it's simple. Also just use a linearisation and pythagore)
            double probe_x = LONG_TERM_INC;
            double probe_y = s(probe_x);
            double probe_dist = sqrt(probe_x*probe_x+probe_y*probe_y);
            double spline_dx = probe_x / (probe_dist/(DT*cur_tar_speed));
            
            
            // Step 2: iterate and evaluate on several x spaced according to this DX
            double eval_x = 0.0;
            for (int i=1; i<=NB_POINTS-prev_size; ++i) {
                eval_x += spline_dx;
                double eval_y = s(eval_x);
                
                // transform into global coordinates and push
                next_x_vals.push_back(ref_x + eval_x*cos(ref_yaw)-eval_y*sin(ref_yaw));
                next_y_vals.push_back(ref_y + eval_x*sin(ref_yaw)+eval_y*cos(ref_yaw));
            }
            
            /*
            double dist_inc = cur_tar_speed*DT;
            for (int i=1; i<=NB_POINTS; ++i) {
                double next_s = car_s + dist_inc*(i+1);
                double next_d = cur_lane*4.+2.;
                vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                
                next_x_vals.push_back(xy[0]);
                next_y_vals.push_back(xy[1]);
            }
            */
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
