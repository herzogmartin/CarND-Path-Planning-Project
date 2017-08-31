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

#define DIST_SPLINE_PTS 30
#define NUM_SPLINE_PTS 3
#define NUM_PATH_PTS 50
#define NUM_PATH_PTS_RESET 10
#define TARGET_SPD 49.5*0.44704 // [m/s]
#define MAX_A_SPEEDUP 3
#define MAX_A_SPEEDDOWN 3
#define A_EMERGENCY 7
#define MIN_DIST_LANECHANGE_FRONT 25
#define MIN_DIST_LANECHANGE_BACK 15

using namespace std;

// for convenience
using json = nlohmann::json;

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
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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
 
  //define target lane of car at beginning
  // lane 0 is left lane
  // lane 1 is middle lane
  // lane 2 is right lane
  int targetLane = 1;
  
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&targetLane,&max_s](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
            
            // convert current data of ego into SI units
            const double ego_x = car_x;
            const double ego_y = car_y;
            const double ego_s = car_s;
            const double ego_d = car_d;
            const double ego_yaw = deg2rad(car_yaw);
            const double ego_spd = car_speed * 0.44704;
            static double ego_targetSpd = 0;
            
            // get size of previous path
            // previous path is the rest of the last path witch is not traveled 
            // by the car
            int previous_path_size = previous_path_x.size();
            
            // check where ego is at the end of path
            double ego_s_future;
            double ego_d_future;
            if ( previous_path_size > 0)
             {
              ego_s_future = end_path_s;
              ego_d_future = end_path_d;
             }
            else
             {
              ego_s_future = ego_s;
              ego_d_future = ego_d;
             }

            bool ego_lane_occupied = false;
            bool ego_lane_emergency = false;
            double ego_lane_spd = 9999;
            double ego_lane_dist = 9999;
            vector<double> lane_spd = {9999,9999,9999};
            vector<double> lane_dist_front = {9999,9999,9999};
            vector<double> lane_dist_back = {9999,9999,9999};
            
            //loop through all other cars
            //the following values are calculated
            //safe speed in ego lane
            //minimal speed of all cars for the three lanes
            //distances of cars in front and back of all lines
            for (int i=0; i<sensor_fusion.size(); i++)
             {
              const double other_vx = sensor_fusion[i][3];
              const double other_vy = sensor_fusion[i][4];
              const double other_v = sqrt(other_vx*other_vx + other_vy*other_vy);
              
              const double other_s = sensor_fusion[i][5];
              const double other_d = sensor_fusion[i][6];
              //other car position in future (assumption other car doesn't change lane)
              const double other_s_future = other_s + previous_path_size*0.02*other_v;
              
              //calculate distance at current position and at path's last position 
              double other_dist_s = other_s - ego_s;
              double other_dist_s_future = other_s_future - ego_s_future;
              //consider cyclic world (other car went across border of world))
              if (other_dist_s < -0.5*max_s)
               {
                other_dist_s += max_s;
               }
              if (other_dist_s_future < -0.5*max_s)
               {
                other_dist_s_future += max_s;
               }

              //check if other car is in critical range in front of ego and ego lane
              if ((other_dist_s_future > 0) && //in front 
                  (other_dist_s_future < 1.5 *ego_spd) && // distance traveled in 1.5 s
                  ((ego_d_future-2<other_d+1) && (ego_d_future+2>other_d-1)) //own lane 4m, other car with 2m
                 )
               {
                ego_lane_occupied = true;
                if (other_dist_s_future < ego_lane_dist)
                 {
                  ego_lane_dist = other_dist_s_future;
                 }
                if (other_v < ego_lane_spd)
                 {
                  ego_lane_spd = other_v;
                 }
                //check if distance is so small that vehicle should fall back
                if (other_dist_s_future < 1.0 *ego_spd)
                 {
                  ego_lane_spd -= 1; //fall back with 1 m/s
                 }
                //check if emergency situation is reached --> break hard and go slower
                if (other_dist_s_future < 0.5 *ego_spd)
                 {
                  ego_lane_spd -= 2; //fall back with another 2 m/s
                  ego_lane_emergency = true;
                 }
                //ensure that target speed is not reversing
                if (ego_lane_spd < 0) ego_lane_spd = 0;
               }
              
              // calculate values for all lanes (lane speed and distance to nearest cars)
              for (int k=0; k<3; k++) // loop all lanes
               {
                //check current position
                if (other_d > k*4 && other_d < (k+1)*4) // 4 m lane width
                 {
                  if (other_dist_s > 0) // other car in front
                   {
                    if (other_dist_s < lane_dist_front[k]) lane_dist_front[k] = other_dist_s;
                    if (other_v < lane_spd[k]) lane_spd[k] = other_v;
                   }
                  else // other car in back
                   {
                    if (-other_dist_s < lane_dist_back[k]) lane_dist_back[k] = -other_dist_s;
                   }
                  //check future position
                  if (other_dist_s_future > 0)
                   {
                    if (other_dist_s_future < lane_dist_front[k]) lane_dist_front[k] = other_dist_s_future;
                    if (other_v < lane_spd[k]) lane_spd[k] = other_v;
                   }
                  else
                   {
                    if (-other_dist_s_future < lane_dist_back[k]) lane_dist_back[k] = -other_dist_s_future;
                   }
                 }
               }
             }
            
            // set target speed
            double target_spd;
            if (ego_lane_occupied)
             {
              target_spd = ego_lane_spd;
             }
            else
             {
              target_spd = TARGET_SPD;
             }
            //std::cout << "Target Speed [mph]: " << target_spd*2.23694 << std::endl;
            
            //check fastest lane (default middle lane)
            int fast_lane = 1;
            for (int i=0; i<3; i++) // loop all lanes; start with left lane
             {
              if (lane_spd[i]>lane_spd[fast_lane]) fast_lane = i;
             }
            
            bool flag_lanechange = false;
            //not on fastest lane and other vehicle in ego lane (near enough)
            //std::cout << "dist veh front ego lane: " << lane_dist_front[targetLane] << std::endl;
            while (fast_lane != targetLane && lane_dist_front[targetLane] < 3.0 *ego_spd)
             {
              //change lane one by one
              double dist_fastlane = abs(ego_d_future-(fast_lane*4+2));
              if (dist_fastlane > 4.1)
               {
                if (fast_lane > targetLane) fast_lane--;
                else fast_lane ++;
                continue;
               }
              else
               {
                //std::cout << "lane change check; dist front: " <<
                //        lane_dist_front[fast_lane] <<
                //        " dist back: " << lane_dist_back[fast_lane] << std::endl;
                // check if it safe to change lane
                if (lane_dist_front[fast_lane] > 1.5*ego_spd &&
                    lane_dist_front[fast_lane] > MIN_DIST_LANECHANGE_FRONT &&
                    lane_dist_back[fast_lane] > 1.0*ego_spd &&
                    lane_dist_back[fast_lane] > MIN_DIST_LANECHANGE_BACK)
                 {
                  targetLane = fast_lane;
                  flag_lanechange = true;
                  //std::cout << "Lane change!" << std::endl;
                 }
                else break; //lane change not safe -> stay on lane
               }
             }
             
             
            
          	// create a list of widely spaced waypoints (distance DIST_SPLINE_PTS)
            // these waypoints will be used for spline interpolation
            vector<double> spline_x;
          	vector<double> spline_y;
            
            //define reference/starting point for starting spline of next path
            double ref_x;
            double ref_y;
            double ref_yaw;
            double prev_ref_x;
            double prev_ref_y;
            // if previous path is not available (start of sim) 

            // use ego data and recalculate path
            if (previous_path_size < 2)
             {
              // add point before current ego position in direction of current yaw
              ref_x = ego_x;
              ref_y = ego_y;
              prev_ref_x = ego_x - cos(ego_yaw);
              prev_ref_y = ego_y - sin(ego_yaw);
              ref_yaw = ego_yaw;
             }
//            // if lane change is done or emergency brake needs to be performed
//            // shorten previous path
//            else if (flag_lanechange || ego_lane_emergency)
//             {
//              if (previous_path_size>NUM_PATH_PTS_RESET)
//               {
//                previous_path_x.erase(previous_path_x.begin()+NUM_PATH_PTS_RESET, previous_path_x.end());
//                previous_path_y.erase(previous_path_y.begin()+NUM_PATH_PTS_RESET, previous_path_y.end());
//                previous_path_size = previous_path_x.size();
//                ref_x = previous_path_x[previous_path_size-1];
//                ref_y = previous_path_y[previous_path_size-1];
//                prev_ref_x = previous_path_x[previous_path_size-2];
//                prev_ref_y = previous_path_y[previous_path_size-2];
//                ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);
//                const vector<double> ego_future = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
//                ego_s_future = ego_future[0];
//                ego_d_future = ego_future[1];
//               }
//             }
            // else: previous path is available use it as starting point
            else
             {
              ref_x = previous_path_x[previous_path_size-1];
              ref_y = previous_path_y[previous_path_size-1];
              prev_ref_x = previous_path_x[previous_path_size-2];
              prev_ref_y = previous_path_y[previous_path_size-2];
              ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);
             }
            // now add the two starting points to spline points
            // (for smooth beginning of spline)
            spline_x.push_back(prev_ref_x);
            spline_y.push_back(prev_ref_y);
            spline_x.push_back(ref_x);
            spline_y.push_back(ref_y);

            // add 3 points in distance DIST_SPLINE_PTS in front to spline points
            // use Frenet coordinates for easy calculation
            for (int i=1; i<=NUM_SPLINE_PTS; i++)
             {
              vector<double> next_spline = getXY(
                  ego_s_future + i * DIST_SPLINE_PTS, // point in s
                  2 + 4*targetLane,      // point in d (4 m lane width)
                  map_waypoints_s, map_waypoints_x, map_waypoints_y);
              spline_x.push_back(next_spline[0]);
              spline_y.push_back(next_spline[1]);
             }
            
            //convert spline points to car coordinates (at future point - end of previous path)
            for (int i=0; i<spline_x.size(); i++)
             {
              const double shift_x = spline_x[i] - ref_x;
              const double shift_y = spline_y[i] - ref_y;
              spline_x[i] = shift_x*cos(-ref_yaw) - shift_y*sin(-ref_yaw);
              spline_y[i] = shift_x*sin(-ref_yaw) + shift_y*cos(-ref_yaw);
             }
            
            //now create the spline
            tk::spline spline;
            spline.set_points(spline_x, spline_y);
            
            // define points for the path made up of (x,y) points that the car will visit sequentially every .02 seconds
            vector<double> next_x_vals;
          	vector<double> next_y_vals;

            //start with all of the points of the previous path
            for (int i=0; i<previous_path_size; i++)
             {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
             }
            
            // now fill up rest of the path with points from spline
            double x_local = 0.0;
            for (int i=1; i<=NUM_PATH_PTS-previous_path_size; i++)
             {
              // accelerate towards target speed
              if (ego_targetSpd < target_spd)
               {
                ego_targetSpd += MAX_A_SPEEDUP*0.02;
                if (ego_targetSpd>target_spd) ego_targetSpd=target_spd;
               }
              else
               {
                //check if emergency brake is requested
                if (ego_lane_emergency)
                  ego_targetSpd -= A_EMERGENCY*0.02;
                else
                  ego_targetSpd -= MAX_A_SPEEDDOWN*0.02;
                if (ego_targetSpd<target_spd) ego_targetSpd=target_spd;
               }
              
              //calculate step size in x according to target speed
              const double step_x = ego_targetSpd*0.02;
              
              //spline points in car coordinate
              x_local += step_x;
              const double y_local = spline(x_local);
              //convert to global coordinates
              const double x_global = ref_x + x_local*cos(ref_yaw)-y_local*sin(ref_yaw);
              const double y_global = ref_y + x_local*sin(ref_yaw)+y_local*cos(ref_yaw);
              //add to path
              next_x_vals.push_back(x_global);
              next_y_vals.push_back(y_global);
             }
                        
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
















































































