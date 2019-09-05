#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "common.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::atan2;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  // init ref spd and ego lane label
  double ref_spd_mph = 0;//base_spd_mph-car_speed>1 ? base_spd_mph : car_speed;
  int lane = 1;//HV lane index: 0-left,1-center,2-right. 2+lane*4 is Frenet d


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ref_spd_mph,&lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          /****************** initialize **********************/
          double car_yaw_rad = deg2rad(car_yaw);
          int prev_path_size = previous_path_x.size();          
          sLaneStatus hvLeftLane(LL_LEFT);//describes neighbor lane space availability for FSM
          sLaneStatus hvRighLane(LL_RIGH);
          sLaneStatus hvOwnnLane(LL_CENT);
          hvOwnnLane.isExist = true;
          hvOwnnLane.isOpen = true;
          switch(lane){
            case 0: hvLeftLane.isExist = false;
                    hvRighLane.isExist = true;
                    break;
            case 1: hvLeftLane.isExist = true;
                    hvRighLane.isExist = true;
                    break;
            case 2: hvLeftLane.isExist = true;
                    hvRighLane.isExist = false;
                    break;
            default:
                    break;
          }
          hvLeftLane.isOpen = true;
          hvRighLane.isOpen = true;

          double pred_hv_s = prev_path_size>0 ? end_path_s : car_s;

          /********** Sensing to get surrounding vehicles ************/
          bool too_close = false;
          bool too_fast = false;

          //TODO: design vehicle sorting and selection module to get nearest front and rear cars in 3 lanes
          for(const auto& rv : sensor_fusion){
            double d = rv[6];
            double vx = rv[3];
            double vy = rv[4];
            double rv_vel = getSpd(vx,vy);
            double rv_s = rv[5];
            double check_s = rv_s;
            check_s = rv_s + prev_path_size*0.02*rv_vel;
            
            // if rv in hv lane or hv target lane and slow: FSM will look for LC possibility
            if(d>lane2d(lane)-2 && d<lane2d(lane)+2){
              if(check_s>pred_hv_s && check_s-pred_hv_s<FOLLOW_DIST){
                too_close = true;
                //ref_spd_mph = mps2mph(rv_vel);
                if(rv_vel<ref_spd_mph){
                  too_fast = true;
                  cout<<"Slow car "<<d<<endl;
                  //cout<<"Front car in: "<<rv_s-car_s<<"m, spd "<<rv_vel/0.447<<", hv spd "<<car_speed<<", ref spd "<<ref_spd_mph<<endl;
                }
              }
            }
            // locate RVs laterally in global, to determine open status of each lane
            // only deal with RVs within a certain longitudinal range
            if(hvLeftLane.isExist){
              if(d>lane2d(lane-1)-2 && d<lane2d(lane-1)+2){
                if(check_s<pred_hv_s+30 && check_s>pred_hv_s-10 && rv_s>car_s-8){
                  hvLeftLane.isOpen = false;
                }
              }
            }
            if(hvRighLane.isExist){
              if(d>lane2d(lane+1)-2 && d<lane2d(lane+1)+2){
                if(check_s<pred_hv_s+30 && check_s>pred_hv_s-10 && rv_s>car_s-8){
                  hvRighLane.isOpen = false;
                }
              }
            }
          }

          /************* Finite State Machine implementation **********************/
          //Slow car in front
          //Activate STATE 2: PrepareLaneChange
          if(too_close){
            if(too_fast){
              //STATE 2: PrepareLaneChange
              //
              // check if hv is during a LaneChange, if not, look for LC
              if(fabs(car_d-end_path_d)<1.0){
                //NOT DuringLaneChange
                //look at lane space opening and make lane change or follow
                //TODO: select left or righ lane change based on lane speed
                if(laneChangeOK(hvLeftLane)){
                  cout<<"LC Left"<<endl;
                  lane--;//action: make lane change, FSM will go to State 3 in next time cycle
                }
                else if(laneChangeOK(hvRighLane)){
                  cout<<"LC Right"<<endl;
                  lane++;//action: make lane change, FSM will go to State 3 in next time cycle
                }
                //DEFAULT action of STATE 2
                else{
                  cout<<"Follow"<<endl;
                  ref_spd_mph -= MAX_ACC;//no LC available, action: stay in STATE 2, follow slow car
                }
              }
              //STATE 3: DuringLaneChange
              //action in this state is : follow and finish path, slow down if slow car in path, NO LC ALLOWED 
              //need to finish this lane change before any further lane changes
              //if there's a slow car in Target lane, need to follow
              //if there's not, FSM will go to STATE 1
              else{
                cout<<"Slow car in target lane during LaneChange"<<endl;
                ref_spd_mph -= MAX_ACC;
              }
            }
          }

          //no slow car in path
          // DEFAULT STATE 1: LaneKeep
          else if(ref_spd_mph<MAX_SPD_MPH){
            ref_spd_mph += MAX_ACC;
          }

          /****************** update waypoints *************************/
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw_rad = car_yaw_rad;
          sWayPoints wpts;
          //vector<double> wptsx_vec, wptsy_vec;

          if(prev_path_size<2){
            double prev_car_x = car_x - cos(car_yaw_rad);
            double prev_car_y = car_y - sin(car_yaw_rad);
            wpts.x_vec.push_back(prev_car_x);
			      wpts.x_vec.push_back(car_x);
            wpts.y_vec.push_back(prev_car_y);
			      wpts.y_vec.push_back(car_y);
          } else {
            ref_x = previous_path_x[prev_path_size-1];
            ref_y = previous_path_y[prev_path_size-1];
            double ref_x_prev = previous_path_x[prev_path_size-2];
            double ref_y_prev = previous_path_y[prev_path_size-2];
            ref_yaw_rad = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
            wpts.x_vec.push_back(ref_x_prev);
            wpts.y_vec.push_back(ref_y_prev);
            wpts.x_vec.push_back(ref_x);
            wpts.y_vec.push_back(ref_y);
          }
          //cout<<"prev_num "<<prev_path_size<<"  car_yaw "<<car_yaw <<"  ref_yaw "<< ref_yaw_rad*57.3 <<endl;

          // fill rest of waypoints
		      double next_wp_s = car_s;
          double next_wp_d = car_d;
		      if(prev_path_size>0){
             next_wp_s = end_path_s;
          }
          
          for(int i=0;i<ADD_WP_NUM;++i){
            next_wp_s += WP_STRIDE;
            next_wp_d = 2+lane*4;
            vector<double> next_wp = getXY(next_wp_s,next_wp_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
            wpts.x_vec.push_back(next_wp[0]);
            wpts.y_vec.push_back(next_wp[1]);
          }
          //cout<<"end_s "<<end_path_s<<"  end_d "<<end_path_d<<"  nex_s "<<next_wp_s<<"  nex_d "<<next_wp_d<<endl;

          /***************** convert waypoints from map coordinate to vehicle coordinate *************/
          /***************** then run spline fit                                         *************/
          /***************** then covert them back to Global X-Y and send to simulator   *************/
          //lcs2vcs(wpts.x_vec,wpts.y_vec,ref_x,ref_y,ref_yaw_rad);
          for(int i=0;i<wpts.x_vec.size();++i){
            double tmpx = wpts.x_vec[i] - ref_x;
            double tmpy = wpts.y_vec[i] - ref_y;
            wpts.x_vec[i] = tmpx*cos(0-ref_yaw_rad) - tmpy*sin(0-ref_yaw_rad);
            wpts.y_vec[i] = tmpx*sin(0-ref_yaw_rad) + tmpy*cos(0-ref_yaw_rad);
          }

          //spline fit
          tk::spline s_fitter;
          s_fitter.set_points(wpts.x_vec,wpts.y_vec);

          //update path points
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(int i=0;i<prev_path_size;++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double look_ahead_x = 30.0;
          double look_ahead_y = s_fitter(look_ahead_x);
          double look_ahead_dist = sqrt(look_ahead_x*look_ahead_x+look_ahead_y*look_ahead_y);
          
          double x_incr = 0, y_incr = 0;
          
          /****** Get path points in Ego coord, convert them into LCS, then update path points sent to simulator *********/
          for(int i=0;i<50-prev_path_size;++i){
            double n = look_ahead_dist/(mph2mps(ref_spd_mph)*0.02);
            double x_pt = x_incr + look_ahead_x/n;
            double y_pt = s_fitter(x_pt);
            x_incr = x_pt;
            double tmpx = x_pt; 
            double tmpy = y_pt; 
            x_pt = tmpx*cos(ref_yaw_rad)-tmpy*sin(ref_yaw_rad);
            y_pt = tmpx*sin(ref_yaw_rad)+tmpy*cos(ref_yaw_rad);
            x_pt += ref_x;
            y_pt += ref_y;
            //vector<double> lcs = vcs2lcs(x_incr,y_incr,ref_x,ref_y,ref_yaw_rad);
            next_x_vals.push_back(x_pt);
            next_y_vals.push_back(y_pt);
          }

          /******* END OF MY ALGORITHM ********/

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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