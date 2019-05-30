#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
//  pid.Init(0.1, 0.0, 0.0); //P-Controller
//  pid.Init(0.1, 0.0, 1.5); //PD-Controller
//  pid.Init(0.1, 0.002, 1.5); //PID-Controller
  pid.Init(0.16, 0.0018, 1.35); //PID-Controller
//  pid.Init(0.13, 0.0018, 1.35); //PID-Controller
//  pid.Init(0.22, 0.0016, 1.2); //PID-Controller
  double cte2 = 0;
  double best_cte2 = 1000;
  int counter = 0;
  double dkp = 0.025;
  double dki = 0.0005;
  double dkd = 0.375;
  int status = 0;  // #1:Kp-update done, #2:Kp&Kd-update done, #3:Kp&Kd&Ki-update done
  double steer_value = 0.0;
  double thr = 0.1; // throttle angle
  double speed_limit = 50.0; //initial limit for updating Kp
  int iteration = 0;


  
  h.onMessage([&pid,&cte2,&best_cte2,&counter,&dkp,&dki,&dkd,&status,&steer_value,&thr,&speed_limit,&iteration](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          pid.UpdateError(cte);
          steer_value = pid.TotalError(); //smoothing filter
          if (steer_value > 1.0){steer_value = 1.0;}
          else if (steer_value < -1.0){steer_value = -1.0;}

          if( (abs(cte)<2.0)&&(abs(angle)<5.0)&&(abs(steer_value)<0.25) ){  // acceleration only when car is under control.
            if(speed < speed_limit * 0.5){
              thr += 0.005;
            }
            else{
              if(speed < speed_limit){thr += 0.001;}
              else{thr -= 0.01;}
            }
          }
          else{
            if(thr>0.1){thr -= 0.1* abs(steer_value);} // quick deceleration for recovery, which is in propotion to steer_value
          }

          // DEBUG
          std::cout << "STATUS:" << status << " Iteration:" << iteration << std::endl;
          std::cout << " Throttle:" << thr  << " Car Speed:" << speed << "/Limit:" << speed_limit << std::endl;
          std::cout << "CTE:" << cte << "Car Angle:" << angle << " Str Angle:" << steer_value  << std::endl;
          std::cout << "Kp :" << pid.Kp << " Ki :" << pid.Ki << " Kd :" << pid.Kd << std::endl;
          std::cout << "dkp:" << dkp << " dki:" << dki << " dkd:" << dkd << std::endl;
          std::cout <<  "CTE2:" << cte2 << " Best-CTE2:" << best_cte2 << std::endl;

          
          ///////////////////////////////////////////////
		  // Twiddle Kp/Ki/Kd parameters
          ///////////////////////////////////////////////
          //Squared error reset every 1000 counts
          int reset_num = 200;
          cte2 += cte * cte;
          counter += 1;
          
          bool twiddle = false;
          if(!twiddle){  //switch parameter tuning mode or fixed parameter mode
            status = 3;
          }            

          if(status!=0){ // speed_limit goes up after Kp adjustment completion
            if(speed_limit<95.0){speed_limit += 0.03;}
          }

          ////////////////////////////////
          //// status & parameter update
          ////////////////////////////////
          // status update
          if(counter == reset_num){
            if(abs(dkp)<0.002){ // 2% of initial Kp
              if(abs(dkd)<0.03){ // 2% of initial Kd
                if(abs(dki)<0.00004){ // 2% of initial Ki
                  if(status!=3){
                    status=3; // Kp Kd Ki optimization done
                    iteration = 0; //iteration conter reset
                  }
                }
                else{
                  if(status != 2){
                    status=2; // Kp Kd optimization done
                    iteration = 0; //iteration conter reset
                  }
                }
              }
              else{
                if(status !=1){ // Kp optimization done
                  status = 1;
                  iteration = 0; //iteration conter reset
                }
              }
            }
            
            // parameter update
            switch(status){
              case 0: // Kp optimizing
                if(best_cte2<cte2){dkp = -0.5 * dkp;}
                else {best_cte2 = cte2;}
                if(twiddle){pid.Kp += dkp;}
                break;   
              case 1: // Kd optimizing
                if(best_cte2<cte2){dkd = -0.5 * dkd;}
                else {best_cte2 = cte2;}
                if(twiddle){pid.Kd += dkd;}
                break;
              case 2: // Ki optimizing
                if(best_cte2<cte2){dki = -0.5 * dki;}
                else {best_cte2 = cte2;}
                if(twiddle){pid.Ki += dki;}
                break;
              case 3:
                if(best_cte2>cte2){best_cte2 = cte2;}
                break;
            }
            counter = 0;           
            cte2 = 0;
            iteration += 1;
          } // end of counter == reset_num
          ////////////////////////////////
          //// End of status & parameter update
          ////////////////////////////////
          

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = thr;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          std::cout << "----------------------------" <<std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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