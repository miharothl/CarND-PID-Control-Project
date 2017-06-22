#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid_steering;
  PID pid_throttle;
  // TODO: Initialize the pid variable.


// Step 1: Set speed to 40Mph. Set Kp. Completes half of the track, doesn't oscillate to much.
//  double Kp = 0.05;
//  double Ki = 0.;
//  double Kd = 0.;

// Step 2: Speed 40Mph. Set Kd. Oscillation dampened. Discrete steering. Late in sharp turns.
//  double Kp = 0.05;
//  double Ki = 0.01;
//  double Kd = 0;

// Step 3: Speed 40Mph. Set Ki. Continuous steering, completes the track.
//  double Kp = 0.05;
//  double Ki = 0.01;
//  double Kd = 2.;


// Step 4: Speed 40Mph. Increased Ki to improve sharp turns. Increased speed to 60Mph. Completes track. Gentle turns.
  double Kp = 0.06;
  double Ki = 0.05;
  double Kd = 2;

  pid_steering.Init(Kp, Ki, Kd);
  pid_throttle.Init(1.4, 10., 5.);

  h.onMessage([&pid_steering, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte_steering = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          pid_steering.UpdateError(cte_steering);
          auto steering_value = pid_steering.TotalError();

          double cte_throttle;
          double target_speed = 40.;
          target_speed = 60.;

          cte_throttle = -(target_speed - speed)/10.;

          pid_throttle.UpdateError(cte_throttle);
          auto throttle_value = pid_throttle.TotalError();

          // DEBUG
          std::cout << "CTE Steering: " << cte_steering << " Steering Value: " << steering_value << std::endl;
          std::cout << "CTE Throttle: " << cte_throttle << " Throttle Value: " << throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steering_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
