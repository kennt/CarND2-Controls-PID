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

//constexpr double target_speed = 42;
constexpr double target_speed = 62;
constexpr double min_speed = 1;

// Max steering value constraint
// Keep the steer value to [-max_steer_value, max_steer_value]
// This helps to reduce the oscillations when going around curves
// Used as a dampener (if < 1)
constexpr double max_steer_value = 1.0;

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

  PID     pid;
  PID     speed_pid;

  // TODO: Initialize the pid variable.
  // If not training, set the p_initial parameters to the desired values
  // with max_steer_value = 1.0, Kp: 0.603622 Ki: 0.0001 Kd: 9.92264, 500/5000
  // with max_steer_value = 0.6, Kp: 0.649051 Ki: 0.00439805 Kd: 7.2 , 2000/6000
  //
  // Due to the oscillations, I resorted to manual tweaking to get a smoother
  // ride.
  //
  std::vector<double> p_initial = { 0.453622, 0.00019805, 11.9};
  std::vector<double> dp_initial = { 1, 0.1, 1 };
  pid.Init(p_initial);
  pid.SetTrainingParams(false /* is training */,
                        dp_initial,
                        6000 /* number of steps per run, gets me one full lap most tries */,
                        2000 /* number of steps to skip before collecting error info */);

  std::vector<double> speedp_initial = { 0.2, 0, 0 };
  speed_pid.Init(speedp_initial);
  
  h.onMessage([&pid, &speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          // If not training, this call can be shortened to:
          //    pid.Update(cte)
          //
          pid.Update(cte,
                     [&ws]() {
                        std::string reset_msg = "42[\"reset\",{}]";
                        ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                        },
                     (speed < min_speed ? true : false));
          speed_pid.Update(speed - target_speed);

          // Keep the values between [-max_steer_value, max_steer_value]
          //
          steer_value = pid.TotalError();
          
          steer_value = fmax(steer_value, -max_steer_value);
          steer_value = fmin(steer_value, max_steer_value);
          
          double constrained_steer_value = steer_value;

          // Adjust the steer value by the speed, the faster
          // the car is moving, the smaller the steer value allowed
          if (fabs(speed) > 11)
            steer_value = steer_value / (1.2*sqrt(fabs(speed-10)));
          
          // Depending on the steer value, adjust the throttle
          // (the tighter the turn the less throttle)
          // throttle-steer curve
          //                            0     1     2     3     4     5     6     7      8       9       10
          double throttle_adjust[] = { 1.00, 1.00, 0.70, 0.50, 0.30, 0.15, 0.10, 0.01, -0.001, -0.002, -0.007 };
          double throttle = speed_pid.TotalError();
          if (speed > 20)
            throttle *= throttle_adjust[static_cast<int>(fabs(constrained_steer_value*10))];

          json msgJson;
          msgJson["steering_angle"] = steer_value / deg2rad(25);
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
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
