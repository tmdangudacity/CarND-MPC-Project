#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);

    //cout << sdata << endl;

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          double px  = j[1]["x"];
          double py  = j[1]["y"];
          double psi = j[1]["psi"];
          double v   = j[1]["speed"];

          /*
          for(unsigned int i = 0; i < ptsx.size(); ++i)
          {
              double d = sqrt( (ptsx[i] - px) * (ptsx[i] - px) + (ptsy[i] - py) * (ptsy[i] - py));

              std::cout << "Original reference point, " << ptsx[i] << ", " << ptsy[i] << ", " << d << std::endl;
          }

          std::cout << "Vehicle point, " << px << ", " << py << ", " << rad2deg(psi) << std::endl;
          */

          vector<double> local_ptsx(ptsx.size());
          vector<double> local_ptsy(ptsy.size());

          Eigen::VectorXd reference_x = Eigen::VectorXd(ptsx.size());
          Eigen::VectorXd reference_y = Eigen::VectorXd(ptsy.size());

          double temp_x = 0.0;
          double temp_y = 0.0;

          for(unsigned int i = 0; i < ptsx.size(); ++i)
          {
              temp_x = ptsx[i] - px;
              temp_y = ptsy[i] - py;

              local_ptsx[i]  =  temp_x * cos(psi) + temp_y * sin(psi);
              reference_x[i] =  local_ptsx[i];

              local_ptsy[i]  = -temp_x * sin(psi) + temp_y * cos(psi);
              reference_y[i] = local_ptsy[i];

              //double d = sqrt(reference_x[i] * reference_x[i] + reference_y[i] * reference_y[i]);
              //std::cout << "Transf. reference point, " << reference_x[i] << ", " << reference_y[i] << ", " << d << std::endl;
          }

          //std::cout << "New vehicle point, " << 0.0 << ", " << 0.0 << std::endl;

          Eigen::VectorXd coeffs = polyfit(reference_x, reference_y, 3);
          //std::cout << "Coeffs: " << coeffs << std::endl;

          //In ENU counter clockwise positive
          double cte  = 0.0 - coeffs[0];
          double epsi = 0.0 - atan(coeffs[1]);
          //std::cout << "Path Y: " << coeffs[0] << ", Xtk error: " << cte << ", Path Hdg: " << rad2deg(atan(coeffs[1])) << ", Hdg error: " << rad2deg(epsi) << std::endl;

          Eigen::VectorXd state(6);
          state << 0.0, 0.0, 0.0, v, cte, epsi;

          //std::cout << "Vehicle states: " << state << std::endl;

          const long DELAY_MS = 100;

          vector<double> solution = mpc.Solve(state, coeffs, (1.0e-3 * DELAY_MS));

          double steer_value = -solution[0] / deg2rad(25.0);
          if(steer_value >  1.0) steer_value =  1.0;
          if(steer_value < -1.0) steer_value = -1.0;

          double throttle_value = solution[1];

          std::cout << "Steer angle solution: " << rad2deg(solution[0])
                    << ", Steer value: "        << steer_value
                    << ", Throttle value: "     << throttle_value
                    << std::endl;

          json msgJson;

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals = mpc.GetSolutionX();
          vector<double> mpc_y_vals = mpc.GetSolutionY();

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals = local_ptsx;
          vector<double> next_y_vals = local_ptsy;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.

          std::cout << "Main thread delayed for " << DELAY_MS << "ms" << std::endl << std::endl;
          this_thread::sleep_for(chrono::milliseconds(DELAY_MS));
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
