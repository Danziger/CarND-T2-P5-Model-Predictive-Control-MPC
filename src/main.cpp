#include "MPC.h"

#include "common/Eigen-3.3/Eigen/Core"
#include "common/Eigen-3.3/Eigen/QR"
#include "common/JSON-Lohmann-2.1.1/json.hpp"
#include "common/format.h"
#include "common/helpers.h"

#include <uWS/uWS.h>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <stdlib.h>


// Lag:
#define LAG_S 0.1
#define LAG_MS 100


// For convenience:
using json = nlohmann::json;
using namespace std;


/**
* Evaluate a polynomial defined by coeffs at x.
*/
double polyeval(const Eigen::VectorXd coeffs, const double x) {
    double result = 0.0;

    for (unsigned int i = 0; i < coeffs.size(); ++i) {
        result += coeffs[i] * pow(x, i);
    }

    return result;
}


/**
* Fit a polynomial.
* Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
*/
Eigen::VectorXd polyfit(
    const Eigen::VectorXd xvals,
    const Eigen::VectorXd yvals,
    const unsigned int order
) {
    const unsigned int xvalsSize = xvals.size();

    assert(xvalsSize == yvals.size());
    assert(order >= 1 && order <= xvalsSize - 1);

    Eigen::MatrixXd A(xvalsSize, order + 1);

    for (Eigen::Index j = 0; j < xvalsSize; ++j) {
        A(j, 0) = 1.0;

        for (size_t i = 0; i < order; ++i) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    return A.householderQr().solve(yvals);
}


// MAIN:

int main(int argc, char **argv) {

    // ARGUMENTS:

    // Lf = 2.67
    // 
    // This value assumes the model presented in the classroom is used.
    //
    // It was obtained by measuring the radius formed by running the vehicle in the
    // simulator around in a circle with a constant steering angle and velocity on a
    // flat terrain.
    //
    // Lf was tuned until the the radius formed by the simulating the model
    // presented in the classroom matched the previous radius.
    //
    // This is the length from front to CoG that has a similar radius:

    const vector<unsigned int> DEF_WEIGHTS = { 750, 750, 10, 1, 10, 200, 10 };
    const vector<unsigned int> DEF_MPC_PARAMS = { 50, 2 };
    vector<unsigned int> WEIGHTS;
    vector<unsigned int> MPC_PARAMS;

    const size_t last_weight_index  = (size_t) min(argc, 8);
    const size_t last_mpc_params_index = (size_t) min(10, max(argc, 8));

    for (size_t i = 1; i < last_weight_index; ++i) {
        WEIGHTS.push_back(strtoul(argv[i], NULL, 0));
    }

    for (size_t i = last_weight_index ; i < 8; ++i) {
        WEIGHTS.push_back(DEF_WEIGHTS[i - 1]);
    }

    for (size_t i = 8; i < last_mpc_params_index; ++i) {
        MPC_PARAMS.push_back(strtoul(argv[i], NULL, 0));
    }

    for (size_t i = last_mpc_params_index; i < 10; ++i) {
        MPC_PARAMS.push_back(DEF_MPC_PARAMS[i - 8]);
    }

    const double MPH_2_MS = argc > 10 ? atof(argv[10]) : 1.0; // 0.447
    const double LF = argc > 11 ? atof(argv[11]) : 2.67;
    const bool SLF = argc > 12 ? argv[12][0] == '1' : false;

    // Print them out:

    cout
        << endl
        << endl
        << "WEIGHTS & OTHER MPC PARAMS" << endl
        << endl
        << "                 CTE     W_CTE = " << WEIGHTS[0] << endl
        << "                EPSI    W_EPSI = " << WEIGHTS[1] << endl
        << "               Speed   W_SPEED = " << WEIGHTS[2] << endl
        << "               Delta   W_DELTA = " << WEIGHTS[3] << endl
        << "        Acceleration     W_ACC = " << WEIGHTS[4] << endl
        << "        Delta Change  W_DDELTA = " << WEIGHTS[5] << endl
        << " Acceleration Change    W_DACC = " << WEIGHTS[6] << endl
        << endl
        << "        Target Speed     V_REF = " << MPC_PARAMS[0] << endl
        << "      Average N Last     AVG_N = " << MPC_PARAMS[1] << endl
        << endl
        << endl
        << "UNITS / CONVERSIONS" << endl
        << endl
        << "   Conversion Factor  MPH_2_MS = " << MPH_2_MS << endl
        << "                  Lf        LF = " << LF << endl
        << "    Steering over Lf       SLF = " << (SLF ? "TRUE" : "FALSE") << endl
        << endl
        << endl;
   
    if (MPH_2_MS != 1) {
        cout << "Converting speed from MPH to m/s using a factor of " << MPH_2_MS << "." << endl << endl;

        MPC_PARAMS[0] = MPC_PARAMS[0] * MPH_2_MS;
    }

    const unsigned int V_REF = MPC_PARAMS[0];

    // Initialize MPC with arguments:
    MPC mpc(WEIGHTS, MPC_PARAMS, MPH_2_MS, LF);

    // MESSAGE PROCESSING:

    uWS::Hub h; // Initialize WebSocket.

    unsigned int total = 0; // For logging purposes.

    h.onMessage([
        &mpc,
        &V_REF,
        &MPH_2_MS,
        &LF,
        &SLF,
        &total
    ](
        uWS::WebSocket<uWS::SERVER> ws,
        char *data,
        size_t length,
        uWS::OpCode opCode
    ) {

        // "42" at the start of the message means there's a websocket message event:
        // - The 4 signifies a websocket message
        // - The 2 signifies a websocket event
        const string sdata = string(data).substr(0, length);

        if (sdata.size() <= 2 || sdata[0] != '4' || sdata[1] != '2') {
            return;
        }

        const string s = helpers::hasData(sdata);

        if (s == "") {
            // Manual driving:

            std::string msg = "42[\"manual\",{}]";

            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            return;
        }

        const auto j = json::parse(s);
        const string event = j[0].get<string>();

        if (event != "telemetry") {
            return;
        }

        // j[1] is the data JSON object:

        const vector<double> ptsx = j[1]["ptsx"];
        const vector<double> ptsy = j[1]["ptsy"];

        const double px = j[1]["x"];
        const double py = j[1]["y"];
        const double psi = j[1]["psi"];
        const double v = ((double) j[1]["speed"]) * MPH_2_MS;
        const double steering = j[1]["steering_angle"];
        const double throttle = ((double) j[1]["throttle"]) * MPH_2_MS;

        // Calculate waypoints in vehicle's coordinates system:

        const unsigned int N = ptsx.size();
        const double cospsi = cos(-psi);
        const double sinpsi = sin(-psi);

        Eigen::VectorXd waypoints_x(N);
        Eigen::VectorXd waypoints_y(N);

        for (size_t i = 0; i < N; ++i) {
            // IDEA/TODO: Should I take lag into account here too?

            const double dx = ptsx[i] - px;
            const double dy = ptsy[i] - py;

            waypoints_x(i) = dx * cospsi - dy * sinpsi;
            waypoints_y(i) = dy * cospsi + dx * sinpsi;
        }

        // Fit a 3rd degree polynomial to the above waypoints:
        const auto coeffs = polyfit(waypoints_x, waypoints_y, 3);

        // In vehicle's coordinate system, px, py and psi are all 0, so this (for a 2nd degree polynomil):
        // const double cte = polyeval(coeffs, px) - py;
        // const double epsi = psi - atan(coeffs[1]);
        // Equals this (for a nth degree polynomial):

        // Calculate the cross track error:
        const double CTE = coeffs[0]; // f(px) = f(0) = coeffs[0]

        // Calculate the orientation error:
        // Due to the sign starting at 0, the orientation error is -f'(px).
        // Derivative of coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + ...
        // Is coeffs[1] + 2 * coeffs[2] * x + ...
        const double EPSI = -atan(coeffs[1]); // psi - f'(px) = 0 - f'(0) = coeffs[1]

        // Calculate steering angle and throttle using MPC, both in between [-1, 1]:
        Eigen::VectorXd state(6);

        // Calculate x, y, psi, v, cte, epsi, taking into accoutn 100ms lag:

        state <<
            v * LAG_S,
            0, // psi = 0 from the vehicle's coordinate system
            -v * LAG_S * steering / LF,
            v + throttle * LAG_S,
            CTE + v * sin(EPSI) * LAG_S,
            EPSI -v * LAG_S * steering / LF;

        const auto vars = mpc.solve(state, coeffs);

        // NOTE: Remember to divide by DEG_2_RAD(25) before you send the steering value back.
        // Otherwise the values will be in between [-DEG_2_RAD(25), DEG_2_RAD(25] instead of [-1, 1].

        const double cost = vars[0];
        double steering_control = -vars[1] / (DEG_2_RAD(25) * (SLF ? LF : 1));
        double throttle_control = vars[2] / MPH_2_MS;

        // This could take into account a few of the predicted cte and epsi 
        if (( (abs(CTE) > 1 && abs(EPSI) > 0.10) || (abs(CTE) > 0.5 && abs(EPSI) > 0.2) ) && v > 50 ) {
            throttle_control = -1;
            // steering_control = 1.25 * abs(steering_control) * (CTE < 0 ? -1 : 1);
            cout << "BRAKE!" << endl;
        }

        // Print stats header once every 10 rows:

        if (++total % 10 == 1) {
            cout
                << " │          │          │          │          │                  │" << endl
                << " │      CTE │     EPSI │    STEER │  THROTLE │             COST │" << endl;
        }

        // Print actual stats:

        // TODO: Add green/yellow/red color to CTE
        // TODO: Implement emergency braking

        cout
            << setprecision(2) << fixed
            << " │ " << setfill(' ') << setw(8) << CTE
            << " │ " << setfill(' ') << setw(8) << EPSI
            << " │ " << setfill(' ') << setw(8) << steering_control
            << " │ " << setfill(' ') << setw(8) << throttle_control
            << " │ " << setfill(' ') << setw(16) << cost
            << " │"
            << endl;

        // Send message back to the emulator:

        json msgJson;
                                    
        msgJson["steering_angle"] = steering_control;
        msgJson["throttle"] = throttle_control;

        // Display the MPC predicted trajectory:
        // Add (x,y) points to list here, points are in reference to the vehicle's coordinate system
        // the points in the simulator are connected by a green line.

        const size_t last_points_index = vars.size();

        vector<double> mpc_x_vals;
        vector<double> mpc_y_vals;

        for (size_t i = 3; i < last_points_index; i += 2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i + 1]);
        }

        msgJson["mpc_x"] = mpc_x_vals;
        msgJson["mpc_y"] = mpc_y_vals;

        // Calculate waypoints in vehicle's coordinates system:
        // Add (x,y) points to list here, points are in reference to the vehicle's coordinate system
        // the points in the simulator are connected by a yellow line.

        vector<double> next_x_vals;
        vector<double> next_y_vals;

        for (size_t i = 4; i < 16 * N + 4; i += 4) {
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
        }

        msgJson["next_x"] = next_x_vals;
        msgJson["next_y"] = next_y_vals;

        const string msg = "42[\"steer\"," + msgJson.dump() + "]";
        
        // TODO: Use the other json notation!

        // Latency
        // The purpose is to mimic real driving conditions where
        // the car does actuate the commands instantly.
        //
        // Feel free to play around with this value but should be to drive
        // around the track with 100ms latency.
        //
        // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
        // SUBMITTING.

        this_thread::sleep_for(chrono::milliseconds(LAG_MS));

        // Send it:
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    });

    // ON HTTP REQUEST:
    // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(

    h.onHttpRequest([](
        uWS::HttpResponse *res,
        uWS::HttpRequest req,
        char *data,
        size_t,
        size_t
     ) {
        const std::string s = "<h1>Hello world!</h1>";

        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // I guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    // ON CONNECTION:

    h.onConnection([&h](
        uWS::WebSocket<uWS::SERVER> ws,
        uWS::HttpRequest req
    ) {
        cout
            << endl
            << " Connected!" << endl
            << endl
            << "──────────────────────────────────────────────────────" << endl
            << endl;
    });

    // ON DISCONNECTION:

    h.onDisconnection([&h](
        uWS::WebSocket<uWS::SERVER> ws,
        int code,
        char *message,
        size_t length
    ) {
        ws.close();

        cout
            << endl
            << "Disconnected!" << endl
            << endl
            << SEPARATOR << endl;
    });

    // START LISTENING:

    const int port = 4567;

    if (h.listen(port)) {

        cout
            << endl
            << " Listening on port " << port << "..." << endl
            << endl
            << "──────────────────────────────────────────────────────" << endl;

    } else {

        cerr
            << endl
            << "Failed to listen on port" << port << "!"
            << endl
            << SEPARATOR << endl;

        return -1;
    }

    h.run();
}
