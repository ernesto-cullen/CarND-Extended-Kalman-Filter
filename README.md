# Extended Kalman Filter Project
### Self-Driving Car Engineer Nanodegree Program

In this project we implement a Kalman Filter in C++ to estimate the state (posicion and velocity in 2D) of a moving object. The measurements come from two noisy sensors, a lidar and a radar. Measurements from these two sensors will be fused to enhance the estimation.


This project uses the [Term 2 simulator](https://github.com/udacity/self-driving-car-sim/releases). The simulator will show a car doing a predetermined circuit, along with the measurements taken and the estimations computed. The simulator communicates with the project using uWebSockets library (the specific commit that works with this version of the simulator is included in the source) to get the estimations as they are calculated, as long as the root mean squared error (RMSE) against the real data (ground thruth).

The project will simulate the real process:

1. take an initial measurement, use it to initialize the system
2. predict the next state of the vehicle
3. get a new measurement, adjust the prediction
4. repeat from point 2.

In this case, the project use a file with entries simulating the sensor measurements.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program
```
["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)
```

OUTPUT: values provided by the c++ program to the simulator
```
["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]
```

I've removed the FusionEKF class and moved all code to KalmanFilter class, as I see no gain from using the FusionEKF class.

Code was edited, compiled, and debugged on Ubuntu 16.04 using [CLion IDE](https://www.jetbrains.com/clion/) which can open an existing project and configure itself using the existing cmake and make files.


## The code

The code is very straightforward in the steps that it follows:

1. configure itself as a server listening in port 4567
```cpp
  uWS::Hub h;

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
  ```

2. The simulator will look for this port and connect to send data and receive calculations. The project calls different functions to respond to the events, like connection and disconnection:
```cpp
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
  ```

3. for each message received from the simulator, the project will parse the sensor data and send it to the KalmanFilter instance for processing. Then it will receive the estimations and RMSE and send them back to the simulator for display. The code is in the `onMessage()` function

4. processing happens in method `processMeasurement()` from KalmanFilter class. This method receives the sensor data and computes the estimation for the next state.
```cpp
void KalmanFilter::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    MatrixXd H;
    MatrixXd R;
    previous_timestamp_ = measurement_pack.timestamp_;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      x_ = Tools::polar2cartesian(measurement_pack.raw_measurements_);
      H = Tools::jacobian(x_);
      R = R_radar_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.6, 0;
      H = H_laser_;
      R = R_laser_;
    }

    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1.e6;
  previous_timestamp_ = measurement_pack.timestamp_;

  //update Q and F with elapsed time
  updateQ(dt);

  F_(0,2) = dt;
  F_(1,3) = dt;

  Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  VectorXd z = measurement_pack.raw_measurements_;
  VectorXd Hx;
  MatrixXd R, H;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    H = Tools::jacobian(Tools::polar2cartesian(z));
    Hx = Tools::cartesian2polar(x_);
    R = R_radar_;
  } else {
    H = H_laser_;
    Hx = H * x_;
    R = R_laser_;
  }
  Update(z, Hx, R, H);
}
```

5. Once the state has been updated, it is sent back to the simulator to display. In this communication, the RMSE is also computed and sent to the simulator.
```cpp
  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
```

where

```cpp
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse.fill(0);

  for (int k = 0; k < estimations.size(); ++k){
    VectorXd diff = estimations[k] - ground_truth[k];
    diff = diff.array() * diff.array();
    rmse += diff;
  }

  rmse /= (double)estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}
```

This is done in a loop controlled by the simulator. In the end, the RMSE is very low which indicates the estimations are pretty close to the ground truth.



## Conclussion

This project serves as a first step into the 'real' programming of sensor signal processing using C++. The simulator provides a welcomed visualization of the data, much more friendly than the raw matrices.
I would like to have more time to play a little with the utilities provided like the data generator, maybe try to modify the simulator to follow a different path. I tried to do it anyway whenever I have a chance.
