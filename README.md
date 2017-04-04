# Extended Kalman Filter Project #

This project is originally forked from https://github.com/udacity/CarND-Extended-Kalman-Filter-Project.  This repository includes starter code, that is used herein.

## Initialization ##

There are two possibilities for initializing the state estimate, depending on which sensor provides a measurement first.

### Lidar Measurement Initialization ###

If the first available measurement comes from the lidar sensor, then the initialization is simple.  We simply take the measured `x` and `y` positions as the initial values for `px` and `py`, and assume `vx` and `vy` are initially `0.0`.

From Constructor function in class KalmanFilter kalman_filter.cpp:
```C++
x_ = VectorXd(4);
x_ << 1, 1, 0, 0;
```

From FusionEKF.cpp:

```C++
else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        ekf_.x_(0) = measurement_pack.raw_measurements_(0);
        ekf_.x_(1) = measurement_pack.raw_measurements_(1);
        previous_timestamp_ = measurement_pack.timestamp_;
    }
```

### Radar Measurement Initialization ###

If the first available measurement comes from the radar sensors, then we need to convert the polar coordinates `rho` and `phi` into Cartesian coordinates.  Unfortunately, we don't know the direction of `rho_dot`, so we can't use this information to initialize `vx` or `vy`.  Like the lidar case, we assume `vx` and `vy` are initially `0.0`.

From tools.cpp
```C++
VectorXd Tools::Polar2Cartesian(const VectorXd& radar_meas) {
    /**
     * Convert polar coordinates to Cartesian.
     */
    VectorXd f_x(4,1);
    f_x << 1,1,0,0;
    
    // From Lesson 14
    
    float rho = radar_meas(0);
    float phi = radar_meas(1);
    //float rho_dot = radar_meas(2);
    
    f_x << rho*cos(phi),
           -rho*sin(phi),
           0, //unknown how to break rho_dot into vx and vy components
           0;
    
    return f_x;
}
```

From FusionEKF.cpp:

```C++
if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.x_ = tools.Polar2Cartesian(measurement_pack.raw_measurements_);
        previous_timestamp_ = measurement_pack.timestamp_;
    }
```

## Performance Visualization ##
Udacity provides a tool to visualize the performance of the extended Kalman filter, and to calculate the RMSE for a single figure 8 path.  The images below show the performance for cases with both lidar and radar, with lidar only, and with radar only.  It's clear from these results that the overall performance is much better for the combined system, and that the radar by itself is a very poor sensor.

For the case where both sensors are used, RMSE for the y position is 0.016.  For the lidar only case, this is more than 10 times worse: 0.193.  For the radar-only case, this is more than 400 times worse: 6.521.

![Whoops, where's my image](data/radar_and_lidar.png)
*Results using both sensors*

![Whoops, where's my image](data/lidar_only.png)
*Results using only lidar*

![Whoops, where's my image](data/radar_only.png)
*Results using only radar*
