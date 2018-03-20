# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

![](https://github.com/evgenyslab/CarND-Extended-Kalman-Filter-Project/blob/master/Result/result.gif)

## Outline
This progam implements sensor fusion through an extended Kalman Filter implmentation that fuses Laser data with radar data to track the 2D cartesian position and velocity of a target object.

The laser measurements are processed through a linear Kalman filter with constant acceleration motion model.

The radar measurements are processed through an Extended Kalman filter with linearized approximation of the measurement matrix using the Jacobian.

The prediction step of the filter was processed with a linear motion model, and thus implemented with a Linear Kalman filter.

## Modifications to the Code

The major modifications to the code were the changes to the tools.h function to implement static methods instead of classes.

## Dependencies

This code depends on
```bash
libuv1-dev libssl-dev gcc g++ cmake make
```
and uWebSockets
```
https://github.com/uWebSockets/uWebSockets 
```

The simulator used for this program is available [here](https://github.com/udacity/self-driving-car-sim/releases)

## Results

The code successfully compiled and produced RMSE errors within the thresholds of (0.11,0.11,0.52,0.52).
