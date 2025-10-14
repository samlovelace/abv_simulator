#ifndef VEHICLE_STATE_HPP
#define VEHICLE_STATE_HPP

// Struct to represent the vehicle's state
struct VehicleState {
    double x;        // Position in the x-direction
    double y;        // Position in the y-direction
    double yaw;      // Yaw angle (orientation) in radians
    double vx;       // Velocity in the x-direction
    double vy;       // Velocity in the y-direction
    double omega;    // Angular velocity about the z-axis
};

#endif // VEHICLE_STATE_HPP