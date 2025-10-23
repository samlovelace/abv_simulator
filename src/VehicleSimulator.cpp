
#include "abv_simulator/VehicleSimulator.h"
#include "abv_simulator/RosTopicManager.h"
#include "robot_idl/msg/abv_state.hpp"
#include "robot_idl/msg/vec3.hpp"
#include <string> 


VehicleSimulator::VehicleSimulator(/* args */) : 
    mUdpServer(std::make_unique<UdpServer>(6969, std::bind(&VehicleSimulator::onRecieved, this, std::placeholders::_1))), 
    mMass(12.7), mIzz(0.35), mThrusterForce(0.15), mMomentArm(0.1), mTimestep(0.05), mDamping(0.0005),
    mVelocity(Eigen::Vector2d::Zero()), mThrustForce(Eigen::Vector3d::Zero()), 
    mTopicManager(std::make_unique<RosTopicManager>())
{ 
    
}

VehicleSimulator::~VehicleSimulator()
{
}

void VehicleSimulator::listen()
{
    mUdpServer->start(); 
    mTopicManager->createPublisher<robot_idl::msg::AbvState>("abv/sim/state"); 
    mTopicManager->spinNode(); 
}

void VehicleSimulator::onRecieved(const std::string& message)
{
    setThrusterCommand(message);  
}

void VehicleSimulator::update()
{
    convertThrusterCommandToForce(getThrusterCommand()); 
    Eigen::Vector3d thrustForceVec_Gl = convertBodyForceToGlobal(); 
    Eigen::Vector2d Fg(thrustForceVec_Gl.x(), thrustForceVec_Gl.y());
    double tau = thrustForceVec_Gl.z(); // torque about vertical axis 

    // compute accelerations based on force/torque applied 
    Eigen::Vector2d a = (1.0 / mMass) * Fg - Eigen::Vector2d(0.5*mDamping * mVelocity.x(), 0.5*mDamping * mVelocity.y()); // linear acceleration
    double alpha = (tau / mIzz) - mDamping * mVehicleState.omega; // angular acceleration 
    
    // update velocity 
    mVelocity += a * mTimestep;
    mVehicleState.omega += alpha * mTimestep;

    // update positions 
    mVehicleState.x += mVelocity.x() * mTimestep;
    mVehicleState.y += mVelocity.y() * mTimestep;
    mVehicleState.yaw  = wrapPi(mVehicleState.yaw + mVehicleState.omega * mTimestep);

    mTopicManager->publishMessage<robot_idl::msg::AbvState>("abv/sim/state", convertToIdl(mVehicleState)); 
}

robot_idl::msg::AbvState VehicleSimulator::convertToIdl(VehicleState aState)
{
    robot_idl::msg::Vec3 position; 
    robot_idl::msg::Vec3 velocity;

    position.x = aState.x; 
    position.y = aState.y; 
    position.z = 0.0; 

    velocity.x = aState.vx; 
    velocity.y = aState.vy; 
    velocity.z = 0.0; 

    robot_idl::msg::Vec3 orientation; 
    orientation.x = 0.0; 
    orientation.y = 0.0; 
    orientation.z = aState.yaw; 

    robot_idl::msg::Vec3 ang_vel; 
    ang_vel.x = 0.0; 
    ang_vel.y = 0.0; 
    ang_vel.z = aState.omega; 

    robot_idl::msg::AbvState state; 
    state.set__position(position); 
    state.set__velocity(velocity); 
    state.set__orientation(orientation); 
    state.set__ang_vel(ang_vel); 

    return state;  
}

Eigen::Vector3d VehicleSimulator::convertBodyForceToGlobal()
{
    double yaw = mVehicleState.yaw; 

    // rotation of abv relative to global 
    Eigen::Matrix3d Rz;
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw),  0,
             0,        0,      1;

    // Transform the vector into the new frame
    return Rz * mThrustForce;
}

inline double VehicleSimulator::wrapPi(double a)
{
    while(a <= -M_PI) a += 2.0*M_PI;
    while(a >   M_PI) a -= 2.0*M_PI;
    return a;
}

void VehicleSimulator::convertThrusterCommandToForce(const std::string& thrusterCommand)
{   
    // assume the thruster command comes in as a string of 0's and 1's
    // 0 = off, 1 = on
    // thrusterCommand = "00000000" means all thrusters are off
    // thrusterCommand = "10000000" means thruster 1 is on, all others are off

    mThrustForce = Eigen::Vector3d::Zero(); // Default case

    if (thrusterCommand == "00000011") {
        mThrustForce = Eigen::Vector3d(2*mThrusterForce, 0, 0); // +x
    } 
    else if (thrusterCommand == "00110000") 
    {
        mThrustForce = Eigen::Vector3d(-2*mThrusterForce, 0, 0); // -x
    } 
    else if (thrusterCommand == "11000000") // +y 
    {
        mThrustForce = Eigen::Vector3d(0, 2*mThrusterForce, 0);
    } 
    else if (thrusterCommand == "00001100") // -y  
    {
        mThrustForce = Eigen::Vector3d(0, -2*mThrusterForce, 0);
    } 
    else if (thrusterCommand == "01000100") // +phi  
    {
        mThrustForce = Eigen::Vector3d(0, 0, 2*mThrusterForce*mMomentArm); 
    } 
    else if (thrusterCommand == "10001000") // -phi
    {
        mThrustForce = Eigen::Vector3d(0, 0, -2*mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "01000010") // +x, +y
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, mThrusterForce, 0);
    } 
    else if (thrusterCommand == "00001001") // +x, -y
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, -mThrusterForce, 0);
    } 
    else if (thrusterCommand == "10010000") // -x, +y
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, mThrusterForce, 0); 
    } 
    else if (thrusterCommand == "00100100") // -x, -y
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, -mThrusterForce, 0);
    } 
    else if (thrusterCommand == "00000001") // +x, +phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, 0, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00000010") // +x, -phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, 0, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00010000") // -x, +phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, 0, mThrusterForce*mMomentArm);
    }
    else if (thrusterCommand == "00100000") // -x, -phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, 0, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "01000000") // +y, +phi
    {
        mThrustForce = Eigen::Vector3d(0, mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "10000000") // +y, -phi
    {
        mThrustForce = Eigen::Vector3d(0, mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00000100") //-y, +phi
    {
        mThrustForce = Eigen::Vector3d(0, -mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00001000") //-y, -phi
    {
        mThrustForce = Eigen::Vector3d(0, -mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "01000001") // +x, +y, +phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00000101") // +x, -y, +phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, -mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "01010000") // -x, +y, +phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00010100") // -x, -y, +phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, -mThrusterForce, mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "10000010") // +x, +y, -phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00001010") // +x, -y, -phi
    {
        mThrustForce = Eigen::Vector3d(mThrusterForce, -mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "10100000") // -x, +y, -phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00101000") // -x, -y, -phi
    {
        mThrustForce = Eigen::Vector3d(-mThrusterForce, -mThrusterForce, -mThrusterForce*mMomentArm);
    } 
    else if (thrusterCommand == "00000000") // all off
    {
        mThrustForce = Eigen::Vector3d::Zero();
    }
    else 
    {
        mThrustForce = Eigen::Vector3d::Zero();
    }

}