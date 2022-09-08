#include "ros/ros.h"
#include "turtlesim_controller/Harmonic.h"
#include "cmath"

#define pi 3.14



bool setvel(turtlesim_controller::Harmonic::Request &req, turtlesim_controller::Harmonic::Response &res){
res.vel = 0.1 + 2*sin (pi*req.x/7 -2*pi/7);
return true;
}




int main(int argc, char **argv)
{
ros::init(argc, argv, "velocity_server");
ros::NodeHandle n;
ros::ServiceServer service= n.advertiseService("/harmonic", setvel);
ros::spin();

return 0;
}

