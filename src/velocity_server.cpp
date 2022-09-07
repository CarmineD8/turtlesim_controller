#include "ros/ros.h"
#include "turtlesim_controller/Velocity.h"


double randMToN(double M, double N)
{return M + (rand() / ( RAND_MAX / (N-M) )) ; }


bool myrandom(turtlesim_controller::Velocity::Request &req, turtlesim_controller::Velocity::Response &res){
res.x = randMToN(req.min, req.max);
res.z = randMToN(req.min, req.max);
return true;
}




int main(int argc, char **argv)
{
ros::init(argc, argv, "velocity_server");
ros::NodeHandle n;
ros::ServiceServer service= n.advertiseService("/vel", myrandom);
ros::spin();

return 0;
}

