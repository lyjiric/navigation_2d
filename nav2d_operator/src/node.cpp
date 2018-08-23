#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav2d_operator/RobotOperator.h>
#include <dynamic_reconfigure/server.h>
#include <nav2d_operator/paraConfig.h>

using namespace ros;

double gFrequency;
double gMaxFreeSpace;
double gSafetyDecay;
double gMaxVelocity;
int gConformanceWeight;
int gSafetyWeight;
int gContinueWeight;
int gEscapeWeight;
bool gPublishRoute;

// Parameter adjustment via GUI
void callback(nav2d_operator::paraConfig &config, uint32_t level)
{
  gFrequency = config.frequency;
  gPublishRoute = config.publish_route;
  gMaxFreeSpace = config.max_free_space;
  gSafetyDecay = config.safety_decay;
  gSafetyWeight = config.safety_weight;
  gConformanceWeight = config.conformance_weight;
  gContinueWeight = config.continue_weight;
  gEscapeWeight = config.escape_weight;
  gMaxVelocity = config.max_velocity;

  ROS_INFO("Reconfigure Request from GUI: %f %s %f %f %d %d %d %d %f",
              config.frequency,
              config.publish_route?"True":"False",
              config.max_free_space,
              config.safety_decay,
              config.safety_weight,
              config.conformance_weight,
              config.continue_weight,
              config.escape_weight,
              config.max_velocity);
}

int main(int argc, char **argv)
{
   dynamic_reconfigure::Server<nav2d_operator::paraConfig>
   server;

   dynamic_reconfigure::Server<nav2d_operator::paraConfig>::
   CallbackType f;

   ros::init(argc, argv, "node");
	NodeHandle n("~/");

	double frequency;
 
        n.param("frequency", frequency, 100.0);
	ROS_INFO("Operator will run at %.2f Hz.", frequency);

	RobotOperator robOp;
	
	Rate loopRate(gFrequency);
	while(ok())
	{
	     spinOnce();

             f = boost::bind(&callback, _1, _2);
             server.setCallback(f);
              
             robOp.setPublishRoute(gPublishRoute);
             robOp.setMaxFreeSpace(gMaxFreeSpace);
             robOp.setSafetyDecay(gSafetyDecay);
             robOp.setSafetyWeight(gSafetyWeight);
             robOp.setConformanceWeight(gConformanceWeight);
             robOp.setEscapeWeight(gEscapeWeight);
             robOp.setMaxVelocity(gMaxVelocity);
	     
             robOp.executeCommand();
	     loopRate.sleep();
	     if(loopRate.cycleTime() > ros::Duration(1.0 / frequency))
			ROS_WARN("Missed desired rate of %.2f Hz! Loop actually took %.4f seconds!",frequency, loopRate.cycleTime().toSec());
	}
	return 0;	
}
