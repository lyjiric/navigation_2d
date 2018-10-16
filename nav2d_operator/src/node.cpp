#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav2d_operator/RobotOperator.h>
#include <dynamic_reconfigure/server.h>
#include <nav2d_operator/paraConfig.h>
#include <nav_msgs/GridCells.h>

using namespace ros;

// Global parameters:
RobotOperator gRobOp;
double gFrequency;
bool gPublishRoute;

// Parameter adjustment via GUI
void callback(nav2d_operator::paraConfig &config, uint32_t level)
{
   gFrequency = config.frequency;
   gRobOp.setPublishRoute(config.publish_route);
   gPublishRoute = config.publish_route;
   gRobOp.setMaxFreeSpace(config.max_free_space);
   gRobOp.setSafetyDecay(config.safety_decay);
   gRobOp.setSafetyWeight(config.safety_weight);
   gRobOp.setConformanceWeight(config.conformance_weight);
   gRobOp.setContinueWeight(config.continue_weight);
   gRobOp.setEscapeWeight(config.escape_weight);
   gRobOp.setMaxVelocity(config.max_velocity);

/*
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
*/
}

int main(int argc, char **argv)
{
   init(argc, argv, NODE_NAME);
   NodeHandle operatorNode("~/");
   double frequency; 
  
   dynamic_reconfigure::Server<nav2d_operator::paraConfig>
   server;

   dynamic_reconfigure::Server<nav2d_operator::paraConfig>::
   CallbackType f;

   f = boost::bind(&callback, _1, _2);
   server.setCallback(f);
   
   if(gFrequency!=0)
   { 
     frequency = gFrequency;
     Rate loopRate(frequency);
     ROS_INFO("Operator is initialized to run at %.2f Hz.", frequency);
   }
   else ROS_ERROR("Operator has a loop rate zero!");

   while(ok())
   {
       spinOnce();

       //Adaption via GUI
//         ROS_INFO("rqt_dynreconfig: PublishRoute=%d", gPublishRoute);
      if(gPublishRoute)
      {
         ROS_INFO("Will publish desired direction on '%s' and control direction on '%s'.", ROUTE_TOPIC, PLAN_TOPIC);
         gRobOp.setPublisherTopics(operatorNode.advertise<nav_msgs::GridCells>(ROUTE_TOPIC, 1), operatorNode.advertise<nav_msgs::GridCells>(PLAN_TOPIC, 1));         
      }

      if((gFrequency!=0)&& (gFrequency!= frequency))
      {
         Rate loopRate(gFrequency);
         loopRate.sleep();
         ROS_INFO("rqt_dynreconfig: Frequency is changed to %.2f Hz", gFrequency); 
         frequency=gFrequency;     
         
         if(loopRate.cycleTime() > ros::Duration(1.0 / frequency))
			ROS_WARN("Missed desired rate of %.2f Hz! Loop actually took %.4f seconds!",frequency, loopRate.cycleTime().toSec());
      }	     
      else ROS_ERROR("Operator has a loop rate zero!");
     
      gRobOp.executeCommand();
       
      
  }//end of while loop

	return 0;	
}
