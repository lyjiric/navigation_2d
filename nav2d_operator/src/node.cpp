#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav2d_operator/RobotOperator.h>
#include <dynamic_reconfigure/server.h>
#include <nav2d_operator/paraConfig.h>
#include <nav_msgs/GridCells.h>

using namespace ros;

// Global parameters:
RobotOperator* gPointer;
double gFrequency;

// Parameter adjustment via GUI
void callback(nav2d_operator::paraConfig &config, uint32_t level)
{
   gFrequency=config.frequency;
   if(gPointer->getPublishRoute() != config.publish_route) 
	gPointer->setPublishRoute(config.publish_route);
   if(gPointer->getMaxFreeSpace() != config.max_free_space)
	gPointer->setMaxFreeSpace(config.max_free_space);
   if(gPointer->getSafetyDecay() != config.safety_decay)
       gPointer->setSafetyDecay(config.safety_decay);
   if(gPointer->getSafetyWeight() != config.safety_weight)
       gPointer->setSafetyWeight(config.safety_weight);
   if(gPointer->getConformanceWeight() !=config.conformance_weight) 
       gPointer->setConformanceWeight(config.conformance_weight);
   if(gPointer->getContinueWeight() !=config.continue_weight)
       gPointer->setContinueWeight(config.continue_weight);
   if(gPointer->getEscapeWeight() !=config.escape_weight)
       gPointer->setEscapeWeight(config.escape_weight);
   if(gPointer->getMaxVelocity() !=config.max_velocity)
       gPointer->setMaxVelocity(config.max_velocity);

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
   
   RobotOperator RobOp;
   gPointer = &RobOp;
   
   double frequency; 
  
   dynamic_reconfigure::Server<nav2d_operator::paraConfig>
   server;

   dynamic_reconfigure::Server<nav2d_operator::paraConfig>::
   CallbackType f;

   f = boost::bind(&callback, _1, _2);
   server.setCallback(f);
 
   if(gFrequency==0)
   {
     ROS_ERROR("Operator receives a setting of loop frequency of zero! Set to 1Hz.");
     frequency = 1;
   }
  else
  {
    ROS_INFO("Operator frequency is initialized to %.2f Hz", gFrequency); 
    frequency = gFrequency; 
  }
  
  Rate loopRate(frequency);

 while(ok())
   {
       spinOnce();

      if((gFrequency!=frequency) && (gFrequency!=0))
      {
         ROS_INFO("rqt_dynreconfig: Frequency is changed to %.2f Hz", gFrequency); 
         Rate loopRate(gFrequency);
         frequency = gFrequency;
      }
      
      if(loopRate.cycleTime() > ros::Duration(1.0 / frequency))
			ROS_WARN("Missed desired rate of %.2f Hz! Loop actually took %.4f seconds!",frequency, loopRate.cycleTime().toSec());
     
      gPointer->executeCommand();
       
      loopRate.sleep();
      
  }//end of while loop

	return 0;	
}

