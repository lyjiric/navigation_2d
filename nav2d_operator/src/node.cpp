#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav2d_operator/RobotOperator.h>
#include <dynamic_reconfigure/server.h>
#include <nav2d_operator/paraConfig.h>
#include <nav_msgs/GridCells.h>

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
//bool gActionSaveParameterSet;

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
  
//  gActionSaveParameterSet = config.action_save;

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
   
   RobotOperator robOp;

   double frequency;
   double max_free_space;
   double safety_decay;
   double max_velocity;
   int conformance_weight;
   int safety_weight;
   int continue_weight;
   int escape_weight;
   bool publish_route;

// Parameter initialization via parameter server
   operatorNode.param("frequency", frequency, 0.2);
   ROS_INFO("Operator is initialized to run at %.2f Hz.", frequency);
   operatorNode.param("publish_route", publish_route, false);
   operatorNode.param("max_free_space", max_free_space, 5.0);
   operatorNode.param("safety_decay", safety_decay, 0.95);
   operatorNode.param("safety_weight", safety_weight, 1);
   operatorNode.param("conformance_weight", conformance_weight, 1);
   operatorNode.param("continue_weight", continue_weight, 1);
   operatorNode.param("escape_weight", escape_weight, 1);
   operatorNode.param("max_velocity", max_velocity, 1.0);
   
   robOp.setPublishRoute(publish_route);
   robOp.setMaxFreeSpace(max_free_space);
   robOp.setSafetyDecay(safety_decay);
   robOp.setSafetyWeight(safety_weight);
   robOp.setConformanceWeight(conformance_weight);
   robOp.setContinueWeight(continue_weight);
   robOp.setEscapeWeight(escape_weight);
   robOp.setMaxVelocity(max_velocity);
  
   dynamic_reconfigure::Server<nav2d_operator::paraConfig>
   server;

   dynamic_reconfigure::Server<nav2d_operator::paraConfig>::
   CallbackType f;

   f = boost::bind(&callback, _1, _2);
   server.setCallback(f);
   
   Rate loopRate(frequency);
   ROS_INFO("Operator is initialized to run at %.2f Hz.", frequency);


   if(publish_route)
       {
          ROS_INFO("Will publish desired direction on '%s' and control direction on '%s'.", ROUTE_TOPIC, PLAN_TOPIC);
         robOp.setPublisherTopics(operatorNode.advertise<nav_msgs::GridCells>(ROUTE_TOPIC, 1), operatorNode.advertise<nav_msgs::GridCells>(PLAN_TOPIC, 1));         
       }


   while(ok())
   {
       spinOnce();

       //Adaption via GUI
       if (gPublishRoute!=publish_route)
       {
          robOp.setPublishRoute(gPublishRoute);
//         ROS_INFO("rqt_dynreconfig: PublishRoute=%d", gPublishRoute);
          if(gPublishRoute)
          {
            ROS_INFO("Will publish desired direction on '%s' and control direction on '%s'.", ROUTE_TOPIC, PLAN_TOPIC);
            robOp.setPublisherTopics(operatorNode.advertise<nav_msgs::GridCells>(ROUTE_TOPIC, 1), operatorNode.advertise<nav_msgs::GridCells>(PLAN_TOPIC, 1));         
          }
         publish_route = gPublishRoute;
       } 

       if(gMaxFreeSpace != max_free_space)
       {
         robOp.setMaxFreeSpace(gMaxFreeSpace);
//         ROS_INFO("rqt_dynreconfig: MaxFreeSpace=%.2f", gMaxFreeSpace);
         max_free_space=gMaxFreeSpace;
       } 

       if(gSafetyDecay!= safety_decay)
       {        
         robOp.setSafetyDecay(gSafetyDecay);
//         ROS_INFO("rqt_dynreconfig: SafetyDecay=%.2f", gSafetyDecay);
         safety_decay = gSafetyDecay;
       }       

       if(gConformanceWeight!=conformance_weight)
       {
         robOp.setConformanceWeight(gConformanceWeight);
//         ROS_INFO("rqt_dynreconfig: ConformanceWeight=%d", gConformanceWeight);
         conformance_weight=gConformanceWeight;
       }

       if(gContinueWeight!=continue_weight)
       {
         robOp.setContinueWeight(gContinueWeight);
//         ROS_INFO("rqt_dynreconfig: ContinueWeight=%d", gContinueWeight);
         continue_weight=gContinueWeight;
       }
       
      if(gSafetyWeight!=safety_weight)
       {
         robOp.setSafetyWeight(gSafetyWeight);
//         ROS_INFO("rqt_dynreconfig: SafetyWeight=%d", gSafetyWeight);
         safety_weight=gSafetyWeight;
       }

      if(gEscapeWeight!=escape_weight)
      {
         robOp.setEscapeWeight(gEscapeWeight);
//         ROS_INFO("rqt_dynreconfig: EscapeWeight=%d", gEscapeWeight);
         escape_weight = gEscapeWeight;
      }      

      if(gMaxVelocity!=max_velocity)
      {
         robOp.setMaxVelocity(gMaxVelocity);
//         ROS_INFO("rqt_dynreconfig: MaxVeloctiy=%.2f", gMaxVelocity);
         max_velocity=gMaxVelocity;
      }	     

      if((gFrequency!=frequency)&&(gFrequency!=0))
      {
         frequency = gFrequency;
         Rate loopRate(frequency);
         ROS_INFO("rqt_dynreconfig: Frequency is changed to %.2f Hz", gFrequency); 
         frequency=gFrequency;     
      }	     
       
    /* if(gActionSaveParameterSet)
     {
       operatorNode.setParam("max_velocity", gMaxVelocity);
       ROS_INFO("OperatorNode: current parameter set is saved locally.");
       gActionSaveParameterSet = false;
     }
*/
                               
      robOp.executeCommand();
       
      loopRate.sleep();
      
      if(loopRate.cycleTime() > ros::Duration(1.0 / frequency))
			ROS_WARN("Missed desired rate of %.2f Hz! Loop actually took %.4f seconds!",frequency, loopRate.cycleTime().toSec());
  }//end of while loop

	return 0;	
}
