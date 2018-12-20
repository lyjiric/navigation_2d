#include <ros/ros.h>

#include <nav2d_navigator/RobotNavigator.h>
#include <nav2d_navigator/para_navConfig.h>

// Global parameters:
RobotNavigator* gPointer;

// Parameter adjustment via GUI
void callback(nav2d_navigator::para_navConfig &config, uint32_t level)
{
   if(gPointer->getFrequency() != config.frequency) 
	gPointer->setFrequency(config.frequency);

   if(gPointer->getInflationRadius() != config.map_inflation_radius)
	gPointer->setInflationRadius(config.map_inflation_radius);

   if(gPointer->getRobotRadius() != config.robot_radius)
       gPointer->setRobotRadius(config.robot_radius);

   if(gPointer->getCommandTargetDistance() != config.command_target_distance)
       gPointer->setCommandTargetDistance(config.command_target_distance);

   if(gPointer->getNavigationGoalDistance() != config.navigation_goal_distance) 
       gPointer->setNavigationGoalDistance(config.navigation_goal_distance);

   if(gPointer->getNavigationGoalAngle() !=config.navigation_goal_angle)
       gPointer->setNavigationGoalAngle(config.navigation_goal_angle);

   if(gPointer->getNavigationHomingDistance() !=config.navigation_homing_distance)
       gPointer->setNavigationHomingDistance(config.navigation_homing_distance);

   if(gPointer->getExplorationGoalDistance() !=config.exploration_goal_distance)
       gPointer->setExplorationGoalDistance(config.exploration_goal_distance);

   if(gPointer->getMinReplanningPeriod() !=config.min_replanning_period)
       gPointer->setMinReplanningPeriod(config.min_replanning_period);
   
  if(gPointer->getMaxReplanningPeriod() !=config.max_replanning_period)
       gPointer->setMaxReplanningPeriod(config.max_replanning_period);
 
// ROS_INFO("Reconfigure Request from Navigator GUI");

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Navigator");
	ros::NodeHandle n;
	
	RobotNavigator robNav;
	
	ros::spin();
	return 0;
}
