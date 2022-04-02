
// include
#include "robot305_localization/robot305_localization.h"

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "robot305_localization");

  //create ros wrapper object
  robotis_op::ROBOT305Localization robot305_localization;

  //set node loop rate
  ros::Rate loop_rate(10);

  //node loop
  while ( ros::ok() )
  {
    robot305_localization.process();

    //execute pending callbacks
    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}
