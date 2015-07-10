#include "GuidanceSensors.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getSensors");
  ros::NodeHandle node_;

  //GuidanceSensors* mGuidanceSensors = new GuidanceSensors( node_ );
  int camNums[2];
  camNums[0] = 0;
  camNums[1] = 1;
  GuidanceSensors* mGuidanceSensors = new GuidanceSensors( node_, 2, camNums );
  if ( mGuidanceSensors->startSampleData() == -1 )
  {
    delete mGuidanceSensors;
    return EXIT_FAILURE;
  }

  while( ros::ok() )
  {
    ros::spinOnce();
  }

  //mGuidanceSensors->stopSampleData();

  delete mGuidanceSensors;


  return EXIT_SUCCESS;
}
