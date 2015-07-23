#include "GuidanceSensors.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getSensors");
  ros::NodeHandle node_;

  //GuidanceSensors* mGuidanceSensors = new GuidanceSensors( node_ );
  int camNums[5];
  for( int i=0; i<5; i++ )
  {
    camNums[i] = i;
  }
  GuidanceSensors* mGuidanceSensors = new GuidanceSensors( node_, 5, camNums );
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
