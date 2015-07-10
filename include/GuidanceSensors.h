#ifndef __GUIDANCE_SENSORS_H__
#define __GUIDANCE_SENSORS_H__

#define NUM_CAM_PAIRS 5

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <DJI_guidance.h>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <DJI_utility.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/fill_image.h"
#include "sensor_msgs/Imu.h"
#include "m100_sensors/Sonar.h"

typedef e_image_data_frequecy e_image_data_frequency; // Dealing with typo in SDK
class GuidanceSensors
{
  public:
    GuidanceSensors( ros::NodeHandle nRosNode );
    GuidanceSensors( ros::NodeHandle nRosNode, int numCamPairs_, int* camPairs_ );
    ~GuidanceSensors();
    int initializeTopics();
    int startSampleData();
    int stopSampleData();

    void lockGuidance();
    void unlockGuidance();

    void copyImages( image_data imData );
    void publishImages();

    void publishSonar( sonar_data* sonar );
    void publishIMU( imu_guidance* imu );

    void setNumCamPairs( int numPairs, int* pairs );
  private:
    int numCamPairs;
    int* camPairs;

    void initializeCameras();
    void destroyTopics();

    // Dealing with typo in SDK.
    int set_image_frequency( e_image_data_frequency frequency ) { return set_image_frequecy( frequency ); };

    struct RectifiedImage
    {
      std::string strTopicName;
      image_transport::CameraPublisher imagePub;
      sensor_msgs::CameraInfo cameraInfo;
      sensor_msgs::Image msgsImg;
      IplImage* imageData;
    };
    RectifiedImage* rectifiedTopicL;
    RectifiedImage* rectifiedTopicR;

    struct DepthImage
    {
      std::string strTopicName;
      std::string strConvTopicName;
      image_transport::Publisher imageDepthPub;
      IplImage* imageData;
      sensor_msgs::Image msgsImg;
      IplImage* convertedImageData;
      sensor_msgs::Image msgsConvImg;
    };
    DepthImage* depthTopic;

    struct SonarData
    {
      std::string strTopicName;
      ros::Publisher sonarPub;
      m100_sensors::Sonar msg;
    };
    SonarData* sonarTopic;

    struct IMUData
    {
      std::string strTopicName;
      ros::Publisher imuPub;
      sensor_msgs::Imu msg;
    };
    IMUData imuTopic;
    // TODO: Add Velocity, IMU, and Obstacle avoidance topics.

    image_transport::ImageTransport nImageTransport;
    ros::NodeHandle nROSNode;
    lock guidanceLock;
    //event guidanceEvent;
};


#endif

