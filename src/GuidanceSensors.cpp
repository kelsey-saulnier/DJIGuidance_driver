#include <stdio.h>
#include <typeinfo>
#include "GuidanceSensors.h"

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define IMAGE_SIZE (IMAGE_WIDTH * IMAGE_HEIGHT)
#define NUM_SENSORS 5
#define CAM_F 262.3829708
#define CAM_CX 164.4641857
#define CAM_CY 117.2315998
#define CAM_BASELINE 0.1505004

GuidanceSensors* g_guidanceSensors;

int dataCallback( int eventType, int dataLength, char*data )
{
  g_guidanceSensors->lockGuidance();
  // Image data event
  if( eventType == e_image_guidance && data != NULL )
  {
    fprintf(stderr,"Starting callback...%lu",time(0));
    image_data imData;
    memcpy( (char*)&imData, data, dataLength );

    g_guidanceSensors->copyImages( imData );
    g_guidanceSensors->publishImages();
  }
  if( eventType == e_sonar_guidance && data != NULL )
  {
    sonar_data *sonar = (sonar_data*)data;
    g_guidanceSensors->publishSonar( sonar );
  }
  if( eventType == e_imu_guidance && data != NULL )
  {
    imu_guidance *imu = (imu_guidance*)data;
    g_guidanceSensors->publishIMU( imu );
  }
  g_guidanceSensors->unlockGuidance();
  return 0;
}

  GuidanceSensors::GuidanceSensors( ros::NodeHandle nRosNode )
:nImageTransport(nRosNode),nROSNode(nRosNode)
{

  fprintf(stderr, "Initializing...");
  int errorCode = init_transfer();
  if(errorCode != 0)
  {
    fprintf(stderr, "Error initiating transfer. File: %s, Line: %d.",
        __FILE__,__LINE__);
    return;
  }
  fprintf(stderr, "Transfer Initialized\n\r");
  numCamPairs = 5;
  camPairs =  new int[numCamPairs];
  for( int camIdx = 0; camIdx < numCamPairs; camIdx++ )
  {
    camPairs[camIdx] = camIdx;
  }
  fprintf(stderr, "Cameras allocated\n\r");
  initializeCameras();
  fprintf(stderr, "setting image freq: %d\n\r",set_image_frequency( (e_image_data_frequecy)5 ));
  fprintf(stderr, "And initialized...\n\r");
  fprintf(stderr, "Frequency set\n\r");
  g_guidanceSensors = this;
  initializeTopics();
  fprintf(stderr, "Initialized topics\n\r");
  set_sdk_event_handler( dataCallback );
  fprintf(stderr, "done.\n\r");
}

GuidanceSensors::GuidanceSensors( ros::NodeHandle nRosNode,
    int numCamPairs_,
    int* camPairs_ ):
  nImageTransport(nRosNode),
  nROSNode(nRosNode),
  numCamPairs(numCamPairs_)
{
  int errorCode = init_transfer();
  if(errorCode != 0)
  {
    fprintf(stderr, "Error initiating transfer. File: %s, Line: %d.",
        __FILE__,__LINE__);
    return;
  }
  camPairs = new int[numCamPairs];
  for( int camIdxNum = 0; camIdxNum < numCamPairs; camIdxNum++ )
  {
    camPairs[camIdxNum] = camPairs_[camIdxNum];
  }
  initializeCameras();
  // NOTE: As far as I can tell this image data frequency thing is not
  // implemented.
  if( numCamPairs == 1 )
  {
    set_image_frequency( (e_image_data_frequecy)20 );
  }
  else if( numCamPairs <= 3 )
  {
    set_image_frequency( (e_image_data_frequecy)10 );
  }
  else
  {
    set_image_frequency( (e_image_data_frequecy)5 );
  }
  g_guidanceSensors = this;
  initializeTopics();
  set_sdk_event_handler( dataCallback );
}

GuidanceSensors::~GuidanceSensors()
{
  stopSampleData();
  destroyTopics();
}

int GuidanceSensors::initializeTopics()
{
  fprintf(stderr, "\tinitializetopics, %d\n\r", numCamPairs);
  //rectifiedTopicL = ( struct RectifiedImage *)calloc( numCamPairs, sizeof( struct RectifiedImage ) );
  rectifiedTopicL = new RectifiedImage[numCamPairs];
  rectifiedTopicR = new RectifiedImage[numCamPairs];
  depthTopic = new DepthImage[numCamPairs];
  sonarTopic = new SonarData[NUM_SENSORS];
fprintf(stderr, "\tmalloc complete\n\r");
  std::ostringstream ostr;
  int camIdxNum;
  for( int camIdx = 0; camIdx<numCamPairs; camIdx++ )
  {
    camIdxNum = camPairs[camIdx];
    fprintf(stderr, "idx:%d, cam:%d   :   ", camIdx, camIdxNum);
    // Topic names.
    ostr << "/sensor/" << camIdxNum << "/rectified_left";
    fprintf(stderr, "One: %lu, All: %lu, Str: %lu, %s\n\r", sizeof(RectifiedImage), sizeof(rectifiedTopicL[camIdx]).strTopicName, sizeof(ostr.str()), typeid(rectifiedTopicL[camIdx].strTopicName).name());
    std::string strTopicName = ostr.str();
    fprintf(stderr, "doneS \n\r");
    rectifiedTopicL[camIdx].strTopicName = strTopicName;
    fprintf(stderr, "doneL \n\r");
    ostr.str("");
    ostr.clear();
    ostr << "/sensor/" << camIdxNum << "/rectified_right";
    rectifiedTopicR[camIdx].strTopicName = ostr.str();
    ostr.str("");
    ostr.clear();
    ostr << "/sensor/" << camIdxNum << "/depth";
    depthTopic[camIdx].strTopicName = ostr.str();
    ostr.str("");
    ostr.clear();
    ostr << "/sensor/" << camIdxNum << "/scaledDepth";
    depthTopic[camIdx].strConvTopicName = ostr.str();
    ostr.str("");
    ostr.clear();

    // Advertise
    rectifiedTopicL[camIdx].imagePub = nImageTransport.advertiseCamera(
        rectifiedTopicL[camIdx].strTopicName, 1);
    rectifiedTopicR[camIdx].imagePub = nImageTransport.advertiseCamera(
        rectifiedTopicR[camIdx].strTopicName, 1);
    depthTopic[camIdx].imageDepthPub = nImageTransport.advertise(
        depthTopic[camIdx].strTopicName, 1);

    // CameraInfo
    rectifiedTopicL[camIdx].cameraInfo.header.frame_id =
      rectifiedTopicL[camIdx].strTopicName;
    rectifiedTopicL[camIdx].cameraInfo.height = IMAGE_HEIGHT;
    rectifiedTopicL[camIdx].cameraInfo.width = IMAGE_WIDTH;
    rectifiedTopicR[camIdx].cameraInfo.header.frame_id =
      rectifiedTopicR[camIdx].strTopicName;
    rectifiedTopicR[camIdx].cameraInfo.height = IMAGE_HEIGHT;
    rectifiedTopicR[camIdx].cameraInfo.width = IMAGE_WIDTH;
    /*
       rectifiedTopicL[camIdx].cameraInfo.distortion_model = "plumb_bob";
       rectifiedTopicR[camIdx].cameraInfo.distortion_model = "plumb_bob";
       rectifiedTopicL[camIdx].cameraInfo.K[0] = CAM_F;
       rectifiedTopicL[camIdx].cameraInfo.K[2] = CAM_CX;
       rectifiedTopicL[camIdx].cameraInfo.K[4] = CAM_F;
       rectifiedTopicL[camIdx].cameraInfo.K[5] = CAM_CY;
       rectifiedTopicL[camIdx].cameraInfo.K[8] = 1;
       rectifiedTopicR[camIdx].cameraInfo.K[0] = CAM_F;
       rectifiedTopicR[camIdx].cameraInfo.K[2] = CAM_CX;
       rectifiedTopicR[camIdx].cameraInfo.K[4] = CAM_F;
       rectifiedTopicR[camIdx].cameraInfo.K[5] = CAM_CY;
       rectifiedTopicR[camIdx].cameraInfo.K[8] = 1;
    // Do I really need this stufff?

    rectifiedTopicL[camIdx].msgsImg.header.frame_id =
    rectifiedTopicL[camIdx].strTopicName;
    rectifiedTopicR[camIdx].msgsImg.header.frame_id =
    rectifiedTopicR[camIdx].strTopicName;
    depthTopic[camIdx].msgsImg.header.frame_id =
    depthTopic[camIdx].strTopicName;
    */
    rectifiedTopicL[camIdx].imageData = cvCreateImage(
        cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
    rectifiedTopicR[camIdx].imageData = cvCreateImage(
        cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
    depthTopic[camIdx].imageData = cvCreateImage(
        cvSize(IMAGE_WIDTH,IMAGE_HEIGHT),IPL_DEPTH_32F, 1);
    depthTopic[camIdx].convertedImageData = cvCreateImage(
        cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 1);

  }
  // Set up sonar topics.
  for( int camIdx = 0; camIdx < NUM_SENSORS; camIdx++ )
  {
    ostr << "/sensor/" << camIdx << "/sonar";
    sonarTopic[camIdx].strTopicName = ostr.str();
    ostr.str("");
    ostr.clear();

    sonarTopic[camIdx].sonarPub = nROSNode.advertise<m100_sensors::Sonar>( sonarTopic[camIdx].strTopicName, (uint32_t)1);
    }

  // Set up imu topics
  ostr << "/sensor/imu";
  imuTopic.strTopicName = ostr.str();
  ostr.str("");
  ostr.clear();
  imuTopic.msg.header.frame_id = imuTopic.strTopicName;
  imuTopic.imuPub = nROSNode.advertise<sensor_msgs::Imu>( imuTopic.strTopicName, 1 );

  return 0;
}

int GuidanceSensors::startSampleData()
{
  int errorCode = start_transfer();
  if( errorCode != 0 )
  {
    return -1;
  }
  return 0;
}

int GuidanceSensors::stopSampleData()
{
  for( int camIdx = 0; camIdx < numCamPairs; camIdx++ )
  {
    cvReleaseImage( &rectifiedTopicL[camIdx].imageData );
    cvReleaseImage( &rectifiedTopicR[camIdx].imageData );
    cvReleaseImage( &depthTopic[camIdx].imageData );
    cvReleaseImage( &depthTopic[camIdx].convertedImageData );
  }
  stop_transfer();
  release_transfer();
}

void GuidanceSensors::lockGuidance()
{
  guidanceLock.enter();
}

void GuidanceSensors::unlockGuidance()
{
  guidanceLock.leave();
}

void GuidanceSensors::copyImages( image_data imData )
{
  // TODO Copy over left, right, and depth images.
  int camIdxNum;
  for( int camIdx = 0; camIdx<numCamPairs; camIdx++ )
  {
    camIdxNum = camPairs[camIdx];
    memcpy( rectifiedTopicL[camIdx].imageData->imageData,
        imData.m_rect_left[camIdxNum], IMAGE_SIZE );
    memcpy( rectifiedTopicR[camIdx].imageData->imageData,
        imData.m_rect_right[camIdxNum], IMAGE_SIZE );
    IplImage *disparityImage = cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT),
       IPL_DEPTH_16S, 1);
    //memcpy( disparityImage->imageData,imData.m_depth[camIdxNum], IMAGE_SIZE * 2 );
    float temp = 16 * CAM_F * CAM_BASELINE;
    int i,j;
    for( i = 0; i<IMAGE_HEIGHT; i++ )
    {
      //short *ptr_in = (short *)(disparityImage->imageData+i*
      //    disparityImage->widthStep);
      short *ptr_in = (short *)(imData.m_depth[camIdxNum]+i*IMAGE_WIDTH*2);
      float *ptr_out = (float *)(depthTopic[camIdx].imageData->imageData+i*
          depthTopic[camIdx].imageData->widthStep);
      for( j = 0; j<IMAGE_WIDTH; j++ )
        {
        float res = 0.0/0.0;
        if( ptr_in[j]>16 )
        {
          res = (temp / ptr_in[j]);
        }
        ptr_out[j] = (float)(res);
      }
    }
    cvReleaseImage( &disparityImage );
    //cvConvertScale(
    //    depthTopic[camIdx].imageData,
    //    depthTopic[camIdx].convertedImageData,
    //    255.0/(1500), 255.0/(1500));
    //cv::applyColorMap(depthTopic[camIdx].convertedImageData->imageData,
    //    depthTopic[camIdx].convertedImageData->imageData, cv::COLORMAP_JET);
  }
}

void GuidanceSensors::publishImages()
{
  int camIdxNum;
  for( int camIdx = 0; camIdx<numCamPairs; camIdx++ )
  {
    camIdxNum = camPairs[camIdx];
    sensor_msgs::fillImage( rectifiedTopicL[camIdx].msgsImg, "8UC1",
        rectifiedTopicL[camIdx].imageData->height,
        rectifiedTopicL[camIdx].imageData->width,
        rectifiedTopicL[camIdx].imageData->widthStep,
        rectifiedTopicL[camIdx].imageData->imageData);
    rectifiedTopicL[camIdx].msgsImg.header.stamp = ros::Time::now();
    rectifiedTopicL[camIdx].cameraInfo.header.stamp =
      rectifiedTopicL[camIdx].msgsImg.header.stamp;
    rectifiedTopicL[camIdx].imagePub.publish( rectifiedTopicL[camIdx].msgsImg,
        rectifiedTopicL[camIdx].cameraInfo );

    sensor_msgs::fillImage( rectifiedTopicR[camIdx].msgsImg, "8UC1",
        rectifiedTopicR[camIdx].imageData->height,
        rectifiedTopicR[camIdx].imageData->width,
        rectifiedTopicR[camIdx].imageData->widthStep,
        rectifiedTopicR[camIdx].imageData->imageData);
    rectifiedTopicR[camIdx].msgsImg.header.stamp = ros::Time::now();
    rectifiedTopicR[camIdx].cameraInfo.header.stamp =
      rectifiedTopicR[camIdx].msgsImg.header.stamp;
    rectifiedTopicR[camIdx].imagePub.publish( rectifiedTopicR[camIdx].msgsImg,
        rectifiedTopicR[camIdx].cameraInfo );

    fillImage( depthTopic[camIdx].msgsImg, "32FC1",
        depthTopic[camIdx].imageData->height,
        depthTopic[camIdx].imageData->width,
        depthTopic[camIdx].imageData->widthStep,
        depthTopic[camIdx].imageData->imageData );
    depthTopic[camIdx].msgsImg.header.stamp = ros::Time::now();
    depthTopic[camIdx].imageDepthPub.publish( depthTopic[camIdx].msgsImg );

    /*fillImage( depthTopic[camIdx].msgsConvImg, "8UC1",
      depthTopic[camIdx].convertedImageData->height,
      depthTopic[camIdx].convertedImageData->width,
      depthTopic[camIdx].convertedImageData->widthStep,
      depthTopic[camIdx].convertedImageData->imageData );
      depthTopic[camIdx].msgsConvImg.header.stamp = ros::Time::now();
      depthTopic[camIdx].imageDepthPub.publish( depthTopic[camIdx].msgsConvImg );
      */
  }
}

void GuidanceSensors::publishSonar( sonar_data* sonar )
{
  for( int camIdx = 0; camIdx < NUM_SENSORS; camIdx++ )
  {
    sonarTopic[camIdx].msg.sonar = sonar[camIdx].sonar*0.001f;
    sonarTopic[camIdx].msg.reliability = sonar[camIdx].reliability;

    sonarTopic[camIdx].sonarPub.publish( sonarTopic[camIdx].msg );
  }
}

void GuidanceSensors::publishIMU( imu_guidance* imu )
{
  geometry_msgs::Quaternion orientation;
  geometry_msgs::Vector3 angular_velocity;
  geometry_msgs::Vector3 linear_acceleration;

  // Guidance does not return an orientation.
  orientation.x = 0;
  orientation.y = 0;
  orientation.z = 0;
  orientation.w = 0;

  angular_velocity.x = imu->gyro_x;
  angular_velocity.y = imu->gyro_y;
  angular_velocity.z = imu->gyro_z;

  linear_acceleration.x = imu->acc_x;
  linear_acceleration.y = imu->acc_y;
  linear_acceleration.z = imu->acc_z;

  imuTopic.msg.orientation = orientation;
  imuTopic.msg.orientation_covariance[0] = -1;

  imuTopic.msg.angular_velocity = angular_velocity;

  imuTopic.msg.linear_acceleration = linear_acceleration;

  imuTopic.msg.header.stamp = ros::Time::now();
  imuTopic.imuPub.publish( imuTopic.msg );
}

void GuidanceSensors::initializeCameras()
{
  int camIdxNum;
  for( int camIdx = 0; camIdx < numCamPairs; camIdx++ )
  {
    camIdxNum = camPairs[camIdx];
    select_rectified_img( (e_vbus_index)camIdxNum, true );
    select_rectified_img( (e_vbus_index)camIdxNum, false );
    select_depth( (e_vbus_index)camIdxNum );
  }
  select_sonar();
  select_imu();
  return;
}

void GuidanceSensors::destroyTopics()
{
  delete[] rectifiedTopicL;
  delete[] rectifiedTopicR;
  delete[] depthTopic;

  delete[] camPairs;

  return;
}
