/** @file  DJI_guidance.h
* 
* @brief Define data struct & data type for USB transfer.
*
*This file define data struct & data type for USB transfer.
*
* @version 1.0.0
*
*************************************/

#ifndef __DJI_GUIDANCE_H__
#define __DJI_GUIDANCE_H__

#ifdef USBDLL
#define SDK_API _declspec(dllexport)
#else
#define SDK_API extern
#endif

#define CAMERA_PAIR_NUM 5

/**
* @enum  e_vbus_index
* @brief Define logical direction of vbus
*/
enum e_vbus_index
{
	e_vbus1 = 1,	/**< logic direction of vbus */
	e_vbus2 = 2,	/**< logic direction of vbus */
	e_vbus3 = 3,	/**< logic direction of vbus */
	e_vbus4 = 4,	/**< logic direction of vbus */
	e_vbus5 = 0	    /**< logic direction of vbus */
};

/**
* @enum  e_image_data_frequecy
* @brief Define frequecy of image data
*/
enum e_image_data_frequecy
{
	e_frequecy_5 =  0,	/**< frequecy of image data */
	e_frequecy_10 = 1,	/**< frequecy of image data */
	e_frequecy_20 = 2	/**< frequecy of image data */
};

/**  
 *     @fn typedef int (*user_call_back)( int event_type, int data_len, char *content );
 *     @brief call back prototypes    
 *     @param event_type use it to identify the data:image,imu,sonar,vo or oa
 *     @param data_len length of the input data
 *     @data input data read from GUIDANCE
 *     @return   error code,if error occur,it will be non zero
 */
typedef int (*user_call_back)( int event_type, int data_len, char *data );

/**
* @enum  e_guidance_event
* @brief Define event type of callback
*/
enum e_guidance_event
{
	e_image_guidance = 0,	/**< called back when image comes */
	e_imu_guidance,	        /**< called back when imu comes */
	e_sonar_guidance,	    /**< called back when sonar comes */
	e_vo_guidance,	        /**< called back when vo output comes */
	e_oa_guidance,	        /**< called back when oa output comes */
	e_event_num
};

/**
*@class  image_data
*@brief Define image data 
*/
class image_data
{
public:
	char  *m_rect_left[CAMERA_PAIR_NUM];	          /**< rectified image of left camera */
	char  *m_rect_right[CAMERA_PAIR_NUM];	      /**< rectified image of right camera */
	char  *m_depth[CAMERA_PAIR_NUM];	              /**< depth */
};

/**
*@class  sonar_data
*@brief Define sonar data 
*/
class sonar_data
{
public:
	unsigned short sonar;	          /**< distance */
	unsigned short reliability;	      /**< reliability of the distance data */
};

/**
*@class  vo_velocity
*@brief Define velocity of vo
*/
class vo_velocity
{
public:
	short vx;	          /**< velocity of x */
	short vy;	          /**< velocity of y */
	short vz;	          /**< velocity of z */
};

/**
*@class  imu_guidance
*@brief Define imu 
*/
class imu_guidance
{
public:
	float acc_x;	/**< acceleration of x */
	float acc_y;	/**< acceleration of y */
	float acc_z;	/**< acceleration of z */

	float gyro_x;	/**< gyro(angular velocity) of x */
	float gyro_y;	/**< gyro(angular velocity) of y */
	float gyro_z;	/**< gyro(angular velocity) of z */
};

/**  
 *     @fn int init_transfer();
 *     @brief init SDK    
 *     @return   error code,if error occur,it will be non zero
 */
SDK_API int init_transfer( void );

/**  
 *     @fn int start_transfer();
 *     @brief send message to GUIDANCE to start data transfer.  
 *     @return   error code,if error occur,it will be non zero
 */
SDK_API int start_transfer( void );

/**  
*     @fn int stop_transfer();
*     @brief send message to GUIDANCE to stop data transfer.    
*     @return   error code,if error occur,it will be non zero
*/
SDK_API int stop_transfer( void );

/**  
*     @fn int release_transfer();
*     @brief release SDK.    
*     @return   error code,if error occur,it will be non zero
*/
SDK_API int release_transfer( void );

/**  
*     @fn int set_sdk_event_handler( user_call_back handler );
*     @brief  set callback,when data from GUIDANCE comes,it will be called by transfer thread.
*     @return   error code,if error occur,it will be non zero
*/
SDK_API int set_sdk_event_handler( user_call_back handler );

/**  
*     @fn int reset_config();
*     @brief  reset subscribe configure,if you want to subscribe data different from last time.
*     @return   error code,if error occur,it will be non zero
*/
SDK_API int reset_config( void );

/**  
*     @fn int select_imu();
*     @brief  subscribe to imu.
*     @return   error code,if error occur,it will be non zero
*/
SDK_API void select_imu( void );

/**  
*     @fn int select_sonar();
*     @brief  subscribe to sonar.
*     @return   error code,if error occur,it will be non zero
*/
SDK_API void select_sonar( void );

/**  
*     @fn int select_vo();
*     @brief  subscribe to VO output, i.e. velocity of GUIDANCE in body coordinate system.
*     @return   error code,if error occur,it will be non zero
*/
SDK_API void select_vo( void );

/**  
*     @fn int select_oa();
*     @brief  subscribe to OA output, i.e. distance from obstacle.
*     @return   error code,if error occur,it will be non zero
*/
SDK_API void select_oa( void );

/**  
*     @fn int select_rectified_img( unsigned int camera_pair_index, bool is_left );
*     @brief  subscribe to rectified image.
*     @param camera_pair_index index of camera pair selected
*     @param is_left whether the image data selected is left
*     @return   error code,if error occur,it will be non zero
*/
SDK_API int select_rectified_img( e_vbus_index camera_pair_index, bool is_left );

/**  
*     @fn int select_depth( unsigned int camera_pair_index );
*     @brief  subscribe depth data.
*     @param camera_pair_index index of camera pair selected
*     @return   error code,if error occur,it will be non zero
*/
SDK_API int select_depth( e_vbus_index camera_pair_index );

/**  
*     @fn int set_image_frequecy( int frequecy );
*     @brief set frequecy of image transfer
*     Set the frequecy of image transfer. As the bandwidth of USB is limited,
*     if you subscribe too much images(feature point,rectified image or depth),the frequecy
*     should be relatively small,otherwise the SDK cannot guarantee the continuity of image transfer.
*     @param frequecy frequecy of image transfer
*     @return   error code,if error occur,it will be non zero
*/
SDK_API int set_image_frequecy( e_image_data_frequecy frequecy );

#endif
