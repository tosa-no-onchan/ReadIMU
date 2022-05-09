/*
* Arduinoを用いてIMUで姿勢推定 on ROS
* https://ssk0109.hatenablog.com/entry/2018/12/13/014117
* ReadIMU.ino
*
*  Serial communication is enabled with the following command:
*  rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
*  rosrun rosserial_python serial_node.py tcp
*
* roscore
* roslaunch rtabmap_ros_my madgwick.launch
*
* MPU9250-SPI-turtleboot3 の IMU ライブラリーを使ってみます。
*  main #4
*
* https://esp32.com/viewtopic.php?t=7665
*/

//#undef ESP32
#include "turtlebot3_sensor.h"

#if defined(BOARD_ESP32)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
//#include <analogWrite.h>
#define LED_BUILTIN 17
//#define LED_BUILTIN 4
#endif

//#define SerialPort SerialUSB
//#define SerialPort Serial

#include <time.h>     // for clock()

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Vector3.h>

#if defined(ESP32)
#include <WiFi.h>
const char SSID[] = "SSID";
const char PASSWORD[] = "PASSWORD";
IPAddress server(192,168,1,170);
const uint16_t serverPort = 11411;
WiFiClient client;
#endif

#define USE_ODOM
#define USE_JOINT

Turtlebot3Sensor sensors;

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
//#define IMU_PUBLISH_FREQUENCY                  200  //hz
//#define IMU_PUBLISH_FREQUENCY                  60  //hz
#define IMU_PUBLISH_FREQUENCY                  15  //hz

#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

//#define FREQUENCY_IMU_DATA_HZ               60  // 90[hz]
//#define FREQUENCY_IMU_DATA_HZ              200  // 90[hz]
#define FREQUENCY_IMU_DATA_HZ                 100  // 90[hz]
//#define FREQUENCY_IMU_DATA_HZ              80  // 90[hz]
//#define FREQUENCY_IMU_DATA_HZ              400  // 400[hz]
//#define FREQUENCY_IMU_DATA_HZ              800  // 400[hz]

#define WHEEL_NUM                        2
#define LEFT                             0
#define RIGHT                            1

float yaw_est;
unsigned long odom_prev_time;

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

// add by nishi from ros_tutlebot3
// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);
// changed by nishi for madgwick_filter  2021.10.7
//ros::Publisher imu_pub("imu/data_raw", &imu_msg);

// Odometry of Turtlebot3
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);
// test by nishi 2021.10.8
//ros::Publisher odom_pub("odom_fox", &odom);

#ifdef USE_JOINT
// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);
#endif

#if defined(USE_MAG)
// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);
//ros::Publisher mag_pub("imu/mag", &mag_msg);
#endif

// prottype
void updateVariable(bool isConnected);
void updateTime();
ros::Time rosNow();
void updateGyroCali(bool isConnected);
void publishImuMsg(void);

#ifdef USE_MAG
void publishMagMsg(void);
#endif

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

void updateJointStates(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateOdometry(void);
void initOdom(void);
void initJointStates(void);

void updateTFPrefix(bool isConnected);

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};


/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
//static uint32_t tTime[10];
static unsigned long tTime[10]={0,0,0,0,0,0,0,0,0,0};

// Multi task
TaskHandle_t th[2];
SemaphoreHandle_t xMutex = NULL;
//SemaphoreHandle_t xSemaphore = NULL;

void update_motor(void *pvParameters){

	xSemaphoreGive(xMutex);

  int ac_cnt2=0;
  clock_t t_start2 ,t_end2;

	while(1){
		//uint32_t t = millis();
		unsigned long t = micros();

		if (t >= tTime[4]){

			//Serial.println("start update_imu()");
			// Update the IMU unit.	add by nishi 2021.8.9
			//sensors.updateIMU();
			bool ok =true;
			if(pdTRUE != xSemaphoreTake(xMutex,10UL)){
				ok=false;
			}
			sensors.copyIMU();
			if(ok){
				xSemaphoreGive(xMutex);
			}
			tTime[4] = t + (1000000UL / FREQUENCY_IMU_DATA_HZ);
		}

    if (t >= tTime[5]){
      sensors.updateIMU();
      //tTime[5] = t + 1000000UL / 100;   // 無負荷     負荷時         負荷時(my)  99.5 Hz
      //tTime[5] = t + 1000000UL / 250;   // 無負荷     負荷時
      //tTime[5] = t + 1000000UL / 450;   // 無負荷     負荷時
      //tTime[5] = t + 1000000UL / 600;   // 無負荷     負荷時
      tTime[5] = t + 1000000UL / 700;   // 無負荷     負荷時           負荷時(my) 686 Hz
      //tTime[5] = t + 1000000UL / 750;   // 無負荷     負荷時
      //tTime[5] = t + 1000000UL / 800;   // 無負荷 789  負荷時 760
      //tTime[5] = t + 1000000UL / 850;   // 無負荷       負荷時 about 800
      //tTime[5] = t + 1000000UL / 950;   // 無負荷       負荷時
      //tTime[5] = t + 1000000UL / 965;   // 無負荷       負荷時(LSM95DS1) 945
      //tTime[5] = t + 1000000UL / 968;   // 無負荷       負荷時(LSM95DS1) 949 - 950
      //tTime[5] = t + 1000000UL / 970;   // 無負荷       負荷時(LSM95DS1) 953
      //tTime[5] = t + 1000000UL / 980;   // 無負荷       負荷時(LSM95DS1) 963
      //tTime[5] = t + 1000000UL / 1000;   // 無負荷       負荷時(LSM95DS1) 985
      ac_cnt2++;
    }

    //#define XXX_400
    #if defined(XXX_400)
    t_end2=clock();
    double t_d = (double)(t_end2 - t_start2) / CLOCKS_PER_SEC;
    if(t_d >= 4.0){
      float hz = (float)ac_cnt2/t_d;
      SERIAL_PORT.print(F("acc_hz:"));
      SERIAL_PORT.println(hz, 4);
      t_start2=clock();
      ac_cnt2=0;
    }
    #endif


		//vTaskDelay(1);
	}
}


void setup() {

	#if defined(BOARD_ESP32)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
	#endif

	// Builtin LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);		// light OFF

	#if defined(ESP32)
	Serial.begin(115200);
	//Serial.begin(57600);
    delay(10);
    Serial.println("start prog.");
	#endif


	#if defined(ESP32)
	WiFi.begin(SSID,PASSWORD);
	Serial.print("WiFi connecting");

	while (WiFi.status() != WL_CONNECTED) {
		Serial.print(".");
		delay(100);
	}
	Serial.println(" connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
	#endif


	#if defined(ESP32)
	Serial.println("nh.getHardware()->setConnection(() call");
	delay(100);
	// set Server IP & port  ESP32 Wi-FI add by nishi 2021.5.6
	nh.getHardware()->setConnection(server, serverPort);
	Serial.println("OK!");
	#else
	// set serial spped
	//nh.getHardware()->setBaud(115200);
	//nh.getHardware()->setBaud(230400);
	nh.getHardware()->setBaud(1000000);
	//nh.getHardware()->setBaud(2000000);
	#endif

	#if defined(ESP32)
	Serial.println("nh.initNode() call");
  #endif
  delay(100);

  nh.initNode();

	#if defined(ESP32)
	IPAddress ipa= nh.getHardware()->getLocalIP();
	Serial.print("ip:");
	Serial.println(ipa.toString());
	#endif

	// Subscriber

	// Publisher
  // add by nishi from ros_tutlebot3
  nh.advertise(imu_pub);
  #ifdef USE_ODOM
	nh.advertise(odom_pub);
  #ifdef USE_JOINT
	// add by nishi  2021.4.26
	nh.advertise(joint_states_pub);
  #endif
  #endif

  delay(100);

	#ifdef USE_MAG
	nh.advertise(mag_pub);
	#endif

	// Setting for SLAM and navigation (odometry, joint states, TF)
  #ifdef USE_ODOM
	// call ros::Publisher() for "/tf"
	tf_broadcaster.init(nh);

  // Setting for IMU
  sensors.init();

	initOdom();

  #ifdef USE_JOINT
	initJointStates();
  #endif
	#endif

	delay(100);

  vSemaphoreCreateBinary( xMutex );
	//xMutex = xSemaphoreCreateMutex();

	if( xMutex != NULL ){
		// start IMU Update Task. add by nishi 2021.8.9
		xTaskCreatePinnedToCore(
			update_motor,"update_motor",4096,NULL,1,&th[0],1);
	}
	else {
		while(1){
			Serial.println("rtos mutex create error, stopped");
			delay(1000);
		}
	}

	#if defined(ESP32)
	Serial.println("setup() OK");
  #endif

}
float gXOffset= 2.50, gYOffset=-1.40, gZOffset= 1.47;

void loop() {

  float qw, qx, qy, qz;

  //uint32_t t = millis();
	unsigned long t = micros();

  updateTime();
  updateVariable(nh.connected());
  updateTFPrefix(nh.connected());

	// Start Gyro Calibration after ROS connection  add by nishi 2021.11.3
	//updateGyroCali(nh.connected());

  //sensors.updateIMU();


	if (t >= tTime[3]){
    //publishImuMsg();
		#ifdef USE_MAG
    publishMagMsg();
		#endif
    tTime[3] = t;


		// feed odom message
		odom.header.stamp = nh.now();
		updateOdometry();
		//odom.header.frame_id = "odom";
		//odom.child_frame_id = "base_footprint";
		
    odom.pose.pose.position.x = 0.0;
		odom.pose.pose.position.y = 0.0;
		odom.pose.pose.position.z = 0.0;

		//odom.pose.pose.orientation.w = qw;
		//odom.pose.pose.orientation.x = qx;
		//odom.pose.pose.orientation.y = qy;
		//odom.pose.pose.orientation.z = qz;

		bool ok =false;

		if(pdTRUE == xSemaphoreTake(xMutex, 10UL)){   // どっちか、NG
			ok=true;
		//}
		// get IMU data
		imu_msg = sensors.getIMU();

    odom.pose.pose.position.x = sensors.tf_dlt[0];
    odom.pose.pose.position.y = sensors.tf_dlt[1];
    odom.pose.pose.position.z = sensors.tf_dlt[2];
    //odom.pose.pose.position.x += sensors.tf_dlt[0];
    //odom.pose.pose.position.y += sensors.tf_dlt[1];
    //odom.pose.pose.position.z += sensors.tf_dlt[2];
    //sensors.tf_dlt[0]=0.0;
    //sensors.tf_dlt[1]=0.0;
    //sensors.tf_dlt[2]=0.0;

		//if(ok){
			xSemaphoreGive(xMutex);   // どちか、NG
		}

    //Serial.print(F("x:"));
    //Serial.print(odom.pose.pose.position.x, 4);
    //Serial.print(F(" y:"));
    //Serial.print(odom.pose.pose.position.y, 4);
    //Serial.print(F(" z:"));
    //Serial.println(odom.pose.pose.position.z, 4);


		qw = imu_msg.orientation.w;
		qx = imu_msg.orientation.x;
		qy = imu_msg.orientation.y;
		qz = imu_msg.orientation.z;

    odom.pose.pose.orientation.w = qw;
    odom.pose.pose.orientation.x = qx;
    odom.pose.pose.orientation.y = qy;
    odom.pose.pose.orientation.z = qz;

		// Velocity expressed in base_link frame
		//odom.twist.twist.linear.x = linear_velocity_est;
		odom.twist.twist.linear.x = 0.0f;
		odom.twist.twist.linear.y = 0.0f;
		//odom.twist.twist.angular.z = angular_velocity_est;
		odom.twist.twist.angular.z = 0.0f;

		// error occured -> [ERROR] [1616575654.217167]: Message from device dropped: message larger than buffer.  by nishi
    #ifdef USE_ODOM
   	odom_pub.publish(&odom);

		// add by nishi for TF  2021.4.26
 		// odometry tf
		updateTF(odom_tf);
 		odom_tf.header.stamp = nh.now();
		// ratbmaap-nishi_stereo_outdoor.launch と TF がバッテイングする? 2021.9.16
 		tf_broadcaster.sendTransform(odom_tf);

    #ifdef USE_JOINT
		// joint states
		//updateJointStates();
		//joint_states.header.stamp = nh.now();
		//joint_states_pub.publish(&joint_states);
    #endif
    #endif

    tTime[3] = t + (1000000UL / IMU_PUBLISH_FREQUENCY);

  }

  nh.spinOnce();
  delay(1);
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  
  if (isConnected)
  {
    if (variable_flag == false)
    {
      sensors.initIMU();
      //initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}
/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  (void)(isConnected);

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}

#ifdef USE_MAG
/*******************************************************************************
* Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  mag_msg = sensors.getMag();

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = mag_frame_id;

  mag_pub.publish(&mag_msg);
}
#endif


/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[80];		// update by nishi 2022.4.25

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");  

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg); 

      isChecked = true;

      odom.pose.pose.position.x=0.0;
      odom.pose.pose.position.y=0.0;
    }
  }
  else
  {
    isChecked = false;
  }
}

/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;

  //Serial.println("updateJointState() : #9 passed!!");
  //Serial.println(*joint_states.position,DEC);
  //Serial.println(*(joint_states.position+1),DEC);

}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  //odom.pose.pose.position.x = odom_pose[0];
  //odom.pose.pose.position.y = odom_pose[1];
  //odom.pose.pose.position.z = 0;
  //odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  //odom.twist.twist.linear.x  = odom_vel[0];
  //odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;

  //motor_tic.left=0;		// add by nishi
  //motor_tic.right=0;	// add by nishi

  yaw_est=0.;			// add by nishi

}

/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  //joint_states.effort_length   = WHEEL_NUM;
  // update by nishi 2021.5.10
  joint_states.effort_length   = 0;

}
