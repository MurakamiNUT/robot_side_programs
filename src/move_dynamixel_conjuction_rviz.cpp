/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "pc_side_programs/Controller.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/DisplayRobotState.h>
//#include <cv_bridge/cv_bridge.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <stdio.h> 
#include <stdlib.h>
std_msgs::Float32MultiArray msg;
ros::Publisher jointstates_pub;
#define Servo_id_array sizeof(Servo_id)/sizeof(Servo_id[0])
//#include "/home/murakami/catkin_ws/src/dynamixel-workbench/dynamixel_workbench_toolbox/include/dynamixel_workbench_toolbox/dynamixel_workbench.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

#define OUT_FLG 0 //debug
#define SERVO_NUM 10
#define CONTROL_TIME 25 //制御周波数 10~25くらい？
#define ALL_SPEED 120  //deg/sのつもり（制御周期早かったりするとおかしくなる）
using namespace std;
void servo_control();
void time_manage();
void pose_set(int line);
int set_vel_accele(int number);
ros::Time ros_start;
bool time_flg = false;

int32_t Velocity_Limit[SERVO_NUM] = {};
float Velocity_Limit_radian[SERVO_NUM] = {}; 
int32_t Acceleration_Limit[SERVO_NUM] = {};
float Acceleration_Limit_radian[SERVO_NUM] = {}; 
float Transition_Time[SERVO_NUM] = {};//各サーボの移動時間
float TT_Max = 0;//Transition_Timeの最高値
// %Tag(CALLBACK)%
//std_msgs::String::ConstPtr&
// %EndTag(CALLBACK)%
double Servo_V[SERVO_NUM] = {};//速度目標値
float Servo_P_deg[SERVO_NUM] = {};//サーボ現在角度
float Servo_P_radian[SERVO_NUM] = {};
bool ARM_MODE = false;
bool emerge = false;



DynamixelWorkbench dxl_wb;
float Arm_C[SERVO_NUM] ={//毎秒どれくらい動くか
 ((3.14159265 / 180) * ALL_SPEED) / 1.5,//0 
 ((3.14159265 / 180) * ALL_SPEED) / 1.5,
 ((3.14159265 / 180) * ALL_SPEED) / 1.5,
 (3.14159265 / 180) * ALL_SPEED,
 (3.14159265 / 180) * ALL_SPEED,
 (3.14159265 / 180) * ALL_SPEED * 1.5,//5
 (3.14159265 / 180) * ALL_SPEED * 1.5,
 (3.14159265 / 180) * ALL_SPEED * 0.8,
 (3.14159265 / 180) * ALL_SPEED,
 (3.14159265 / 180) * ALL_SPEED//9
};
uint8_t Servo_id[SERVO_NUM] = {2,3,4,5,6,7,8,9,10,11};
bool Servo_id_MT[SERVO_NUM] = {0,0,0,0,0,0,1,1,0,0};
bool Servo_XH430_V350[SERVO_NUM]  = {0,0,0,0,0,0,1,0,1,1};
//value値
float Priset_deg[11][SERVO_NUM] = {
  //{2048,2048,2048,2048,2048,2048,2048,2048,2048,2048},
{7223, 232161, -189950, -30343, 3857, -4, 1408, -26178, 2122, 2067},//標準姿勢
  //{0,240000,-213000,-42000,0,0,0,0,2048,2048},//標準姿勢値
{7223, 227016, -131310, 56044, 0, 0, -1103, -155237, 3419, 2067},//シャットダウン姿勢
{7223, 209144, -119384, 109140, 1063, 3995, -891, -94374, 1588, 2020},//フリッパー監視

{7223, 131753, -131310, 56044, 0, 0, 1409, -105377, 2218, 2067},//レディネス
{7223, 232312, -217538, -22496, 652, 2091, 1101, 112624, 2371, 2019},//階段上り
{7223, 232312, -217538, -22496, 652, 2091, 1101, 112624, 2371, 2019},//階段下り
{7223, 133335, -60565, 24, 1608, -4, 1002, -52450, 2395, 2012},//バルブ
{7223, 232312, -217538, -22496, 652, 2091, 1101, 112624, 2371, 2019},//メーターチェック
{7223, 232312, -217538, -22496, 652, 2091, 1101, 112624, 2371, 2019},//自分監視
{7223, 131753, -131310, 56044, -151870, 0, 1409, -105377, 2218, 2067},//熱カメラ
{7223, 233535, -212257, -41007, -151870, -4, 1364, -290511, 2395, 2012},//ジャングル
};
//各サーボ最高速 radian/s
/*
float Priset_Speed[SERVO_NUM]={
  3.546835,3.546835,3.546835,3.550990,3.550990,3.550990,3.237411,3.550990,3.237411,3.237411
};
*/


int32_t key;
bool flg = false;

void chatterCallback(const pc_side_programs::Controller& controller_){
#if 0
  if(controller_.R3) ARM_MODE = true;
  if(controller_.L3) ARM_MODE = false;
  if(!emerge){
	      Servo_V[8] = controller_.Up_Down       * Arm_C[8];

	      Servo_V[9] = controller_.Left_Right    * Arm_C[9];
	  if(ARM_MODE){
	    //コントローラ値の受け取り
	      //Servo_V[0] = controller_.LS_Left_Right * -Arm_C[0];
	      //Servo_V[1] = controller_.LS_Up_Down    * -Arm_C[1];



	      Servo_V[4] = controller_.RS_Left_Right * Arm_C[4];
	      Servo_V[3] = controller_.RS_Up_Down    * -Arm_C[3];
	      Servo_V[7] = controller_.RS_Up_Down       * Arm_C[7];
	      if(controller_.L1)         Servo_V[6] = Arm_C[6];
	      else if(controller_.L2)    Servo_V[6] = -Arm_C[6];
	      else Servo_V[6] = 0;

	      if(controller_.R1){        
          if(controller_.RS_Up_Down != 0)  Servo_V[7] = Arm_C[7]*2; 
          else                             Servo_V[7] = Arm_C[7];
	      }
	      else if(controller_.R2){    
          if(controller_.RS_Up_Down != 0)  Servo_V[7] = -Arm_C[7]*2; 
          else                             Servo_V[7] = -Arm_C[7];
	      }
	      //else Servo_V[7] = 0;

	      //if(controller_.Triangle)   Servo_V[2] = Arm_C[2];
	      //else if(controller_.Cross) Servo_V[2] = -Arm_C[2];
	      else Servo_V[2] = 0;

	      if(controller_.Circle)   Servo_V[5] = Arm_C[5];
	      else if(controller_.Square) Servo_V[5] = -Arm_C[5];
	      else Servo_V[5] = 0;


    const char *log;
    if(controller_.Select){
      for(int i = 0;i < Servo_id_array; i++){
            /*
        dxl_wb.writeRegister(Servo_id[i],"Goal_Velocity",dxl_wb.convertVelocity2Value(Servo_id[i],0),&log);
        ROS_INFO("data:[ID%d::%s]",i+2, log);
        dxl_wb.writeRegister(Servo_id[i],"Goal_Acceleration",35,&log);
        ROS_INFO("data:[ID%d::%s]",i+2, log);
        dxl_wb.writeRegister(Servo_id[i],"Profile_Velocity",dxl_wb.convertVelocity2Value(Servo_id[i],0),&log);
        ROS_INFO("data:[ID%d::%s]",i+2, log);
        dxl_wb.writeRegister(Servo_id[i],"Profile_Acceleration",35,&log);
        ROS_INFO("data:[ID%d::%s]",i+2, log);
        */
		  for(int i=0; i < Servo_id_array; i++){
        dxl_wb.getRadian(Servo_id[i], &Servo_P_radian[i], &log);
		      //dxl_wb.reboot(Servo_id[i],&log);
		      //dxl_wb.torque(Servo_id[i],1,&log);
		  }
      }
/*
		if(!flg){
		  flg = true;
		} 
*/
	      }
//	      else flg = false;
	      if(controller_.Start){
	      for(int i = 0;i < Servo_id_array; i++){
		  dxl_wb.writeRegister(Servo_id[i],"Goal_Velocity",dxl_wb.convertVelocity2Value(Servo_id[i],0.5),&log);
		  ROS_INFO("data:[ID%d::%s]",i+2, log);
		  dxl_wb.writeRegister(Servo_id[i],"Goal_Acceleration",35,&log);
		  ROS_INFO("data:[ID%d::%s]",i+2, log);
		  dxl_wb.writeRegister(Servo_id[i],"Profile_Velocity",dxl_wb.convertVelocity2Value(Servo_id[i],0.5),&log);
		  ROS_INFO("data:[ID%d::%s]",i+2, log);
		  dxl_wb.writeRegister(Servo_id[i],"Profile_Acceleration",35,&log);
		  ROS_INFO("data:[ID%d::%s]",i+2, log);
		}
		  dxl_wb.writeRegister(Servo_id[7],"Goal_Velocity",dxl_wb.convertVelocity2Value(Servo_id[7],3.1415),&log);
		  ROS_INFO("data:[ID%d::%s]",7+2, log);
		  dxl_wb.writeRegister(Servo_id[7],"Goal_Acceleration",70,&log);
		  ROS_INFO("data:[ID%d::%s]",7+2, log);
		
		}
	  }
  }
else{
	      for(int i = 0;i < Servo_id_array; i++){
Servo_V[i] = 0;
		}
}
#else
#endif
}
void chatterCallback_key(const std_msgs::Int32& msg){
  key = msg.data;
  ROS_INFO("data:[%d]", key);
  const char *log;
  switch(key){
  //   case 97://規定姿勢
	// pose_set(0);
  //   break;
  //   case 121://規定姿勢
	// pose_set(1);
  //   break;

  //   case 100://ふりっぱー監視
	// pose_set(2);
  //   break;

  //   case 110://レディネス
	// pose_set(3);
  //   break;

  //   case 114://階段上り
	// pose_set(4);
  //   break;

  //   case 102://階段下り
	// pose_set(5);
  //   break;

  //   case 98://バルブ
	// pose_set(6);
  //   break;

  //   case 109://メーターチェック
	// pose_set(7);
  //   break;

  //   case 101://自分監視
	// pose_set(8);
  //   break;

  //   case 104://熱カメラ
	// pose_set(9);
  //   break;

  //   case 106://ジャングル
	// pose_set(10);
  //   break;
  //   case 46://手首右回転
	// // Servo_P_radian[5] = Servo_P_radian[5] + 1.5707;
  //   break;
  //   case 44://手首右回転
	// // Servo_P_radian[5] = Servo_P_radian[5] - 1.5707;
  //   break;
  //   case 119://カメラ前
	// Servo_P_radian[9] = 0;
  //   break;
  //   case 115://カメラ後ろ
	// Servo_P_radian[9] = 3.14;
  //   break;
//stop command
    case 111:{
	emerge = false;
    	break;
}

    case 112:{
	emerge = true;
    	break;
}

    // case 99://c
    //   FILE *outputfile;
    //   std::string str ="/home/kimuralab-nuc1/pose/" + std::to_string(ros::Time::now().toSec());
    //   const char* p = str.c_str();
    //   outputfile = fopen(p,"w");
    //   fprintf(outputfile, "{"); // ファイルに書き出し
    //   for(int i=0; i < Servo_id_array; i++){
    //     fprintf(outputfile, "%d",dxl_wb.convertRadian2Value(Servo_id[i], Servo_P_radian[i])); // ファイルに書き出し
    //     if(i != Servo_id_array-1)fprintf(outputfile, ", "); // ファイルに書き出し
    //   }
    //   fprintf(outputfile, "}"); // ファイルに書き出し
    //   fclose(outputfile);          // ファイルをクローズ(閉じる)

    //   for(int i=0; i < Servo_id_array; i++){
    //     ROS_INFO("data:[ID%d::%d]",i+2, dxl_wb.convertRadian2Value(Servo_id[i], Servo_P_radian[i]));
    //   }
    // break;
  }
}
float correction[3] = {0, 0, 0};
float dxl_value[3] = {0,0,0};
void chatterCallback_joint(const moveit_msgs::DisplayRobotState& msg){
  if(!emerge){
    // for(int i = 0; i < 3; i++){
    //   if(msg.position[i] > 3.1415926535)  dxl_value[i] = msg.position[i] - 6.28318531;
    //   else dxl_value[i] = msg.position[i];
    // //   ROS_INFO("Servo_P_radian:[ID%d::%f]",i+2,dxl_value[i] - correction[i] );
    //   Servo_P_radian[i] = msg.position[i] - correction[i];
        for(int i = 0; i < 6; i++){
            dxl_value[i] = msg.state.joint_state.position[i];
            // ROS_INFO("Servo_P_radian:[ID%d::%f]",i,dxl_value[i] - correction[i] );            
            Servo_P_radian[i] = msg.state.joint_state.position[i] - correction[i];    
        }
    }
  
}
void servo_control(){
  static double V_Step[SERVO_NUM];
  //if(ARM_MODE){
    for(int id = 0; id < SERVO_NUM; id++){
      //受け取った値よりステップ値を計算
      V_Step[id] = Servo_V[id] / CONTROL_TIME;
      //ステップ値分角度値に加算
      Servo_P_radian[id] += V_Step[id];
      //多回転モード(MT)でないサーボは一周で角度制限
      //後でサーボ個別に設定できるように変更
      if(!Servo_id_MT[id]){
        if(Servo_P_radian[id] > 3.1415)  Servo_P_radian[id] = 3.1415;
        else if(Servo_P_radian[id] < -3.1415)  Servo_P_radian[id] = -3.1415;
      }
      else{}
    } 
    if(OUT_FLG){
      //for(int i = 0;i < 7; i++) ROS_INFO("data:[%f]", Servo_P_radian[i]);
      for(int i = 0;i < Servo_id_array; i++) ROS_INFO("data:[ID%d::%d]",i, dxl_wb.convertRadian2Value(Servo_id[i], Servo_P_radian[i]));
      
    }
  //}
}
void time_manage(){
  const char *log;
  if(time_flg){
  ROS_INFO("ROS: %f::%f",ros::Time::now().toSec(),ros_start.toSec());
    if(TT_Max < (ros::Time::now().toSec() - ros_start.toSec())){
      for(int i = 0;i < Servo_id_array; i++){
        if(!Servo_XH430_V350[i]){//8.10.11を除く
        //高速設定
          //dxl_wb.writeRegister(Servo_id[i],"Goal_Velocity",0,&log);
          //dxl_wb.writeRegister(Servo_id[i],"Goal_Acceleration",0,&log);

          //変化したかの確認
          dxl_wb.readRegister(Servo_id[i],"Goal_Velocity",&Velocity_Limit[i],&log);
          Velocity_Limit_radian[i] = dxl_wb.convertValue2Velocity(Servo_id[i],Velocity_Limit[i]);
          ROS_INFO("data:[ID%d::velocity::%d]",i+2, Velocity_Limit[i]);
          ROS_INFO("data:[ID%d::%s]",i+2, log);

          dxl_wb.readRegister(Servo_id[i],"Goal_Acceleration",&Acceleration_Limit[i],&log);
          ROS_INFO("data:[ID%d::acceleration::%d]",i+2,Acceleration_Limit[i] );
          ROS_INFO("data:[ID%d::%s]",i+2, log);
        }
      }
      TT_Max = 0;
      time_flg = false;
    }
  }
}

int set_vel_accele(int number){
  const char *log;
  float low_speed[SERVO_NUM] =        {2, 2, 2, 2, 2, 2, 2, 2, 2, 2 };
  float low_acceleration[SERVO_NUM] = {3, 3, 3, 3, 3, 3, 3, 3, 3, 3 };
  float rise_time[SERVO_NUM] = {};
      for(int i=0; i < Servo_id_array; i++){
        if(!Servo_XH430_V350[i]){//8.10.11を除く
          //サーボ移動方向の特定  現在角度から目標角度を引いて0以上ならば＋方向へ回転
          if((Servo_P_radian[i] - dxl_wb.convertValue2Radian(Servo_id[i], Priset_deg[number][i]) > 0)){
            if(low_speed[i] < 0)        low_speed[i]        *= -1;
          }
          else{
            if(low_speed[i] > 0)        low_speed[i]        *= -1;
          }
          //低速設定 速度及び加速度
          dxl_wb.writeRegister(Servo_id[i],"Goal_Velocity",dxl_wb.convertVelocity2Value(Servo_id[i],low_speed[i]),&log);
          ROS_INFO("data:[ID%d::%s]",i+2, log);
          dxl_wb.writeRegister(Servo_id[i],"Goal_Acceleration",low_acceleration[i],&log);
          ROS_INFO("data:[ID%d::%s]",i+2, log);

          //変化したかの確認
          dxl_wb.readRegister(Servo_id[i],"Goal_Velocity",&Velocity_Limit[i],&log);
          Velocity_Limit_radian[i] = dxl_wb.convertValue2Velocity(Servo_id[i],Velocity_Limit[i]);
          ROS_INFO("data:[ID%d::velocity::%d]",i+2, Velocity_Limit[i]);
          ROS_INFO("data:[ID%d::%s]",i+2, log);

          dxl_wb.readRegister(Servo_id[i],"Goal_Acceleration",&Acceleration_Limit[i],&log);
          ROS_INFO("data:[ID%d::acceleration::%d]",i+2,Acceleration_Limit[i] );
          ROS_INFO("data:[ID%d::%s]",i+2, log);

          //到達までの時間計測 
          Acceleration_Limit_radian[i] = ((fabs(low_acceleration[i]) * 58000) / 3600) * 2 * 3.14159265 * 0.0033;
          //立ち上がり立ち下がり時間合計
          rise_time[i] = (fabs(Velocity_Limit_radian[i]) / fabs(Acceleration_Limit_radian[i]) * 2);
          float rise_move_radian[SERVO_NUM] = {};
          float move_radian[SERVO_NUM] = {};
          rise_move_radian[i] = (rise_time[i] * fabs(Velocity_Limit_radian[i])) / 2;
          move_radian[i] = fabs(Servo_P_radian[i] - dxl_wb.convertValue2Radian(Servo_id[i], Priset_deg[number][i]));
          if( move_radian[i] < fabs(rise_move_radian[i])){
            Transition_Time[i] = 2 * sqrt(move_radian[i] / Acceleration_Limit_radian[i]);
          }
          else{
            Transition_Time[i] = rise_time[i] - (fabs(move_radian[i] - rise_move_radian[i]) / fabs(Velocity_Limit_radian[i]));
          }
         // Transition_Time[i] = fabs(dxl_wb.convertValue2Radian(Servo_id[i], Priset_deg[0][i]) - Servo_P_radian[i]) / low_speed[i];
          //ROS_INFO("data:[ID%d::transition_time::%f]",i+2,Transition_Time[i] );
          if(TT_Max < Transition_Time[i]) TT_Max = Transition_Time[i];//最も時間のかかる移動
        }
      }
      ROS_INFO("data:[TT_Max:::%f]", TT_Max);
  
}
void pose_set(int line){
  //速度制限・加速度制限セット
  set_vel_accele(line);
  //時間計測開始
  ros_start = ros::Time::now();
  time_flg = true;
  Servo_P_radian[7] = Servo_P_radian[7] + (Servo_P_radian[3] - dxl_wb.convertValue2Radian(Servo_id[3], Priset_deg[line][3]))*0.8; 
  for(int i=0; i < Servo_id_array; i++){
    //移動開始//グリッパーは動かさない
   //if(!((Servo_id[i] == 8) || (Servo_id[i] == 9)))  Servo_P_radian[i] = dxl_wb.convertValue2Radian(Servo_id[i], Priset_deg[line][i]);
    if(!((Servo_id[i] == 8) || (Servo_id[i] == 9)))  msg.data[i] = dxl_wb.convertValue2Radian(Servo_id[i], Priset_deg[line][i]);
  }
  jointstates_pub.publish(msg);
}
int main(int argc, char **argv)
{
  std::string port_name = "/dev/USB_Serial";
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "move_dynamixel_conjuction_rviz");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("Arduino", 1000, chatterCallback);
  ros::Subscriber sub_key = n.subscribe("key", 1000, chatterCallback_key);
  ros::Subscriber sub_robot_state = n.subscribe("display_robot_state", 1000, chatterCallback_joint);
  ros::Publisher jointstates_pub = n.advertise<std_msgs::Float32MultiArray>("dynamixel_jointstates", 1000);
  uint32_t baudrate = 1000000;
  //uint32_t baudrate = 57600;
  bool result = false;
  const char *log;
  bool result_id[SERVO_NUM] = {};
  

    result = dxl_wb.init(port_name.c_str(), baudrate, &log);
    if (result == false){
      ROS_WARN("%s", log);
      ROS_WARN("Failed to init");
    }
    else{
      ROS_INFO("Succeed to init(%d)", baudrate);
    }
    for(int i = 0;i < Servo_id_array; i++){
      uint16_t model_number = 0;
      //pingのやり取り
      result = dxl_wb.ping(Servo_id[i], &model_number, &log);
      if (result == false){
        ROS_WARN("%s", log);
        ROS_WARN("Failed to ping ID = %d",Servo_id[i]);
      }
      else{
        ROS_INFO("Succeed to ping");
        ROS_INFO(",Servo_id : %d",Servo_id[i]);
      }
      if(Servo_id_MT[i]){
        //多回転モード設定
        result_id[i] = dxl_wb.setOperatingMode(Servo_id[i],4,&log);
        if (result_id[i] == false){
          ROS_WARN("%s", log);
        }
        else{
          ROS_INFO("Succeed to change extendedMode mode ID = %d",Servo_id[i]);
        }
        result_id[i] = dxl_wb.torque(Servo_id[i],0,&log);
      }
      else{
        //  モード設定
        result_id[i] = dxl_wb.jointMode(Servo_id[i],  0,0, &log);
        if (result_id[i] == false){
          ROS_WARN("%s", log);
          ROS_WARN("Failed to joint mode ID = %d",Servo_id[i]);
        }
        else{
          ROS_INFO("Succeed to change jointMode mode ID = %d",Servo_id[i]);
        }
      }
      //各サーボ初期角度取得
      if(result_id[i] == true){
        dxl_wb.getRadian(Servo_id[i], &Servo_P_radian[i], &log);
        //Servo_P_deg[i] = dxl_wb.convertRadian2Value(Servo_id[i], Servo_P_radian[i]);
      }
    }

    for(int i = 0;i < Servo_id_array; i++){
        dxl_wb.writeRegister(Servo_id[i],"Goal_Velocity",dxl_wb.convertVelocity2Value(Servo_id[i],0.3),&log);
        ROS_INFO("data:[ID%d::%s]",i+2, log);
        dxl_wb.writeRegister(Servo_id[i],"Goal_Acceleration",10,&log);
        ROS_INFO("data:[ID%d::%s]",i+2, log);
  	}
      
  ros::Rate loop_rate(CONTROL_TIME);
while(ros::ok()){
  servo_control();
  time_manage();
  for(int i=0; i < Servo_id_array; i++){
    if(result_id[i] == true)dxl_wb.goalPosition(Servo_id[i], (int32_t)dxl_wb.convertRadian2Value(Servo_id[i], Servo_P_radian[i]),&log);
  }
  //ROS_INFO("data:[%d]", 1);

     // for(int i = 0;i < 7; i++) ROS_INFO("data:[%d]",  dxl_wb.convertRadian2Value(Servo_id[i], Servo_P_radian[i]));
  ros::spinOnce();
  loop_rate.sleep();
}
}
// %EndTag(FULLTEXT)%
