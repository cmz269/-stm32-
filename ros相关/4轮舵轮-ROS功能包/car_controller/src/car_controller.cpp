/*
*******************************************
*/
#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
//创建一个serial类
serial::Serial sp;

#define  to_rad  0.017453f  //角度转弧度
//只简单输出前轮的行程和前轮的转角,未涉及坐标转换
float speed_x;//前驱三轮车前轮速度
float turn_z;//前驱三轮车前轮转角

uint8_t FLAG_USART; //串口发送标志
uint16_t count_1;//计数器

uint8_t Flag_start=0;//下位机运行标志
float x_mid_speed; //
float y_mid_speed; //
float z_mid_speed; //
float z_mid_angle; //
float angle_A,angle_B,angle_C,angle_D;//发送到下位机的4个轮子的角度
float speed_A,speed_B,speed_C,speed_D;//发送到下位机的4个轮子的速度
int size;
float Data_US[12];//发送到下位机的数据数组
float Data_UR[22];//接收来自下位机的数据数组

void send_data(void);//串口发送协议函数

typedef unsigned char byte;
float b2f(byte m0, byte m1, byte m2, byte m3)//float 型解算为4个字节
{
//求符号位
    float sig = 1.;
    if (m0 >=128.)
        sig = -1.;
  
//求阶码
    float jie = 0.;
     if (m0 >=128.)
    {
        jie = m0-128.  ;
    }
    else
    {
        jie = m0;
    }
    jie = jie * 2.;
    if (m1 >=128.)
        jie += 1.;
  
    jie -= 127.;
//求尾码
    float tail = 0.;
    if (m1 >=128.)
        m1 -= 128.;
    tail =  m3 + (m2 + m1 * 256.) * 256.;
    tail  = (tail)/8388608;   //   8388608 = 2^23

    float f = sig * pow(2., jie) * (1+tail);
 
    return f;
}

void chatterCallback(const geometry_msgs::Twist &msg)//获取键盘控制的回调函数
{
/*四轮四驱*/  
//	  ROS_INFO("X_linear: [%g]", msg.linear.x);//
//    ROS_INFO("Y_linear: [%g]", msg.linear.y);//
//    ROS_INFO("Z_linear: [%g]", msg.linear.z);//
//    ROS_INFO("X_angular: [%g]", msg.angular.x);//
//    ROS_INFO("Y_angular: [%g]", msg.angular.y);//
//    ROS_INFO("Z_angular: [%g]", msg.angular.z);//
//    ROS_INFO("-------------");

    x_mid_speed =msg.linear.x;//将这个值作为X方向的速度目标
    z_mid_angle =msg.linear.z;//将这个值作为Z旋转时的速度目标
    z_mid_speed =msg.angular.z;//将这个值作为Y方向的速度目标

    if(x_mid_speed > +1.1)x_mid_speed = +1.1;
    if(x_mid_speed < -1.1)x_mid_speed = -1.1;

    if(z_mid_speed > +1.1)z_mid_speed = +1.1;
    if(z_mid_speed < -1.1)z_mid_speed = -1.1;

    if(z_mid_angle > +1.1)z_mid_angle = +1.1;
    if(z_mid_angle < -1.1)z_mid_angle = -1.1;


            if(x_mid_speed>0 && z_mid_speed==0 && z_mid_angle==0){//按下 I 键 
			               speed_A= x_mid_speed;
				       speed_B= x_mid_speed; 
				       speed_C= x_mid_speed; 
				       speed_D= x_mid_speed;  
				       angle_A=0;angle_B=0;angle_C=0;angle_D=0;
			        }//前进
       else if(x_mid_speed<0 && z_mid_speed==0 && z_mid_angle==0){//按下 < 键 
			               speed_A= x_mid_speed;
				       speed_B= x_mid_speed; 
				       speed_C= x_mid_speed; 
				       speed_D= x_mid_speed;  
				       angle_A=0;angle_B=0;angle_C=0;angle_D=0;
			        }//后退

       else if(x_mid_speed==0 && z_mid_angle==0 && z_mid_speed>0){//按下 J 键
			               speed_A= z_mid_speed*0.5;
				       speed_B= z_mid_speed*0.5; 
				       speed_C= z_mid_speed*0.5; 
				       speed_D= z_mid_speed*0.5;  
				       angle_A=+90;angle_B=+90;angle_C=+90;angle_D=+90;
			        }//左移
       else if(x_mid_speed==0 && z_mid_angle==0 && z_mid_speed<0){//按下 L 键
			               speed_A= z_mid_speed*0.5;
				       speed_B= z_mid_speed*0.5; 
				       speed_C= z_mid_speed*0.5; 
				       speed_D= z_mid_speed*0.5;  
				       angle_A=+90;angle_B=+90;angle_C=+90;angle_D=+90;
			        }//右移

       else if(x_mid_speed==0 && z_mid_angle>0 && z_mid_speed==0){//按下 T 键
			         speed_A= z_mid_angle;
				       speed_B= -z_mid_angle; 
				       speed_C= -z_mid_angle; 
				       speed_D= z_mid_angle;  
				       angle_A=+54.78;angle_B=-54.78;angle_C=+54.78;angle_D=-54.78;
			        }//左自转
       else if(x_mid_speed==0 && z_mid_angle<0 && z_mid_speed==0){//按下 B 键
			               speed_A= z_mid_angle;
				       speed_B= -z_mid_angle; 
				       speed_C= -z_mid_angle; 
				       speed_D= z_mid_angle;  
				       angle_A=+54.78;angle_B=-54.78;angle_C=+54.78;angle_D=-54.78;
			        }//右自转

       else if(x_mid_speed>0 && z_mid_angle==0 && z_mid_speed>0){//按下 U 键
			               speed_A= x_mid_speed;
				       speed_B= x_mid_speed; 
				       speed_C= x_mid_speed; 
				       speed_D= x_mid_speed;  
				       angle_A=+45;angle_B=+45;angle_C=+45;angle_D=+45;
			        }//左斜上
       else if(x_mid_speed>0 && z_mid_angle==0 && z_mid_speed<0){//按下 O 键
			               speed_A= x_mid_speed;
				       speed_B= x_mid_speed; 
				       speed_C= x_mid_speed; 
				       speed_D= x_mid_speed;  
				       angle_A=-45;angle_B=-45;angle_C=-45;angle_D=-45;
			        }//右斜上
	else if(x_mid_speed<0 && z_mid_angle==0 && z_mid_speed<0){//按下 M 键
			               speed_A= x_mid_speed;
				       speed_B= x_mid_speed; 
				       speed_C= x_mid_speed; 
				       speed_D= x_mid_speed;  
				       angle_A=-45;angle_B=-45;angle_C=-45;angle_D=-45;
			        }//左斜下
        else if(x_mid_speed<0 && z_mid_angle==0 && z_mid_speed>0){//按下 > 键
			               speed_A= x_mid_speed;
				       speed_B= x_mid_speed; 
				       speed_C= x_mid_speed; 
				       speed_D= x_mid_speed;  
				       angle_A=+45;angle_B=+45;angle_C=+45;angle_D=+45;
			        }//右斜下
  
   if(x_mid_speed==0 && z_mid_speed==0 && z_mid_angle==0)Flag_start=0;
   else Flag_start=1;


   FLAG_USART=1;//

}


int main(int argc, char **argv){

    ros::init(argc, argv, "listener");

    ros::NodeHandle np;//为这个进程的节点创建一个句柄

    ros::Subscriber sub = np.subscribe("cmd_vel", 200, chatterCallback);//订阅键盘控制

    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;

  double y = 0.0;

  double th = 0.0;

 

  double vx = 0.0;

  double vy = 0.0;

  double vth = 0.0;

 

  ros::Time current_time, last_time;

  current_time = ros::Time::now();

  last_time = ros::Time::now();


   //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");

    }
    else
    {
        return -1;
    }  
     memset(Data_US, 0, sizeof(uint8_t)*12);		
    for(uint8_t j=0;j<3;j++)send_data(); 
		

   ros::Rate loop_rate(250);//设置循环间隔，即代码执行频率 250 HZ

   while(ros::ok())
   {
		 
         float angular_velocity_x = Data_UR[9]*0.001064;//角速度转换成 rad/s
				 float angular_velocity_y = Data_UR[10]*0.001064;//角速度转换成 rad/s
				 float angular_velocity_z = Data_UR[11]*0.001064;//角速度转换成 rad/s			 
				 float accelerated_speed_x = Data_UR[12]/2048;//转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
				 float accelerated_speed_y = Data_UR[13]/2048;//转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
				 float accelerated_speed_z = Data_UR[14]/2048;//转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
//--------------------------------------------------------------------------------------------------------//
//-----------------------------该方式计算线速度未验证是否正确-----------------------------//		 
//--------------------------------------------------------------------------------------------------------//		 
		 //设定车子正负方向 ，车子前进方向为X正，后退为X负，左移为Y正，右移为Y负
		 //轮子从上往下看，逆时针转为正角度，顺时针转为负角度
   float Power_A_X =	Data_UR[5] * cos(Data_UR[1]*to_rad);    //A轮X方向速度
   float Power_A_Y =	Data_UR[5] * sin(Data_UR[1]*to_rad);    //A轮Y方向速度
		
   float Power_B_X =	Data_UR[6] * cos(Data_UR[2]*to_rad);    //A轮X方向速度
   float Power_B_Y =	Data_UR[6] * sin(Data_UR[2]*to_rad);    //A轮Y方向速度
		
   float Power_C_X =	Data_UR[7] * cos(Data_UR[3]*to_rad);    //A轮X方向速度
   float Power_C_Y =	Data_UR[7] * sin(Data_UR[3]*to_rad);    //A轮Y方向速度
		
   float Power_D_X =	Data_UR[8] * cos(Data_UR[4]*to_rad);    //A轮X方向速度
   float Power_D_Y =	Data_UR[8] * sin(Data_UR[4]*to_rad);    //A轮Y方向速度	
    	
    
    vx  = (Power_A_X + Power_B_X + Power_C_X + Power_D_X)/4 ;//底盘当前X方向线速度 m/s	
    vy  = (Power_A_Y + Power_B_Y + Power_C_Y + Power_D_Y)/4 ;//底盘当前Y方向线速度 m/s	
    vth = angular_velocity_z;//设备当前Z轴角速度 rad/s
//--------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------//		

    current_time = ros::Time::now();//记录当前时间
		
    //以给定机器人速度的典型方式计算里程计
    double dt = (current_time - last_time).toSec();

    double delta_x = ((vx * cos(th) - vy * sin(th)) * dt)/2;

    double delta_y = ((vx * sin(th) + vy * cos(th)) * dt)/2;

    double delta_th = vth * dt;

 

      x += delta_x;//X轴速度累积位移 m

      y += delta_y;//Y轴速度累积位移 m

      th += delta_th;//Z轴角速度累积求车体朝向角度  rad //存在漂移



      //因为所有的里程表都是6自由度的，所以我们需要一个由偏航创建的四元数
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

 

      //首先，我们将通过tf发布转换

      geometry_msgs::TransformStamped odom_trans;

      odom_trans.header.stamp = current_time;

      odom_trans.header.frame_id = "odom";

      odom_trans.child_frame_id = "base_link";

 

      odom_trans.transform.translation.x = x;

      odom_trans.transform.translation.y = y;

      odom_trans.transform.translation.z = 0.0;

      odom_trans.transform.rotation = odom_quat;

 

      //发送转换

      odom_broadcaster.sendTransform(odom_trans);


       //接下来，我们将通过ROS发布里程计信息

       nav_msgs::Odometry odom;

       odom.header.stamp = current_time;

       odom.header.frame_id = "odom";

 

       //设置位置
       odom.pose.pose.position.x = x;

       odom.pose.pose.position.y = y;

       odom.pose.pose.position.z = th;

       odom.pose.pose.orientation = odom_quat;

 

       //设定速度
       odom.child_frame_id = "base_link";

       odom.twist.twist.linear.x = vx;

       odom.twist.twist.linear.y = vy;

       odom.twist.twist.angular.z = vth;

 


       //发布消息
       odom_pub.publish(odom);

       last_time = current_time;//保存为上次时间


        ros::spinOnce();//执行回调处理函数，完后继续往下执行
				
				if(FLAG_USART==1){	//若接收到键盘控制，则发送数据到下位机，同时接收下位机发送上来的数据		
		     /*四轮四驱*/				
        							  /*<01>*/Data_US[0]  = Flag_start;//电机启动开关，1启动 0停止
							          /*<02>*/Data_US[1]  = angle_A; 
							          /*<03>*/Data_US[2]  = angle_B ; 
							          /*<04>*/Data_US[3]  = angle_C ; 
							          /*<05>*/Data_US[4]  = angle_D ; //ABCD四轮的当前转角 deg
							          /*<06>*/Data_US[5]  = speed_A ;
							          /*<07>*/Data_US[6]  = speed_B ;    
							          /*<08>*/Data_US[7]  = speed_C ;    
							          /*<09>*/Data_US[8]  = speed_D ; //ABCD四轮的当前线速度 m/s
							          /*<10>*/Data_US[9]  = 0 ;//预留位  
							          /*<11>*/Data_US[10] = 0 ;//预留位 
							          /*<12>*/Data_US[11] = 0 ;//预留位
	
                                                                   send_data(); //发送指令控制电机运行
		    }FLAG_USART=0;
		
               //获取下位机的数据				
               size_t n = sp.available();//获取缓冲区内的字节数

               if(n>0)
               {
		
                 uint8_t buffer[92];
                   if(buffer[0]!=0XAA)sp.read(buffer, 1);//读出数据
                   if(buffer[0]==0XAA)sp.read(buffer, 92);//读出数据
                   if(buffer[0]==0XAA && buffer[1]==0XF1)
                   {               
                         uint8_t sum; 
	                 for(uint8_t j=0;j<91;j++)sum+=buffer[j];    //计算校验和	
                         if(buffer[91] == sum+buffer[0])
                         {	
                            for(uint8_t i=0;i<22;i++){
				Data_UR[i] =  b2f( buffer[4*i+3],  buffer[4*i+4],  buffer[4*i+5],  buffer[4*i+6] );
                            }	                       										 	
			              				
	                  }sum =0;
 			  memset(buffer, 0, sizeof(uint8_t)*92);						
                    }
							          /*<01>*///Data_UR[0] ;//电机启动开关，1启动 0停止
							          /*<02>*///Data_UR[1] ; 
							          /*<03>*///Data_UR[2] ; 
							          /*<04>*///Data_UR[3] ; 
							          /*<05>*///Data_UR[4] ;//ABCD四轮的当前转角 deg
							          /*<06>*///Data_UR[5] ;
							          /*<07>*///Data_UR[6] ;    
							          /*<08>*///Data_UR[7] ;    
							          /*<09>*///Data_UR[8] ;//ABCD四轮的当前线速度 m/s
							          /*<10>*///Data_UR[9] ; 
							          /*<11>*///Data_UR[10]; 
							          /*<12>*///Data_UR[11];//XYZ三轴角速度原始数值
							          /*<13>*///Data_UR[12]; 
							          /*<14>*///Data_UR[13]; 
							          /*<15>*///Data_UR[14];//XYZ三轴加速度原始数值
                        /*<16>*///Data_UR[15];			  
                        /*<17>*///Data_UR[16];
                        /*<18>*///Data_UR[17];//XYZ三轴角度		deg					 
							          /*<19>*///Data_UR[18];//电池电压
							          /*<20>*///Data_UR[19];//预留位
							          /*<21>*///Data_UR[20];//预留位
							          /*<22>*///Data_UR[21];//预留位	

							 }
							count_1++;
              if(count_1>24){//显示频率降低为10HZ
                  count_1=0;
								
		            if(Data_UR[0]==1)
                  ROS_INFO("[01] Flag_start: ON");//下位机电机启动/停止标志，1启动，0停止
                else
                  ROS_INFO("[01] Flag_start: OFF");//下位机电机启动/停止标志，1启动，0停止

                  ROS_INFO("[02] Current_angle_A: [%.2f deg]", Data_UR[1]); //A轮转向角 deg
                  ROS_INFO("[03] Current_angle_B: [%.2f deg]", Data_UR[2]); //B轮转向角 deg
                  ROS_INFO("[04] Current_angle_C: [%.2f deg]", Data_UR[3]); //C轮转向角 deg
                  ROS_INFO("[05] Current_angle_D: [%.2f deg]", Data_UR[4]); //D轮转向角 deg				 

                  ROS_INFO("[06] Current_linear_A: [%.2f m/s]", Data_UR[5]); //A轮线速度 m/s
                  ROS_INFO("[07] Current_linear_B: [%.2f m/s]", Data_UR[6]); //B轮线速度 m/s 
                  ROS_INFO("[08] Current_linear_C: [%.2f m/s]", Data_UR[7]); //C轮线速度 m/s 
                  ROS_INFO("[09] Current_linear_D: [%.2f m/s]", Data_UR[8]); //D轮线速度 m/s

                  ROS_INFO("[10] gyro_Roll: [%d ]",   (int)Data_UR[9]); //X轴角速度原始数据 gyro_Roll
                  ROS_INFO("[11] gyro_Pitch: [%d ]", (int)Data_UR[10]); //Y轴角速度原始数据 gyro_Pitch
                  ROS_INFO("[12] gyro_Yaw: [%d ]",   (int)Data_UR[11]); //Z轴角速度原始数据 gyro_Yaw
								
                  ROS_INFO("[13] accel_x: [%d ]",   (int)Data_UR[12]); //X轴加速度原始数据 accel_x
                  ROS_INFO("[14] accel_y: [%d ]",  (int)Data_UR[13]); //Y轴加速度原始数据 accel_x
                  ROS_INFO("[15] accel_z: [%d ]",  (int)Data_UR[14]); //Z轴加速度原始数据 accel_x 

                  ROS_INFO("[16] Roll: [%.2f deg]",   Data_UR[15]); //X轴角度 deg
                  ROS_INFO("[17] Pitch: [%.2f deg]",  Data_UR[16]); //Y轴角度 deg
                  ROS_INFO("[18] Yaw: [%.2f deg]",  Data_UR[17]); //Z轴角度 deg	 							
								
                  ROS_INFO("[19] Voltage: [%.2f V]", Data_UR[18]/100); // 电池电压
                  ROS_INFO("-----------------------"); 
																		
                }					 


        loop_rate.sleep();//循环延时时间
   }
    

   memset(Data_US, 0, sizeof(uint8_t)*12);
   for(uint8_t j=0;j<3;j++)send_data();
   //关闭串口
   sp.close();  

    return 0;
}


//************************发送12个数据**************************// 

void send_data(void)
{
	  uint8_t len=12;
    uint8_t tbuf[53];

    unsigned char *p;
				
    for(uint8_t i=0;i<len;i++){
	            p=(unsigned char *)&Data_US[i];
        tbuf[4*i+4]=(unsigned char)(*(p+3));
        tbuf[4*i+5]=(unsigned char)(*(p+2));
        tbuf[4*i+6]=(unsigned char)(*(p+1));
        tbuf[4*i+7]=(unsigned char)(*(p+0));
    }						
//fun:功能字 0XA0~0XAF
//data:数据缓存区，48字节
//len:data区有效数据个数

    tbuf[len*4+4]=0;  //校验位置零
    tbuf[0]=0XAA;   //帧头
    tbuf[1]=0XAA;   //帧头
    tbuf[2]=0XF1;    //功能字
    tbuf[3]=len*4;    //数据长度
    for(uint8_t i=0;i<(len*4+4);i++)tbuf[len*4+4]+=tbuf[i]; //计算和校验   
  try
  {
    sp.write(tbuf, len*4+5);//发送数据下位机(数组，字节数)

  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //如果发送数据失败，打印错误信息
  }
}  

