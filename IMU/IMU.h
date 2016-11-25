#ifndef IMU__H
#define IMU__H
#include "L3GD20.h"
#include "LSM303D.h"
#include "mbed.h"

#define Kp 14.88f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.055f   // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.01f
#define Pi 3.14159265358979323846f
#define kapho 0.05f     
#define kaph2   0.01f
#define GYRO_CTIME 3
class IMU
{
    public:
        IMU();
        float angles[3];//欧拉角pitch、roll、yaw
        int ready; //Flag
        float gyroOffset[3];
        int beInited; //初始化q相关变量标志位
        void getYawPitchRoll();//公用函数，获得四元数转换成的欧拉角
        void getDebug();//用于调试的传感器ADC值输出
        void updateSensor(void);//得到四元数
        Serial HC05;    //蓝牙串口输出
    private:
            float avT,avPre,avNow;//针对采样周期计数所设置的变量
            float exInt, eyInt, ezInt;//误差积分
            float q0, q1, q2, q3;//全局四元数
            float q0q0 ;
            float q0q1 ;
            float q0q2 ;
            float q0q3 ;
            float q1q1 ;
            float q1q2 ;
            float q1q3 ;
            float q2q2 ;   
            float q2q3 ;
            float q3q3 ;
            float values[9];//九轴数据值数组
            float pre_ax,pre_ay,pre_az,pre_gx,pre_gy,pre_gz;//用于互补滤波
            float invSqrt(float x);//快速计算平方倒数
            Timer conTime;  
            vectors gyro;  //供L3GD20使用的类成员
            L3GD20 _gyro;//声明L3GD20陀螺仪对象
            LSM303D _compass;//声明LSM303D传感器对象
            void getValues();//获得传感器ADC值
            void AHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);//四元数更新
            void initAngles(float ax, float ay, float az, float mx, float my, float mz);//初始化相关量
            float yaw_offset;
};
#endif