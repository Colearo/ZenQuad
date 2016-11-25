#ifndef PID_H
#define PID_H
#include "mbed.h"

// 积分限幅大小
#define INTEGRAL_MAX     200.0f
#define INTEGRAL_MIN     -200.0f

// PID输出限幅大小
#define PID_OUTPUT_MAX   800.0f
#define PID_OUTPUT_MIN   -800.0f

// PWM输出限幅大小
#define PWM_OUT_MIN      1000
#define PWM_OUT_MAX      2000

// 遥控中值(不是相同的，要调)
#define Ail_Mid    1491.0
#define Ele_Mid    1497.0
#define Rud_Mid    1497.0

// Roll的PID参数
#define Roll_Kp         1.9
#define Roll_Rate_Kp    0.90
#define Roll_Rate_Ti    0.00
#define Roll_Rate_Td    0.00

// Pitch的PID参数
#define Pitch_Kp        2.4
#define Pitch_Rate_Kp   0.60
#define Pitch_Rate_Ti   0.00
#define Pitch_Rate_Td   0.00

// Yaw的PID参数
#define Yaw_Kp          1.3
#define Yaw_Rate_Kp     0.60
#define Yaw_Rate_Ti     0.00
#define Yaw_Rate_Td     0.00

class PID{
public:
        PID();
        // PID更新时间
        float Time_dt;
        float avPre,avNow;//周期计数变量
        Timer conTime;
        
        //姿态结算之后获得的三个姿态角
        float Roll,Pitch,Yaw;
    
        //重力加速度
        float init_gy,init_gx,init_gz;
        
        // 欧拉角PID输出值
        float PID_Roll, PID_Pitch, PID_Yaw;
    
        // 遥控值
        float Motor_Ail;
        float Motor_Ele;
        float Motor_Thr;//throttle 油门
        float Motor_Rud;
        
        // 电机转速
        uint16_t Motor_1, Motor_2, Motor_3, Motor_4;
        
        // PID欧拉角总误差值
        float Roll_Err_Sum;
        float Pitch_Err_Sum;
        float Yaw_Err_Sum;

        // PID欧拉角上一次误差值
        float Roll_Err_Last;
        float Pitch_Err_Last;
        float Yaw_Err_Last;
        
        void initial(float angles[],float CH1,float CH2,float CH3,float CH4);  
        //计算四个电机的输出值，结果直接更新PID的成员
        void motorCalculate(void);
        
        void getDebug(void);

        // Roll的PID计算
        void PID_Roll_Calculate(void);
        // Pitch的PID计算
        void PID_Pitch_Calculate(void);
        // Yaw的PID计算
        void PID_Yaw_Calculate(void);
        // PWM输出限幅
        float limitPWM(float accelerator);
        // 遥控值处理计算，四个通道
        void motorExpectationCalculate(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4);
        //获取时间间隔
        void getPIDTime();
};
#endif
