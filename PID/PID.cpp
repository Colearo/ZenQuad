#include "PID.h"
PID::PID(){
    
        Time_dt = 0.0;
        avNow = 0.0;
        avPre = 0.0;
        conTime.start();
        
        Roll = 0.0;
        Pitch = 0.0;
        Yaw = 0.0;
    
        init_gy = 0.0;
        init_gx = 0.0;
        init_gz = 0.0;
    
        // 遥控值
        Motor_Ail = 0.0;
        Motor_Ele = 0.0;
        Motor_Thr = 0.0;//throttle 油门
        Motor_Rud = 0.0;
        
        
        // 欧拉角PID输出值
         PID_Roll = 0.0;
         PID_Pitch = 0.0;
         PID_Yaw = 0.0;
        
        // 电机转速
         Motor_1 = 0;
         Motor_2 = 0;
         Motor_3 = 0;
         Motor_4 = 0;
    
        // PID欧拉角总误差值
         Roll_Err_Sum   = 0.0;
         Pitch_Err_Sum  = 0.0;
         Yaw_Err_Sum    = 0.0;

        // PID欧拉角上一次误差值
         Roll_Err_Last  = 0.0;
         Pitch_Err_Last = 0.0;
         Yaw_Err_Last   = 0.0;
}

//Initial获取遥控值，重力加速度，姿态角
void PID::initial(float angles[],float CH1,float CH2,float CH3,float CH4){    
//      Motor_Ail = CH1;
//      Motor_Ele = CH2;
//      Motor_Thr = CH3;//throttle 油门
//      Motor_Rud = CH4;
        motorExpectationCalculate(CH1,CH2,CH3,CH4);
        Yaw =angles[2];
        Pitch = angles[1];
        Roll = angles[0];
}
    


// Roll的PID计算
void PID::PID_Roll_Calculate(void)
{
    float Proportion;//比例
    float Integral;//积分
    float Derivative;//微分
    float Error, Output;

    // 外环结果输入内环作误差值
    Error = Roll_Kp * (Motor_Ail - Roll) + init_gy * 57.295780f;
    
    
    // 总误差
    Roll_Err_Sum += Error;
    
    // PID计算
    Proportion = Roll_Rate_Kp * Error;
    Integral   = Roll_Rate_Ti * Roll_Err_Sum * Time_dt;
    Derivative = Roll_Rate_Td * (Error - Roll_Err_Last) / Time_dt;
    
    
    
    // 积分限幅
  if(Integral > INTEGRAL_MAX)
  {
            Integral = INTEGRAL_MAX;
  }
  if(Integral < INTEGRAL_MIN)
  {
            Integral = INTEGRAL_MIN;
  }
  
    // PID之和为输出
  Output = Proportion + Integral + Derivative;

    // PID输出限幅
  if(Output > PID_OUTPUT_MAX)
  {
            Output = PID_OUTPUT_MAX;
    }
  if(Output < PID_OUTPUT_MIN)
  {
            Output = PID_OUTPUT_MIN;
    }

    // 记录为上一次误差
  Roll_Err_Last = Error;
    
    // 输出PID最终值
    PID_Roll = Output;
}

// Pitch的PID计算
void PID::PID_Pitch_Calculate(void)
{
    float Proportion;
    float Integral;
    float Derivative;
    float Error, Output;

    Error = Pitch_Kp * (Pitch - Motor_Ele) + init_gx * 57.295780f;

    Pitch_Err_Sum += Error;

    Proportion = Pitch_Rate_Kp * Error;
    Integral   = Pitch_Rate_Ti * Pitch_Err_Sum * Time_dt;
    Derivative = Pitch_Rate_Td * (Error - Pitch_Err_Last) / Time_dt;

  // 积分限幅大小
  if(Integral > INTEGRAL_MAX)
  {
            Integral = INTEGRAL_MAX;
  }
  if(Integral < INTEGRAL_MIN)
  {
            Integral = INTEGRAL_MIN;
  }
  
  Output = Proportion + Integral + Derivative;

  if(Output > PID_OUTPUT_MAX)
  {
            Output = PID_OUTPUT_MAX;
    }
  if(Output < PID_OUTPUT_MIN)
  {
            Output = PID_OUTPUT_MIN;
    }

    Pitch_Err_Last = Error;
    
    
    PID_Pitch = Output;
}

// Yaw的PID计算
void PID::PID_Yaw_Calculate(void)
{
    float Proportion;
    float Integral;
    float Derivative;
    float Error, Output;

    // Yaw角不做外环，直接使用内环
    Error = init_gz * 57.295780f - Motor_Rud;
    
    Yaw_Err_Sum += Error;

    Proportion = Yaw_Rate_Kp * Error;
    Integral   = Yaw_Rate_Ti * Yaw_Err_Sum * Time_dt;
    Derivative = Yaw_Rate_Td * (Error - Yaw_Err_Last) / Time_dt;
    
  if(Integral > INTEGRAL_MAX)
  {
            Integral = INTEGRAL_MAX;
  }
  if(Integral < INTEGRAL_MIN)
  {
            Integral = INTEGRAL_MIN;
  }
  
  Output = Proportion + Integral + Derivative;

  if(Output > PID_OUTPUT_MAX)
  {
            Output = PID_OUTPUT_MAX;
    }
  if(Output < PID_OUTPUT_MIN)
  {
            Output = PID_OUTPUT_MIN;
    }

  Yaw_Err_Last = Error;
    
    PID_Yaw = Output;
}


//单位可能有错
void PID::getPIDTime(){
    //获取系统时间
//    avNow = HAL_GetTick();//获取当前时间
//    Time_dt = ((float)(avNow-avPre)/1000.0f);//获取时间间隔
//    printf("time:%f\n",Time_dt);
//    avPre = avNow;
      Time_dt = conTime.read()/2.0f;
      conTime.reset();
}

void PID::motorCalculate(void)
{
    // 获取PID更新时间(具体获取时间)
    getPIDTime();

    // 三轴PID计算
    PID_Roll_Calculate();
    PID_Pitch_Calculate();
    PID_Yaw_Calculate();
    
    //PID计算完之后得到三个角的输出值，从而计算四个舵机的输出值

    // x模式电机转速融合公式
    // 从1-4分别为：左前顺时针、右前逆时针、左后逆时针、右后顺时针
    //（这部分有可能要修改)
    Motor_1 = (uint16_t)limitPWM(Motor_Thr + PID_Pitch + PID_Roll - PID_Yaw);
    Motor_2 = (uint16_t)limitPWM(Motor_Thr + PID_Pitch - PID_Roll + PID_Yaw);
    Motor_3 = (uint16_t)limitPWM(Motor_Thr - PID_Pitch - PID_Roll - PID_Yaw);
    Motor_4 = (uint16_t)limitPWM(Motor_Thr - PID_Pitch + PID_Roll + PID_Yaw);
    
    /*
    网上版
    [size=4]Motor[2] = (int16_t)(Thr - Pitch - Roll - Yaw );    //M3
    Motor[0] = (int16_t)(Thr + Pitch + Roll - Yaw );    //M1
    Motor[3] = (int16_t)(Thr - Pitch + Roll + Yaw );    //M4
    Motor[1] = (int16_t)(Thr + Pitch - Roll + Yaw );    //M2
    
    旋转规则
    Roll方向旋转，则电机1电机2同侧出力，电机0电机3反向出力
    Pitch方向旋转，则电机2电机3同侧出力，电机0电机1反向出力
    Yaw方向旋转，则电机1电机3同侧出力，电机0电机2反向出力
    
    */
    
    /*
    大师版
    
        motor1 = limit(Motor_Thr + pid_pitch + pid_roll + pid_yaw); //
        motor2 = limit(Motor_Thr + pid_pitch - pid_roll - pid_yaw); //
        motor3 = limit(Motor_Thr - pid_pitch + pid_roll - pid_yaw); //
        motor4 = limit(Motor_Thr - pid_pitch - pid_roll + pid_yaw); //
        
    */
    
    // 起飞前电机转速安全保护
    if(Motor_Thr <= 1050)
    {
            Motor_1 = 990;
            Motor_2 = 990;
            Motor_3 = 990;
            Motor_4 = 990;
    }
}

// PWM输出限幅
float PID::limitPWM(float accelerator)
{
    if(accelerator > PWM_OUT_MAX)
    {
        accelerator = PWM_OUT_MAX;
    }
    else if(accelerator < PWM_OUT_MIN)
    {
        accelerator = PWM_OUT_MIN;
    }
    else 
    {
        accelerator = accelerator;
    }

    return accelerator;
}

// 遥控值处理计算
void PID::motorExpectationCalculate(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4)
{
    // 遥控值限幅
  if(ch1 < 1000) { ch1=1000; }
  if(ch1 > 2000) { ch1=2000; }

  if(ch2 < 1000) { ch2=1000; }
  if(ch2 > 2000) { ch2=2000; }

  if(ch3 < 1000) { ch3=1000; }
  if(ch3 > 2000) { ch3=2000; }

  if(ch4 < 1000) { ch4=1000; }
  if(ch4 > 2000) { ch4=2000; }

  // 三通道遥控值零偏处理及范围缩小，油门通道数值不处理
  Motor_Ail = (float)((ch1 - Ail_Mid) * 0.16);
  Motor_Ele = (float)((ch2 - Ele_Mid) * 0.16);
  Motor_Thr = (float)ch3;
  Motor_Rud = (float)((ch4 - Rud_Mid) * 0.20);
}

void PID::getDebug()
    
{
    
//  printf("Thr:%f\n",Motor_Thr);
//  
//  
    printf("Motor 1:%d\n",Motor_1);
    printf("Motor 2:%d\n",Motor_2);
    printf("Motor 3:%d\n",Motor_3);
    printf("Motor 4:%d\n",Motor_4);
//    printf("Motor Ail:%f\n",Motor_Ail);
//    printf("Motor Ele:%f\n",Motor_Ele);
//    printf("Motor Thr:%f\n",Motor_Thr);
//    printf("Motor Rud:%f\n",Motor_Rud);
//  printf("Roll1:%f\n",Roll);
//  printf("Yaw1:%f\n",Yaw);
//  printf("Pitch1:%f\n",Pitch);
}