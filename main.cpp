#include "mbed.h"
#include "rtos.h"
#include "PwmIn.h"
#include "IMU.h"
#include "PID.h"

#define P0 1013.25
#define STK_SIZE 1024
#define BUF_SIZE 256

PwmOut Motor1(PC_8);//4个电机信号输出口
PwmOut Motor2(PB_15);
PwmOut Motor3(D6);
PwmOut Motor4(D9);

PwmIn CH1(PB_7); //1通道 横滚
PwmIn CH2(PA_13);//2通道 俯仰
PwmIn CH3(PA_15);//接收机3通道 油门
PwmIn CH4(PA_14);//4通道

IMU imu;
PID pid;

static unsigned char stk1[STK_SIZE];
static unsigned char stk2[STK_SIZE];
static char test_buf[BUF_SIZE];
unsigned char ready = 0;

//电机初始化
void motorInit()
{
    Motor1.period_us(20000);
    Motor2.period_us(20000);
    Motor3.period_us(20000);
    Motor4.period_us(20000);
    Motor1.pulsewidth_us(1998);
    Motor2.pulsewidth_us(1998);
    Motor3.pulsewidth_us(1998);
    Motor4.pulsewidth_us(1998);
    wait(1.8);
    Motor1.pulsewidth_us(995);
    Motor2.pulsewidth_us(995);
    Motor3.pulsewidth_us(995);
    Motor4.pulsewidth_us(995);
}

//Return String from UART
static char *get_input()
{
    size_t i;
    int c;

    for (i = 0; i < BUF_SIZE-1; ++i)
    {
        c = imu.HC05.getc();
        imu.HC05.printf("%c", c);    // echo
        if (c == '\r' || c == '\n')
        {
            imu.HC05.printf("\n");
            break;
        }
        test_buf[i] = (char)c;
    }

    test_buf[i] = 0;
    return test_buf;
}

static int geti()//String to Int
{
    return atoi(get_input());
}

void imu_thread(void const *args)
{
    while(true) {
        imu.updateSensor();
        Thread::wait(4);
    }
}

void debug_thread(void const *args)
{
    while(true) {
//        imu.getDebug();
//        pid.getDebug();
//        printf("%f\n%f\n%f\n%f\n",CH1.pulsewidth()*1000000,CH2.pulsewidth()*1000000,CH3.pulsewidth()*1000000,CH4.pulsewidth()*1000000);
        imu.HC05.printf("\r\n* MENU *\r\n");
        imu.HC05.printf("1. IMU Status\r\n");
        imu.HC05.printf("2. PID_Status\r\n");
        imu.HC05.printf("3. Start_Cali\r\n");
        imu.HC05.printf("4. Keep_Lowest_Thr\r\n");
        imu.HC05.printf("5. Start_PID\r\n");
        
        imu.HC05.printf(" select? ");
        switch(geti())
        {
        case 1:
            imu.getDebug();
            break;
        case 2:
            pid.getDebug();
            imu.HC05.printf("Motor 1:%d\n",pid.Motor_1);
            imu.HC05.printf("Motor 2:%d\n",pid.Motor_2);
            imu.HC05.printf("Motor 3:%d\n",pid.Motor_3);
            imu.HC05.printf("Motor 4:%d\n",pid.Motor_4);
            break;
        case 3:
            imu.getYawPitchRoll();
            break;
        //case 4:
//            Motor1.pulsewidth_us(995);
//            Motor2.pulsewidth_us(995);
//            Motor3.pulsewidth_us(995);
//            Motor4.pulsewidth_us(995);
//            BLE.printf("Keep Finished.\n");
//            break;
//        case 5:
//            ready = 1;
//            BLE.printf("Ready.\n");
//            break;
        default:
            imu.HC05.printf("invalid choice\r\n");
            break;
        }
        Thread::wait(300);
    }
}

void pidcntl_thread(void const *args)
{
    while(true) {  
            if((pid.Motor_1==990)&&(pid.Motor_2==990)&&(pid.Motor_3==990)&&(pid.Motor_4==990))
            {
                ready = 1;
            }     
            imu.getYawPitchRoll();
            pid.initial(imu.angles,CH1.pulsewidth()*1000000,CH2.pulsewidth()*1000000,CH3.pulsewidth()*1000000,CH4.pulsewidth()*1000000);
            pid.motorCalculate();
            if(ready == 0){
                Motor1.pulsewidth_us(995); 
                Motor2.pulsewidth_us(995);
                Motor3.pulsewidth_us(995);
                Motor4.pulsewidth_us(995);
            }
            else{     
                Motor1.pulsewidth_us(pid.Motor_1); 
                Motor2.pulsewidth_us(pid.Motor_2);
                Motor3.pulsewidth_us(pid.Motor_3);
                Motor4.pulsewidth_us(pid.Motor_4);
            }
        Thread::wait(8);
    }
}
  
int main()
{
    motorInit();
    Thread t1(imu_thread);
    Thread t2(debug_thread);
    Thread t3(pidcntl_thread);
    while(true) {
    }
}


