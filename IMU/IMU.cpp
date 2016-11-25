#include <IMU.h>

void usart1_niming_report(uint8_t fun,uint8_t*data,uint8_t len)     //测试

{
    uint8_t send_buf[32];
    uint8_t i;
    if(len>28)return;    //最多28字节数据 
    send_buf[len+3]=0;  //校验数置零
    send_buf[0]=0X88;   //帧头
    send_buf[1]=fun;    //功能字
    send_buf[2]=len;    //数据长度
    for(i=0;i<len;i++)send_buf[3+i]=data[i];     //复制数据
    for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];    //计算校验和 
    for(i=0;i<len+4;i++)printf("%c",send_buf[i]);   //发送数据到串口1 
}

/******************************IMU****************************/
IMU::IMU():
    HC05(PA_11,PA_12),_gyro(PB_9,PB_8),_compass(PB_9,PB_8)
{
    HC05.baud(38400);
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    exInt = 0.0;
    eyInt = 0.0;
    ezInt = 0.0;
    ready = 0;
    yaw_offset = 0;
    beInited = 0;
    pre_ax = pre_ay = pre_az = pre_gx = pre_gy = pre_gz = 0;
    conTime.start();
    avT = halfT;
    _compass.read();
    _gyro.read(&gyro.x,&gyro.y,&gyro.z);
    values[3]= _compass.acc.x;
    values[4]= _compass.acc.y;
    values[5]= _compass.acc.z;
    values[0]= gyro.x;
    values[1]= gyro.y;
    values[2]= gyro.z;
    values[6]= _compass.mag.x;
    values[7]= _compass.mag.y;
    values[8]= _compass.mag.z;
}
//IMU里面get传感器参数
void IMU::getValues() {  
/**********************************************************************
   IMU里面get传感器参数
***********************************************************************/
    //vector_normalize(&gy);
    //加速度、陀螺仪、磁力
//    pre_ax=values[3];//记录前一次的acc和gyro数据
//    pre_ay=values[4];
//    pre_az=values[5];
//    pre_gx=values[0];
//    pre_gy=values[1];
//    pre_gz=values[2];
//    values[3]= _compass.acc.x*(1-kapho)+pre_ax*(kapho);//进行按kapho和kaph2比例的迭代，互补滤波
//    values[4]= _compass.acc.y*(1-kapho)+pre_ay*(kapho);
//    values[5]= _compass.acc.z*(1-kapho)+pre_az*(kapho);
//    values[0]= gyro.x* (1-kaph2)+pre_gx*(kaph2) - gyroOffset[0];
//    values[1]= gyro.y*(1-kaph2)+pre_gy*(kaph2)- gyroOffset[1];
//    values[2]= gyro.z*(1-kaph2)+pre_gz*(kaph2)- gyroOffset[2];
//    values[6]= _compass.mag.x;
//    values[7]= _compass.mag.y;
//    values[8]= _compass.mag.z;  
//    values[0]= gyro.x;
//    values[1]= gyro.y;
//    values[2]= gyro.z;
//    values[3]= _compass.acc.x;
//    values[4]= _compass.acc.y;
//    values[5]= _compass.acc.z;
//    values[6]= _compass.mag.x;
//    values[7]= _compass.mag.y;
//    values[8]= _compass.mag.z;  
    //互补滤波
}

void IMU::AHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float accNorm;//加速度归一化变量
    float   qNorm;//四元数归一化变量
    float magNorm;//磁力计归一化变量
    float hx, hy, bx, bz;//分别是余弦矩阵的各个分量
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;//误差积分的分量
         
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
    //当且仅当加速度计的ADC值有效时才进行
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        //重力加速度各分量归一化
        accNorm = invSqrt(ax*ax + ay*ay + az*az);       
        ax = ax * accNorm;
        ay = ay * accNorm;
        az = az * accNorm;
        //磁力计各分量归一化
        magNorm = invSqrt(mx*mx + my*my + mz*mz);          
        mx = mx * magNorm;
        my = my * magNorm;
        mz = mz * magNorm;
        //计算地球的磁场方向
        hx = 2.0f*(mx*(0.5f - q2q2 - q3q3) + my*(q1q2 - q0q3) + mz*(q1q3 + q0q2));
        hy = 2.0f*(mx*(q1q2 + q0q3) + my*(0.5f - q1q1 - q3q3) + mz*(q2q3 - q0q1));        
        bx = sqrt((hx*hx) + (hy*hy));
        bz = 2.0f*(mx*(q1q3-q0q2)+my*(q2q3 + q0q1)+mz*(0.5f - q1q1 - q2q2));     
        //估计重力和磁场方向向量
        vx = 2*(q1q3 - q0q2);
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - 0.5f + q3q3;
        wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
        wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
        wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2);   
        //计算误差，误差是计算的参考系方向向量与传感器ADC值方向向量的叉积 
        ex = (ay*vz - az*vy) + (my*wz - mz*wy);
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
        
        //如果误差不为0，才计算
        if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
            //计算误差关于时间的比例积分
            exInt += ex * Ki * avT;
            eyInt += ey * Ki * avT; 
            ezInt += ez * Ki * avT;
            // 同时调整陀螺仪的测量值
            gx += Kp*ex + exInt;
            gy += Kp*ey + eyInt;
            gz += Kp*ez + ezInt;
            }
        }
          float qa=q0;
          float qb=q1;
          float qc=q2;
          // 按照一阶龙格库塔方法计算四元数
          
          q0 += (-qb*gx - qc*gy - q3*gz)* avT;
          q1 += (qa*gx + qc*gz - q3*gy)* avT;
          q2 += (qa*gy - qb*gz + q3*gx)* avT;
          q3 += (qa*gz + qb*gy - qc*gx)* avT;  
          // 四元数归一化
          qNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);//计算归一化变量
          q0 = q0 * qNorm;
          q1 = q1 * qNorm;
          q2 = q2 * qNorm;
          q3 = q3 * qNorm;
}

//函数名：updateSensor(void)
//描述: 更新四元数
void IMU::updateSensor(void) {
//  getValues();   
    _compass.read();
    _gyro.read(&gyro.x,&gyro.y,&gyro.z);
    for(int i = 0;i<15;i++){
        pre_ax=values[3];
        pre_ay=values[4];
        pre_ay=values[4];
        pre_gx=values[0];
        pre_gy=values[1];
        pre_gz=values[2];
        values[3]= _compass.acc.x*(1-kapho)+pre_ax*(kapho);//进行按kapho和kaph2比例的迭代，互补滤波
        values[4]= _compass.acc.y*(1-kapho)+pre_ay*(kapho);
        values[5]= _compass.acc.z*(1-kapho)+pre_az*(kapho);
        values[0]= gyro.x* (1-kaph2)+pre_gx*(kaph2) - gyroOffset[0];
        values[1]= gyro.y*(1-kaph2)+pre_gy*(kaph2)- gyroOffset[1];
        values[2]= gyro.z*(1-kaph2)+pre_gz*(kaph2)- gyroOffset[2];
        values[6]= _compass.mag.x;
        values[7]= _compass.mag.y;
        values[8]= _compass.mag.z;  
    }
        values[3] = -values[3];
        values[4] = -values[4];
        values[5] = -values[5];
//    pre_ax=values[3];//记录前一次的acc和gyro数据
//    pre_ay=values[4];
//    pre_az=values[5];
//    pre_gx=values[0];
//    pre_gy=values[1];
//    pre_gz=values[2];
//    values[3]= _compass.acc.x*(1-kapho)+pre_ax*(kapho);//进行按kapho和kaph2比例的迭代，互补滤波
//    values[4]= _compass.acc.y*(1-kapho)+pre_ay*(kapho);
//    values[5]= _compass.acc.z*(1-kapho)+pre_az*(kapho);
//    values[0]= gyro.x* (1-kaph2)+pre_gx*(kaph2) - gyroOffset[0];
//    values[1]= gyro.y*(1-kaph2)+pre_gy*(kaph2)- gyroOffset[1];
//    values[2]= gyro.z*(1-kaph2)+pre_gz*(kaph2)- gyroOffset[2];
//    values[6]= _compass.mag.x;
//    values[7]= _compass.mag.y;
//    values[8]= _compass.mag.z;  
//AHRSUpdate(values[0] *0.0174f , values[1] *0.0174f , values[2] *0.0174f, values[3], values[4], values[5], values[6], values[7], values[8]);
}

//函数名：invSqrt(float x)
//描述：求平方根的倒数
//该函数是经典的Carmack求平方根算法，效率极高，使用魔数0x5f375a86
float IMU::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//函数名：getYawPitchRoll(void)
//描述：四元数转换得到欧拉角
void IMU::getYawPitchRoll() {
  static float startTime = 0.0f;
  float offsetSum[3] = {0.0f};
  static uint16_t offsetCount = 0;
  float rotMatrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };
  if(!beInited){
      updateSensor();
      initAngles(-values[3], -values[4], -values[5], values[6], values[7], values[8]);
      beInited = 1;
  }
  
  //更新时钟计时
  avNow = conTime.read();//获得当前时间，单位s
  avT=((float)(avNow-avPre)/2.0f);//deltaT/2的时间间隔
  avPre = avNow;//重复，将前一时基赋值为现时间
  
  if(!ready){
    if(startTime == 0) startTime = avNow;
    updateSensor();
    offsetSum[0] += values[0];
    offsetSum[1] += values[1];
    offsetSum[2] += values[2];
    offsetCount++;
    if(avNow > startTime + GYRO_CTIME){
        gyroOffset[0] = offsetSum[0]/offsetCount;
        gyroOffset[1] = offsetSum[1]/offsetCount;
        gyroOffset[2] = offsetSum[2]/offsetCount;
        
        offsetCount = 0;
        offsetSum[0] = 0.0f;
        offsetSum[0] = 0.0f;
        offsetSum[0] = 0.0f;
        
        ready = 1;
        startTime = 0;
    } 
    
    return;
  }
  getValues();
  AHRSUpdate(values[0] *0.0174f , values[1] *0.0174f , values[2] *0.0174f, -values[3], -values[4], -values[5], values[6], values[7], values[8]);
  
  //旋转矩阵的各个元素
  rotMatrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
  rotMatrix[1] = 2.f * (q1*q2 + q0*q3);  // 12
  rotMatrix[2] = 2.f * (q1*q3 - q0*q2);  // 13
  rotMatrix[3] = 2.f * (q1*q2 - q0*q3);  // 21
  rotMatrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
  rotMatrix[5] = 2.f * (q2*q3 + q0*q1);  // 23
  rotMatrix[6] = 2.f * (q1*q3 + q0*q2);  // 31
  rotMatrix[7] = 2.f * (q2*q3 - q0*q1);  // 32
  rotMatrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33
  float c = invSqrt(rotMatrix[1]*rotMatrix[1]+rotMatrix[0]*rotMatrix[0]);
  
  angles[0] = atan2f(rotMatrix[5], rotMatrix[8]) * 180.0f / Pi;  //! Roll
  angles[1] = -asinf(rotMatrix[2]) * 180.0f / Pi;                //! Pitch
  
  angles[2] = atan2f(rotMatrix[1], rotMatrix[0]) * 180.0f / Pi;  //! Yaw
//  angles[2] = ((asin(fabs(rotMatrix[1])/c) * (1 - fabs(sin(angles[2])))) + (acos(fabs(rotMatrix[0])/c) * fabs(sin(angles[2]))))* 180.0f / Pi;
//   if ((rotMatrix[1] > 0) && (rotMatrix[0] < 0)) angles[2] = 180 - angles[2];
//    if ((rotMatrix[1] < 0) && (rotMatrix[0] < 0)) angles[2] = 180 + angles[2];
//    if ((rotMatrix[1] < 0) && (rotMatrix[0] > 0)) angles[2] = 360 - angles[2];
}

//函数名：initAngles
//描述：第一次初始化四元数参量
void IMU::initAngles(float ax, float ay, float az, float mx, float my, float mz) {
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2f(-ay, -az);
    initialPitch = atan2f(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}
//函数名：getDebug(void)
//描述：和上位机通讯
void IMU::getDebug()
{
//    uint8_t _cnt=0;
//printf("G-x:%6.2f,G-y:%6.2f,G-z:%6.2f\n",values[0],values[1],values[2]);
//    uint8_t send_buf[6]={0x00};
//  send_buf[_cnt++]=0x88;   //帧头
//  send_buf[_cnt++]=0xA1;    //功能字
//  send_buf[_cnt++]=0x1C;    //数据长度
//  int16_t temp=(int)values[3];
//  send_buf[_cnt++]=(temp>>8)&0xFF;
//  send_buf[_cnt++]=(temp)&0xFF;
//  temp=(int)values[4];
//  send_buf[_cnt++]=(temp>>8)&0xFF;
//  send_buf[_cnt++]=(temp)&0xFF;
//  temp=(int)values[5];
//  send_buf[_cnt++]=(temp>>8)&0xFF;
//  send_buf[_cnt++]=(temp)&0xFF;
//  temp=(int)values[0];
//  send_buf[_cnt++]=(temp>>8)&0xFF;
//  send_buf[_cnt++]=(temp)&0xFF;
//  temp=(int)values[1];
//  send_buf[_cnt++]=(temp>>8)&0xFF;
//  send_buf[_cnt++]=(temp)&0xFF;
//  temp=(int)values[2];
//  send_buf[_cnt++]=(temp>>8)&0xFF;
//  send_buf[_cnt++]=(temp)&0xFF;
//  temp=(int)values[6];
//  send_buf[_cnt++]=(temp>>8)&0xFF;
//  send_buf[_cnt++]=(temp)&0xFF;
//  temp=(int)values[7];
//  send_buf[_cnt++]=(temp>>8)&0xFF;
//  send_buf[_cnt++]=(temp)&0xFF;
//  temp=(int)values[8];
//  send_buf[_cnt++]=(temp>>8)&0xFF;
//  send_buf[_cnt++]=(temp)&0xFF;
    //  send_buf[3]=((uint16_t)values[3]>>8)&0xFF;
//    uint16_t temp=(int16_t)angles[2];
//    send_buf[_cnt++]=(temp>>8)&0xFF;
//    send_buf[_cnt++]=(temp)&0xFF;
//    temp=(int16_t)angles[1];
//    send_buf[_cnt++]=(temp>>8)&0xFF;
//    send_buf[_cnt++]=(temp)&0xFF;
//    temp=(int16_t)angles[0];
//    send_buf[_cnt++]=(temp>>8)&0xFF;
//    send_buf[_cnt++]=(temp)&0xFF;
//    usart1_niming_report(0xA1,send_buf,6);
//  send_buf[4]=((uint16_t)values[3])&0xFF;
//  send_buf[5]=((uint16_t)values[4]>>8)&0xFF;
//  send_buf[6]=((uint16_t)values[4])&0xFF;
//  send_buf[7]=((uint16_t)values[5]>>8)&0xFF;
//  send_buf[8]=((uint16_t)values[5])&0xFF;
//  send_buf[9]=((uint16_t)values[0]>>8)&0xFF;
//  send_buf[10]=((uint16_t)values[0])&0xFF;
//  send_buf[11]=((uint16_t)values[1]>>8)&0xFF;
//  send_buf[12]=((uint16_t)values[1])&0xFF;
//  send_buf[13]=((uint16_t)values[2]>>8)&0xFF;
//  send_buf[14]=((uint16_t)values[2])&0xFF;
//  send_buf[15]=((uint16_t)values[6]>>8)&0xFF;
//  send_buf[16]=((uint16_t)values[6])&0xFF;
//  send_buf[17]=((uint16_t)values[7]>>8)&0xFF;
//  send_buf[18]=((uint16_t)values[7])&0xFF;
//  send_buf[19]=((uint16_t)values[8]>>8)&0xFF;
//  send_buf[20]=((uint16_t)values[8])&0xFF;
//  send_buf[21]=0x00;//(roll>>8)&0xFF;
//  send_buf[22]=0x1f;//(roll)&0xFF;
//  send_buf[23]=0x00;//(pitch>>8)&0xFF;
//  send_buf[24]=0x2e;//(pitch)&0xFF;
//  send_buf[25]=0x00;//(yaw>>8)&0xFF;
//  send_buf[26]=0x20;//(yaw)&0xFF;
//  send_buf[_cnt++]=0x00;
//  send_buf[_cnt++]=0x00;
//  send_buf[_cnt++]=0x00;
//  send_buf[_cnt++]=0x00;
//  send_buf[_cnt]=0x00;
//    printf("ACC-x:%.1f,ACC-y:%.1f,ACC-z:%.1f\n",_compass.acc.x,_compass.acc.y,_compass.acc.z);
    //printf("Magx:%.1f,Magy:%.1f,Magz:%.1f\n",_compass.mag.x,_compass.mag.y,_compass.mag.z);
  HC05.printf("Pose: roll:%.2f pitch:%.2f yaw:%.2f\n",angles[0],angles[1],angles[2]);
//  _compass.monitor();
}
/*********************************IMU***********************************/
