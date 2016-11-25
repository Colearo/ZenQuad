/** Tilt-compensated compass interface Library for the STMicro LSM303D 3-axis magnetometer, 3-axis acceleromter
 *
 * Based on
 * 
 * Michael Shimniok http://bot-thoughts.com
 *
 * test program by tosihisa and 
 *
 * Pololu sample library for LSM303DLH breakout by ryantm:
 *
 * Copyright (c) 2011 Pololu Corporation. For more information, see
 *
 * http://www.pololu.com/
 * http://forum.pololu.com/
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
 
#include "mbed.h"
#include "LSM303D.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FILTER_SHIFT 1      // used in filtering acceleromter readings

const int addr_ls303d = 0x3A;

enum REG_ADDRS {
    /* --- Mag --- */
    OUT_TEMP    = 0x05,
    OUT_X_M     = 0x08,
    OUT_Y_M     = 0x0A,
    OUT_Z_M     = 0x0C,
    /* --- Acc --- */
    CTRL_REG1_A = 0x20,
    CTRL_REG2_A = 0x21,
    CTRL_REG5_A = 0x24,
    CTRL_REG6_A = 0x25,
    CTRL_REG7_A = 0x26,
    OUT_X_A     = 0x28,
    OUT_Y_A     = 0x2A,
    OUT_Z_A     = 0x2C,
};

bool LSM303D::write_reg(int addr_i2c,int addr_reg, char v)
{
    char data[2] = {addr_reg, v}; 
    return LSM303D::_compass.write(addr_i2c, data, 2) == 0;
}

bool LSM303D::read_reg(int addr_i2c,int addr_reg, char *v)
{
    char data = addr_reg; 
    bool result = false;
    
    __disable_irq();
    if ((_compass.write(addr_i2c, &data, 1) == 0) && (_compass.read(addr_i2c, &data, 1) == 0)){
        *v = data;
        result = true;
    }
    __enable_irq();
    return result;
}



LSM303D::LSM303D(PinName sda, PinName scl):
    _compass(sda, scl), _offset_x(0), _offset_y(0), _offset_z(0), _scale_x(1), _scale_y(1), _scale_z(1), _filt_ax(0), _filt_ay(0), _filt_az(9000)
{
    char reg_v;
    //_compass.frequency(100000);
    
    //Chip Setup
    
    //CTRL 0 (1Fh)all Default
    
    //CTRL 1 (20h)   
    reg_v = 0;
    reg_v |= 0x04 << 4;     /* ACC 25Hz Autosample  */
    reg_v |= 0x07;          /* X/Y/Z axis enable. */
    write_reg(addr_ls303d,CTRL_REG1_A,reg_v); //
    reg_v = 0;
    read_reg(addr_ls303d,CTRL_REG1_A,&reg_v);
    
    //CTRL 2 (21h)
    reg_v = 0x00;          
    reg_v |= 0x03 << 6;     /* 50Hz Antialias Filter */
    reg_v |= 0x00 << 3;     /* +/- 2g */
    write_reg(addr_ls303d,CTRL_REG2_A,reg_v);
    
    //CTRL 3 (22h) all Default
    
    //CTRL 4 (23h) all Default
    
    //CTRL 5 (24h)
    reg_v = 0x00;          
    reg_v |= 0x01 << 7;     /* Temp Sensor Enable 1*/
    reg_v |= 0x03 << 5;     /* Mag high Res  3*/
    reg_v |= 0x03 << 2;     /* Mag 25Hz Autosample*/
    write_reg(addr_ls303d,CTRL_REG5_A,reg_v);
    
    //CTRL 6 (25h)
    reg_v = 0x00;          
    reg_v |= 0x00 << 5;     /* +-2 Gauss*/
    write_reg(addr_ls303d,CTRL_REG6_A,reg_v); //25h

    //CTRL 7 (26h)
    reg_v = 0x00;          
    reg_v |= 0x00 << 0;     /* Mag Continuous Conversation*/
    write_reg(addr_ls303d,CTRL_REG7_A,reg_v); //26h

    min.x =-2000;
    min.y =-2000;
    min.z = 1000;
        
    max.x = -1000;
    max.y = -1000;
    max.z =  4000;
    
    spreed.x = 6000;
    spreed.y = 6000;
    spreed.z = 6000;
}


void LSM303D::setOffset(float x, float y, float z)
{
    _offset_x = x;
    _offset_y = y;
    _offset_z = z;   
}

void LSM303D::setScale(float x, float y, float z)
{
    _scale_x = x;
    _scale_y = y;
    _scale_z = z;
}

void LSM303D::set_vectors()
{

    // Perform simple lowpass filtering
    // Intended to stabilize heading despite
    // device vibration such as on a UGV
    _filt_ax += acc_raw.x - (_filt_ax >> FILTER_SHIFT);
    _filt_ay += acc_raw.y - (_filt_ay >> FILTER_SHIFT);
    _filt_az += acc_raw.z - (_filt_az >> FILTER_SHIFT);

    acc.x = (float) (_filt_ax >> FILTER_SHIFT);
    acc.y = (float) (_filt_ay >> FILTER_SHIFT);
    acc.z = (float) (_filt_az >> FILTER_SHIFT);
    
    // offset and scale
    //mag.x = (3 * mag.x + ((mag_raw.x + _offset_x) * _scale_x))/4;
    //mag.y = (3 * mag.y + ((mag_raw.y + _offset_y) * _scale_y))/4;
    //mag.z = (3 * mag.z + ((mag_raw.z + _offset_z) * _scale_z))/4; 
    mag.x = (mag.x + ((mag_raw.x + (((max.x - min.x)/2) - max.x)) * (10000/(max.x - min.x))) )/2;
    mag.y = (mag.y + ((mag_raw.y + (((max.y - min.y)/2) - max.y)) * (10000/(max.y - min.y))) )/2;
    mag.z = (mag.z + ((mag_raw.z + (((max.z - min.z)/2) - max.z)) * (10000/(max.z - min.z))) )/2;
   
    vector_normalize(&mag);  
}

void LSM303D::read()
{
    char data[1] = { OUT_X_A | (1<<7)}; //Page 23 In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddress field. In other words, SUB(7) must be equal to ‘1’ while SUB(6-0) represents the address of the first register to be read.
    char out[6] = {0,0,0,0,0,0};    
    
    _compass.write( addr_ls303d, data,1);
    _compass.read( addr_ls303d, out, 6);
    
    acc_raw.x= short( (((short)out[1]) << 8) | out[0] );
    acc_raw.y= short( (((short)out[3]) << 8) | out[2] );
    acc_raw.z= short( (((short)out[5]) << 8) | out[4] );   

    data[0] =  (OUT_X_M | (1<<7));

    _compass.write( addr_ls303d, data, 1 );
    _compass.read ( addr_ls303d, out, 6 );
    
    mag_raw.x= short( (((short)out[1]) << 8)| out[0]);
    mag_raw.y= short( (((short)out[3]) << 8)| out[2]);
    mag_raw.z= short( (((short)out[5]) << 8)| out[4]); 
  
    set_vectors();
    
    calc_pos();
}

void LSM303D::monitor(void)
{
    if((acc.x == 0.0f) && (acc.y == 0.0f) && (acc.z == 0.0f))
    {
        _compass.stop();
    }       
}

void LSM303D::calc_pos(void)
{   
  float x;
  float y;
  float c;
    ////////////////////////////////////////////////
    // compute heading       
    ////////////////////////////////////////////////
    vectors from = {1,0,0};
    vectors temp_a = acc;
    
    // normalize
    vector_normalize(&temp_a);

    // compute E and N
    vectors E;                           //vector East
    vectors N;                           //vector Nord
    vector_cross(&mag,&temp_a,&E);
    vector_normalize(&E);
    vector_cross(&temp_a,&E,&N);
    
    // compute heading
    x = vector_dot(&E,&from);
    y = vector_dot(&N,&from);
    c = sqrt(x*x + y*y);
    //hdg = atan2(x, y)) * 180/M_PI;
    hdg = atan2(x, y); // * 180/M_PI;
    hdg = ((asin(fabs(x)/c) * (1 - fabs(sin(hdg)))) + (acos(fabs(y)/c) * fabs(sin(hdg))))* 180 / M_PI;
    if ((x > 0) && (y < 0)) hdg = 180 - hdg;
    if ((x < 0) && (y < 0)) hdg = 180 + hdg;
    if ((x < 0) && (y > 0)) hdg = 360 - hdg;
    
    //if (hdg <   0) hdg += 360;
    //if (hdg > 360) hdg -= 360;
    
    vector_norm_xz(&temp_a);
    pitch = (asin(temp_a.x) * 180/M_PI) + 25;
    
}

void LSM303D::set_limits(int mode)
{
   if(mode == 1)                            //first cal.
    {
     if (max.x < mag_raw.x) max.x = mag_raw.x;
     if (min.x > mag_raw.x) min.x = mag_raw.x;
     if (max.y < mag_raw.y) max.y = mag_raw.y;
     if (min.y > mag_raw.y) min.y = mag_raw.y; 
     if (max.z < mag_raw.z) max.z = mag_raw.z;
     if (min.z > mag_raw.z) min.z = mag_raw.z;
   }
   if(mode == 2)                            //autorecal.
    {   
     if (max.x < mag_raw.x) max.x += 5;
     if (min.x > mag_raw.x) min.x -= 5;
     if (max.y < mag_raw.y) max.y += 5;
     if (min.y > mag_raw.y) min.y -= 5; 
     if (max.z < mag_raw.z) max.z += 5;
     if (min.z > mag_raw.z) min.z -= 5;  
    }
       
   if ((max.x - min.x) > spreed.x) {max.x--; min.x++;};
   if ((max.y - min.y) > spreed.y) {max.y--; min.y++;};
   if ((max.z - min.z) > spreed.z) {max.z--; min.z++;}; 
}

void LSM303D::frequency(int hz)
{
    _compass.frequency(hz);
}