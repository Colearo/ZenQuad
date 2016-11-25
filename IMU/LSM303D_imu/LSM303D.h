#include "mbed.h"
#include "vector.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
 *
 * @code
 * #include "mbed.h"
 * #include "LSM303D.h"
 *
 * Serial debug(USBTX,USBRX);
 * LSM303D compass(p28, p27);
 *
 * int main() 
 * { 
 *
 *  int count;
 *    
 *    debug.baud(115200);
 *    debug.printf("LSM303D Test\x0d\x0a");
 *    
 *    compass.setOffset(460, 610, -290);
 *    compass.setScale(0.93, 0.92, 1.00);
 *    
 *    while(1) 
 *    {
 *      compass.read();
 *      count++;
 *      if (count%10==0) debug.printf("Heading: %5.1f\n\r Pitch: %4.1f\n\r",compass.hdg, compass.pitch );
 *      wait(0.050);
 *    }
 * }
 * @endcode
 */
class LSM303D {
    public:
        //! ACC Raw readings with x,y and z axis
        vectors acc_raw;
        
        //! MAG Raw readings with x,y and z axis
        vectors mag_raw;
        
        //! ACC Filtert readings with x,y and z axis
        vectors acc;
        
        //! MAG Normal readings with x,y and z axis
        vectors mag;
        
        //! Heading
        float  hdg;
        
        //! Pitch
        float  pitch;
        
        //! MAG Minimal readings
        vectors min; 
        
        //! MAG Maximal readings
        vectors max;
        
        //! MAG Min-Max estimated range
        vectors spreed;   
        
        /** Create a new interface for an LSM303D
         *
         * @param sda is the pin for the I2C SDA line
         * @param scl is the pin for the I2C SCL line
         */
        LSM303D(PinName sda, PinName scl);

        /** sets the x, y, and z offset corrections for hard iron calibration
         * 
         * Calibration details here:
         *  http://mbed.org/users/shimniok/notebook/quick-and-dirty-3d-compass-calibration/
         *
         * If you gather raw magnetometer data and find, for example, x is offset
         * by hard iron by -20 then pass +20 to this member function to correct
         * for hard iron.
         *
         * @param x is the offset correction for the x axis
         * @param y is the offset correction for the y axis
         * @param z is the offset correction for the z axis
         */
        void setOffset(float x, float y, float z);
        
        /** sets the scale factor for the x, y, and z axes
         *
         * Calibratio details here:
         *  http://mbed.org/users/shimniok/notebook/quick-and-dirty-3d-compass-calibration/
         *
         * Sensitivity of the three axes is never perfectly identical and this
         * function can help to correct differences in sensitivity.  You're
         * supplying a multipler such that x, y and z will be normalized to the
         * same max/min values
         */
        void setScale(float x, float y, float z);      
        
        /** read the raw accelerometer and compass values
         * calc Heading and Pitch
         */
        void read();
    
    
        /** Set Mag Limits 
         */
        void set_limits(int mode);
    
    
        /** sets the I2C bus frequency
         *
         * @param frequency is the I2C bus/clock frequency, either standard (100000) or fast (400000)
         */
        void frequency(int hz);
        
        void monitor();
    
    private:
        I2C _compass;
        float _offset_x;
        float _offset_y;
        float _offset_z;
        float _scale_x;
        float _scale_y;
        float _scale_z;
        long _filt_ax;
        long _filt_ay;
        long _filt_az;

       
        bool write_reg(int addr_i2c,int addr_reg, char v);
        bool read_reg(int addr_i2c,int addr_reg, char *v);
        void calc_pos(void);
        void set_vectors(void);
        
};
