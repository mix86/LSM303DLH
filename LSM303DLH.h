#include "mbed.h"
#include "myvector.h"

#include "LSM303DLH_RegisterDef.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/** Tilt-compensated compass interface Library for the STMicro LSM303DLH 3-axis magnetometer, 3-axis acceleromter
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 *
 *
 * Base on the @see doccumentation https://cdn-shop.adafruit.com/datasheets/LSM303DLHC.PDF
 * @code
 * #include "mbed.h"
 * #include "LSM303DLH.h"
 *
 * Serial debug(USBTX,USBRX);
 * LSM303DLH compass(p28, p27);
 *
 * int main() {
 *   float hdg;
 *   debug.format(8,Serial::None,1);
 *   debug.baud(115200);
 *   debug.printf("LSM303DLH Test\x0d\x0a");
 *   compass.setOffset(29.50, -0.50, 4.00); // example calibration
 *   compass.setScale(1.00, 1.03, 1.21);    // example calibration
 *   while(1) {
 *     hdg = compass.heading();
 *     debug.printf("Heading: %.2f\n", hdg);
 *     wait(0.1);
 *   }
 * }
 * @endcode 
 * 
 * @author Salco <JeSuisSalco@gmail.com>
 */
 
 /** enable for MSB @ lower address */
//#define LSM303_LITLE_ENDIAN

class LSM303DLH {
    public:
        /** Create a new interface for an LSM303DLH
         *
         * @param sda is the pin for the I2C SDA line
         * @param scl is the pin for the I2C SCL line
         */
        LSM303DLH(PinName sda, PinName scl);
        /** Create a new interface for an LSM303DLH
         *
         * @param ptrI2C is a pointer from existing I2C
         */
        LSM303DLH(I2C* ptrI2C);
        /** Destructor of the class
         */
        ~LSM303DLH();
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
         *
         * @param a is the accelerometer 3d vector, written by the function
         * @param m is the magnetometer 3d vector, written by the function
         */
        bool read(myvector &a, myvector &m);
        /** read the raw accelerometer values
         *
         * @param a is the accelerometer 3d vector, written by the function
         */
        bool read_acc_raw(myvector *a);
        /** read the raw compass values
         *
         * @param m is the magnetometer 3d vector, written by the function
         */
        bool read_mag_raw(myvector *m);
        
        /** returns the magnetic heading with respect to the y axis
         *
         */
        float heading(void);
        
        /** returns the heading with respect to the specified vector
         *
         */
        float heading(myvector from);
    
        /** sets the I2C bus frequency
         *
         * @param frequency is the I2C bus/clock frequency, either standard (100000) or fast (400000)
         */
        void frequency(int hz);

      private:
        enum DEV_ADDRS {
            /* --- Mag --- */
            addr_mag   = 0x3c,
            /* --- Acc --- */
            addr_acc   = 0x32,//0x30;
            };
        
        enum REG_ADDRS {
            /* --- Mag --- */
            CRA_REG_M   = 0x00,
            CRB_REG_M   = 0x01,
            MR_REG_M    = 0x02,
            OUT_X_M     = 0x03,
            OUT_Y_M     = 0x05,
            OUT_Z_M     = 0x07,
            SR_REG_M    = 0x09,
            /* --- Acc --- */
            CTRL_REG1_A = 0x20,
            CTRL_REG4_A = 0x23,
            STATUS_REG_A= 0x27,
            OUT_X_A     = 0x28,
            OUT_Y_A     = 0x2A,
            OUT_Z_A     = 0x2C,
        };
        
        I2C* m_ptr_I2C;//_compass;
        float _offset_x;
        float _offset_y;
        float _offset_z;
        float _scale_x;
        float _scale_y;
        float _scale_z;
        long _filt_ax;
        long _filt_ay;
        long _filt_az;
        int8_t m_FS;
        int8_t m_GN;
        bool m_have_createdI2C;
        void init(void) ;
        
        bool write_reg(int addr_i2c,int addr_reg, uint8_t v);
        bool write_reg(int addr_i2c,int addr_reg, char v);
        
        bool read_reg(int addr_i2c,int addr_reg, uint8_t *v);
        bool read_reg(int addr_i2c,int addr_reg, char *v);
        
        bool read_reg_short(int addr_i2c,int addr_reg, short *v);
        bool read_reg_short(DEV_ADDRS addr_i2c,REG_ADDRS addr_reg, OUT_XYZ_t *dataRead);
        
        int8_t get_FullScall_selection(void);
        float get_acc_value_in_g(OUT_XYZ_t* dataOut);
};
