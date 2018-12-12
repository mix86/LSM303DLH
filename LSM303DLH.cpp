/** LSM303DLH Interface Library
 *
 * Michael Shimniok http://bot-thoughts.com
 * Modified by @author Salco <JeSuisSalco@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
#include "mbed.h"
#include "LSM303DLH.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FILTER_SHIFT 6      // used in filtering acceleromter readings





bool LSM303DLH::write_reg(int addr_i2c,int addr_reg, uint8_t v)
{
    return this->write_reg(addr_i2c,addr_reg,(char)v);
}

bool LSM303DLH::write_reg(int addr_i2c,int addr_reg, char v)
{
    bool result=false;
    char data[2] = {addr_reg, v};
    //__disable_irq();
    result = m_ptr_I2C->write(addr_i2c, data, 2) == 0;
    if(result == false) debug("Unable to Write \n");
    
    //__enable_irq(); 
    return result;
}

bool LSM303DLH::read_reg(int addr_i2c,int addr_reg, uint8_t *v)
{
    return this->read_reg(addr_i2c,addr_reg,(char*)v);
}


bool LSM303DLH::read_reg(int addr_i2c,int addr_reg, char *v)
{
    char data = addr_reg; 
    bool result = false;
    
    //__disable_irq();
    if(m_ptr_I2C->write(addr_i2c, &data, 1) == 0)
    {        
        if (m_ptr_I2C->read(addr_i2c, &data, 1) == 0)
        {            
            *v = data;
            result = true;
        }
        else
        {
            debug("Unable to Read \n");
        }
    }
    else
    {
        debug("Unable to Write \n");
    }
    //__enable_irq();
    return result;
}

bool LSM303DLH::read_reg_short(int addr_i2c,int addr_reg, short *v)
{
    
    char *pv = (char *)v;
    bool result;
    
    result =  read_reg(addr_i2c,addr_reg+0,pv+1);
    result &= read_reg(addr_i2c,addr_reg+1,pv+0);
  
    
    return result;
}

bool LSM303DLH::read_reg_short(DEV_ADDRS addr_i2c,REG_ADDRS addr_reg, OUT_XYZ_t *dataRead)
{
     bool result=true;
     
     switch(addr_reg)
     {
         case OUT_X_A:
         case OUT_Y_A:
         case OUT_Z_A:
         
         case OUT_X_M:
         case OUT_Y_M:
         case OUT_Z_M:
            result &= read_reg(addr_i2c,addr_reg  ,&(dataRead->UT_L_A));//LSB at lower
            result &= read_reg(addr_i2c,addr_reg+1,&(dataRead->UT_H_A));
            #if defined(LSM303_LITLE_ENDIAN)
                dataRead.value = (dataRead.byte[1]<<8)+(dataRead.byte[0]); //swap the reading
            #endif
             break;
        default:
            result = false;
            break;
    }
    
    return result;
}

LSM303DLH::~LSM303DLH()
{
    if(m_have_createdI2C)
    {
        delete m_ptr_I2C;
    }
}
LSM303DLH::LSM303DLH(PinName sda, PinName scl)     
{
   m_ptr_I2C = new I2C(sda, scl);
   m_have_createdI2C = true;
   this->init(); 
}
LSM303DLH::LSM303DLH(I2C* ptrI2C)
{
    m_ptr_I2C = ptrI2C;
    m_have_createdI2C = false;
    this->init();
}

void LSM303DLH::init(void)
{
    char reg_v;
 
    _offset_x = 0; 
    _offset_y = 0;
    _offset_z = 0; 
    _scale_x  = 0; 
    _scale_y  = 0; 
    _scale_z  = 0; 
    _filt_ax  = 0; 
    _filt_ay  = 0; 
    _filt_az  = 6000;
 
 
    m_ptr_I2C->frequency(100000);
    
    ((Ctrl_Reg1_A_t*)&reg_v)->byte = 0;
    ((Ctrl_Reg1_A_t*)&reg_v)->ODR |= 0b0010;     /* Normal mode  */
    ((Ctrl_Reg1_A_t*)&reg_v)->Xen |= 1;          /* X/Y/Z axis enable. */
    ((Ctrl_Reg1_A_t*)&reg_v)->Yen |= 1;
    ((Ctrl_Reg1_A_t*)&reg_v)->Zen |= 1;
    write_reg(addr_acc,CTRL_REG1_A,reg_v);
    //not sure if we need to read the register
    //reg_v = 0;
    //read_reg(addr_acc,CTRL_REG1_A,&reg_v);

    ((Ctrl_Reg4_A_t*)&reg_v)->byte = 0;
    ((Ctrl_Reg4_A_t*)&reg_v)->BDU |= 1;     //full read befor update
    //(((Ctrl_Reg4_A_t*)&reg_v)->HR) |= 1;      //hi res
    #if defined(LSM303_LITLE_ENDIAN)
        ((Ctrl_Reg4_A_t*)&reg_v)->BLE |= 1;     /* 1: data MSB @ lower address */
    #endif
    ((Ctrl_Reg4_A_t*)&reg_v)->FS |= 0b01; ;     /* +/- 4g */
    write_reg(addr_acc,CTRL_REG4_A,reg_v);

    /* -- mag --- */
    debug("in MAG \n");
    ((CRA_REG_M_t*)&reg_v)->byte = 0;
    ((CRA_REG_M_t*)&reg_v)->DO |= 0b100;     /* Minimum data output rate = 15Hz */
    write_reg(addr_mag,CRA_REG_M,reg_v);

    reg_v = 0;
    //reg_v |= 0x01 << 5;     /* +-1.3Gauss */
    ((CRB_REG_M_t*)&reg_v)->GN |= 0b111;     /* +-8.1Gauss */
    write_reg(addr_mag,CRB_REG_M,reg_v);

   ((MR_REG_M_t*)&reg_v)->byte = 0;
   //((MR_REG_M_t*)&reg_v)->MD  |= 0;              /* Continuous-conversion mode */
    write_reg(addr_mag,MR_REG_M,reg_v);
    
    //put here since we dont change it during the execution
    m_FS = get_FullScall_selection();
    
    read_reg(addr_mag,CRB_REG_M ,&reg_v);
    m_GN = (((CRB_REG_M_t*)&reg_v)->GN)-1;
}

void LSM303DLH::setOffset(float x, float y, float z)
{
    _offset_x = x;
    _offset_y = y;
    _offset_z = z;   
}

void LSM303DLH::setScale(float x, float y, float z)
{
    _scale_x = x;
    _scale_y = y;
    _scale_z = z;
}
//#define _FS 4
bool LSM303DLH::read(myvector &a, myvector &m)
{
    
    bool result = true;
    //short a_x, a_y, a_z;
    //short m_x, m_y, m_z;
    #if defined(CHECK_TIME_SEQUENCE)
        Timer t;
        int usec1, usec2;
        
        t.reset();
        t.start();
    
        usec1 = t.read_us();
    #endif
   
    static myvector local_a, local_m;
    
    result &= read_acc_raw(&local_a);
    
    result &= read_mag_raw(&local_m);
    
    
    #if defined(CHECK_TIME_SEQUENCE)
        usec2 = t.read_us();
        
        debug("%d %d %d\n", usec1, usec2, usec2-usec1);//if (debug) debug->printf("%d %d %d\n", usec1, usec2, usec2-usec1);
    #endif
    if(result == true)
    {
        // Perform simple lowpass filtering
        // Intended to stabilize heading despite
        // device vibration such as on a UGV
        
       
        //float( a[i] ) * pow(2.,(fs+1)) / 32768.
        
        //x/8 = reading /0xFFFF(655355)
        
        
        
       // _filt_ax = _filt_ax + (a_x - (_filt_ax >> FILTER_SHIFT));
       /* _filt_ax = _filt_ax + (ax_test - (_filt_ax >> FILTER_SHIFT));
        _filt_ay += a_y - (_filt_ay >> FILTER_SHIFT);
        _filt_az += a_z - (_filt_az >> FILTER_SHIFT);
    
        
        
        a.x = (float) (_filt_ax >> FILTER_SHIFT);
        a.y = (float) (_filt_ay >> FILTER_SHIFT);
        a.z = (float) (_filt_az >> FILTER_SHIFT);*/
        
        a = local_a;
 
        
        // offset and scale        
        m.x = (/*m_*/local_m.x + _offset_x) * _scale_x;
        m.y = (/*m_*/local_m.y + _offset_y) * _scale_y;
        m.z = (/*m_*/local_m.z + _offset_z) * _scale_z;
    
    }
    
    return result;
}


// Returns the number of degrees from the -Y axis that it
// is pointing.
float LSM303DLH::heading()
{
    return heading((myvector){0,-1,0});
}

float LSM303DLH::heading(myvector from)
{
    myvector a, m;

    read(a, m);
    
    ////////////////////////////////////////////////
    // compute heading       
    ////////////////////////////////////////////////

    myvector temp_a = a;
    // normalize
    vector_normalize(&temp_a);
    //vector_normalize(&m);

    // compute E and N
    myvector E;
    myvector N;
    vector_cross(&m,&temp_a,&E);
    vector_normalize(&E);
    vector_cross(&temp_a,&E,&N);
    
    // compute heading
    float heading = atan2(vector_dot(&E,&from), vector_dot(&N,&from)) * 180/M_PI;
    if (heading < 0) heading += 360;
    
    return heading;
}

void LSM303DLH::frequency(int hz)
{
    m_ptr_I2C->frequency(hz);
}

int8_t LSM303DLH::get_FullScall_selection(void)
{
    char data_read_acc =0;
    read_reg(addr_acc,CTRL_REG4_A,&data_read_acc);
        
    return 2<<((((Ctrl_Reg4_A_t*)&data_read_acc)->FS));
}

float LSM303DLH::get_acc_value_in_g(OUT_XYZ_t* dataOut)
{
   return (float) (dataOut->value / (float)(32768 /*half of the ADC resolution*/ / m_FS/*+- 4g*/));
}

bool LSM303DLH::read_acc_raw(myvector *a)
{
    bool result = true;
    char data_read_acc =0;
    OUT_XYZ_t dataOut;
    
    read_reg(addr_acc,STATUS_REG_A,&data_read_acc);
   
    if(((Status_Reg_A_t*)&data_read_acc)->ZYXDA)//new data
    {
        result &= read_reg_short(addr_acc,OUT_X_A,&dataOut);
        
        if(result)
        {
            a->x = get_acc_value_in_g(&dataOut);
        }
        else
        {
            debug("error reading \n");
        }
        
        if(result)
        {
            result &= read_reg_short(addr_acc,OUT_Y_A,&dataOut);                
        }
        if(result)
        {
            a->y = get_acc_value_in_g(&dataOut);
        }
        else
        { 
            debug("error reading \n");
        }
        
        if(result)
        { 
            result &= read_reg_short(addr_acc,OUT_Z_A,&dataOut);
        }
        if(result)
        {
            a->z = get_acc_value_in_g(&dataOut);
        }
        else
        { 
            debug("error reading \n");
        }
    }
    
    return result;
}
bool LSM303DLH::read_mag_raw(myvector *m)
{
    bool result = true;
    char data_read_mag =0;
    OUT_XYZ_t dataOut;
    
    read_reg(addr_mag,SR_REG_M,&data_read_mag);
    
    
    /**@todo not sure if the reading of magnetometer is Litle or big endian, I assume its change like the accelerometer.
     * You can try to find the answer if you care.
     */
     
    if(((SR_Reg_M_t*)&data_read_mag)->DRDY)
    {    
        float gainxy[] = { 1100., 855., 670., 450., 400., 330., 230. };
        float gainz[]  = {  980., 760., 600., 400., 355., 295., 205. };
        
        result &= read_reg_short(addr_mag,OUT_X_M,&dataOut);
        if(result)
        {
            //dataOut.value = (dataOut.byte[0]<<8)+(dataOut.byte[1]);//only a test            
            m->x = float(dataOut.value) / gainxy[m_GN];
        }
        
        result &= read_reg_short(addr_mag,OUT_Y_M,&dataOut);
        if(result)
        {
            //dataOut.value = (dataOut.byte[0]<<8)+(dataOut.byte[1]);//only a test            
            m->y = float(dataOut.value) / gainxy[m_GN];
        }
            
        result &= read_reg_short(addr_mag,OUT_Z_M,&dataOut);
        if(result)
        {
            //dataOut.value = (dataOut.byte[0]<<8)+(dataOut.byte[1]);//only a test            
            m->z = float(dataOut.value) / gainz[m_GN];
        }        
    }
    
    return result;
}