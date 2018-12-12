/*
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#ifndef __LSM303DLH_REGISTERDEF_H_
#define __LSM303DLH_REGISTERDEF_H_

/** @defgroup LSM303_Register
  * @author      Salco <JeSuisSalco@gmail.com>
  */

/** @defgroup regA
  * Register for accelerometer
  * @ingroup LSM303_Register 
  * @{        
  */

//#if defined(DOXYGEN_ONLY)

/**@note add LSM303DLH:: to use the docc doxygen for no reason*/

/** CTRL_REG1_A structure definition
 */ 
union Ctrl_Reg1_A_t 
{
    uint8_t byte; /**< Value in byte.*/
    struct{
        uint8_t Xen:1; /**< X axis enable. Default value: 1.*/
        uint8_t Yen:1; /**< Y axis enable. Default value: 1.*/
        uint8_t Zen:1; /**< Z axis enable. Default value: 1.*/
        uint8_t LPen:1;/**< Low-power mode enable. Default value: 0.*/
        uint8_t ODR:4; /**< Data rate selection. Default value: 0.*/
    };    
};

/** CTRL_REG2_A structure definition
 */ 
union Ctrl_Reg2_A_t
{
    uint8_t byte; /**< Value in byte.*/
    struct{
        uint8_t HPIS1:1;    /**< High pass filter enabled for AOI function on Interrupt 1.*/
        uint8_t HPIS2:1;    /**< High pass filter enabled for AOI function on Interrupt 2.*/
        uint8_t HPCLICK:1;  /**< High pass filter enabled for CLICK function.*/
        uint8_t FDS:1;      /**< Filtered data selection. Default value: 0.*/
        uint8_t HPCF:2;     /**< High pass filter cut-off frequency selection.*/
        uint8_t HPM:2;      /**< High pass filter mode selection. Default value: 00*/
    };    
};

/** CTRL_REG3_A structure definition
 */ 
union Ctrl_Reg3_A_t
{
    uint8_t byte; /**< Value in byte.*/
    struct{
        uint8_t _Reserved:1;    /**< Do not use.*/
        uint8_t I1_OVERRUN:1;  /**< FIFO overrun interrupt on INT1. Default value 0.*/
        uint8_t I1_WTM:1;      /**< FIFO watermark interrupt on INT1. Default value 0.*/
        uint8_t I1_DRDY2:1;    /**< DRDY2 interrupt on INT1. Default value 0.*/
        uint8_t I1_DRDY1:1;    /**< DRDY1 interrupt on INT1. Default value 0.*/
        uint8_t I1_AOI2:1;     /**< AOI2 interrupt on INT1. Default value 0.*/
        uint8_t I1_AOI1:1;     /**< AOI1 interrupt on INT1. Default value 0.*/
        uint8_t I1_CLICK:1;    /**< CLICK interrupt on INT1. Default value 0.*/
    };    
};

/** CTRL_REG4_A structure definition
 */ 
union Ctrl_Reg4_A_t
{
    uint8_t byte; /**< Value in byte.*/
    struct{
        uint8_t SIM:1;      /**< SPI serial interface mode selection. Default value: 0.*/
        uint8_t _Reserved:2; /**< Do not use.*/
        uint8_t HR:1;       /**< High resolution output mode: Default value: 0.*/
        uint8_t FS:2;       /**< Full-scale selection. Default value: 00.*/
        uint8_t BLE:1;      /**< Big/little endian data selection. Default value 0.*/
        uint8_t BDU:1;      /**< Block data update. Default value: 0.*/
    };    
};

/** CTRL_REG5_A structure definition
 */ 
union Ctrl_Reg5_A_t
{
    uint8_t byte; /**< Value in byte.*/
    struct{
        uint8_t D4D_INT2:1;      /**< 4D detection is enabled on INT2 when 6D bit on INT2_CFG is set to 1.*/                
        uint8_t LIR_INT2:1;      /**< Latch interrupt request on INT2_SRC register, with INT2_SRC register 
                                      cleared by reading INT2_SRC itself. Default value: 0.*/
        uint8_t D4D_INT1:1;      /**< 4D detection is enabled on INT1 when 6D bit on INT1_CFG is set to 1.*/                
        uint8_t LIR_INT1:1;      /**< Latch interrupt request on INT1_SRC register, with INT1_SRC register 
                                      cleared by reading INT1_SRC itself. Default value: 0.*/
        uint8_t _Reserved:2;      /**< Do not use.*/
        uint8_t FIFO_EN:1;       /**< FIFO enable. Default value: 0.*/
        uint8_t BOOT:1;          /**< Reboot memory content. Default value: 0.*/
    };   
};



/** CTRL_REG6_A structure definition
 */ 
union Ctrl_Reg6_A_t
{
    uint8_t byte; /**< Value in byte.*/
    struct{
        uint8_t _Reserved:1;     /**< Do not use.*/
        uint8_t H_LACTIVE:1;    /**< Interrupt active high, low. Default value 0.*/    
        uint8_t __Reserved:1;     /**< Do not use.*/            
        uint8_t P2_ACT:1;       /**< Active function status on PAD2. Default value 0.*/
        uint8_t BOOT_I1:1;      /**< Reboot memory content on PAD2. Default value: 0.*/                
        uint8_t I2_INT2:1;      /**< Interrupt 2 on PAD2. Default value 0.*/
        uint8_t I2_INT1:1;      /**< Interrupt 1 on PAD2. Default value 0.*/
        uint8_t I2_CLICKen:1;   /**< CLICK interrupt on PAD2. Default value 0.*/
    };   
};

 /** Status_Reg_A structure definition
 */ 
union Status_Reg_A_t
{
    uint8_t byte; /**< Value in byte.*/
    struct{
        
        uint8_t XDA:1;          /**< X axis new data available. Default value: 0.*/    
        uint8_t YDA:1;          /**< Y axis new data available. Default value: 0.*/            
        uint8_t ZDA:1;          /**< Z axis new data available. Default value: 0.*/
        uint8_t ZYXDA:1;        /**< X, Y, and Z axis new data available. Default value: 0.*/                
        uint8_t XOR:1;          /**< X axis data overrun. Default value: 0.*/
        uint8_t YOR:1;          /**< Y axis data overrun. Default value: 0.*/
        uint8_t ZOR:1;          /**< Z axis data overrun. Default value: 0.*/
        uint8_t ZYXOR:1;        /**< X, Y, and Z axis data overrun. Default value: 0.*/
    };   
};

/** OUT_XYZ structure. The value is expressed in 2’s complement
*/
union OUT_XYZ_t
{
    int16_t value;      /**< Value in signed integer.*/
    uint8_t byte[2];
    struct{
        uint8_t UT_L_A; /**< Low register.*/
        uint8_t UT_H_A; /**< High register.*/
    };
};
/** @} */ // end of regA

/** @defgroup regM  
  *  Register for magnetometer
  * @ingroup LSM303_Register 
  *  @{        
  */
 
/** SR_Reg_M structure definition
 */ 
union SR_Reg_M_t
{
    uint8_t byte; /**< Value in byte.*/
    struct{
        uint8_t DRDY:1;      /**< Data output register lock.*/                
        uint8_t LOCK:1;      /**< Data ready bit.*/ 
        uint8_t _Reserved:8; /**< Do not use.*/
    };   
};

/** CRA_REG_M structure definition
 */ 
union CRA_REG_M_t
{
    uint8_t byte; /**< Value in byte.*/
    struct{
        uint8_t _Reserved:2;    /**< Do not use.  Must be set to 0*/
        uint8_t DO:3;           /**< Data output rate bits.*/
        uint8_t __Reserved:2;   /**< Do not use.  Must be set to 0*/
        uint8_t TEMP_EN:1;      /**< Temperature sensor enable.*/
    };   
};

/** CRB_REG_M structure definition
 */ 
union CRB_REG_M_t
{
    uint8_t byte; /**< Value in byte.*/
    struct{
        uint8_t _Reserved:5;      /**< Do not use.  Must be set to ‘0*/
        uint8_t GN:3;           /**< Gain configuration.*/                               
    };   
};

/** MR_REG_M structure definition
 */ 
union MR_REG_M_t
{
    uint8_t byte; /**< Value in byte.*/
    struct{
        uint8_t MD:2;           /**< Mode select bits. These bits select the operation mode of this device.*/    
        uint8_t _Reserved:6;    /**< Do not use.  Must be set to 0*/
                                   
    };   
};
/** @} */ // end of regM

#endif //__LSM303DLH_REGISTERDEF_H_