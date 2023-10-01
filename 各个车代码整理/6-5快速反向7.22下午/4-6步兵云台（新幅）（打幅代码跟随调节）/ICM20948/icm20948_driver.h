
#ifndef __ICM20948_DRIVER_H_
#define __ICM20948_DRIVER_H_


#ifdef __cplusplus
 extern "C" {
#endif

#define __ICM94
#define __USE_GYROSCOPE
#define __USE_ACCELEROMETER
//#define __USE_MAGNETOMETER
#define __USE_ICTEMPERATURE

#if defined(__USE_MAGNETOMETER)
#define __USE_AK09916
//#define __USE_IST8308
#endif

/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4xx.h"

//---设置陀螺仪和加速度计的量程---//
#define ICM20948_GYRO_RANGE_2000
#define ICM20948_ACCEL_RANGE_8G
//----设置滤波陀螺仪和加速度计的参数-----//
#define ICM20948_GYRO_FILTER_119dB
#define ICM20948_ACCEL_FILTER_111dB



//------以下切勿更改------------//

# define MPU_GYRO_RANGLE_250                (0x0 << 1)
# define MPU_GYRO_RANGLE_500                (0x1 << 1)
# define MPU_GYRO_RANGLE_1000               (0x2 << 1)
# define MPU_GYRO_RANGLE_2000               (0x3 << 1)

# define MPU_ACCEL_RANGLE_2G                 (0x0 << 1)
# define MPU_ACCEL_RANGLE_4G                 (0x1 << 1)
# define MPU_ACCEL_RANGLE_8G                 (0x2 << 1)
# define MPU_ACCEL_RANGLE_16G                (0x3 << 1)

# define MPU_GYRO_FILTER_196dB                (0x0 << 3)
# define MPU_GYRO_FILTER_152dB                (0x1 << 3)
# define MPU_GYRO_FILTER_119dB               	(0x2 << 3)
# define MPU_GYRO_FILTER_51dB               	(0x3 << 3)
# define MPU_GYRO_FILTER_24dB               	(0x4 << 3)
# define MPU_GYRO_FILTER_12dB               	(0x5 << 3)
# define MPU_GYRO_FILTER_6dB               		(0x6 << 3)
# define MPU_GYRO_FILTER_361dB               	(0x7 << 3)

# define MPU_ACCEL_FILTER_246dB               (0x0 << 3)
# define MPU_ACCEL_FILTER_111dB               (0x2 << 3)
# define MPU_ACCEL_FILTER_50dB               	(0x3 << 3)
# define MPU_ACCEL_FILTER_24dB               	(0x4 << 3)
# define MPU_ACCEL_FILTER_12dB               	(0x5 << 3)
# define MPU_ACCEL_FILTER_6dB               	(0x6 << 3)
# define MPU_ACCEL_FILTER_473dB               (0x7 << 3)

#define ENABLE_GYRO_DLPF 		1
#define DISABLE_GYRO_DLPF 	0
#define ENABLE_ACCEL_DLPF 	1
#define DISABLE_ACCEL_DLPF 	0

#define MAG_SEN 0.15f //转换成 uT

#ifdef ICM20948_GYRO_RANGE_2000
#define MPU_GYRO_RANGLE MPU_GYRO_RANGLE_2000
#define GYRO_SEN 0.00106526443603169529841533860381f

#elif defined(ICM20948_GYRO_RANGE_1000)
#define MPU_GYRO_RANGLE MPU_GYRO_RANGLE_1000
#define GYRO_SEN 0.0005326322180158476492076f

#elif defined(ICM20948_GYRO_RANGE_500)
#define MPU_GYRO_RANGLE MPU_GYRO_RANGLE_500
#define GYRO_SEN 0.0002663161090079238246038f

#elif defined(ICM20948_GYRO_RANGE_250)
#define MPU_GYRO_RANGLE MPU_GYRO_RANGLE_250
#define GYRO_SEN 0.000133158054503961923019f

#else
#error "Please set the right range of gyro (2000 , 1000, 500 or 250)"
#endif



//  加速度计原始数据转换成m/s2 加速度计范围可以在h文件中修改
#ifdef ICM20948_ACCEL_RANGE_2G
#define MPU_ACCEL_RANGLE MPU_ACCEL_RANGLE_2G
#define ACCEL_SEN 0.00059814453125f

#elif defined(ICM20948_ACCEL_RANGE_4G)
#define MPU_ACCEL_RANGLE MPU_ACCEL_RANGLE_4G
#define ACCEL_SEN 0.0011962890625f

#elif defined(ICM20948_ACCEL_RANGE_8G)
#define MPU_ACCEL_RANGLE MPU_ACCEL_RANGLE_8G
#define ACCEL_SEN 0.002392578125f

#else
#error "Please set the right range of accel (16G , 8G, 4G or 2G)"
#define

#endif

#ifdef ICM20948_GYRO_FILTER_196dB
	#define MPU_GYRO_FCHOICE ENABLE_GYRO_DLPF
	#define MPU_GYRO_FILTER MPU_GYRO_FILTER_196dB
#elif defined ICM20948_GYRO_FILTER_152dB
	#define MPU_GYRO_FCHOICE ENABLE_GYRO_DLPF
	#define MPU_GYRO_FILTER MPU_GYRO_FILTER_152dB
#elif defined ICM20948_GYRO_FILTER_119dB
	#define MPU_GYRO_FCHOICE ENABLE_GYRO_DLPF
	#define MPU_GYRO_FILTER MPU_GYRO_FILTER_119dB
#elif defined ICM20948_GYRO_FILTER_51dB
	#define MPU_GYRO_FCHOICE ENABLE_GYRO_DLPF
	#define MPU_GYRO_FILTER MPU_GYRO_FILTER_51dB
#elif defined ICM20948_GYRO_FILTER_24dB
	#define MPU_GYRO_FCHOICE ENABLE_GYRO_DLPF
	#define MPU_GYRO_FILTER MPU_GYRO_FILTER_24dB
#elif defined ICM20948_GYRO_FILTER_12dB
	#define MPU_GYRO_FCHOICE ENABLE_GYRO_DLPF
	#define MPU_GYRO_FILTER MPU_GYRO_FILTER_12dB
#elif defined ICM20948_GYRO_FILTER_6dB
	#define MPU_GYRO_FCHOICE ENABLE_GYRO_DLPF
	#define MPU_GYRO_FILTER MPU_GYRO_FILTER_6dB
#elif defined ICM20948_GYRO_FILTER_361dB
	#define MPU_GYRO_FCHOICE DISABLE_GYRO_DLPF
	#define MPU_GYRO_FILTER MPU_GYRO_FILTER_361dB
#error "Please set the right filter of gyro"
#endif

#ifdef ICM20948_ACCEL_FILTER_246dB
	#define MPU_ACCEL_FCHOICE ENABLE_ACCEL_DLPF
	#define MPU_ACCEL_FILTER MPU_ACCEL_FILTER_246dB
#elif defined ICM20948_ACCEL_FILTER_111dB
	#define MPU_ACCEL_FCHOICE ENABLE_ACCEL_DLPF
	#define MPU_ACCEL_FILTER MPU_ACCEL_FILTER_111dB
#elif defined ICM20948_ACCEL_FILTER_50dB
	#define MPU_ACCEL_FCHOICE ENABLE_ACCEL_DLPF
	#define MPU_ACCEL_FILTER MPU_ACCEL_FILTER_50dB
#elif defined ICM20948_ACCEL_FILTER_24dB
	#define MPU_ACCEL_FCHOICE ENABLE_ACCEL_DLPF
	#define MPU_ACCEL_FILTER MPU_ACCEL_FILTER_24dB
#elif defined ICM20948_ACCEL_FILTER_12dB
	#define MPU_ACCEL_FCHOICE ENABLE_ACCEL_DLPF
	#define MPU_ACCEL_FILTER MPU_ACCEL_FILTER_12dB
#elif defined ICM20948_ACCEL_FILTER_6dB
	#define MPU_ACCEL_FCHOICE ENABLE_ACCEL_DLPF
	#define MPU_ACCEL_FILTER MPU_ACCEL_FILTER_6dB
#elif defined ICM20948_ACCEL_FILTER_473dB
	#define MPU_ACCEL_FCHOICE DISABLE_ACCEL_DLPF
	#define MPU_ACCEL_FILTER MPU_ACCEL_FILTER_473dB
#error "Please set the right filter of accel "
#endif


/* Exported types --------------------------------------------------------------------------*/
/* Exported constants ----------------------------------------------------------------------*/

/* ---- ICM20948 ---------------------------------------------- */

#define ICM20948_I2C_ADDR               ((uint8_t)0xD0)   /* AD0 = 0 */
//#define ICM20948_I2C_ADDR               ((uint8_t)0xD2)   /* AD0 = 1 */
#define ICM20948_DeviceID               ((uint8_t)0xEA)

/* USER BANK 0 REGISTER */
#define ICM20948_WHO_AM_I               ((uint8_t)0x00)
#define ICM20948_USER_CTRL              ((uint8_t)0x03)
#define ICM20948_LP_CONFIG              ((uint8_t)0x05)
#define ICM20948_PWR_MGMT_1             ((uint8_t)0x06)
#define ICM20948_PWR_MGMT_2             ((uint8_t)0x07)
#define ICM20948_INT_PIN_CFG            ((uint8_t)0x0F)
#define ICM20948_INT_ENABLE             ((uint8_t)0x10)
#define ICM20948_INT_ENABLE_1           ((uint8_t)0x11)
#define ICM20948_INT_ENABLE_2           ((uint8_t)0x12)
#define ICM20948_INT_ENABLE_3           ((uint8_t)0x13)
#define ICM20948_I2C_MST_STATUS         ((uint8_t)0x17)
#define ICM20948_INT_STATUS             ((uint8_t)0x19)
#define ICM20948_INT_STATUS_1           ((uint8_t)0x1A)
#define ICM20948_INT_STATUS_2           ((uint8_t)0x1B)
#define ICM20948_INT_STATUS_3           ((uint8_t)0x1C)
#define ICM20948_DELAY_TIMEH            ((uint8_t)0x28)
#define ICM20948_DELAY_TIMEL            ((uint8_t)0x29)
#define ICM20948_ACCEL_XOUT_H           ((uint8_t)0x2D)
#define ICM20948_ACCEL_XOUT_L           ((uint8_t)0x2E)
#define ICM20948_ACCEL_YOUT_H           ((uint8_t)0x2F)
#define ICM20948_ACCEL_YOUT_L           ((uint8_t)0x30)
#define ICM20948_ACCEL_ZOUT_H           ((uint8_t)0x31)
#define ICM20948_ACCEL_ZOUT_L           ((uint8_t)0x32)
#define ICM20948_GYRO_XOUT_H            ((uint8_t)0x33)
#define ICM20948_GYRO_XOUT_L            ((uint8_t)0x34)
#define ICM20948_GYRO_YOUT_H            ((uint8_t)0x35)
#define ICM20948_GYRO_YOUT_L            ((uint8_t)0x36)
#define ICM20948_GYRO_ZOUT_H            ((uint8_t)0x37)
#define ICM20948_GYRO_ZOUT_L            ((uint8_t)0x38)
#define ICM20948_TEMP_OUT_H             ((uint8_t)0x39)
#define ICM20948_TEMP_OUT_L             ((uint8_t)0x3A)
#define ICM20948_SLV_SENS_DATA_00       ((uint8_t)0x3B)
#define ICM20948_SLV_SENS_DATA_01       ((uint8_t)0x3C)
#define ICM20948_SLV_SENS_DATA_02       ((uint8_t)0x3D)
#define ICM20948_SLV_SENS_DATA_03       ((uint8_t)0x3E)
#define ICM20948_SLV_SENS_DATA_04       ((uint8_t)0x3F)
#define ICM20948_SLV_SENS_DATA_05       ((uint8_t)0x40)
#define ICM20948_SLV_SENS_DATA_06       ((uint8_t)0x41)
#define ICM20948_SLV_SENS_DATA_07       ((uint8_t)0x42)
#define ICM20948_SLV_SENS_DATA_08       ((uint8_t)0x43)
#define ICM20948_SLV_SENS_DATA_09       ((uint8_t)0x44)
#define ICM20948_SLV_SENS_DATA_10       ((uint8_t)0x45)
#define ICM20948_SLV_SENS_DATA_11       ((uint8_t)0x46)
#define ICM20948_SLV_SENS_DATA_12       ((uint8_t)0x47)
#define ICM20948_SLV_SENS_DATA_13       ((uint8_t)0x48)
#define ICM20948_SLV_SENS_DATA_14       ((uint8_t)0x49)
#define ICM20948_SLV_SENS_DATA_15       ((uint8_t)0x4A)
#define ICM20948_SLV_SENS_DATA_16       ((uint8_t)0x4B)
#define ICM20948_SLV_SENS_DATA_17       ((uint8_t)0x4C)
#define ICM20948_SLV_SENS_DATA_18       ((uint8_t)0x4D)
#define ICM20948_SLV_SENS_DATA_19       ((uint8_t)0x4E)
#define ICM20948_SLV_SENS_DATA_20       ((uint8_t)0x4F)
#define ICM20948_SLV_SENS_DATA_21       ((uint8_t)0x50)
#define ICM20948_SLV_SENS_DATA_22       ((uint8_t)0x51)
#define ICM20948_SLV_SENS_DATA_23       ((uint8_t)0x52)
#define ICM20948_FIFO_EN_1              ((uint8_t)0x66)
#define ICM20948_FIFO_EN_2              ((uint8_t)0x67)
#define ICM20948_FIFO_RST               ((uint8_t)0x68)
#define ICM20948_FIFO_MODE              ((uint8_t)0x69)
#define ICM20948_FIFO_COUNTH            ((uint8_t)0x70)
#define ICM20948_FIFO_COUNTL            ((uint8_t)0x71)
#define ICM20948_FIFO_R_W               ((uint8_t)0x72)
#define ICM20948_DATA_RDY_STATUS        ((uint8_t)0x74)
#define ICM20948_FIFO_CFG               ((uint8_t)0x76)
#define ICM20948_REG_BANK_SEL           ((uint8_t)0x7F)

/* USER BANK 1 REGISTER */
#define ICM20948_SELF_TEST_X_GYRO       ((uint8_t)0x02)
#define ICM20948_SELF_TEST_Y_GYRO       ((uint8_t)0x03)
#define ICM20948_SELF_TEST_Z_GYRO       ((uint8_t)0x04)
#define ICM20948_SELF_TEST_X_ACCEL      ((uint8_t)0x0E)
#define ICM20948_SELF_TEST_Y_ACCEL      ((uint8_t)0x0F)
#define ICM20948_SELF_TEST_Z_ACCEL      ((uint8_t)0x10)
#define ICM20948_XA_OFFS_H              ((uint8_t)0x14)
#define ICM20948_XA_OFFS_L              ((uint8_t)0x15)
#define ICM20948_YA_OFFS_H              ((uint8_t)0x17)
#define ICM20948_YA_OFFS_L              ((uint8_t)0x18)
#define ICM20948_ZA_OFFS_H              ((uint8_t)0x1A)
#define ICM20948_ZA_OFFS_L              ((uint8_t)0x1B)
#define ICM20948_TIMEBASE_CORRECTION    ((uint8_t)0x28)

/* USER BANK 2 REGISTER */
#define ICM20948_GYRO_SMPLRT_DIV        ((uint8_t)0x00)
#define ICM20948_GYRO_CONFIG_1          ((uint8_t)0x01)
#define ICM20948_GYRO_CONFIG_2          ((uint8_t)0x02)
#define ICM20948_XG_OFFS_USRH           ((uint8_t)0x03)
#define ICM20948_XG_OFFS_USRL           ((uint8_t)0x04)
#define ICM20948_YG_OFFS_USRH           ((uint8_t)0x05)
#define ICM20948_YG_OFFS_USRL           ((uint8_t)0x06)
#define ICM20948_ZG_OFFS_USRH           ((uint8_t)0x07)
#define ICM20948_ZG_OFFS_USRL           ((uint8_t)0x08)
#define ICM20948_ODR_ALIGN_EN           ((uint8_t)0x09)
#define ICM20948_ACCEL_SMPLRT_DIV_1     ((uint8_t)0x10)
#define ICM20948_ACCEL_SMPLRT_DIV_2     ((uint8_t)0x11)
#define ICM20948_ACCEL_INTEL_CTRL       ((uint8_t)0x12)
#define ICM20948_ACCEL_WOM_THR          ((uint8_t)0x13)
#define ICM20948_ACCEL_CONFIG           ((uint8_t)0x14)
#define ICM20948_ACCEL_CONFIG_2         ((uint8_t)0x15)
#define ICM20948_FSYNC_CONFIG           ((uint8_t)0x52)
#define ICM20948_TEMP_CONFIG            ((uint8_t)0x52)
#define ICM20948_MOD_CTRL_USR           ((uint8_t)0x54)

/* USER BANK 3 REGISTER */
#define ICM20948_I2C_MST_ODR_CONFIG     ((uint8_t)0x00)
#define ICM20948_I2C_MST_CTRL           ((uint8_t)0x01)
#define ICM20948_I2C_MST_DELAY_CTRL     ((uint8_t)0x02)
#define ICM20948_I2C_SLV0_ADDR          ((uint8_t)0x03)
#define ICM20948_I2C_SLV0_REG           ((uint8_t)0x04)
#define ICM20948_I2C_SLV0_CTRL          ((uint8_t)0x05)
#define ICM20948_I2C_SLV0_DO            ((uint8_t)0x06)
#define ICM20948_I2C_SLV1_ADDR          ((uint8_t)0x07)
#define ICM20948_I2C_SLV1_REG           ((uint8_t)0x08)
#define ICM20948_I2C_SLV1_CTRL          ((uint8_t)0x09)
#define ICM20948_I2C_SLV1_DO            ((uint8_t)0x0A)
#define ICM20948_I2C_SLV2_ADDR          ((uint8_t)0x0B)
#define ICM20948_I2C_SLV2_REG           ((uint8_t)0x0C)
#define ICM20948_I2C_SLV2_CTRL          ((uint8_t)0x0D)
#define ICM20948_I2C_SLV2_DO            ((uint8_t)0x0E)
#define ICM20948_I2C_SLV3_ADDR          ((uint8_t)0x0F)
#define ICM20948_I2C_SLV3_REG           ((uint8_t)0x10)
#define ICM20948_I2C_SLV3_CTRL          ((uint8_t)0x11)
#define ICM20948_I2C_SLV3_DO            ((uint8_t)0x12)
#define ICM20948_I2C_SLV4_ADDR          ((uint8_t)0x13)
#define ICM20948_I2C_SLV4_REG           ((uint8_t)0x14)
#define ICM20948_I2C_SLV4_CTRL          ((uint8_t)0x15)
#define ICM20948_I2C_SLV4_DO            ((uint8_t)0x16)
#define ICM20948_I2C_SLV4_DI            ((uint8_t)0x17)

#define ICM20948_I2C_SLVx_EN            ((uint8_t)0x80)
#define ICM20948_I2C_SLV4_DONE          ((uint8_t)0x40)
#define ICM20948_I2C_SLV4_NACK          ((uint8_t)0x10)


/* ---- AK09916 ----------------------------------------------- */

#define AK09916_I2C_ADDR                ((uint8_t)0x18)
#define AK09916_CompanyID               ((uint8_t)0x48)
#define AK09916_DeviceID                ((uint8_t)0x09)

#define AK09916_AKM                     ((uint8_t)0x00) // company id
#define AK09916_WIA                     ((uint8_t)0x01)
#define AK09916_RSV1                    ((uint8_t)0x02) // reserved register for AKM
#define AK09916_RSV2                    ((uint8_t)0x03) // reserved register for AKM
#define AK09916_ST1                     ((uint8_t)0x10)
#define AK09916_HXL                     ((uint8_t)0x11)
#define AK09916_HXH                     ((uint8_t)0x12)
#define AK09916_HYL                     ((uint8_t)0x13)
#define AK09916_HYH                     ((uint8_t)0x14)
#define AK09916_HZL                     ((uint8_t)0x15)
#define AK09916_HZH                     ((uint8_t)0x16)
#define AK09916_TMPS                    ((uint8_t)0x17) // dummy register
#define AK09916_ST2                     ((uint8_t)0x18)

#define AK09916_CNTL1                   ((uint8_t)0x30) // dummy register
#define AK09916_CNTL2                   ((uint8_t)0x31)
#define AK09916_CNTL3                   ((uint8_t)0x32)
#define AK09916_TS1                     ((uint8_t)0x33) // internal test register. Do not access these registers
#define AK09916_TS2                     ((uint8_t)0x34) // internal test register. Do not access these registers

#define AK09916_STATUS_DRDY             ((uint8_t)0x01)
#define AK09916_STATUS_DOR              ((uint8_t)0x02)
#define AK09916_STATUS_HOFL             ((uint8_t)0x08)


/* Exported functions ----------------------------------------------------------------------*/  
/* Exported functions ----------------------------------------------------------------------*/  



extern uint8_t ID;
extern uint8_t MagID;

uint8_t   ICM20948_init(void);
uint8_t   ICM94_ReadReg( uint8_t readAddr );
uint8_t  ICM94_AUX_ReadReg( uint8_t slaveAddr, uint8_t readAddr );



void ICM94_Config( void );
void ICM94_SetSPIFastMode( void );
void ICM94_WriteReg( uint8_t writeAddr, uint8_t writeData );
void ICM94_WriteRegs( uint8_t writeAddr, uint8_t *writeData, uint8_t lens );
void ICM94_ReadRegs( uint8_t readAddr, uint8_t *readData, uint8_t lens );
void ICM94_AUX_WriteReg( uint8_t slaveAddr, uint8_t writeAddr, uint8_t writeData );
void ICM94_AUX_WriteRegs( uint8_t slaveAddr, uint8_t writeAddr, uint8_t *writeData, uint8_t lens );
void ICM94_AUX_ReadRegs( uint8_t slaveAddr, uint8_t readAddr, uint8_t *readData, uint8_t lens );
void ICM94_SwitchUserBank( uint8_t bank );




#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
