#include "icm20948_driver.h"
#include "arm_math.h"
#include "spi.h"
#include "main.h"

uint8_t ID;
uint8_t MagID;


#define AUX_READ_TIMEOUT 256
#define ICM20948_INIT_REG_LENS 14


//--------------------------------------------
//数据读取
//--------------------------------------------
void  ICM94_WriteReg(uint8_t writeAddr, uint8_t writeData)
{
	//return myiic_write_reg(ICM20602_ADDRESS,reg,val);
	CS_L();
	spi1_write_reg(writeAddr,writeData);
	CS_H();
}
//bank change
void ICM94_SwitchUserBank(uint8_t bank)
{
  ICM94_WriteReg(ICM20948_REG_BANK_SEL,(bank& 0xCF) << 4);
}

uint8_t ICM94_ReadReg(uint8_t readAddr)
{
	uint8_t res;
	//return myiic_read_reg(ICM20602_ADDRESS,reg);
	CS_L();
	res = spi1_read_reg(readAddr);
	CS_H();
	return res;
}

void  ICM94_ReadRegs( uint8_t readAddr, uint8_t *readData, uint8_t lens )
{
	//return myiic_read_buffer(ICM20602_ADDRESS,reg,len,buffer);
	CS_L();
	spi1_read_reg_buffer(readAddr,readData,lens);
	CS_H();

}
/**
  * @brief  ICM94_AUX_WriteRegs
  */
void ICM94_AUX_WriteReg( uint8_t slaveAddr, uint8_t writeAddr, uint8_t writeData )
{
  uint8_t  status;
  uint32_t timeout = AUX_READ_TIMEOUT;

  ICM94_SwitchUserBank(3);
  ICM94_WriteReg(ICM20948_I2C_SLV4_ADDR, slaveAddr >> 1);
  delay_ms(2);
  ICM94_WriteReg(ICM20948_I2C_SLV4_REG, writeAddr);
  delay_ms(2);
  ICM94_WriteReg(ICM20948_I2C_SLV4_DO, writeData);
  delay_ms(2);
  ICM94_WriteReg(ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVx_EN);
  delay_ms(2);
  ICM94_SwitchUserBank(0);

  do {
    status = ICM94_ReadReg(ICM20948_I2C_MST_STATUS);
    delay_ms(1);
  } while (((status & ICM20948_I2C_SLV4_DONE) == 0) && (timeout--));
}
void ICM94_AUX_WriteRegs( uint8_t slaveAddr, uint8_t writeAddr, uint8_t *writeData, uint8_t lens )
{
  uint8_t  status;
  uint32_t timeout = AUX_READ_TIMEOUT;

  ICM94_SwitchUserBank(3);
  ICM94_WriteReg(ICM20948_I2C_SLV4_ADDR, slaveAddr >> 1);
  delay_ms(2);
  for (uint8_t i = 0; i < lens; i++) {
    ICM94_SwitchUserBank(3);
    ICM94_WriteReg(ICM20948_I2C_SLV4_REG, writeAddr + i);
    delay_ms(2);
    ICM94_WriteReg(ICM20948_I2C_SLV4_DO, writeData[i]);
    delay_ms(2);
    ICM94_WriteReg(ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVx_EN);
    delay_ms(2);
    ICM94_SwitchUserBank(0);
    do {
      status = ICM94_ReadReg(ICM20948_I2C_MST_STATUS);
      delay_ms(2);
    } while (((status & ICM20948_I2C_SLV4_DONE) == 0) && (timeout--));
  }
}

/**
  * @brief  ICM94_AUX_ReadReg
  */
uint8_t ICM94_AUX_ReadReg( uint8_t slaveAddr, uint8_t readAddr )
{
  uint8_t status;
  uint8_t readData;
  uint32_t timeout = AUX_READ_TIMEOUT;

  ICM94_SwitchUserBank(3);
  ICM94_WriteReg(ICM20948_I2C_SLV4_ADDR, (slaveAddr >> 1) | 0x80);
  delay_ms(2);
  ICM94_WriteReg(ICM20948_I2C_SLV4_REG, readAddr);
  delay_ms(2);
  ICM94_WriteReg(ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVx_EN);
  delay_ms(2);
  ICM94_SwitchUserBank(0);

  do {
    status = ICM94_ReadReg(ICM20948_I2C_MST_STATUS);
    delay_ms(2);
  } while (((status & ICM20948_I2C_SLV4_DONE) == 0) && (timeout--));

  ICM94_SwitchUserBank(3);
  readData = ICM94_ReadReg(ICM20948_I2C_SLV4_DI);
  ICM94_SwitchUserBank(0);

  return readData;
}

/**
  * @brief  ICM94_AUX_ReadRegs
  */

void ICM94_AUX_ReadRegs( uint8_t slaveAddr, uint8_t readAddr, uint8_t *readData, uint8_t lens )
{
  uint8_t status;
  uint32_t timeout = AUX_READ_TIMEOUT;

  ICM94_SwitchUserBank(3);
  ICM94_WriteReg(ICM20948_I2C_SLV4_ADDR, (slaveAddr >> 1) | 0x80);
  delay_ms(1);
  for (uint8_t i = 0; i< lens; i++) {
    ICM94_SwitchUserBank(3);
    ICM94_WriteReg(ICM20948_I2C_SLV4_REG, readAddr + i);
    delay_ms(1);
    ICM94_WriteReg(ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVx_EN);
    delay_ms(1);
    ICM94_SwitchUserBank(0);
    do {
      status = ICM94_ReadReg(ICM20948_I2C_MST_STATUS);
      delay_ms(1);
    } 
		while (((status & ICM20948_I2C_SLV4_DONE) == 0) && (timeout--));

    ICM94_SwitchUserBank(3);
    readData[i] = ICM94_ReadReg(ICM20948_I2C_SLV4_DI);
    delay_ms(1);
    ICM94_SwitchUserBank(0);
  }
}

/**
  * @brief  ICM94_AUX_SLVx_Config
  */
void ICM94_AUX_SLVx_Config( uint8_t slv, uint8_t slaveAddr, uint8_t readAddr, uint8_t lens )
{
  uint8_t offset = slv << 2;

  ICM94_SwitchUserBank(3);
  ICM94_WriteReg(ICM20948_I2C_SLV0_ADDR + offset, (slaveAddr >> 1) | 0x80);
  delay_ms(1);
  ICM94_WriteReg(ICM20948_I2C_SLV0_REG + offset, readAddr);
  delay_ms(1);
  ICM94_WriteReg(ICM20948_I2C_SLV0_CTRL + offset, ICM20948_I2C_SLVx_EN | (lens & 0x0F));
  delay_ms(1);
  ICM94_SwitchUserBank(0);
}


//--------------------------------------------
//icm20948???
//--------------------------------------------
uint8_t ICM20948_init()
{
	uint8_t ICM20948_InitData[ICM20948_INIT_REG_LENS][2] = {
    {0x20, ICM20948_USER_CTRL},             /* [0]  USR0, Release AUX I2C               */
    {0x80, ICM20948_PWR_MGMT_1},            /* [1]  USR0, Reset Device                  */
    {0x01, ICM20948_PWR_MGMT_1},            /* [2]  USR0, Clock Source 自动选择最佳时钟源 */
    {0x30, ICM20948_USER_CTRL},             /* [3]  USR0, Set I2C_MST_EN, I2C_IF_DIS    */
    {0x10, ICM20948_INT_PIN_CFG},           /* [4]  USR0, Set INT_ANYRD_2CLEAR          */
    {0x01, ICM20948_INT_ENABLE},            /* [5]  USR0, Set RAW_RDY_EN Enable I2C master interrupt to propagate to interrupt pin 1.                */
    {0x00, ICM20948_PWR_MGMT_2},            /* [6]  USR0, Enable all Accel & Gyro            */

    {0x00, ICM20948_GYRO_SMPLRT_DIV},       /* [7]  USR2, Sample Rate Divider 1.1khz          */
    {0x00, ICM20948_GYRO_CONFIG_1},         /* [8]  USR2, default : +-250dps            */
    {0x00, ICM20948_ACCEL_CONFIG},          /* [9]  USR2, default : +-2G                */
    {0x00, ICM20948_ACCEL_CONFIG_2},        /* [10] USR2, default : AccLPS_460Hz low-power mode时使用 */
    {0x00, ICM20948_TEMP_CONFIG},           /* [11] USR2, DLPF                          */

    {0x07, ICM20948_I2C_MST_CTRL},          /* [12] USR3, Set INT_ANYRD_2CLEAR          */
    {0x80, ICM20948_I2C_MST_DELAY_CTRL},    /* [13] USR3, Delays Shadowing              */
  };
	
	
	ICM20948_InitData[8][0]  |= MPU_GYRO_FILTER | (MPU_GYRO_RANGLE) | MPU_GYRO_FCHOICE;       
	//  GYRO_DLPFCFG = 0,3DB BW 196.6hz  +-1000 dps; GYRO_FCHOICE = 1; 
	//0 196.6; 1 151.8; 2 119.5; 3 51.2; 4 23.9; 5 11.6; 6 5.7; 7 361.4 rate 1125/(1+GYRO_SMPLRT_DIV)Hz
  ICM20948_InitData[9][0]  |= MPU_ACCEL_FILTER | (MPU_ACCEL_RANGLE) | MPU_ACCEL_FCHOICE;       
	//  ACCEL_DLPFCFG = 0,3DB BW 246.0; +-8g; ACCEL_FCHOICE = 1;
	//0 246; 1 246; 2 111.4; 3 50.4; 4 23.9; 5 11.5; 6 5.7; 7 473 rate 1125/(1+ACCEL_SMPLRT_DIV)Hz
  ICM20948_InitData[11][0] |= (1 << 0);                             
	// TEMP_DLPCFG = 1

  ICM94_SwitchUserBank(0);
  ICM94_WriteReg(ICM20948_USER_CTRL, ICM94_ReadReg(ICM20948_USER_CTRL) & ~ICM20948_InitData[0][0]); // release aux i2c
  delay_ms(10);
  ICM94_WriteReg(ICM20948_InitData[1][1], ICM20948_InitData[1][0]);     // reset device
  delay_ms(10);
  ICM94_WriteReg(ICM20948_InitData[2][1], ICM20948_InitData[2][0]);     // set clock source
  delay_ms(1);
  ICM94_WriteReg(ICM20948_InitData[3][1], ICM20948_InitData[3][0]);     // set I2C_MST_EN, I2C_IF_DIS
  delay_ms(1);
  ICM94_WriteReg(ICM20948_InitData[4][1], ICM20948_InitData[4][0]);     // set INT_ANYRD_2CLEAR
  delay_ms(1);
  ICM94_WriteReg(ICM20948_InitData[5][1], ICM20948_InitData[5][0]);     // set RAW_RDY_EN
  delay_ms(1);
  ICM94_WriteReg(ICM20948_InitData[6][1], ICM20948_InitData[6][0]);     // enable accel & gyro
  delay_ms(1);
	
  ID = ICM94_ReadReg(ICM20948_WHO_AM_I);
	MagID=ICM94_AUX_ReadReg(AK09916_I2C_ADDR, AK09916_WIA);
	
  ICM94_SwitchUserBank(2);
  ICM94_WriteReg(ICM20948_InitData[7][1], ICM20948_InitData[7][0]);     // set gyro sample rate divider  1Khz
  delay_ms(1);
  ICM94_WriteReg(ICM20948_InitData[8][1], ICM20948_InitData[8][0]);     // set gyro full-scale range, filter
  delay_ms(1);
  ICM94_WriteReg(ICM20948_InitData[9][1], ICM20948_InitData[9][0]);     // set accel full-scale range, filter
  delay_ms(1);
  ICM94_WriteReg(ICM20948_InitData[10][1], ICM20948_InitData[10][0]);   // set samples average
  delay_ms(1);
  ICM94_WriteReg(ICM20948_InitData[11][1], ICM20948_InitData[11][0]);   // set INT_ANYRD_2CLEAR
  delay_ms(1);

  ICM94_SwitchUserBank(3);
  ICM94_WriteReg(ICM20948_InitData[12][1], ICM20948_InitData[12][0]);   // set temp filter
  delay_ms(1);
  ICM94_WriteReg(ICM20948_InitData[13][1], ICM20948_InitData[13][0]);   // delays shadowing
  delay_ms(1);
	
	ICM94_AUX_WriteReg(AK09916_I2C_ADDR, AK09916_CNTL3, 0x01);
  delay_ms(10);
  ICM94_AUX_WriteReg(AK09916_I2C_ADDR, AK09916_CNTL2,0x00);
  delay_ms(1);
  ICM94_AUX_WriteReg(AK09916_I2C_ADDR, AK09916_CNTL2,0x08);
  delay_ms(1);
	ICM94_AUX_SLVx_Config(0, AK09916_I2C_ADDR, AK09916_ST1, 9);
	delay_ms(10);
	ICM94_SwitchUserBank(0);//bank 0
//  SPI1_SetSpeed(SPI_BaudRatePrescaler_4);		//
	return 0;
}


