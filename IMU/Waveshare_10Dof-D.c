#include "Waveshare_10Dof-D.h"

IMU_ST_SENSOR_DATA gstGyroOffset ={0,0,0};

#ifdef __cplusplus
extern "C" {
#endif

void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float invSqrt(float x);

void icm20948init(void);
bool icm20948Check(void);
void icm20948GyroRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z);
void icm20948AccelRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z);
void icm20948MagRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z);
bool icm20948MagCheck(void);
void icm20948CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal);
void icm20948GyroOffset(void);
void icm20948ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data);
void icm20948WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data);

bool bmp280Check(void);
void bmp280Init(void);

#define IIC_Dev  "/dev/i2c-1"

int fd;

void i2cInit(void)
{
  if ((fd = open(IIC_Dev, O_RDWR)) < 0)
  {
    printf("Failed to open the i2c bus.\n");
  }
  else
  {
    printf("I2C bus opened successfully.\n");
  }
  return;
}

uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr)
{
  uint8_t u8Ret;
  if (ioctl(fd, I2C_SLAVE, DevAddr) < 0)
  {
    printf("Failed to acquire bus access and/or talk to slave. DevAddr: 0x%02X\n", DevAddr);
    return 0;
  }
  write(fd, &RegAddr, 1);
  read(fd, &u8Ret, 1);
  return u8Ret;
}

void I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t value)
{
  int8_t *buf;

  if (ioctl(fd, I2C_SLAVE, DevAddr) < 0)
  {
    printf("Failed to acquire bus access and/or talk to slave. DevAddr: 0x%02X\n", DevAddr);
    return;
  }
  buf = malloc(2);
  buf[0] = RegAddr;
  buf[1] = value;
  write(fd, buf, 2);
  free(buf);
  return;
}

#define Kp 4.50f
#define Ki 1.0f

float angles[3];
float q0, q1, q2, q3; 

void imuInit(IMU_EN_SENSOR_TYPE *penMotionSensorType, IMU_EN_SENSOR_TYPE *penPressureType)
{
  bool bRet = false;
  
  i2cInit();
  bRet = icm20948Check();
  if (bRet == true)
  {
    printf("ICM20948 detected.\n");
    *penMotionSensorType = IMU_EN_SENSOR_TYPE_ICM20948;
    icm20948init();
  }
  else
  {
    printf("ICM20948 not detected. Address: 0x%02X\n", I2C_ADD_ICM20948);
    *penMotionSensorType = IMU_EN_SENSOR_TYPE_NULL;
  }
  
  bRet = bmp280Check();
  if (bRet == true)
  {
    printf("BMP280 detected.\n");
    *penPressureType = IMU_EN_SENSOR_TYPE_BMP280;
    bmp280Init();
  }
  else
  {
    printf("BMP280 not detected.\n");
    *penPressureType = IMU_EN_SENSOR_TYPE_NULL;
  }

  q0 = 1.0f;  
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;

  return;
}

bool icm20948Check(void)
{
  bool bRet = false;
  uint8_t who_am_i = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_WIA);
  printf("ICM20948 WHO_AM_I register: 0x%02X\n", who_am_i);
  if (who_am_i == REG_VAL_WIA)
  {
    bRet = true;
  }
  return bRet;
}

void icm20948init(void)
{
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET);
  delay(10);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1, REG_VAL_RUN_MODE);

  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_SMPLRT_DIV, 0x07);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_CONFIG_1, REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_1000DPS | REG_VAL_BIT_GYRO_DLPF);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_SMPLRT_DIV_2, 0x07);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG, REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g);

  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
  delay(100);
  icm20948GyroOffset();
  icm20948MagCheck();
  icm20948WriteSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_WRITE, REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_20HZ);
  return;
}

void icm20948GyroRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
  uint8_t u8Buf[6];
  int16_t s16Buf[3] = {0}; 
  uint8_t i;
  int32_t s32OutBuf[3] = {0};
  static ICM20948_ST_AVG_DATA sstAvgBuf[3];
  static int16_t ss16c = 0;
  ss16c++;

  u8Buf[0] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_XOUT_L); 
  u8Buf[1] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_XOUT_H);
  s16Buf[0] = (u8Buf[1] << 8) | u8Buf[0];

  u8Buf[0] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_YOUT_L); 
  u8Buf[1] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_YOUT_H);
  s16Buf[1] = (u8Buf[1] << 8) | u8Buf[0];

  u8Buf[0] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_ZOUT_L); 
  u8Buf[1] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_ZOUT_H);
  s16Buf[2] = (u8Buf[1] << 8) | u8Buf[0];

  for (i = 0; i < 3; i++) 
  {
    icm20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
  }
  *ps16X = s32OutBuf[0] - gstGyroOffset.s16X;
  *ps16Y = s32OutBuf[1] - gstGyroOffset.s16Y;
  *ps16Z = s32OutBuf[2] - gstGyroOffset.s16Z;

  return;
}

void icm20948AccelRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
  uint8_t u8Buf[2];
  int16_t s16Buf[3] = {0}; 
  uint8_t i;
  int32_t s32OutBuf[3] = {0};
  static ICM20948_ST_AVG_DATA sstAvgBuf[3];

  u8Buf[0] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_XOUT_L); 
  u8Buf[1] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_XOUT_H);
  s16Buf[0] = (u8Buf[1] << 8) | u8Buf[0];

  u8Buf[0] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_YOUT_L); 
  u8Buf[1] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_YOUT_H);
  s16Buf[1] = (u8Buf[1] << 8) | u8Buf[0];

  u8Buf[0] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_ZOUT_L); 
  u8Buf[1] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_ZOUT_H);
  s16Buf[2] = (u8Buf[1] << 8) | u8Buf[0];

  for (i = 0; i < 3; i++) 
  {
    icm20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
  }
  *ps16X = s32OutBuf[0];
  *ps16Y = s32OutBuf[1];
  *ps16Z = s32OutBuf[2];

  return;
}

void icm20948MagRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
  uint8_t counter = 20;
  uint8_t u8Data[MAG_DATA_LEN];
  int16_t s16Buf[3] = {0}; 
  uint8_t i;
  int32_t s32OutBuf[3] = {0};
  static ICM20948_ST_AVG_DATA sstAvgBuf[3];
  while (counter > 0)
  {
    delay(10);
    icm20948ReadSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ, REG_ADD_MAG_ST2, 1, u8Data);
    
    if ((u8Data[0] & 0x01) != 0)
      break;
    
    counter--;
  }
  
  if (counter != 0)
  {
    icm20948ReadSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ, REG_ADD_MAG_DATA, MAG_DATA_LEN, u8Data);
    s16Buf[0] = ((int16_t)u8Data[1] << 8) | u8Data[0];
    s16Buf[1] = ((int16_t)u8Data[3] << 8) | u8Data[2];
    s16Buf[2] = ((int16_t)u8Data[5] << 8) | u8Data[4];
  }

  for (i = 0; i < 3; i++) 
  {
    icm20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
  }
  
  *ps16X = s32OutBuf[0];
  *ps16Y = -s32OutBuf[1];
  *ps16Z = -s32OutBuf[2];
  return;
}

void icm20948ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data)
{
  uint8_t i;
  uint8_t u8Temp;

  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_ADDR, u8I2CAddr);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_REG, u8RegAddr);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | u8Len);

  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
  
  u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL);
  u8Temp |= REG_VAL_BIT_I2C_MST_EN;
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
  delay(5);
  u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
  
  for (i = 0; i < u8Len; i++)
  {
    *(pu8data + i) = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_EXT_SENS_DATA_00 + i);
  }
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3);
  
  u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL);
  u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN) & (REG_VAL_BIT_MASK_LEN));
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, u8Temp);
  
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
}

void icm20948WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data)
{
  uint8_t u8Temp;
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_ADDR, u8I2CAddr);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_REG, u8RegAddr);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_DO, u8data);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN | 1);

  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);

  u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL);
  u8Temp |= REG_VAL_BIT_I2C_MST_EN;
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
  delay(5);
  u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);

  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3);

  u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL);
  u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN) & (REG_VAL_BIT_MASK_LEN));
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, u8Temp);

  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
  return;
}

void icm20948CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{ 
  uint8_t i;
  
  *(pAvgBuffer + ((*pIndex)++)) = InVal;
  *pIndex &= 0x07;
  
  *pOutVal = 0;
  for (i = 0; i < 8; i++) 
  {
    *pOutVal += *(pAvgBuffer + i);
  }
  *pOutVal >>= 3;
}

void icm20948GyroOffset(void)
{
  uint8_t i;
  int16_t s16Gx = 0, s16Gy = 0, s16Gz = 0;
  int32_t s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;
  for (i = 0; i < 32; i++)
  {
    icm20948GyroRead(&s16Gx, &s16Gy, &s16Gz);
    s32TempGx += s16Gx;
    s32TempGy += s16Gy;
    s32TempGz += s16Gz;
    delay(10);
  }
  gstGyroOffset.s16X = s32TempGx >> 5;
  gstGyroOffset.s16Y = s32TempGy >> 5;
  gstGyroOffset.s16Z = s32TempGz >> 5;
  return;
}

bool icm20948MagCheck(void)
{
  bool bRet = false;
  uint8_t u8Ret[2];
  
  icm20948ReadSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ, REG_ADD_MAG_WIA1, 2, u8Ret);
  if ((u8Ret[0] == REG_VAL_MAG_WIA1) && (u8Ret[1] == REG_VAL_MAG_WIA2))
  {
    bRet = true;
  }
  
  return bRet;
}

bool bmp280Check(void)
{
    bool bRet = false;
    if (0x58 == I2C_ReadOneByte(BMP280_ADDR, BMP280_REGISTER_CHIPID))
    {
        bRet = true;
    }
    return bRet;
}

void bmp280Init(void)
{
  I2C_WriteOneByte(BMP280_ADDR, BMP280_REGISTER_CONTROL, 0xFF);
  I2C_WriteOneByte(BMP280_ADDR, BMP280_REGISTER_CONFIG, 0x14);
  bmp280ReadCalibration();
}

void bmp280ReadCalibration(void)
{
  uint8_t lsb, msb; 
  
  /* read the temperature calibration parameters */  
  lsb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_T1_LSB_REG);
  msb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_T1_MSB_REG);
  bmp280.T1 = msb << 8 | lsb;
  lsb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_T2_LSB_REG);
  msb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_T2_MSB_REG);
  bmp280.T2 = msb << 8 | lsb;  
  lsb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_T3_LSB_REG);
  msb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_T3_MSB_REG);
  bmp280.T3 = msb << 8 | lsb;  
  
  /* read the pressure calibration parameters */  
  lsb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P1_LSB_REG);
  msb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P1_MSB_REG);    
  bmp280.P1 = msb << 8 | lsb;  
  lsb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P2_LSB_REG);
  msb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P2_MSB_REG);      
  bmp280.P2 = msb << 8 | lsb;  
  lsb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P3_LSB_REG);
  msb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P3_MSB_REG);  
  bmp280.P3 = msb << 8 | lsb;  
  lsb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P4_LSB_REG);
  msb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P4_MSB_REG);         
  bmp280.P4 = msb << 8 | lsb;    
  lsb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P5_LSB_REG);
  msb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P5_MSB_REG);           
  bmp280.P5 = msb << 8 | lsb;  
  lsb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P6_LSB_REG);
  msb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P6_MSB_REG);          
  bmp280.P6 = msb << 8 | lsb;  
  lsb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P7_LSB_REG);
  msb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P7_MSB_REG);           
  bmp280.P7 = msb << 8 | lsb;  
  lsb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P8_LSB_REG);
  msb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P8_MSB_REG);         
  bmp280.P8 = msb << 8 | lsb;  
  lsb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P9_LSB_REG);
  msb = I2C_ReadOneByte(BMP280_ADDR, BMP280_DIG_P9_MSB_REG);            
  bmp280.P9 = msb << 8 | lsb; 
}

#ifdef __cplusplus
}
#endif
