#include "Waveshare_10Dof-D.h"


int main(int argc, char* argv[])
{
	IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
	IMU_ST_ANGLES_DATA stAngles;
	IMU_ST_SENSOR_DATA stGyroRawData;
	IMU_ST_SENSOR_DATA stAccelRawData;
	IMU_ST_SENSOR_DATA stMagnRawData;
	int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

	imuInit(&enMotionSensorType, &enPressureType);
	if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
	{
		printf("Motion sersor is ICM-20948\n" );
	}
	else
	{
		printf("Motion sersor NULL\n");
	}
	if(IMU_EN_SENSOR_TYPE_BMP280 == enPressureType)
	{
		printf("Pressure sersor is BMP280\n");
	}
	else
	{
		printf("Pressure sersor NULL\n");
	}

	while(1)
	{
		imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
		pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);
		printf("\r\n /-------------------------------------------------------------/ \r\n");
		printf("\r\n Roll: %.2f     Pitch: %.2f     Yaw: %.2f \r\n",stAngles.fRoll, stAngles.fPitch, stAngles.fYaw);
		printf("\r\n Acceleration: X: %d     Y: %d     Z: %d \r\n",stAccelRawData.s16X, stAccelRawData.s16Y, stAccelRawData.s16Z);
		printf("\r\n Gyroscope: X: %d     Y: %d     Z: %d \r\n",stGyroRawData.s16X, stGyroRawData.s16Y, stGyroRawData.s16Z);
		printf("\r\n Magnetic: X: %d     Y: %d     Z: %d \r\n",stMagnRawData.s16X, stMagnRawData.s16Y, stMagnRawData.s16Z);
		printf("\r\n Pressure: %.2f     Altitude: %.2f \r\n",(float)s32PressureVal/100, (float)s32AltitudeVal/100);
		printf("\r\n Temperature: %.1f \r\n", (float)s32TemperatureVal/100);
		delay(100);
	}
	return 0;
}
