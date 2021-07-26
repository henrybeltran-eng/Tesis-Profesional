#ifndef __MPU_9250_H__
#define __MPU_9250_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>

#define MPU9250_ADDR 			0xD0
#define SMPLRT_DIV_REG 			0x19
#define GYRO_CONFIG_REG 		0x1B
#define ACCEL_CONFIG_REG 		0x1C
#define ACCEL_XOUT_H_REG 		0x3B
#define TEMP_OUT_H_REG 			0x41
#define GYRO_XOUT_H_REG 		0x43
#define PWR_MGMT_1_REG 			0x6B
#define WHO_AM_I_REG 			0x75
#define ACCEL_CONFIG_2 			0X1D
#define MPU9250_CONFIG			0X1A
#define MPU9250_BYPASSCONFG 	0X37
#define MPU9250_USERCONTROL 	0x6A
#define M_PI 					3.14159265358979323846

#define AK8963_ADDRESS   		0x18
#define WHO_AM_I_AK8963  		0x00
#define AK8963_XOUT_L	 		0x03
#define AK8963_CNTL     	 	0x0A // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_FUSE_ROM			0xF  // Fuse mode rom and 16 bit output
#define AK8963_Mscale			0x10
#define AK8963_measu_mode_2 	0x6
#define AK8963_ASAX 			0x10
#define AK8963_Status			0x2
#define AK8963_READY_MASK 		0x01

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
void MPU9250_Init(void);
void initAK8963(void);
void MPU9250_Read_Accel (void);
void MPU9250_Read_Gyro (void);
void MPU9250_Read_Temp (void);
void MPU9250_Read_Mag (void);
void MPU9250_DistanceAngle(void);
void MPU9250_LectureWithFilter (void);
void MPU9250_CalibracionAG(void);
void MPU9250_CalibracionMag(void);
void zeros (void);


int16_t Accel_X_Raw, Accel_Y_Raw, Accel_Z_Raw, Gyro_X_Raw, Gyro_Y_Raw, Gyro_Z_Raw, Temp_Raw;
int16_t Mag_X_Raw, Mag_Y_Raw, Mag_Z_Raw;
uint8_t sens_Mg_X, sens_Mg_Y, sens_Mg_Z;
float Ax, Ay, Az, Gx, Gy, Gz,Temp, Mgx, Mgy, Mgz;
float Ax_min, Ay_min, Az_min, Gx_min, Gy_min, Gz_min;
float Ax_max, Ay_max, Az_max, Gx_max, Gy_max, Gz_max;
float Ax_offset, Ay_offset, Az_offset, Gx_offset, Gy_offset, Gz_offset;
float RollPrev, PitchPrev, YawPrev, Roll, Pitch, Yaw;
float XPrev, YPrev, ZPrev, X, Y, Z;
float DataFilter[9][11], MPU9250Filtered[9], MPU9250HistoryData[9][11];
int VarMag;

char Display_Buff[50];

#ifdef __cplusplus
}
#endif

#endif /*__MPU_9250_H__*/
