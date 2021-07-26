#include "main.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "ST7735.h"
#include "GFX_FUNCTIONS.h"
#include "MPU_9250.h"

float Ax_min = 32768, Ay_min = 32768, Az_min = 32768, Gx_min = 32768, Gy_min = 32768, Gz_min = 32768;
float Ax_max = -32768, Ay_max = -32768, Az_max = -32768, Gx_max = -32768, Gy_max = -32768, Gz_max = -32768;
float Ax_offset = 0, Ay_offset = 0, Az_offset = 0, Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;
float DataFilter[9][11] = {0.040102,	0.136742,	-0.058701,	-0.250898,	0.027643,	0.299618,	0.027643,	-0.250898,	-0.058701,	0.136742,	0.040102,
		0.022800,	0.136430,	-0.039376,	-0.263937,	0.019273,	0.318292,	0.019273,	-0.263937,	-0.039376,	0.136430,	0.022800,
		0.009095,	0.131460,	-0.019619,	-0.275009,	0.010047,	0.336816,	0.010047,	-0.275009,	-0.019619,	0.131460,	0.009095,
		0.116778,	0.078793,	-0.119725,	-0.167477,	0.051069,	0.207181,	0.051069,	-0.167477,	-0.119725,	0.078793,	0.116778,
		0.028273,	-0.020501,	-0.048846,	-0.026479,	0.023817,	0.049578,	0.023817,	-0.026479,	-0.048846,	-0.020501,	0.028273,
		0.018045,	0.132587,	-0.030312,	-0.268317,	0.014768,	0.326753,	0.014768,	-0.268317,	-0.030312,	0.132587,	0.018045,
		0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,
		0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,
		0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000};

char buff[100];
extern float Ax_offset, Ay_offset, Az_offset, Gx_offset, Gy_offset, Gz_offset;

void MPU9250_Init(void){
	uint8_t check, data;
	HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,WHO_AM_I_REG,1,&check,1,1000);  //Check the conection with MPU9250
	if (check == 0x71){
		//Print the true connection
		sprintf(buff,"Hello, My name is MPU9250, I'm happy to meet you\r\n"); 
		HAL_UART_Transmit(&huart1,buff,strlen(buff),1000);
		//Select de clock Source
		data = 0x0;
		HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,PWR_MGMT_1_REG,1,&data,1,1000);
		//select the sample rate time
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,SMPLRT_DIV_REG,1,&data,1,1000);
		//Configure the accel scale -+ 2g
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,ACCEL_CONFIG_REG,1,&data,1,1000);
		//Configure the Gyro Scale 250�/s
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,GYRO_CONFIG_REG,1,&data,1,1000);
		//disable I2C master interface.Precondition to enable bypass multiplexer of the I2C master interface
		data = 0x0;
		HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,MPU9250_USERCONTROL,1,&data,1,1000);
		//enable I2C master interface bypass multiplexer
		data = 0x2;
		HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDR,MPU9250_BYPASSCONFG,1,&data,1,1000);
	}
}
void initAK8963(void){
	uint8_t check, data;
	//chech te correct conection with AK8963 (Magnetometer)
		HAL_I2C_Mem_Read(&hi2c1,AK8963_ADDRESS,0x0,1,&check,1,1000);
		if (check == 0x48){
			sprintf(buff,"Hello, My name is AK8963, I'm happy to meet you\r\n");
			HAL_UART_Transmit(&huart1,buff,strlen(buff),1000);
		}
	//setup the magnetometer:Fuse ROM access mode and 16 bit output
	data = 0x1F;
	HAL_I2C_Mem_Write(&hi2c1,AK8963_ADDRESS,AK8963_CNTL,1,&data,1,1000);
	
	uint8_t Rec_Data[3];
	//read the sensitivity adjustment values
	HAL_I2C_Mem_Read(&hi2c1,AK8963_ADDRESS,AK8963_ASAX,1,Rec_Data,3,1000);
	
	sens_Mg_X = (Rec_Data[0]-128)*0.5/128+1;
	sens_Mg_Y = (Rec_Data[0]-128)*0.5/128+1;
	sens_Mg_Z = (Rec_Data[0]-128)*0.5/128+1;
	//reset the magnetometer to power down mode
	data = 0x0;
	HAL_I2C_Mem_Write(&hi2c1,AK8963_ADDRESS,AK8963_CNTL,1,&data,1,1000);
	HAL_Delay(1000);

	data = 0x16;
	HAL_I2C_Mem_Write(&hi2c1,AK8963_ADDRESS,AK8963_CNTL,1,&data,1,1000);
}
void MPU9250_Read_Accel (){
	uint8_t Rec_Data[6];
	
	HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,ACCEL_XOUT_H_REG,1,Rec_Data,6,1000);
	
	Accel_X_Raw = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]);
	Accel_Y_Raw = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]);
	Accel_Z_Raw = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]);
	
	Ax = (Accel_X_Raw-Ax_offset)*9.806/16384.0;
	Ay = (Accel_Y_Raw-Ay_offset)*9.806/16384.0;
	Az = (Accel_Z_Raw-Az_offset)*9.806/16384.0;
}
void MPU9250_Read_Gyro (void){
	uint8_t Rec_Data[6];
	
	HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,GYRO_XOUT_H_REG,1,Rec_Data,6,1000);
	
	Gyro_X_Raw = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]);
	Gyro_Y_Raw = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]);
	Gyro_Z_Raw = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]);
	
	Gx = (Gyro_X_Raw-Gx_offset)/131.0;
	Gy = (Gyro_Y_Raw-Gy_offset)/131.0;
	Gz = (Gyro_Z_Raw-Gz_offset)/131.0;
}
void MPU9250_Read_Temp (void){
	uint8_t Data[2];
	HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,TEMP_OUT_H_REG,1,Data,2,1000);
	
	Temp_Raw = (int16_t)(Data[0]<<8 | Data[1]);
	Temp = ((float)Temp_Raw)/333.87+21.0;
}
void MPU9250_Read_Mag (void){
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&hi2c1,AK8963_ADDRESS,AK8963_XOUT_L,1,Rec_Data,7,1000);
	Mag_X_Raw = (int16_t)(Rec_Data[0] | Rec_Data[1]<<8);
	Mag_Y_Raw = (int16_t)(Rec_Data[2] | Rec_Data[3]<<8);
	Mag_Z_Raw = (int16_t)(Rec_Data[4] | Rec_Data[5]<<8);

	Mgx = (float)Mag_X_Raw*(4800.0)/32768.0;
	Mgy = (float)Mag_Y_Raw*(4800.0)/32768.0;
	Mgz = -(float)Mag_Z_Raw*(4800.0)/32768.0;
}

void MPU9250_DistanceAngle(void){

}
void MPU9250_LectureWithFilter (void){
	MPU9250_Read_Gyro();
	MPU9250_Read_Accel();
	MPU9250_Read_Mag();

	float Var[9]= {Gx, Gy, Gz, Ax, Ay, Az-9.806, Mgx, Mgy, Mgz};
	float NewVar[9][11];
	float sensitivy[9] = {1/131,1/131,1/131,9.806/16384.0,9.806/16384.0,9.806/16384.0,0,0,0};

	for(int i=0; i<9; i++){
		MPU9250Filtered[i] = 0;
	}
	for(int i=0; i<9; i++){
		MPU9250HistoryData[i][0] = Var[i];
	}
	for(int i=0; i<9; i++){
		for(int j=0; j<11; j++){
			NewVar[i][j] = MPU9250HistoryData[i][j]*DataFilter[i][j];
		}
	}
	for(int i=0; i<9; i++){
		for(int j=0; j<11; j++){
			MPU9250Filtered[i] = MPU9250Filtered[i]+NewVar[i][j];
		}
	}
	for(int i=0; i<9; i++){
		for(int j=9; j>0; j--){
			MPU9250HistoryData[i][j+1] = MPU9250HistoryData[i][j];
		}
	}
	MPU9250Filtered[5] += 9.806;
//	for(int i=0; i<9; i++){
//		MPU9250Filtered[i] = MPU9250Filtered[i]*sensitivy[i];
//	}
}

void MPU9250_CalibracionAG(void){
	float Porcentaje;
	for(int i=0;i<50;i++){
		uint8_t Rec_Data[6];

		HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,ACCEL_XOUT_H_REG,1,Rec_Data,6,1000);

		Accel_X_Raw = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]);
		Accel_Y_Raw = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]);
		Accel_Z_Raw = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]);

		Ax = Accel_X_Raw/16384.0;
		Ay = Accel_Y_Raw/16384.0;
		Az = Accel_Z_Raw/16384.0;

		HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,GYRO_XOUT_H_REG,1,Rec_Data,6,1000);

		Gyro_X_Raw = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]);
		Gyro_Y_Raw = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]);
		Gyro_Z_Raw = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]);

		Gx = (Gyro_X_Raw/131.0);
		Gy = (Gyro_Y_Raw/131.0);
		Gz = (Gyro_Z_Raw/131.0);
	}
	sprintf(Display_Buff,"Toma de datos");
	ST7735_WriteString(20,30,Display_Buff, Font_7x10, WHITE,BLACK);
	for(int i=0;i<200;i++){
		uint8_t Rec_Data[6];

		HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,ACCEL_XOUT_H_REG,1,Rec_Data,6,1000);

		Accel_X_Raw = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]);
		Accel_Y_Raw = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]);
		Accel_Z_Raw = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]);

		Ax = Accel_X_Raw;
		Ay = Accel_Y_Raw;
		Az = Accel_Z_Raw;

		HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,GYRO_XOUT_H_REG,1,Rec_Data,6,1000);

		Gyro_X_Raw = (int16_t)(Rec_Data[0]<<8 | Rec_Data[1]);
		Gyro_Y_Raw = (int16_t)(Rec_Data[2]<<8 | Rec_Data[3]);
		Gyro_Z_Raw = (int16_t)(Rec_Data[4]<<8 | Rec_Data[5]);

		Gx = (Gyro_X_Raw);
		Gy = (Gyro_Y_Raw);
		Gz = (Gyro_Z_Raw);

		if(Ax < Ax_min){
			Ax_min = Ax;
		}
		if(Ay < Ay_min){
			Ay_min = Ay;
		}
		if(Az < Az_min){
			Az_min = Az;
		}
		if(Gx < Gx_min){
			Gx_min = Gx;
		}
		if(Gy < Gy_min){
			Gy_min = Gy;
		}
		if(Gz < Gz_min){
			Gz_min = Gz;
		}
		if(Ax > Ax_max){
			Ax_max = Ax;
		}
		if(Ay > Ay_max){
			Ay_max = Ay;
		}
		if(Az > Az_max){
			Az_max = Az;
		}
		if(Gx > Gx_max){
			Gx_max = Gx;
		}
		if(Gy > Gy_max){
			Gy_max = Gy;
		}
		if(Gz > Gz_max){
			Gz_max = Gz;
		}
		Porcentaje = i*100/200;
		sprintf(Display_Buff,"%.0f",Porcentaje);
		ST7735_WriteString(55,50,Display_Buff, Font_11x18, WHITE,BLACK);
		drawRect(14,78,100,20,WHITE);
		fillRect(14,78,(int)Porcentaje,20,WHITE);
	}
	Ax_offset = (Ax_min+Ax_max)/2;
	Ay_offset = (Ay_min+Ay_max)/2;
	Az_offset = (Az_min+Az_max)/2-32768/2;
	Gx_offset = (Gx_min+Gx_max)/2;
	Gy_offset = (Gy_min+Gy_max)/2;
	Gz_offset = (Gz_min+Gz_max)/2;
}

void MPU9250_CalibracionMag(void){
	int Max = 0, Min = 0;
	if (VarMag == 0){
		sprintf(Display_Buff," Calibracion en");
		ST7735_WriteString(15 ,0 ,Display_Buff, Font_7x10, WHITE,BLACK);
		for (float i = 0; i<200; i++){
			sprintf(Display_Buff," X = %.0f",i);
			ST7735_WriteString(15 ,12 ,Display_Buff, Font_11x18, WHITE,BLACK);
			MPU9250_Read_Mag();
			if (Mgz > Max){
				Max = Mgz;
			}
			if (Mgy > Max){
				Max = Mgy;
			}
			ST7735_DrawPixel(Mgy+64, Mgz+78, RED);
		}
	}
	else if (VarMag == 1){
		sprintf(Display_Buff," Calibracion en");
		ST7735_WriteString(15 ,0 ,Display_Buff, Font_7x10, WHITE,BLACK);
		for (float i = 0; i<200; i++){
			sprintf(Display_Buff," Y = %.0f",i);
			ST7735_WriteString(15 ,12 ,Display_Buff, Font_11x18, WHITE,BLACK);
			MPU9250_Read_Mag();
			if (Mgz > Max){
				Max = Mgz;
			}
			if (Mgx > Max){
				Max = Mgx;
			}
			ST7735_DrawPixel(Mgx+64, Mgz+78, RED);
		}
	}
	else if (VarMag == 2){
		sprintf(Display_Buff," Calibracion en");
		ST7735_WriteString(15 ,0 ,Display_Buff, Font_7x10, WHITE,BLACK);
		for (float i = 0; i<200; i++){
			sprintf(Display_Buff," Z = %.0f",i);
			ST7735_WriteString(15 ,12 ,Display_Buff, Font_11x18, WHITE,BLACK);
			MPU9250_Read_Mag();
			if (Mgx > Max){
				Max = Mgx;
			}
			if (Mgy > Max){
				Max = Mgy;
			}
			ST7735_DrawPixel(Mgx+64, Mgy+78, RED);
		}
	}
}

void zeros (void){
	for (int i = 0; i<9; i++){
		for(int j = 0; j<11; j++){
			 MPU9250Filtered[i] = 0;
			 MPU9250HistoryData[i][j] = 0;
		}
	}
}
