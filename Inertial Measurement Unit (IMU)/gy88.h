/*
 * gy88.h
 *
 * Created: 7/6/2018 4:16:24 PM
 *  Author: Abheesh Khanal
 */ 


#ifndef GY88_H_
#define GY88_H_

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#include "hmccompass.h"
#include "MPU6050.h"
#include "General-function.h"

static MPU6050 mpu;
static float timeStep = 0.01;
static float unfiltered_roll, unfiltered_pitch, unfiltered_yaw;
static float filtered_roll, filtered_pitch, filtered_yaw;
unsigned long previousTime;
bool readFirstData = true;

Vector normAccel;

void initGY88(){
	mpu.Init();
	init_HMC5883L();
	initialise_timeperiod();
	previousTime = millis();
}

uint16_t compass_tilt_compensation(float roll_radians, float pitch_radians,float mag_y, float mag_x, float mag_z)
{

	float tilt_compensated_heading;
	float MAG_X;
	float MAG_Y;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;
	int16_t heading;
	

	cos_roll = cos(roll_radians);
	sin_roll = sin(roll_radians);
	cos_pitch = cos(pitch_radians);
	sin_pitch = sin(pitch_radians);


	#if 0
	MAG_X = mag_x*cos_pitch+mag_y*sin_roll*sin_pitch+mag_z*cos_roll*sin_pitch;
	MAG_Y = mag_y*cos_roll+mag_z*sin_roll;
	tilt_compensated_heading = atan2f(-MAG_Y,MAG_X);
	#else
	MAG_X = mag_x * cos_pitch - mag_z * sin_pitch;
	MAG_Y = mag_x * sin_roll * sin_pitch - mag_y * cos_roll + mag_z * sin_roll * cos_pitch;
	tilt_compensated_heading = atan2(-MAG_Y,MAG_X);
	//tilt_compensated_heading = atan2f(-mag_y,mag_x); // works fine when leveled
	#endif

	//convert to degree from 0-3599
	heading = tilt_compensated_heading * RAD_TO_DEG;
	if ( heading < 0 )
	{
		heading += 360;
	}

	return heading;
}

static float getAverageCompensatedYaw(){
	float average_unfiltered_yaw = 0;
	for (uint8_t i =0; i<10; i++)
	{
		average_unfiltered_yaw += compass_tilt_compensation(filtered_roll * DEG_TO_RAD , filtered_pitch * DEG_TO_RAD ,read_rawX(),read_rawY(),read_rawZ() );
	}
	average_unfiltered_yaw = average_unfiltered_yaw/10;
	return average_unfiltered_yaw;
}

void getRollPitchYawGY88(float *roll, float *pitch , float *yaw){
	normAccel = mpu.readNormalizeAccel();
	mpu.rateGyro();
	unfiltered_roll = atan2(normAccel.XAxis,normAccel.ZAxis) * RAD_TO_DEG;
	unfiltered_pitch = atan2(normAccel.YAxis,normAccel.ZAxis) * RAD_TO_DEG;
	read_Compass();
	if((millis() - previousTime)>=10){
		previousTime = millis();
		if(readFirstData){
			filtered_roll = unfiltered_roll;
			filtered_pitch = unfiltered_pitch;
			filtered_yaw = unfiltered_yaw;
			readFirstData = false;
		}
		filtered_roll = 0.93 * (filtered_roll + mpu.getGyroRateX() * timeStep) + 0.07 * unfiltered_roll;
		filtered_pitch = 0.93 * (filtered_pitch + mpu.getGyroRateY() * timeStep) + 0.07 * unfiltered_pitch;
		unfiltered_yaw = getAverageCompensatedYaw();
		filtered_yaw = 0.93 * (filtered_yaw + mpu.getGyroRateZ() * timeStep) + 0.07 * unfiltered_yaw;
			
	}
	*roll = filtered_roll;
	*pitch = filtered_pitch;
	*yaw = filtered_yaw;
}





#endif /* GY88_H_ */