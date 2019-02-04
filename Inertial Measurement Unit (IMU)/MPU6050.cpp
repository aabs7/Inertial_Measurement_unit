/*
 * MPU6050.cpp
 *
 * Created: 6/28/2018 6:36:29 PM
 *  Author: Abheesh Khanal
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <stdlib.h>
#include <math.h>
#include "MPU6050.h"
#include "TWI.h"
#include "MPU6050_res_define.h"
#include <util/delay.h>

MPU6050::MPU6050(){
	// Reset calibrate values
	dg.XAxis = 0;
	dg.YAxis = 0;
	dg.ZAxis = 0;
	useCalibrate = false;
	//reset calibration offset for accelero//
	offsetAccelero.XAxis = 0;
	offsetAccelero.YAxis = 0;
	offsetAccelero.ZAxis = 0;

	// Reset threshold values
	tg.XAxis = 0;
	tg.YAxis = 0;
	tg.ZAxis = 0;
	actualThreshold = 0;
	//0.007633f for 250dps,0.015267 for 500dps, 0.030487f for 1000 dps , 0.060975f for 2000dps
	dpsPerDigit = .007633f;			//for 250DPS
	// 0.000061f for 2g, 0.000122f for 4g, 0.000244f for 8g , 0.0004882f for 16g
	rangePerDigit = .000061f;		//for 2G RANGE
}


void MPU6050::Init(void){
	_delay_ms(150);										/* Power up time >100ms */
	i2c_start(0xD0);								/* Start with device write address */

	i2c_write(PWR_MGMT_1);								/* Write to power management register */
	i2c_write(0x00);									/* X axis gyroscope reference frequency */
	i2c_stop();

	i2c_start(0xD0);
	i2c_write(SMPLRT_DIV);								/* Write to sample rate register */
	i2c_write(0x19);									/* 1KHz sample rate */
	i2c_stop();

	
	i2c_start(0xD0);
	i2c_write(CONFIG);									/* Write to Configuration register */
	i2c_write(0x00);									/* Fs = 8KHz */
	i2c_stop();

	i2c_start(0xD0);
	i2c_write(GYRO_CONFIG);								/* Write to Gyro configuration register */
	i2c_write(0x00);									/* Full scale range +/- 250 degree/C */
	i2c_stop();

	i2c_start(0XD0);
	i2c_write(INT_ENABLE);								/* Write to interrupt enable register */
	i2c_write(0x01);
	i2c_stop();
	
	
	i2c_start(MPU6050_WRITE);
	i2c_write(MPU6050_ACCEL_CONFIG);   //set pointer to accelerometer configuration register
	i2c_write(0x00);                  //set accelerometer scale to 2g
	i2c_stop();
}

void MPU6050::startReadLocation(unsigned char addr)
{
	i2c_start_wait(MPU6050_WRITE);								/* I2C start with device write address */
	i2c_write(addr);							/* Write start location address from where to read */
	i2c_rep_start(MPU6050_READ);							/* I2C start with device read address */
}

Vector MPU6050::readRawAccel(void){
	Vector rra;
	startReadLocation(ACCEL_XOUT_H);
	rra.XAxis = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	rra.YAxis = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	rra.ZAxis = (((int)i2c_readAck()<<8) | (int)i2c_readNak());
	ra.XAxis = rra.XAxis - offsetAccelero.XAxis;
	ra.YAxis = rra.YAxis - offsetAccelero.YAxis;
	ra.ZAxis = rra.ZAxis;
	i2c_stop();
	return ra;
}

 Vector MPU6050::readNormalizeAccel(void)
 {
 	readRawAccel();
 
 	na.XAxis = ra.XAxis * rangePerDigit;// * 9.80665f;
 	na.YAxis = ra.YAxis * rangePerDigit;// * 9.80665f;
 	na.ZAxis = ra.ZAxis * rangePerDigit;// * 9.80665f;
 
 	return na;
 }

Vector MPU6050::readRawGyro(void){
	startReadLocation(GYRO_XOUT_H);
	rg.XAxis = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	rg.YAxis = (((int)i2c_readAck()<<8) | (int)i2c_readAck());
	rg.ZAxis = (((int)i2c_readAck()<<8) | (int)i2c_readNak());
	i2c_stop();
	return rg;
}


Vector MPU6050::readNormalizeGyro(void)
{
	readRawGyro();

	if (useCalibrate)
	{
		ng.XAxis = (rg.XAxis - dg.XAxis) * dpsPerDigit;
		ng.YAxis = (rg.YAxis - dg.YAxis) * dpsPerDigit;
		ng.ZAxis = (rg.ZAxis - dg.ZAxis) * dpsPerDigit;
	} else
	{
		ng.XAxis = rg.XAxis * dpsPerDigit;
		ng.YAxis = rg.YAxis * dpsPerDigit;
		ng.ZAxis = rg.ZAxis * dpsPerDigit;
	}

	if (actualThreshold)
	{
		if (abs(ng.XAxis) < tg.XAxis) ng.XAxis = 0;
		if (abs(ng.YAxis) < tg.YAxis) ng.YAxis = 0;
		if (abs(ng.ZAxis) < tg.ZAxis) ng.ZAxis = 0;
	}

	return ng;
}

float MPU6050::readTemperature(void)
{
	int16_t T;
	startReadLocation(TEMP_OUT_H);
	T = (((int)i2c_readAck()<<8) | (int)i2c_readNak());
	i2c_stop();
	return (float)T/340 + 36.53;
}

//
// Calibrate algorithm

void MPU6050::calibrateAccelero(uint8_t samples){
	float sumX = 0;
	float sumY = 0;
	for (uint8_t i = 0; i<samples; ++i){
		readRawAccel();
		sumX += ra.XAxis;
		sumY += ra.YAxis;
		//sum of Z force is not taken as it is force exerted by gravity.
		_delay_ms(5);
	}
	offsetAccelero.XAxis = sumX / samples;
	offsetAccelero.YAxis = sumY / samples;
}

void MPU6050::calibrateGyro(uint8_t samples)
{
	// Set calibrate
	useCalibrate = true;

	// Reset values
	float sumX = 0;
	float sumY = 0;
	float sumZ = 0;
	float sigmaX = 0;
	float sigmaY = 0;
	float sigmaZ = 0;

	// Read n-samples
	for (uint8_t i = 0; i < samples; ++i)
	{
		readRawGyro();
		sumX += rg.XAxis;
		sumY += rg.YAxis;
		sumZ += rg.ZAxis;

		sigmaX += rg.XAxis * rg.XAxis;
		sigmaY += rg.YAxis * rg.YAxis;
		sigmaZ += rg.ZAxis * rg.ZAxis;

		_delay_ms(5);
	}

	// Calculate delta vectors
	dg.XAxis = sumX / samples;
	dg.YAxis = sumY / samples;
	dg.ZAxis = sumZ / samples;

	// Calculate threshold vectors
	th.XAxis = sqrt((sigmaX / 50) - (dg.XAxis * dg.XAxis));
	th.YAxis = sqrt((sigmaY / 50) - (dg.YAxis * dg.YAxis));
	th.ZAxis = sqrt((sigmaZ / 50) - (dg.ZAxis * dg.ZAxis));

	// If already set threshold, recalculate threshold vectors
	if (actualThreshold > 0)
	{
		setThreshold(actualThreshold);
	}
}

// Get current threshold value
uint8_t MPU6050::getThreshold(void)
{
	return actualThreshold;
}

// Set treshold value
void MPU6050::setThreshold(uint8_t multiple)
{
	if (multiple > 0)
	{
		// If not calibrated, need calibrate
		if (!useCalibrate)
		{
			calibrateGyro();
		}

		// Calculate threshold vectors
		tg.XAxis = th.XAxis * multiple;
		tg.YAxis = th.YAxis * multiple;
		tg.ZAxis = th.ZAxis * multiple;
	} else
	{
		// No threshold
		tg.XAxis = 0;
		tg.YAxis = 0;
		tg.ZAxis = 0;
	}

	// Remember old threshold value
	actualThreshold = multiple;
}

void MPU6050::rateGyro(){
	readRawGyro();
	gyroRateX = (rg.XAxis)/131;
	gyroRateY = (rg.YAxis)/131;
	gyroRateZ = (rg.ZAxis)/131;
}

float MPU6050::getGyroRateX(){
	return gyroRateX;
}
float MPU6050::getGyroRateY(){
	return gyroRateY;
}
float MPU6050::getGyroRateZ(){
	return gyroRateZ;
}