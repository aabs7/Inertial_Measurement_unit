/*
 * MPU6050.h
 *
 * Created: 6/28/2018 1:07:32 PM
 *  Author: Prakash Chaudhary
 */ 


#ifndef MPU6050_H_
#define MPU6050_H_


#define F_CPU 16000000UL

#include <avr/io.h>


#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector{
	float XAxis;
	float YAxis;
	float ZAxis;
	};
#endif



class MPU6050
{
	public:
	MPU6050();
	void Init(void);
	float readTemperature(void);
	
	Vector readRawAccel(void);
	Vector readNormalizeAccel(void);
	
	Vector readRawGyro(void);
	Vector readNormalizeGyro(void);
	
	void calibrateGyro(uint8_t samples = 50);
	void calibrateAccelero(uint8_t samples = 50);
	void setThreshold(uint8_t multiple = 1);
	uint8_t getThreshold(void);
	void rateGyro(void);
	float getGyroRateX(void);
	float getGyroRateY(void);
	float getGyroRateZ(void);
	
	private:
	void startReadLocation(unsigned char addr);
	
	Vector ra , rg;	//Raw vectors
	Vector na , ng;	//normalized vectors
	Vector tg, dg; // Threshold and Delta for Gyro
	Vector offsetAccelero;
	Vector th;     // Threshold
	float gyroRateX,gyroRateY,gyroRateZ;
	
	float dpsPerDigit,rangePerDigit;
	float actualThreshold;
	bool useCalibrate;
	};








#endif /* MPU6050_H_ */