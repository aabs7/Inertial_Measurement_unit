/*
 * gy81Gyro.h
 *
 * Created: 7/3/2018 12:48:50 AM
 *  Author: Abheesh Khanal
 */ 


#ifndef GY81GYRO_H_
#define GY81GYRO_H_


#include "TWI.h"

#define ITG3205_Address 0x68

#define ITG3205_WRITE	0xD0
#define ITG3205_READ	0xD1

class ITG3205
{
	public:
	//HMC5883L();

	//MagnetometerRaw ReadRawAxis();
	//MagnetometerScaled ReadScaledAxis();
	
	//int SetMeasurementMode(uint8_t mode);
	//int SetScale(float gauss);

	//char* GetErrorText(int errorCode);

	void itg3205initGyro();
	void itg3205ReadGyro();
	void itg3205CalGyro();
	float itg3205GyroX();
	float itg3205GyroY();
	float itg3205GyroZ();
	float itg3205Temp();
	
	protected:
	//void Write(int address, int byte);
	//uint8_t* Read(int address, int length);

	short x,y,z,temp; // yes, public, what the heck
	int g_offx,g_offy,g_offz;

	private:
	//float m_Scale;
};





#endif /* GY81GYRO_H_ */