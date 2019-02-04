/*
 * gy81Gyro.cpp
 *
 * Created: 7/3/2018 12:49:56 AM
 *  Author: Abheesh Khanal
 */ 

#include "gy81Gyro.h"
#include <util/delay.h>

void ITG3205::itg3205initGyro()
{
	i2c_init();
	i2c_start(ITG3205_WRITE);
	i2c_write(0x3E);
	i2c_write(0x00);
	i2c_stop();
	
	i2c_start(ITG3205_WRITE);
	i2c_write(0x15);
	i2c_write(0x07);			//8 ms per sample
	i2c_stop();
	
	i2c_start(ITG3205_WRITE);
	i2c_write(0x16);
	i2c_write(0x1E);   // +/- 2000 dgrs/sec, 1KHz, 1E, 19
	i2c_stop();
	
	i2c_start(ITG3205_WRITE);
	i2c_write(0x17);
	i2c_write(0x00);
	i2c_stop();
}

void ITG3205::itg3205ReadGyro()
{
	i2c_start(ITG3205_WRITE); 	// address of the accelerometer
	i2c_write(0x1B); 							// set read pointer to data
	i2c_stop();
	
	i2c_rep_start(ITG3205_READ);

	temp = i2c_readAck() << 8;
	temp |= i2c_readAck();
	x = i2c_readAck() << 8;
	x |= i2c_readAck();
	y = i2c_readAck() << 8;
	y |= i2c_readAck();
	z = i2c_readAck() << 8;
	z |= i2c_readNak();
	
	x = x - g_offx;
	y = y - g_offy;
	z = z - g_offz;
}

void ITG3205::itg3205CalGyro()
{
	int tmpx = 0;
	int tmpy = 0;
	int tmpz = 0;

	g_offx = 0;
	g_offy = 0;
	g_offz = 0;
	
	for (char i = 0;i<10;i++)
	{
		_delay_ms(10);
		itg3205ReadGyro();
		tmpx += x;
		tmpy += y;
		tmpz += z;
	}
	g_offx = tmpx/10;
	g_offy = tmpy/10;
	g_offz = tmpz/10;
}

float ITG3205::itg3205GyroX()
{
	return x;// / 14.375;
}

float ITG3205::itg3205GyroY()
{
	return y;// / 14.375;
}

float ITG3205::itg3205GyroZ()
{
	return z;// / 14.375;
}

float ITG3205::itg3205Temp()
{
	return (35+((temp+13200) / 280));
}

