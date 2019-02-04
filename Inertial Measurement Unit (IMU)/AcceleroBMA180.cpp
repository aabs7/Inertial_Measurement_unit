/*
 * AcceleroBMA180.cpp
 *
 * Created: 7/5/2018 6:40:06 PM
 *  Author: Abheesh Khanal
 */ 
#ifndef F_CPU
#define F_CPU 16000000ul
#endif

#include "AcceleroBMA180.h"
#include "TWI.h"
#include <util/delay.h>
#include "General-function.h"
#include "uart.h"

void BMA180::bma180Write(char add, char data){
	i2c_start(BMA180_WRITE_ADDRESS);
	i2c_write(add);
	i2c_write(data);
	i2c_stop();
}
void BMA180::bma180EnableWrite()
{
	bma180Write(0x0D,0x10);
	_delay_ms(10);
}

void BMA180::bma180SoftReset(){
	bma180Write(0x10,0xB6);
	_delay_ms(100);	//give time for accelerometer
}

void BMA180::bma180init(){
	bma180EnableWrite();
	bma180SoftReset();
	bma180SetFilter(F10HZ);
	bma180SetGSensitivty(G1);
	_delay_ms(10);
}

void BMA180::bma180GetIDs(int *id, int *version){
	i2c_start(BMA180_WRITE_ADDRESS);
	i2c_write(0x00);		//set pointer to data
	i2c_stop();
	
	i2c_rep_start(BMA180_READ_ADDRESS);
	*id = i2c_readAck();
	*version = i2c_readNak();
	
	i2c_stop();
}

void BMA180::bma180SetFilter(FILTER f){
	i2c_start(BMA180_WRITE_ADDRESS);
	i2c_write(0x20);		//read from here
	i2c_stop();
	
	i2c_rep_start(BMA180_READ_ADDRESS);
	uint8_t data = i2c_readNak();
	i2c_stop();
	i2c_start(BMA180_WRITE_ADDRESS);
	i2c_write(0x20);
	i2c_write((data & 0x0F) | f<<4);	//low pass filter to 10Hz
	i2c_stop();
	_delay_ms(10);
}
void BMA180::bma180SetGSensitivty(GSENSITIVITY maxg)	
{
	i2c_start(BMA180_WRITE_ADDRESS);
	i2c_write(0x35);
	i2c_stop();
	i2c_rep_start(BMA180_READ_ADDRESS);
	uint8_t data = i2c_readNak();
	i2c_stop();
	
	i2c_start(BMA180_WRITE_ADDRESS);
	i2c_write(0x35);
	i2c_write((data & 0xf1) | maxg << 1);//range +/- 2g
	i2c_stop();
	gSense = maxg;
	
}


void BMA180::bma180ReadAccel()
{
	i2c_start(BMA180_WRITE_ADDRESS); 	// address of the accelerometer
	i2c_write(0x02); 							// set read pointer to data
	i2c_stop();
	
	i2c_rep_start(BMA180_READ_ADDRESS);

	// read in the 3 axis data, each one is 16 bits
	// print the data to terminal
	//uart_puts("Accelerometer: X = ");
	x = i2c_readAck();
	x |= i2c_readAck() << 8;
	x = x >> 2;
	y = i2c_readAck();
	y |= i2c_readAck() << 8;
	y = y >> 2;
	z = i2c_readAck();
	z |= i2c_readAck() << 8;
	z = z >> 2;
	temp = i2c_readNak();
	
	i2c_stop();
}

float BMA180::bma180GetgSense()
{
	switch(gSense)
	{
		case G1: 	return 1.0 * 0.0001250;
		case G15: 	return 1.5 * 0.0001875;
		case G2: 	return 2.0 * 0.0002500;
		case G3: 	return 3.0 * 0.0003750;
		case G4: 	return 4.0 * 0.0005000;
		case G8: 	return 8.0 * 0.0009900;
		case G16: 	return 16.0 * 0.0019800;
	}
}

float BMA180::bma180FloatX()
{
	return x;
}

float BMA180::bma180FloatY()
{
	return y;
}

float BMA180::bma180FloatZ()
{
	return z;
}

float BMA180::bma180GravityX()
{
	float data = bma180GetgSense();
	return x * data;
}

float BMA180::bma180GravityY()
{
	float data = bma180GetgSense();
	return y * data;
}

float BMA180::bma180GravityZ()
{
	float data = bma180GetgSense();
	return z * data;
}

float BMA180::bma180Temp()
{
	return map((int8_t)temp,-128,127,-400,875)/10.0;;
}
