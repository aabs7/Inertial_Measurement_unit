/*
 * compass.h
 *
 * Created: 12/24/2017 1:00:45 AM
 *  Author: TopsyKreet
 */ 
#ifndef COMPASS_H_
#define COMPASS_H_

// Library for 3 axis magnetometer with QMC5883L chip i.e Chinese sensor
#include <avr/io.h>
#include <math.h>
#include "TWI.h"

#define QMC5883L_WRITE 0x1a // write address  //7-bit address=0x0d
#define QMC5883L_READ 0x1b // read address


int16_t raw_x;
int16_t raw_y;
int16_t raw_z;
uint16_t Angle;
////for calibrating///////////
int16_t x_max = 0;
int16_t y_max = 0;
int16_t x_min = 0;
int16_t y_min = 0;
//////////////////////////////
int16_t X_offset = -113;
int16_t Y_offset = -98;
/////////////////////////////


void init_QMC5883L(void)
{
	i2c_init();
	//Set reset period register
	i2c_start(QMC5883L_WRITE);
	i2c_write(0x0B); //control register 3
	i2c_write(0x01);
	i2c_stop();

	//Configure mode of data
	i2c_start(QMC5883L_WRITE);
	i2c_write(0x09); // set pointer to control register 1
	i2c_write(0x11); // write 0x1D to CRA  Number of samples per output=256  ,data output rate=10Hz
	i2c_stop();
}

void read_Compass(void)
{
	i2c_start(QMC5883L_WRITE);
	i2c_write(0x00); //set pointer to X-axis LSB
	i2c_stop();
	
	i2c_rep_start(QMC5883L_READ);
	// Read the registers one by one
	
	raw_x = i2c_readAck();
	raw_x |=((uint8_t)i2c_readAck())<<8;
	//raw_x -= X_OFFSET;
	
	raw_y = i2c_readAck();
	raw_y |= ((uint8_t)i2c_readAck())<<8;
	//raw_y -= Y_OFFSET;
	
	raw_z = i2c_readAck();
	raw_z |=((uint8_t)i2c_readNak())<<8;
	
	i2c_stop();	
}
int16_t read_rawX(void){
	return (raw_x-X_offset);
}
int16_t read_rawY(void){
	return (raw_y-Y_offset);
}

void calibrate_compass(void){
	for (int i = 0;i<1000;i++){
		read_Compass();
		
		if(raw_x > x_max)
			x_max = raw_x;
		else if(raw_x < x_min)
			x_min = raw_x;
			
		if(raw_y > y_max)
			y_max = raw_y;
		else if(raw_y < y_min)
			y_min = raw_y;
		
		_delay_ms(10);
	}
	
	X_offset = ((x_max + x_min))/2;
	Y_offset = ((y_min + y_max))/2;
}

int16_t getoffset_X(void){
	return X_offset;
}
int16_t getoffset_Y(void){
	return Y_offset;
}

uint16_t get_Angle(){
	read_Compass();
	raw_x = raw_x - X_offset;
	raw_y = raw_y - Y_offset;
	Angle = (atan2((double)raw_y,(double)raw_x)* 180 / 3.14159265 +180.0);
	return Angle;
}



#endif /* COMPASS_H_ */