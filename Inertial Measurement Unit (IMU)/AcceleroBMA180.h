/*
 * AcceleroBMA180.h
 *
 * Created: 7/5/2018 6:30:11 PM
 *  Author: Abheesh Khanal
 */ 


#ifndef ACCELEROBMA180_H_
#define ACCELEROBMA180_H_

#define BMA180_ADDRESS			 0x38
#define BMA180_WRITE_ADDRESS	 0x80
#define BMA180_READ_ADDRESS		 0x81

class BMA180
{
	public:
	typedef enum {F10HZ=0,F20HZ=1,F40HZ, F75HZ,F15HZ0,F300HZ,F600HZ,F1200HZ,HIGHPASS,BANDPASS} FILTER;
	typedef enum {G1=0,G15=1,G2,G3,G4,G8,G16}GSENSITIVITY;
	
	void bma180init(void);
	void bma180SetGSensitivty(GSENSITIVITY maxg);
	void bma180SetFilter(FILTER f);
	void bma180SoftReset();
	void bma180GetIDs(int *id, int *version);
	void bma180EnableWrite();
	
	void bma180ReadAccel();
	float bma180GetgSense();
	
	float bma180FloatX();
	float bma180FloatY();
	float bma180FloatZ();
	
	float bma180GravityX();
	float bma180GravityY();
	float bma180GravityZ();
	
	float bma180Temp();
	
	protected:
	
	GSENSITIVITY gSense;
	short x,y,z,temp; // yes, public, what the heck
	
	void bma180Write(char add,char data);
	
	private:
	
	
};



#endif /* ACCELEROBMA180_H_ */