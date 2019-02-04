/*
 * kalman_filter.h
 *
 * Created: 1/1/2017 9:41:02 PM
 *  Author: dhiraj.basnet
 */ 


#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
//#include "accelerometer.h"
//#include "gyrometer.h"

class Kalman
{
	private:
	float rate;  // Unbiased rate calculated from the rate and the calculated bias
	float y;       // Angle difference
	float S;       // Estimate error

	float Q_angle = 0.001;    //Initially assumed values for noise parameters
	float Q_bias = 0.003;
	float R_measure = 0.03;

	float angle = 0.0;// Reset the angle to 0
	float bias = 0.0; // Reset bias to 0

	float P_00 = 0.0;//nce the bias is 0 and we know the starting angle, the error covariance matrix is set like this
	float P_01 = 0.0;
	float P_10 = 0.0;
	float P_11 = 0.0;
	
	public:
		
		
	// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
	float kalman_angle(float newAngle, float newRate, float dt)
	{

		// Predict stage-Project the state ahead

		/* Step 1 */
		rate = newRate - bias;
		angle += dt * rate;

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		P_00 += dt * (dt*P_11 - P_01 - P_10 + Q_angle);
		P_01 -= dt * P_11;
		P_10 -= dt * P_11;
		P_11 += Q_bias * dt;

		//  Measurement Update- Update estimate with measurement zk (newAngle)
		/* Step 3 */
		y = newAngle - angle;

		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		S = P_00 + R_measure;

		/* Step 5 */
		float K_0 = P_00 / S;
		float K_1 = P_10 / S;

		/* Step 6 */
		angle += K_0 * y;
		bias += K_1 * y;

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		P_00 -= K_0 * P_00;
		P_01 -= K_0 * P_01;
		P_10 -= K_1 * P_00;
		P_11 -= K_1 * P_01;

		return angle;
	}
};




#endif /* KALMAN_FILTER_H_ */