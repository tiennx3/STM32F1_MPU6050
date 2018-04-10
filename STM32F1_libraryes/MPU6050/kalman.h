#ifndef __KALMAN_H
#define __KALMAN_H

float dt = 0.01; // T Sampling
float Q_angle = 0.005;
float Q_bias = 0.003;
float R_measure = 0.03;
float bias = 0; // Reset bias
float rate=0;
float angle=0;
float S; // estimate error
float y; // different angle
float P_00 = 0 , P_01 = 0 , P_10 =0 ,P_11 =0;
float K_0 =0,K_1=0; // Kalman gain
 
float Kalman(float newAngle, float newRate){
        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        //angle = X_Raw_Gyro_Angle;
        rate = newRate - bias;
        angle += dt * rate;
 
        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        P_00 += dt * ( dt*P_11 - P_10 - P_01 + Q_angle);
        P_01 -= dt * P_11;
        P_10 -= dt * P_11;
        P_11 += Q_bias * dt;
 
        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        S = P_00 + R_measure;
        /* Step 5 */
        K_0 = P_00 / S;
        K_1 = P_10 / S;
 
        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        y = newAngle - angle;
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

 #endif /* __KALMAN_H */
