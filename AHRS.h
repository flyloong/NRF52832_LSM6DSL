//
//  LSM9SDS0_AHRS.h
//  LSM9SDS0 AHRS
//
//  Created by Nicholas Robinson on 04/19/14.
//  Copyright (c) 2014 Nicholas Robinson. All rights reserved.
//

#ifndef __AHRS_H__
#define __AHRS_H__

#include "math.h"


// Madgwick Constants
#define GyroMeasError PI * (10.0f / 180.0f)
#define beta sqrt(3.0f / 4.0f) * GyroMeasError


#define Kp 2.0f 
#define Ki 0.000f
#define PI 3.1416f


extern  float pitch,pitch2, yaw, roll;

		static float q[4]={1,0,0,0};
                   static   float rMat[3][3];
		static float dt=0.002;
                static float mahonyErrors[3]={0};
		void madgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
		void mahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
                void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
                void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
                
                void AHRS_updateEulerAngles(void);
		
		 float invSqrt(float x) ;

#endif // __LSM9DS0_AHRS_H__
