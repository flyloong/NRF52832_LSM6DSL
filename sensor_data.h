#ifndef _SENSOR_DATA_H_
#define _SENSOR_DATA_H_


//#include "imu_6axes.h"
//#include "magneto.h"
//#include "pressure.h"
//#include "board.h"
//#include "steval_fcu001_v1.h"
//#include "component.h"
#include <string.h>
#include <stdint.h>
struct T_Data{
            uint8_t R0_continuous;
          uint8_t R1_3D;
          uint8_t R2_pressure;
          uint8_t R3_power;
          uint8_t R4_direction;
          uint8_t R7_msg;
          uint8_t first_byte[8];
          uint8_t Data_All[20] ;
          int Euler[3];
         int Pressure;
         uint8_t Pressure_Cnt;
         uint8_t Power;
         uint8_t Direction[5];
          uint8_t Direction_RealTime[3];
         uint8_t Msg;
        uint16_t timestamp;
         uint8_t Check_sum;
};
struct R_Data{
          uint8_t Data_Receive_Flag;
          uint8_t R0_continuous;
          uint8_t R1_3D;
          uint8_t R2_pressure;
          uint8_t R3_power;
          uint8_t R4_direction;
          uint8_t R5_shake;
          uint8_t R6_LED;
          uint8_t R7_MSG;
          uint8_t first_byte[8];
          uint8_t second_byte[8];
          uint8_t Data_All[20] ;
         uint8_t Time_Cnt;
        uint8_t shake_level ;
        uint8_t Lamp_level;
        uint8_t LR_direction_threshold;
        uint8_t UD_direction_threshold;
        uint8_t which_hand;
        uint8_t smoothness_level;
        uint8_t Pressure_Th;
        uint8_t Pressure_Time;
        uint8_t direction_mode;
        uint8_t Pressure_mode;
        uint8_t Get_Write_Flash;
        uint8_t Pressure_Calibration;
        uint8_t Pressure_Calibration_Auto;
        uint8_t Check_sum;
};
extern struct T_Data mT_Data;
extern struct R_Data mR_Data;
void Sensor_SPI_Init(void);
void Sensor_Init(void);
void Sensor_Init_Power_off(void);
 void Sensor_Init_2(void);
 void Sensor_Init_3(void);
//  void Sensor_Init_4(void);
  void Sensor_Init_5(void);
    void Sensor_Init_6(void);
uint8_t Read_Status(void);
__UINT8_T_TYPE__ test(void);
 float KalmanFilter(const float ResrcData,float *x_last,float *p_last, float ProcessNiose_Q,float MeasureNoise_R);
void Reset_R_Data(void);
#endif /* _SENSOR_DATA_H_ */
