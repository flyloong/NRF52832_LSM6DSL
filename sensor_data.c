
#include "sensor_data.h"
#include "nrf_drv_spi.h"
//#include "nrf_drv_twi.h"
//#include "twi_master.h"
#include "nrf_gpio.h"
//#include "app_util_platform.h"
#include "AHRS.h"
#include "ble_nus.h"
#include "LSM6DSL_ACC_GYRO_driver.h"



static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(0);  /**< SPI instance. */


static uint8_t       m_tx_buf[2] ;           /**< TX buffer. */
static uint8_t          m_rx_buf2=0;

static   uint8_t       m_rx_buf[13];    /**< RX buffer. */

  uint8_t regValue[12];
 int16_t Sensor_Raw_Data[6];
 float Sensor_Raw_Data_Temp[3];
float GYROSCOPE_ANGLE_RATIO=0.0000174532925f;
float G_Sensitivity = 70;
float A_Sensitivity = 0.244;
float Gyro_Temp,Acc_Temp,Gyro_Data[3],Gyro_Offset[3]={0,0,0},Gyro_Offset_Temp[3],Acc_Data[3],Acc_Offset[3]={0};

#define SPI_CS_PIN   25  /**< SPI CS Pin.*/
static volatile bool spi_xfer_done=true;  /**< Flag used to indicate that SPI instance completed the transfer. */
int spi_cnt=0;
void spi_event_handler(nrf_drv_spi_evt_t const * p_event  )
{
    spi_xfer_done = true;
}
void Sensor_SPI_Init(void){
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = 25;
    spi_config.miso_pin = 24;
    spi_config.mosi_pin = 23;
    spi_config.sck_pin  = 22;
    
    spi_config.ss_pin   = 20;
    spi_config.miso_pin = 17;
    spi_config.mosi_pin = 18;
    spi_config.sck_pin  = 19;
    
     APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
}
uint8_t Sensor_IO_Write_NO_Ack(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite){
	uint8_t tx_buf[2];
	tx_buf[0]=WriteAddr;
	tx_buf[1]=*pBuffer;
        nrf_drv_spi_transfer(&spi, tx_buf, 2, NULL, 0);
	return 0;
}

uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite){
	uint8_t tx_buf[2];
	tx_buf[0]=WriteAddr;
	tx_buf[1]=*pBuffer;
        spi_xfer_done = false;
        nrf_drv_spi_transfer(&spi, tx_buf, 2, NULL, 0);
        while(spi_xfer_done == false);     
	return 0;
}

uint8_t Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer2, uint16_t nBytesToRead){
        
  uint8_t tx_buf=( ReadAddr) | 0x80;
         spi_xfer_done = false;       
            nrf_drv_spi_transfer(&spi,&tx_buf,1,m_rx_buf,nBytesToRead+1);
            if(nBytesToRead==1){
              while(spi_xfer_done == false);    
            }
          *pBuffer2=m_rx_buf[1];
	return 0;
}

void Get_Sensor_RawData(void){
  	Sensor_IO_Read(NULL,LSM6DSL_ACC_GYRO_OUTX_L_G, regValue, 12);     
        for(int i=0;i<6;i++){
         Sensor_Raw_Data[i] = ( ( ( ( int16_t )m_rx_buf[2*i+2] ) << 8 ) + ( int16_t )m_rx_buf[2*i+1] );
        }
}

void Get_Sensor_Data(void){
          Gyro_Data[0]=((float)Sensor_Raw_Data[0]-Gyro_Offset[0])*GYROSCOPE_ANGLE_RATIO*G_Sensitivity;
          Gyro_Data[1]=((float)Sensor_Raw_Data[1]-Gyro_Offset[1])*GYROSCOPE_ANGLE_RATIO*G_Sensitivity;
          Gyro_Data[2]=((float)Sensor_Raw_Data[2]-Gyro_Offset[2])*GYROSCOPE_ANGLE_RATIO*G_Sensitivity;
          Acc_Data[0]=(Sensor_Raw_Data[3]-Acc_Offset[0])*A_Sensitivity;
          Acc_Data[1]=(Sensor_Raw_Data[4]-Acc_Offset[1])*A_Sensitivity;
          Acc_Data[2]=(Sensor_Raw_Data[5]-Acc_Offset[2])*A_Sensitivity;   
}

uint8_t test(void){
    Get_Sensor_RawData();
    Get_Sensor_Data();
    madgwickQuaternionUpdate(Acc_Data[0],Acc_Data[1],Acc_Data[2],Gyro_Data[0],Gyro_Data[1],Gyro_Data[2],0,0,0);
    AHRS_updateEulerAngles();
    return 0;
}

void Sensor_Init(void){
 uint8_t Who_Am_I=0; 
        while(Who_Am_I!=0x6A){
        Sensor_IO_Read(NULL,LSM6DSL_ACC_GYRO_WHO_AM_I_REG, &Who_Am_I, 1);  
        }
       m_tx_buf[0]=0x44;
      do{
        Sensor_IO_Write(NULL,LSM6DSL_ACC_GYRO_CTRL3_C,m_tx_buf,1);//Enable BDU  
        Sensor_IO_Read(NULL,LSM6DSL_ACC_GYRO_CTRL3_C,&m_rx_buf2,1);  
      } while( m_tx_buf[0]!=m_rx_buf2);
        m_tx_buf[0]=0x7C;//833
      do{
        Sensor_IO_Write(NULL,LSM6DSL_ACC_GYRO_CTRL2_G,m_tx_buf,1);//Gyro 1660Hz 2000dps  
        Sensor_IO_Read(NULL,LSM6DSL_ACC_GYRO_CTRL2_G,&m_rx_buf2,1);  
      } while( m_tx_buf[0]!=m_rx_buf2);   
       do{
        Sensor_IO_Write(NULL,LSM6DSL_ACC_GYRO_CTRL1_XL,m_tx_buf,1);//Acc 1660Hz 8G
        Sensor_IO_Read(NULL,LSM6DSL_ACC_GYRO_CTRL1_XL,&m_rx_buf2,1);  
      } while( m_tx_buf[0]!=m_rx_buf2);   
      m_tx_buf[0]=0x00;        
      do{
        Sensor_IO_Write(NULL,LSM6DSL_ACC_GYRO_CTRL6_G,m_tx_buf,1);//LSM6DSL_ACC_GYRO_W_LowPower_XL
        Sensor_IO_Read(NULL,LSM6DSL_ACC_GYRO_CTRL6_G,&m_rx_buf2,1);  
      } while( m_tx_buf[0]!=m_rx_buf2);   
}
