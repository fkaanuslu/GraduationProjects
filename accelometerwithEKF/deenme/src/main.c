#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <math.h>
#include <time.h>
#include "kalmanfilter.h"

											 /*-------type defs------*/
GPIO_InitTypeDef GPIO_InitStruct;
SPI_InitTypeDef SPI_InitStruct;




uint8_t x_adress = 0x29 , y_adress = 0x2B , z_adress = 0x2D; // from LIS302DL datasheet. Their origins


							    /*---------------------GPIO CONFİG------------------------------*/
void GPIO_Config (void)
{
	/*-------Activating clock busses for PD12 PD13 PD13 PD15 - PA5 PA6 PA7 - PE3 pins--------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/*--------Activating Leds--------*/
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 |  GPIO_Pin_13 |  GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOD, &GPIO_InitStruct);


	/*---------Activating SPI Pins--------*/
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOE, &GPIO_InitStruct);


	/*--------Activating SPI Pins--------*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;


	GPIO_Init(GPIOA, &GPIO_InitStruct);


}

								/*---------------------SPI CONFİG-------------------------------*/
void SPI_Config (void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // i set baundrate to 2 in order to work fast
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge; //use second edge
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High; // while risign
	/*
	 *
	 *This paragrafh also from the datasheet of LIS302DL
	 *
	 * The registers embedded inside the LIS302DL may be accessed through I2C and SPI serial
	 * interfaces. The latter may be SW configured to operate either in 3-wire or 4-wire interface mode.
	 * The serial interfaces are mapped onto the same pads. To select/exploit the I2C interface, CS
     * line must be tied %%%% HİGH %%%%
     *
     *
     *
     *
     * */

	// SPI_InitStruct.SPI_CRCPolynomial -- > No need for that
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // both reading and writing
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// düşük önclili yükske öncelikli seçme
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;// Master
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set ; // In order to control my SPI I need to choose hardware or
	//sofware. I chose software

	SPI_Init(SPI1, &SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE); // since it is a peripheral you need to give this command to your stm32f407

	GPIO_SetBits(GPIOE, GPIO_Pin_3); // To use the SPI make it level lojik 1

}
								/*---------------------SPI Writing------------------------------*/
void SPI_Write(uint8_t adress, uint8_t data)
{
	/*------while sending data it must be low------------*/
	GPIO_ResetBits(GPIOE, GPIO_Pin_3); //While you are sending an data make it in low level. Also you can find it in datasheet

	/*------Sending address-------*/
	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE))
		; // wait to enable until TXE flag interrupted
	// inside of while loop when it became 1 inside it means TXE flag enabled. So you can get out of the while loop


	SPI_I2S_SendData(SPI1, adress); // hangi adreste çalışacağımı söylüyorum

	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE))
		;// same lojik like above but this time its controlling the RX (receiver side) finding if its arrived.

	SPI_I2S_ReceiveData(SPI1);

	/*------Sending data-------*/ // same thing like above
	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE))
		;
	SPI_I2S_SendData(SPI1, data);

	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE))
		;

	SPI_I2S_ReceiveData(SPI1);

	GPIO_SetBits(GPIOE, GPIO_Pin_3); // after u finished set it to lojik 1
}
								/*---------------------SPI Reading------------------------------*/
uint8_t SPI_Read(uint8_t adress, uint8_t data)
{

	GPIO_ResetBits(GPIOE, GPIO_Pin_3);

	 adress = adress | 0x80; // okuma işlemi yapılacak

	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE))
		;

	SPI_I2S_SendData(SPI1, adress);

	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE))
		;

	SPI_I2S_ReceiveData(SPI1);


	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE))
			;
	SPI_I2S_SendData(SPI1, data);

	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE))
			;

	SPI_I2S_ReceiveData(SPI1);


	GPIO_SetBits(GPIOE, GPIO_Pin_3);

	return SPI_I2S_ReceiveData(SPI1);

}


int main(void)
{
  GPIO_Config();
  SPI_Config ();
  SPI_Write(0x20, 0x67); // activate control register 1. (You can find it in datasheet)


  float x_real, y_real, z_real;
  float measured_x, measured_y, measured_z;
  float estimated_x, estimated_y, estimated_z;


  int lower = -100, upper = 100;

  srand(time(0));
  int random = (rand() % (upper - lower + 1)) + lower;

  while (1)
  {
	  SimpleKalmanFilter( 2, 2, 0.01);
	  /* SimpleKalmanFilter(e_mea, e_est, q);
 	 	 e_mea: Measurement Uncertainty
 	 	 e_est: Estimation Uncertainty
 	 	 q: Process Noise*/

	  //For X dimension
	  x_real = SPI_Read(x_adress, 0x00); // read the real values
	  measured_x = x_real + random;
	  estimated_x = updateEstimate(measured_x);
	  //For Y dimension
	  y_real = SPI_Read(y_adress, 0x00);
	  measured_y = y_real + random;
	  estimated_y = updateEstimate(measured_y);
	  //For Z dimension
	  z_real = SPI_Read(z_adress, 0x00);
	  measured_z = z_real + random;
	  estimated_z = updateEstimate(measured_z);


	  	  	  	  	  	  /*---------Algorithm---------*/
	  /*Explaining :
	   *
	   * It is a simple code. It does the same thing like if statement but with ternary operators to look cool
	   *
	   *For instance :
	   *
	   *Light pin 12 and pin 15 when x above 200
	   *
	   * */
	  x_real > 200 ? GPIO_SetBits(GPIOD, GPIO_Pin_12) : GPIO_ResetBits(GPIOD, GPIO_Pin_12);
	  x_real > 200 ? GPIO_SetBits(GPIOD, GPIO_Pin_15) : GPIO_ResetBits(GPIOD, GPIO_Pin_15);
	  y_real > 200 ? GPIO_SetBits(GPIOD, GPIO_Pin_13) : GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	  z_real > 200 ? GPIO_SetBits(GPIOD, GPIO_Pin_14) : GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	  z_real > 200 ? GPIO_SetBits(GPIOD, GPIO_Pin_15) : GPIO_ResetBits(GPIOD, GPIO_Pin_15);

  }
}






void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
