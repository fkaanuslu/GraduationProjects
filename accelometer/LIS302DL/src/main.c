
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

											 /*-------type defs------*/
GPIO_InitTypeDef GPIO_InitStruct;
SPI_InitTypeDef SPI_InitStruct;



uint8_t x, y, z;
uint8_t x_adress = 0x29 , y_adress = 0x2B , z_adress = 0x2D;


							/*---------------------GPIO CONFİG-------------------------------*/
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
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // baundrate'i 2 ye ayarladım çünkü clock hattı 84MHz'de çalışıyor
	// 2 ye bölerek en hızlı şekilde çalışmasını sağladım.
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge; //bit yakalama yaparken 2. kenarı kullan
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	/*
	 * The registers embedded inside the LIS302DL may be accessed through I2C and SPI serial
	 * interfaces. The latter may be SW configured to operate either in 3-wire or 4-wire interface mode.
	 * The serial interfaces are mapped onto the same pads. To select/exploit the I2C interface, CS
     * line must be tied %%%% HİGH %%%%
     * */
	// SPI_InitStruct.SPI_CRCPolynomial -- > gerek duymadım bunu yapmaya
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // hem okuma hem yazma için kullanıyorum
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// düşük önclili yükske öncelikli seçme
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;// master olucak
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set ; // donanım tarafından mı kontrol edilecek yazılım tarafından mı? yazılım tarafından.

	SPI_Init(SPI1, &SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE);

	GPIO_SetBits(GPIOE, GPIO_Pin_3); // lojik 1'e getirerek kullanılabilir hale getirdim.

}


void SPI_Write(uint8_t adress, uint8_t data)
{
	/*------while sending data it must be low------------*/
	GPIO_ResetBits(GPIOE, GPIO_Pin_3); // SPI üzerinden data gönderirken low'a çekmen lazım

	/*------Sending address-------*/
	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE))
		; // TXE interruptı enable olana kadar bekle
	// içerisi 1 olduğunda tersliyecem 0 olucak ve while'dan çıkıcam

	SPI_I2S_SendData(SPI1, adress); // hangi adreste çalışacağımı söylüyorum

	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE))
		;// receivere tarafında data geldimi kontrolü

	SPI_I2S_ReceiveData(SPI1);

	//aynı şeyi data için yapıyorum
	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE))
		;
	SPI_I2S_SendData(SPI1, data);

	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE))
		;// receivere tarafında data geldimi kontrolü

	SPI_I2S_ReceiveData(SPI1);

	GPIO_SetBits(GPIOE, GPIO_Pin_3);
}


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

  SPI_Write(0x20, 0x67); // control register 1 i aktif ediyorum
  while (1)
  {
	  x = SPI_Read(x_adress, 0x00);
	  y = SPI_Read(y_adress, 0x00);
	  z = SPI_Read(z_adress, 0x00);

	  if(x > 200)
		  GPIO_SetBits(GPIOD, GPIO_Pin_12);
	  else
		  GPIO_ResetBits(GPIOD, GPIO_Pin_12);

	  if(x > 200)
	  	  GPIO_SetBits(GPIOD, GPIO_Pin_15);
	  else
	  	  GPIO_ResetBits(GPIOD, GPIO_Pin_15);

	  if(y > 200)
	  	  GPIO_SetBits(GPIOD, GPIO_Pin_13);
	  else
		  GPIO_ResetBits(GPIOD, GPIO_Pin_13);


	  if(z > 200)
		  GPIO_SetBits(GPIOD, GPIO_Pin_14);
	  else
		  GPIO_ResetBits(GPIOD, GPIO_Pin_14);

	  if(z > 200)
	  	  GPIO_SetBits(GPIOD, GPIO_Pin_15);
	  else
	  	  GPIO_ResetBits(GPIOD, GPIO_Pin_15);


  }
}


void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
