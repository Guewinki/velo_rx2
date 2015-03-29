//My board
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __DISCOVER_BOARD_H
#define __DISCOVER_BOARD_H


/* MACROs for SET, RESET or TOGGLE Output port */

#define GPIO_HIGH(a,b) 		a->ODR|=b
#define GPIO_LOW(a,b)		a->ODR&=~b
#define GPIO_TOGGLE(a,b) 	a->ODR^=b

#define BTN_PORT	GPIOC
#define BTN_PIN		GPIO_Pin_1

#define LED1_PORT	GPIOE     //green
#define LED1_PIN		GPIO_Pin_7
#define LED2_PORT		GPIOC       //blue
#define LED2_PIN		GPIO_Pin_7	//GPIOC->ODR addr 0x500a

//UART
#define USART_TX GPIOC					//also PC3,PC5
#define USART_TX_PIN GPIO_Pin_3
#define USART_RX GPIOC					//also PC2,PC6
#define USART_RX_PIN GPIO_Pin_2

//NRF24L01 transciever
#define NRF_CE GPIOB				//tx,rx mode activation
#define NRF_CE_PIN GPIO_Pin_2
#define NRF_CSN GPIOB				//chip select active-low
#define NRF_CSN_PIN GPIO_Pin_3

//SPI hardware
#define SPI_SCK    GPIOC
#define SPI_SCK_PIN   GPIO_Pin_6
#define SPI_MOSI    GPIOA
#define SPI_MOSI_PIN  GPIO_Pin_3
#define SPI_MISO    GPIOA					//port 
#define SPI_MISO_PIN  GPIO_Pin_2		//

//alarm out
#define ALM_PORT	GPIOD
#define ALM_PIN		GPIO_Pin_4    //pb6 ??

//I2C
// Q O O O
//SDA i2cadr CS SCL
#define I2C_SDA    GPIOC					//i2c data open drain 25pin
#define I2C_SDA_PIN  GPIO_Pin_0
#define I2C_CS    GPIOD           //1-I2C 0-SPI
#define I2C_CS_PIN GPIO_Pin_7
#define I2C_ADR    GPIOD          //I2C LSB addr
#define I2C_ADR_PIN GPIO_Pin_6
#define I2C_SCL    GPIOD					//i2c clock
#define I2C_SCL_PIN  GPIO_Pin_5



//PC1,PB6, PB5 down
#endif


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
/*
//area below doesn't use in this project
//SPI hardware
#define SPI_SCK    GPIOB
#define SPI_SCK_PIN   GPIO_Pin_5
#define SPI_MOSI    GPIOB
#define SPI_MOSI_PIN  GPIO_Pin_6
#define SPI_MISO    GPIOB
#define SPI_MISO_PIN  GPIO_Pin_7

//C55 as in LCD_cellphone.c
#define SIEM_CS  GPIOC								//RED: V+ 3v3
#define SIEM_CS_PIN  GPIO_Pin_3       //RED GREEN: CS
#define SIEM_SCK    GPIOC
#define SIEM_SCK_PIN   GPIO_Pin_6    //BLUE: clock
#define SIEM_MOSI    GPIOC
#define SIEM_MOSI_PIN  GPIO_Pin_5    //GREEN: data out
#define SIEM_DC GPIOC
#define SIEM_DC_PIN GPIO_Pin_4		//YELLOW: data/cmd switcher
#define SIEM_RES GPIOC 			    
#define SIEM_RES_PIN GPIO_Pin_2 //METAL: reset

//radio rec
#define RFREC    GPIOD					//radio signal receive port
#define RFREC_PIN  GPIO_Pin_7

//I2C as in i2c.c
#define I2C_SDA    GPIOD					//i2c data open drain
#define I2C_SDA_PIN  GPIO_Pin_6
#define I2C_SCL    GPIOD					//i2c clock open drain
#define I2C_SCL_PIN  GPIO_Pin_5

//sig SFM-1440 buzzer
#define SIG    GPIOD					//alarm signal
#define SIG_PIN  GPIO_Pin_1		//white to PD1,black to GND

*/
