/*
Receiver velo rx2 Discover Board
via SPI and NRF24L01
prepare
buzzer ok
test lis ERROR! PC1 fault?
PC1 fault!
fixed.works.
*/
//Includes ------------------------------------------------------------------*/

#include "stm8l15x.h"
#include "discover_board.h" //ok
#include "main.h"
#include "stdio.h"  //for virual com port
#include "my_interface.h" //i2c functions
#include "nrf24l01p.h"  //NRF24L01 radio module
#include "mydelay.h"	//delay,holds all the programm


//defines see in main.h

//Global Variables
volatile unsigned char fl,btnState,btnDelay;		//flags
volatile unsigned char rxreg;	//receive byte for UART
unsigned char rxdata[10];  //array of rec data from UART usartRx: 0,1..5 cells
unsigned char rxcnt=0; //rec data bytes counter UART usartRx
unsigned char cmd;		//collected data from UART usartRx as command
uint8_t address[5] = { 0xE2, 0xE4, 0x23, 0xE4, 0x02 };	//nrf address
unsigned char x=0,y=0,z=0;	//for lis302 axises
unsigned char x1=0,y1=0,z1=0;	//for accel read values
unsigned char dx=0,dy=0,dz=0;	//for lis delta count values

//Uid uid;

//Functions
void lis302_init(){
	i2cstart();
	txbyte2(0x38,1);		//dev adrr,write
	txbyte2(0x20,1);		//addr ctrl_reg1
	txbyte2(0x47,1);		//data
	//txbyte2(0x10,1);		//data for ctrl_reg2 filter on,doesnt work
	i2cstop();
}
//i2c init at velo_i2c.c
void testlis(){
	unsigned char data,i;
	//test existence of LIS302DL
	//lis302dl 'who_am_I' byte read
	i2cstart();
	txbyte2(0x38,1);		//we want to write
	txbyte2(0x0f,1);		//WHO AM I addr
	i2cstart();
	txbyte2(0x39,1);		//we want to read
	data=rxbyte2();
	nack();
	i2cstop();
	//data=59=0x3b
	if(data==0x3b){ //if data WHO AM I correct - blink
		for(i=0;i<5;i++){
			GPIO_HIGH(LED2_PORT,LED2_PIN);
			mydelay_ms(200);
			GPIO_LOW(LED2_PORT,LED2_PIN);
			mydelay_ms(200);
		}
	}
  else{
    usartMsg("Test lis ERROR!",1);
  }
}
void readlis_xyz(uint8_t *x,uint8_t *y,uint8_t *z){
	unsigned char data;
	//check rdy data from lis
	i2cstart();
	txbyte2(0x38,1);		//dev adrr,write
	txbyte2(0x27,1);		//addr status
	i2cstart();
	txbyte2(0x39,1);		//dev addr,read
	data=rxbyte2();
	nack();
	i2cstop();
	if((data & 0x08)==0x08){		//data rdy
		i2cstart();
		txbyte2(0x38,1);		//dev adrr,write
		txbyte2(0x29,1);		//Reg addr X
		i2cstart();
		txbyte2(0x39,1);		//dev addr,read
		*x=rxbyte2();
		nack();
		i2cstop();
		i2cstart();
		txbyte2(0x38,1);		//dev adrr,write
		txbyte2(0x2b,1);		//Reg addr Y
		i2cstart();
		txbyte2(0x39,1);		//dev addr,read
		*y=rxbyte2();
		nack();
		i2cstop();
		i2cstart();
		txbyte2(0x38,1);		//dev adrr,write
		txbyte2(0x2D,1);		//Reg addr Z
		i2cstart();
		txbyte2(0x39,1);		//dev addr,read
		*z=rxbyte2();
		nack();
		i2cstop();
	}
}
//gives abs delta value
//in:2 variables
//out: abs delta value
unsigned char delta(unsigned char d1,unsigned char d2){
	unsigned char dt;
	dt=d1-d2;
	if(dt>>7){
		dt=~dt+1;
	}
	return dt;
}
//reads accel data and checks treshold
//out: 1-overpeak delta 0-normal delta value
unsigned char readcheckAccel(){
	unsigned char data;
	//check rdy data from lis
	i2cstart();
	txbyte2(0x38,1);		//dev adrr,write
	txbyte2(0x27,1);		//addr status
	i2cstart();
	txbyte2(0x39,1);		//dev addr,read
	data=rxbyte2();
	nack();
	i2cstop();
	if((data & 0x08)==0x08){		//data rdy
		i2cstart();
		txbyte2(0x38,1);		//dev adrr,write
		txbyte2(0x29,1);		//Reg addr X
		i2cstart();
		txbyte2(0x39,1);		//dev addr,read
		x=rxbyte2();
		nack();
		i2cstop();
		i2cstart();
		txbyte2(0x38,1);		//dev adrr,write
		txbyte2(0x2b,1);		//Reg addr Y
		i2cstart();
		txbyte2(0x39,1);		//dev addr,read
		y=rxbyte2();
		nack();
		i2cstop();
		i2cstart();
		txbyte2(0x38,1);		//dev adrr,write
		txbyte2(0x2D,1);		//Reg addr Z
		i2cstart();
		txbyte2(0x39,1);		//dev addr,read
		z=rxbyte2();
		nack();
		i2cstop();
		//set delta
		if(fl&FTLISDT){			//if fist time is set - count delta
			dx=delta(x1,x);
			dy=delta(y1,y);
			dz=delta(z1,z);
		}
		x1=x;
		y1=y;
		z1=z;
		fl|=FTLISDT;	//set flag first time 
	}
	if((dx>PEAK)|(dy>PEAK)|(dz>PEAK)){
		return 1;		//over PEAK delta
	}
	else
		return 0;		//normal delta
}
/*
//craft spi
unsigned char nrf24l01p_spi_rw(uint8_t data_out){
	unsigned char data_in;
  while ((SPI1->SR&SPI_FLAG_TXE) != SPI_FLAG_TXE);   		//TXE wait to be empty
  SPI1->DR = data_out;
  while ((SPI1->SR&SPI_FLAG_TXE) != SPI_FLAG_TXE);   		//TXE
  while ((SPI1->SR&SPI_FLAG_RXNE) != SPI_FLAG_RXNE);   	//RXNE
  data_in=SPI1->DR;
	return data_in;
}
//craft ss
//SS_PIN = CSN, CE_PIN= CE
void nrf24l01p_spi_ss(nrf24l01p_spi_ss_level_t level)
{
  if(level==NRF24L01P_SPI_SS_LOW)
    GPIO_LOW(NRF_CSN,NRF_CSN_PIN);
  else
    GPIO_HIGH(NRF_CSN,NRF_CSN_PIN);
}
//NRF another proc
//NRF communication procedure
//SPI transmitt and receive
unsigned char SPI_rw(unsigned char data_out){
	unsigned char data_in;
  while ((SPI1->SR&SPI_FLAG_TXE) != SPI_FLAG_TXE);   		//TXE wait to be empty
  SPI1->DR = data_out;
  while ((SPI1->SR&SPI_FLAG_TXE) != SPI_FLAG_TXE);   		//TXE
  while ((SPI1->SR&SPI_FLAG_RXNE) != SPI_FLAG_RXNE);   	//RXNE
  data_in=SPI1->DR;
	return data_in;
}
//write multiple
unsigned char SPI_Write_Buf(unsigned char reg,unsigned char *pBuf,unsigned char bytes){
	GPIO_LOW(NRF_CSN,NRF_CSN_PIN);
	unsigned char status,i;
	status=SPI_rw(reg);
	for(i=0;i<bytes;i++){
		SPI_rw(*pBuf++);
	}
	asm("nop");
	GPIO_HIGH(NRF_CSN,NRF_CSN_PIN);
	return status;
}
//read multiple
unsigned char SPI_Read_Buf(unsigned char reg,unsigned char *pBuf,unsigned char bytes){
	GPIO_LOW(NRF_CSN,NRF_CSN_PIN);
	unsigned char status,i;
	status=SPI_rw(reg);
	for(i=0;i<bytes;i++){
		pBuf[i]=SPI_rw(0);		//read to pBuf
	}
	asm("nop");
	GPIO_HIGH(NRF_CSN,NRF_CSN_PIN);
	return status;
}
//write reg
void SPI_RW_Reg(unsigned char reg,unsigned char value){
	GPIO_LOW(NRF_CSN,NRF_CSN_PIN);
	SPI_rw(reg);
	SPI_rw(value);
	asm("nop");
	GPIO_HIGH(NRF_CSN,NRF_CSN_PIN);
}
//read reg
unsigned char SPI_Read(unsigned char reg){
	unsigned char value;
	GPIO_LOW(NRF_CSN,NRF_CSN_PIN);
	SPI_rw(reg);
	value=SPI_rw(0);
	asm("nop");
	GPIO_HIGH(NRF_CSN,NRF_CSN_PIN);
	return value;
}
*/
//USART
/*
//usart byte to symbols
//convert byte to digit sequence,NO CR LF!
void usartShowByteDec(unsigned char byte){
	unsigned char digit[3],cnt;
	digit[0]=(byte/100)+0x30;			//msb
	digit[1]=(byte/10%10)+0x30;
	digit[2]=(byte%10)+0x30;			//lsb
	unsigned char *digitptr;
	digitptr=&digit[0];	//takes addressof first element array
	for(cnt=0;cnt<3;cnt++){
		usartTx(digitptr++);	//increase address
	}
  USART_SendData8(USART1,0x20);	//SPACE
  while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
}
//in:byte to convert byte to digit sequence,CR LF included
//out:usart byte to hexsymbols: 0x23 is 23,0xff is ff
void usartShowByteHex(unsigned char byte){
	unsigned char digit[2],cnt;
	cnt=byte>>4;		//0x00..0x09,A
	if(cnt>=0x0a)
		digit[0]=cnt-0x0a+0x41;			//msb
	else
		digit[0]=cnt+0x30;			//msb
	cnt=byte&0x0f;		//0x00..0x09,A
	if(cnt>=0x0a)
		digit[1]=cnt-0x0a+0x41;			//msb
	else
		digit[1]=cnt+0x30;			//msb
	unsigned char *digitptr;
	cnt=0x30;
	digitptr=&cnt;
	usartTx(digitptr);	//
	cnt=0x78;
	digitptr=&cnt;
	usartTx(digitptr);	//
	digitptr=&digit[0];	//takes addressof first element array
	for(cnt=0;cnt<2;cnt++){
		usartTx(digitptr++);	//increase address
	}
  USART_SendData8(USART1,0x20);	//SPACE
  while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
}
//uses address pointer
void usartTx(unsigned char *sendbyte){
  while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
  USART_SendData8(USART1,(unsigned char) *sendbyte);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
}
//message send:if enter=1 adds CR LF,else no CRLF
void usartMsg(unsigned char *msg,unsigned char enter){
  unsigned char * tmp=msg;
  while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
  while(*tmp){
    USART_SendData8(USART1,(unsigned char)*tmp);
    while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
    tmp++;
  }
	if(enter){
	  USART_SendData8(USART1,0xd);	//CR
	  while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	  USART_SendData8(USART1,0xa);	//LF
	  while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	}
}
*/
//craft
void nrf_init2(uint8_t *address,unsigned char nrfmode){
	//init
	mydelay_ms(200);
	GPIO_LOW(NRF_CE,NRF_CE_PIN);
	if(nrfmode==MODETX){
		nrf24l01p_get_clear_irq_flags();
   	nrf24l01p_close_pipe(NRF24L01P_ALL);
		nrf24l01p_open_pipe(NRF24L01P_TX, FALSE);
   	nrf24l01p_set_rx_payload_width(NRF24L01P_PIPE0, NRF_PAYLOAD_LENGTH);
 		nrf24l01p_set_crc_mode(NRF24L01P_CRC_16BIT);
   	nrf24l01p_set_address_width(NRF24L01P_AW_5BYTES);
   	nrf24l01p_set_address(NRF24L01P_TX, address);
		nrf24l01p_set_address(NRF24L01P_PIPE0, address);
   	nrf24l01p_set_operation_mode(NRF24L01P_PTX);
   	nrf24l01p_set_rf_channel(NRF_CHANNEL);
 		nrf24l01p_set_power_mode(NRF24L01P_PWR_UP);
   	mydelay_ms(NRF_POWER_UP_DELAY);
		//usartMsg("MODETX configured\r\n");
    //
		//usartMsg("TRANSM selected\r\n");
	}
	if(nrfmode==MODERX){
		nrf24l01p_get_clear_irq_flags();
		nrf24l01p_close_pipe(NRF24L01P_ALL);
   	nrf24l01p_open_pipe(NRF24L01P_PIPE0, FALSE);
		nrf24l01p_set_crc_mode(NRF24L01P_CRC_16BIT);
		nrf24l01p_set_address_width(NRF24L01P_AW_5BYTES);
		nrf24l01p_set_address(NRF24L01P_TX, address);
		nrf24l01p_set_address(NRF24L01P_PIPE0, address);
		nrf24l01p_set_operation_mode(NRF24L01P_PRX);
		nrf24l01p_set_rx_payload_width(NRF24L01P_PIPE0, NRF_PAYLOAD_LENGTH);
		nrf24l01p_set_rf_channel(NRF_CHANNEL);
		nrf24l01p_set_power_mode(NRF24L01P_PWR_UP);
		mydelay_ms(NRF_POWER_UP_DELAY);
		mydelay_ms(NRF_POWER_UP_DELAY);
		GPIO_HIGH(NRF_CE,NRF_CE_PIN);
		//usartMsg("MODERX configured\r\n");
		//
		//usartMsg("REC selected\r\n");
	}
	if(nrfmode==MODETXA){
		nrf24l01p_get_clear_irq_flags();
   	nrf24l01p_close_pipe(NRF24L01P_ALL);
		nrf24l01p_open_pipe(NRF24L01P_PIPE0,TRUE);
   	nrf24l01p_set_crc_mode(NRF24L01P_CRC_16BIT);
		nrf24l01p_set_auto_retr(RF_RETRANSMITS, RF_RETRANS_DELAY); //retransm on
   	nrf24l01p_set_address_width(NRF24L01P_AW_5BYTES);	//addr
   	nrf24l01p_set_address(NRF24L01P_TX, address);		//addr
		nrf24l01p_set_address(NRF24L01P_PIPE0, address);	//addr
		
		//nrf24l01p_set_rx_payload_width(NRF24L01P_PIPE0, NRF_PAYLOAD_LENGTH);
   	nrf24l01p_set_operation_mode(NRF24L01P_PTX);
		
   	nrf24l01p_set_rf_channel(NRF_CHANNEL);
 		nrf24l01p_set_power_mode(NRF24L01P_PWR_UP);
   	mydelay_ms(NRF_POWER_UP_DELAY);
		mydelay_ms(NRF_POWER_UP_DELAY);
		//usartMsg("MODETXA configured\r\n");
		//
		//usartMsg("TRANSM selected\r\n");
	}
	if(nrfmode==MODERXA){
		nrf24l01p_get_clear_irq_flags();
   	nrf24l01p_close_pipe(NRF24L01P_ALL);
		nrf24l01p_open_pipe(NRF24L01P_PIPE0, TRUE);
   	nrf24l01p_set_crc_mode(NRF24L01P_CRC_16BIT);
		nrf24l01p_set_auto_retr(RF_RETRANSMITS, RF_RETRANS_DELAY); //retransm on
   	nrf24l01p_set_address_width(NRF24L01P_AW_5BYTES);	//addr
   	nrf24l01p_set_address(NRF24L01P_TX, address);		//addr
		nrf24l01p_set_address(NRF24L01P_PIPE0, address);	//addr
		
		nrf24l01p_set_operation_mode(NRF24L01P_PRX);
		nrf24l01p_set_rx_payload_width(NRF24L01P_PIPE0, NRF_PAYLOAD_LENGTH);
		
   	nrf24l01p_set_rf_channel(NRF_CHANNEL);
 		nrf24l01p_set_power_mode(NRF24L01P_PWR_UP);
		mydelay_ms(NRF_POWER_UP_DELAY);
		mydelay_ms(NRF_POWER_UP_DELAY);
		GPIO_HIGH(NRF_CE,NRF_CE_PIN);
		//usartMsg("MODERXA configured\r\n");
		//
		//usartMsg("REC selected\r\n");
	}
} //end of nrf_init2

//func to trigger nrf mode
void RX_mode(void){	//receiver mode:
	if(fl&RXF){			//first time rx_mode run
		fl&=~RXF;	//clear flag
		nrf_init2(address,MODERX);	//change to receive mode
		usartMsg("Changed to Receive mode",1);
	}
	//if received radio data,show it via com port
	if(nrf24l01p_get_irq_flags() & (1 << NRF24L01P_IRQ_RX_DR)){
	  nrf24l01p_clear_irq_flag(NRF24L01P_IRQ_RX_DR);
	  unsigned char payload[NRF_PAYLOAD_LENGTH];
	  nrf24l01p_read_rx_payload((uint8_t*)payload);//, sizeof(payload));
		usartMsg("Received from transmitter",0);
	  unsigned char k;
		for(k=0; k<NRF_PAYLOAD_LENGTH; k++){
      //show data to uart
			//usartShowByteDec(payload[k]);								//print rx buf
			usartTx(&payload[k]);
		}
  	usartMsg(".",1);
		//flush buffer?
	}
	//if we have data from com to radio send
	if(fl&COMRX){
		fl&=~COMRX;	//clear com data from user flag
		fl|=RXF;	//clear flag to reinitialize rx mode
		nrf_init2(address,MODETX);	//change mode to TX
		static uint8_t payload[NRF_PAYLOAD_LENGTH];
		unsigned char i;
		for(i=0;i<NRF_PAYLOAD_LENGTH;i++){
			payload[i] = rxdata[i];	//fill tx buffer
			rxdata[i]=0x20;	//clear array: fill with spaces
		}
    //send data
		nrf24l01p_write_tx_payload(payload,NRF_PAYLOAD_LENGTH);
		GPIO_HIGH(NRF_CE,NRF_CE_PIN);	//on
		mydelay_ms(10);
		GPIO_LOW(NRF_CE,NRF_CE_PIN);		//and off CE
    //end of sending,wait for ack
		unsigned char timecnt=15;
		do{
			timecnt--;
			mydelay_ms(1);
		} while ((timecnt!=0)&&(!(nrf24l01p_get_irq_flags() & (1 << NRF24L01P_IRQ_TX_DS))));
		nrf24l01p_clear_irq_flag(NRF24L01P_IRQ_TX_DS);
		usartMsg("NRF TX send",1);
	}
}
//timer4: clock 2000000Hz/Prescaler(1-32768)or(2^0 - 2^15)/value ARR=Ftmr4
void tmr4_init(void){
	CLK->PCKENR1|=0x04; //enable TIM4 clock
  TIM4_DeInit();
  //TIM4_TimeBaseInit(TIM4_Prescaler_32768,100); // every 1.6sec
	//TIM4_TimeBaseInit(TIM4_Prescaler_32768,6); // every 98ms=almost 100ms
	TIM4_TimeBaseInit(TIM4_Prescaler_16384,1); // every almost 10ms
	//TIM4_TimeBaseInit(TIM4_Prescaler_1,1); // every xx us
	TIM4_SetCounter(0x00); //reset counter
  TIM4_ITConfig(TIM4_IT_Update,ENABLE);
  //TIM4_Cmd(ENABLE);		//for button handle
	TIM4_Cmd(DISABLE);
}
//blink :2000000(sysclock)/200(prescaler)=10000/100(Arr)=100Hz=10ms
//blink :2000000(sysclock)/200(prescaler)=10000/5(Arr)=2000Hz=0.5ms
//blink :2000000(sysclock)/200(prescaler)=10000/2(Arr)=5000Hz=0.2ms
//blink :2000000(sysclock)/200(prescaler)=10000/10000(Arr)=1Hz=1000ms
void tmr1_init(void){
	CLK->PCKENR2|=0x02;   //enable TIM1 clock
  TIM1->PSCRH=0;
  TIM1->PSCRL=199;  //prescaler
	TIM1->ARRH=(1000)>>8;  //ARR High
  TIM1->ARRL=(1000)& 0xff;   //ARR low
  TIM1->EGR|=TIM1_EGR_UG;	//update generation,reinit counter
  TIM1->CR1|=TIM1_CR1_URS;
  TIM1->CR1&=~TIM1_CR1_DIR;	//
  TIM1->IER|=TIM1_IER_UIE;
 	//TIM1->CR1|=TIM1_CR1_CEN;  //enable TMR1
	TIM1->CR1&=~TIM1_CR1_CEN;  //disable TMR1
}
void init(){
  //init
  //CLK->CKDIVR|=0x03;//sysclock prescaler sysclocksource/8	= 2Mhz by default
  //CLK->CKDIVR|=0x04;//sysclock prescaler sysclocksource/16 1Mhz
	GPIO_Init(LED1_PORT, LED1_PIN, GPIO_Mode_Out_PP_Low_Slow);
  GPIO_Init(LED2_PORT, LED2_PIN, GPIO_Mode_Out_PP_Low_Slow);
	GPIO_Init(BTN_PORT,BTN_PIN,GPIO_Mode_In_FL_No_IT);
  GPIO_Init(ALM_PORT, ALM_PIN, GPIO_Mode_Out_PP_Low_Fast);
  GPIO_Init(I2C_CS, I2C_CS_PIN, GPIO_Mode_Out_PP_High_Slow); //1-i2c,0-spi
  GPIO_Init(I2C_ADR, I2C_ADR_PIN, GPIO_Mode_Out_PP_Low_Fast); //lsb addr i2c
  //
  EXTI->CR1|=0x08;	//falling edge only	1<<3 for button
	//SPI init NRF24L01
	SYSCFG_REMAPPinConfig(REMAP_Pin_SPI1Full,ENABLE);  //remap SPI to PC,PA if set
  GPIO_Init(NRF_CE,NRF_CE_PIN,GPIO_Mode_Out_PP_Low_Fast);	//NRF tx rx activation
  GPIO_Init(NRF_CSN,NRF_CSN_PIN,GPIO_Mode_Out_PP_High_Fast);	//NRF chip select,act low
  GPIO_Init(SPI_MOSI,SPI_MOSI_PIN,GPIO_Mode_Out_PP_High_Fast);
  GPIO_Init(SPI_SCK,SPI_SCK_PIN,GPIO_Mode_Out_PP_High_Fast);
  GPIO_Init(SPI_MISO,SPI_MISO_PIN,GPIO_Mode_In_FL_No_IT); //PU as variant
  SPI_DeInit(SPI1);
  CLK->PCKENR1|=0x10; //enable clock for SPI1
  SPI_Init(
					 SPI1,SPI_FirstBit_MSB,SPI_BaudRatePrescaler_64,			//32us pulse 32kHz
					 SPI_Mode_Master,SPI_CPOL_Low,SPI_CPHA_1Edge,	//
					 SPI_Direction_2Lines_FullDuplex,SPI_NSS_Soft,0x00
						 );
  SPI_Cmd(SPI1,ENABLE);  //disable while uart acts
  //USART
  GPIO_Init(USART_TX,USART_TX_PIN,GPIO_Mode_Out_PP_Low_Fast);  //TX as out
  GPIO_Init(USART_RX,USART_RX_PIN,GPIO_Mode_In_FL_No_IT);  //RX as In
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1,ENABLE);
  //SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortA,ENABLE);  //remap usart to PA if set
  USART_DeInit(USART1);
  USART_Init(
						 USART1,9600,USART_WordLength_8b,USART_StopBits_1,
						 USART_Parity_No,USART_Mode_Tx|USART_Mode_Rx);		//
  //USART_ITConfig(USART1,USART_IT_TXE,ENABLE); //enable Tx int
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);  //enable Rx int
  USART_Cmd(USART1,ENABLE);
}
//tasks
//dd- delay of 10us:32..56 sounds good. 12..13 good peep
void buzzer(unsigned char dd){
	unsigned char ii;
	for(ii=0;ii<10;ii++){
		GPIO_HIGH(ALM_PORT,ALM_PIN);
		mydelay_10us(dd);
		GPIO_LOW(ALM_PORT,ALM_PIN);
		mydelay_10us(dd);
		GPIO_HIGH(ALM_PORT,ALM_PIN);
		mydelay_10us(dd);
		GPIO_LOW(ALM_PORT,ALM_PIN);
		mydelay_10us(dd);
		GPIO_HIGH(ALM_PORT,ALM_PIN);
		mydelay_10us(dd);
		GPIO_LOW(ALM_PORT,ALM_PIN);
		mydelay_10us(dd);
		GPIO_HIGH(ALM_PORT,ALM_PIN);
		mydelay_10us(dd);
		GPIO_LOW(ALM_PORT,ALM_PIN);
		mydelay_10us(dd);
		GPIO_HIGH(ALM_PORT,ALM_PIN);
		mydelay_10us(dd);
		GPIO_LOW(ALM_PORT,ALM_PIN);
		mydelay_10us(dd);
		GPIO_HIGH(ALM_PORT,ALM_PIN);
		mydelay_10us(dd);
		GPIO_LOW(ALM_PORT,ALM_PIN);
		mydelay_10us(dd);
	}
}
void alarm(){
	buzzer(32);
	buzzer(37);
	buzzer(40);
	buzzer(32);
	buzzer(37);
	buzzer(40);
	mydelay_ms(300);
}
void confirm(unsigned char mode){
  if(mode==1){
    buzzer(32);
    buzzer(37);
    buzzer(40);
    buzzer(32);
    buzzer(37);
    buzzer(40);
  }
  if(mode==2){
    buzzer(50);
    buzzer(45);
    buzzer(32);
    buzzer(50);
    buzzer(45);
    buzzer(32);
  }
}
//MAIN
void main(void){
	fl=0;   //clear all flags
	init();				//base cpu,ports init
	i2cinit();		//i2c ports and reset seq init
	GPIO_HIGH(I2C_CS,I2C_CS_PIN);
	GPIO_LOW(I2C_ADR,I2C_ADR_PIN);
	tmr1_init();
	tmr4_init();
  nrf_init2(address,MODERX);	//init by default
  lis302_init();
	GPIO_HIGH(LED1_PORT,LED1_PIN);
	mydelay_ms(200);
	GPIO_LOW(LED1_PORT,LED1_PIN);
  mydelay_ms(200);
  GPIO_HIGH(LED2_PORT,LED2_PIN);
	mydelay_ms(200);
	GPIO_LOW(LED2_PORT,LED2_PIN);
  
	//TIM4_Cmd(ENABLE);
	//TIM1->CR1|=TIM1_CR1_CEN;  //enable TMR1
	//enableInterrupts();
	mydelay_ms(1000);
	usartMsg("NRF24L01 VELO ALARM v03",1);
	//test sequences
	testlis();
	usartMsg("UART Test message",1);
	fl|=RXF;	//set first time flag for RXF
  unsigned char xx,yy,zz; //lis302 axis data
	while(1){
    /*
		readlis_xyz(&xx,&yy,&zz);
		usartMsg("X axis:",0);
		usartShowByteDec(xx);
		usartMsg("Y axis:",0);
		usartShowByteDec(yy);
		usartMsg("Z axis:",0);
		usartShowByteDec(zz);
		usartMsg(" ",1);
		if(readcheckAccel()){
			usartMsg("PEAK exceeded! ALARM!",1);
      //alarm();
		}
    */
		//mydelay_ms(500);
		RX_mode();
	}
}
//----------------------------------------------------------------