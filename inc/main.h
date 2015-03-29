#ifndef __MAIN_H
#define __MAIN_H
//fl bits,my
#define COMRX 0x01		//set if com data from user rdy
#define TXF 0x02			//first time flag
#define RXF 0x04			//
#define TXPLF 0x08		//transmitt with ack payload
#define RXPLF 0x10		//receive with ack payload
#define FTLISDT 0x20	//first time data from lis302 flag

//mode of transciever
#define TRANSM 0x00		//transmitter mode
#define REC 0x01			//receiver mode
//mode of alarm
#define IDLE 0   //at startup
#define ARMED 1  //read accelerometer,compare with treshold
#define ALARM 2   //ALARM signal!
//data for accelerometer
#define PEAK 2		//delta peak value critical for alarm

//for NRF
#define  NRF_CHANNEL 120
#define  NRF_POWER_UP_DELAY 4
#define  NRF_PAYLOAD_LENGTH 3		//also buffer for com data
#define RF_RETRANSMITS 5
#if (RF_PAYLOAD_LENGTH <= 18)
#define RF_RETRANS_DELAY 250
#else
#define RF_RETRANS_DELAY 500
#endif
//func nrfmodes
#define MODETX 0		//transmitt wo ack
#define MODERX 1		//receive wo ack
#define MODETXA 2		//transmitt with ack
#define MODERXA 3
#define MODETXB 4		//transmitt with data back
#define MODERXB 5

//base funcs
void tmr1_init();
void tmr4_init();
void init();
//variable functions
void nrf_init2(unsigned char*,unsigned char);
//void usartTx(unsigned char *);
//void usartMsg(unsigned char *,unsigned char);
//void usartShowByte(unsigned char);
unsigned char delta(unsigned char d1,unsigned char d2);
#endif