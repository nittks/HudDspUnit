
/*
SRCLK		�V�t�g���W�X�^�N���b�N�BFF�֎�荞��
SRCLR		�N���A�BFF�����Z�b�g����
SER			�f�[�^
RCLK		�o�͊m��B�o��FF�̃N���b�N
OE			�o�̓|�[�g�n�C�C���s�[
*/

#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "drvOutShiftReg_inc.h"
#include "drvOutShiftReg.h"
#include "hardware.h"

//LINK���Z�b�g�����
static DRV_LED_7SEG_DATA	drvLed7SegData;
static unsigned char		stateSegNo=0;

unsigned long bitReverceSort( unsigned long inData , unsigned char size );

//********************************************************************************
// ������
//********************************************************************************
void initDrvOutShiftReg( void )
{
	unsigned char	i;
	
	stateSegNo	= 0;
	for( i=0; i<LED_7SEG_DIGIT_NUM ; i++ ){
		drvLed7SegData.val[i] = 0;
	}
	drvLed7SegData.brightRed	= 0;
	drvLed7SegData.brightGreen	= 0;
	drvLed7SegData.brightBlue	= 0;

	//�V�t�g���W�X�^�o�͏�����
	SER_OFF;
//	OE_ON;
	RCLK_OFF;
	SRCLK_OFF;

	SRCLR_ON;
	WAIT_SREG;
	SRCLR_OFF;	
	
//	START_PWM1;
//	START_PWM2;
	
}


//********************************************************************************
// Lnk����Z�b�g
//********************************************************************************
void setDrvOutShiftReg7seg( DRV_LED_7SEG_DATA *inP )
{
	drvLed7SegData = *inP;
}

//********************************************************************************
// ���C������
//********************************************************************************
void drvOutShiftRegMain( void )
{
//	static unsigned char	tmpspeed=0;
	unsigned char	i,j;
	unsigned char	sendData[CPLD_SEND_DATA_NUM];
	unsigned char	sendDataByte;

	for( i=0 ; i<LED_7SEG_DIGIT_NUM ; i++ ){
		sendData[i]	= drvLed7SegData.val[i];
	}

	sendData[i++]	=	drvLed7SegData.brightRed;
	sendData[i++]	=	drvLed7SegData.brightGreen;
	sendData[i++]	=	drvLed7SegData.brightBlue;

	cli();	
	//�V�t�g���W�X�^�o��
	for( i=0 ; i<CPLD_SEND_DATA_NUM ; i++ ){
		sendDataByte	= sendData[i];
		for( j=0 ; j<sizeof(char)*8 ; j++ ){		//8bit�̗ǂ���`���@�͂Ȃ����H
			//2)�f�[�^�Z�b�g
			//��ʂ���1bit���Z�b�g�B�Z�b�g�����猳�f�[�^�ϐ����V�t�g�B
			SER_SET( sendDataByte >> 7 );		//1bit���Z�b�g
			sendDataByte	<<= 1;
			WAIT_SREG;

			//3)�N���b�Nposedge
			SRCLK_ON;
			WAIT_SREG;
			SRCLK_OFF;
			WAIT_SREG;
		}
	}
	//4)2-3�����[�v
	//5)�o�̓|�[�g�փZ�b�g
	RCLK_ON;
	WAIT_SREG;
	RCLK_OFF;

	sei();

}

