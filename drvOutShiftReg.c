
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
static DRV_LED_BAR_DATA		drvLedBarData;
static DRV_LED_7SEG_DATA	drvLed7SegData;
static DRV_BITF_BYTE		drvLedAnode;
static DRV_BITF_SHIFT_REG	outShiftReg[LED_BAR_NUM];
static unsigned char		stateSegNo=0;
static unsigned char		preSetBright7seg;
static unsigned char		preSetBrightBarled;

unsigned long bitReverceSort( unsigned long inData , unsigned char size );

//********************************************************************************
// ������
//********************************************************************************
void initDrvOutShiftReg( void )
{
	unsigned char	i;
	
	stateSegNo	= 0;
	for( i=0; i<LED_7SEG_NUM ; i++ ){
		drvLed7SegData.data[i].data = 0;
	}
	for( i=0; i<LED_BAR_NUM ; i++ ){
		drvLedBarData.data[i].data = 0;
	}
	preSetBright7seg	= OCR1B_MAX;
	preSetBrightBarled	= OCR2A_MAX;

	SET_COM1B;
	SET_WGM1;
	SET_ICR1;

	SET_COM2A;
	SET_WGM2;

	SET_OCR1B( OCR1B_MAX );
	SET_OCR2A( OCR2A_MAX );

	//�V�t�g���W�X�^�o�͏�����
	SER_OFF;
	OE_ON;
	RCLK_OFF;
	SRCLK_OFF;
	SRCLR_OFF;
	
	START_PWM1;
	START_PWM2;
	
}

//********************************************************************************
// Lnk����Z�b�g
//********************************************************************************
void setDrvOutShiftRegLedBar( DRV_LED_BAR_DATA *inP )
{
	drvLedBarData = *inP;
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
//LED�o�́B16ms����(60FPS)��4�Z�O�A1�Z�O4ms�����ŏ���
void drvOutShiftRegMain( void )
{
//	static unsigned char	tmpspeed=0;
	unsigned char	i;
	DRV_BITF_SHIFT_REG	tmp;
	DRV_BITF_BYTE	tmp7seg[LED_7SEG_NUM];	
	DRV_BITF_10BIT	tmpLedBar[LED_BAR_NUM];
	
	//7�Z�O�̖��邳���ύX���ꂽ
	if( drvLed7SegData.bright != preSetBright7seg ){
		SET_OCR1B( drvLed7SegData.bright );
		preSetBright7seg = drvLed7SegData.bright;
	}
	//�o�[LED�̖��邳���ύX���ꂽ
	if( drvLedBarData.bright != preSetBrightBarled ){
		SET_OCR2A( drvLedBarData.bright );
		preSetBrightBarled = drvLedBarData.bright;
	}

	//1�Z�O��(�����̓�)��1��f�[�^���擾���A�o�̓o�b�t�@�փZ�b�g����
	if( stateSegNo == 0 ){
		
		//�Z�O�����g�f�[�^������ւ�
		for( i=0 ; i< LED_7SEG_NUM ; i++ ){
			tmp7seg[i].data		= drvLed7SegData.data[LED_7SEG_NUM-(i+1)].data;		
			tmpLedBar[i].data	= bitReverceSort( (unsigned long)drvLedBarData.data[LED_7SEG_NUM-(i+1)].data , LED_BAR_SEG );
		}
		//7�Z�O�ƃo�[LED�̃f�[�^���V���A���f�[�^�Ƃ��ėp��		
		for( i=0 ; i< LED_7SEG_NUM ; i++ ){
			outShiftReg[i].data	= (
				(((unsigned long)tmp7seg[i].data & 0x00FF) << 10) |
				(tmpLedBar[i].data & 0x3FF)
			);
		}
	}

	cli();	
	//��x�S����
	ALL_OFF;

	//�V�t�g���W�X�^�o��
	//1)�����l�Z�b�g
	RCLK_OFF;
	SRCLK_OFF;
	ALL_OFF;
	SRCLR_ON;
	ALL_OFF;
	WAIT_SREG;
	SRCLR_OFF;

	tmp.data = outShiftReg[stateSegNo].data;
	for( i=0 ; i<SER_DATA_NUM ; i++ ){
		//2)�f�[�^�Z�b�g
		//���ʂ���1bit���Z�b�g�B�Z�b�g�����猳�f�[�^�ϐ����V�t�g�B
		SER_SET( ~(tmp.data & 0x01) );		//1bit���Z�b�g
		tmp.data >>= 1;
		WAIT_SREG;

		//3)�N���b�Nposedge
		SRCLK_ON;
		WAIT_SREG;
		SRCLK_OFF;
		WAIT_SREG;
	}
	//4)2-3�����[�v
	//5)�o�̓|�[�g�փZ�b�g

	RCLK_ON;
	//�A�m�[�h�o��
	drvLedAnode.data	= (1 << stateSegNo);
	ANODE_SET( drvLedAnode.data );

	sei();
	
	//���̃Z�O�����g��
	stateSegNo++;
	if( stateSegNo >= LED_BAR_NUM ){
		stateSegNo = 0;
	}

}

//********************************************************************************
// �r�b�g���є��]
//********************************************************************************
unsigned long bitReverceSort( unsigned long inData , unsigned char size )
{
	//���<->���ʁA�ׂ荇��8bit�A4bit�A2bit�A1bit�̏��ɓ���ւ���
	inData = ((inData & 0x0000FFFF)<<16) | ((inData>>16) & 0x0000FFFF);
	inData = ((inData & 0x00FF00FF)<< 8) | ((inData>> 8) & 0x00FF00FF);
	inData = ((inData & 0x0F0F0F0F)<< 4) | ((inData>> 4) & 0x0F0F0F0F);
	inData = ((inData & 0x33333333)<< 2) | ((inData>> 2) & 0x33333333);
	inData = ((inData & 0x55555555)<< 1) | ((inData>> 1) & 0x55555555);

	//�t�]���ċ󂫂ƂȂ�������bit���l�߂�
	inData = inData >> ((sizeof(unsigned long)*8)-size);

	return( inData );
}




