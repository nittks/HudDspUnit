#ifndef LNK_OUT_LED_INC_H
#define LNK_OUT_LED_INC_H

#define CNTMAX		((unsigned char)10)	//8bit
#define CNT100MSMAX	((unsigned char)20)	//5ms���荞��20���100ms

#define DOT_ON( inData )	( inData|0x01)		//7�Z�O�̓_(dot)��\������

enum{
	LED_7SEG_0,
	LED_7SEG_1,
	LED_7SEG_2,
	LED_7SEG_3,
	LED_7SEG_4,
	LED_7SEG_5,
	LED_7SEG_6,
	LED_7SEG_7,
	LED_7SEG_8,
	LED_7SEG_9,
	LED_7SEG_BLANK,
	LED_7SEG_MAX
};

//7�Z�O�A�Z�O�����g�p�^�[��
static const unsigned char	led7SegBit[LED_7SEG_MAX]={
//���=a,����=.
	0b00111111,
	0b00000110,
	0b01011011,
	0b01001111,
	0b01100110,
	0b01101101,
	0b01111101,
	0b00000111,
	0b01111111,
	0b01100111,
	0b00000000,
};

#endif