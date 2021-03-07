#ifndef DRV_OUT_SHIFT_REG_INC_H
#define DRV_OUT_SHIFT_REG_INC_H

#define CNTMAX		((unsigned char)10)	//8bit
#define CNT100MSMAX	((unsigned char)20)	//5ms���荞��20���100ms

//bitPos 0-7
//********************************************************************************
// �V�t�g���W�X�^
//********************************************************************************
#define	CPLD_SEND_DATA_NUM	6


//�o��bit�|�W�V����
#define	SRCLR_POS	3
#define	SRCLK_POS	4
#define	RCLK_POS	5
#define	OE_POS		1
#define	SER_POS		2

//�J�\�[�h���e�|�[�g
#define	SHIFT_REG_PORT	PORTC.IN
#define	SRCLR_ON	(SHIFT_REG_PORT &= ~(1<<SRCLR_POS))	//Low�A�N�e�B�u
#define	SRCLR_OFF	(SHIFT_REG_PORT |=  (1<<SRCLR_POS))
#define	SRCLK_ON	(SHIFT_REG_PORT |=  (1<<SRCLK_POS))
#define	SRCLK_OFF	(SHIFT_REG_PORT &= ~(1<<SRCLK_POS))
#define	RCLK_ON		(SHIFT_REG_PORT |=  (1<<RCLK_POS))
#define	RCLK_OFF	(SHIFT_REG_PORT &= ~(1<<RCLK_POS))
#define	OE_ON		(SHIFT_REG_PORT &= ~(1<<OE_POS))	//Low�A�N�e�B�u
#define	OE_OFF		(SHIFT_REG_PORT |=  (1<<OE_POS))
#define	SER_ON		(SHIFT_REG_PORT |=  (1<<SER_POS))
#define	SER_OFF		(SHIFT_REG_PORT &= ~(1<<SER_POS))
#define	SER_DATA_NUM	((unsigned char)18)		//1�t���[���ŃV�t�g���W�X�^�փZ�b�g����f�[�^bit��
#define	SER_SET( data )	(SHIFT_REG_PORT = ((SHIFT_REG_PORT & (~(1<<SER_POS))) | ((data&0x01)<<SER_POS)))
												//SHIFT_REG_PORT-SER_POS bit��data���Z�b�g����
//�A�m�[�h��
#define	ANODE_PORT	PORTC
#define	ANODE_PORT_MASK		0xF0		//PORTC��ʂ͓��́A�����v���A�b�v�̂���1�ݒ���ێ�
#define	ANODE_POS	0
#define	ANODE_SET( data )	(ANODE_PORT = (ANODE_PORT_MASK | (data << ANODE_POS)))
#define	ALL_OFF				(ANODE_PORT &= (ANODE_PORT_MASK & 0xF0))		//�A�m�[�h����Low�ɂ���(PB4-7

#define	WAIT_SREG	(_delay_us( 1 ))	//�M���ω���̃E�F�C�g


//********************************************************************************
// PWM
//********************************************************************************
//����
#define	TOP				0xFF	//TOP�l=255

//7�Z�O�pPWM�ݒ� �^�C�}1B
#define	REG_WGM1		0x5		//8bit����PWM����
#define	REG_COM1B		0x2		//�񔽓]����
#define	REG_CS1			0x3		//8����(8MHz/8=1Mhz
#define	REG_ICR1		TOP
#define	OCR1B_MAX		TOP

#define	SET_COM1B			(TCCR1A |= (REG_COM1B&0x03)<<COM1B0)
#define	SET_WGM1			(TCCR1A |= (REG_WGM1&0x03)<<WGM10);(TCCR1B |= ((REG_WGM1>>2)<<WGM12))

#define START_PWM1			(TCCR1B|=  (REG_CS1<<CS10))
#define STOPT_PWM1			(TCCR1B&= ~(REG_CS1<<CS10))

#define SET_ICR1			(ICR1H |= REG_ICR1>>8);(ICR1L = REG_ICR1&0x0F)
#define	SET_OCR1B( data )	(OCR1B = (((unsigned short)data*OCR1B_MAX)/100))


//�o�[LED�pPWM�ݒ� �^�C�}2A
#define	REG_WGM2		0x3		//8bit����PWM����
#define	REG_COM2A		0x2		//�񔽓]����
#define	REG_CS2			0x3		//8����(8MHz/8=1Mhz
#define REG_ICR2		TOP
#define	OCR2A_MAX		TOP

#define	SET_COM2A		(TCCR2A |= (REG_COM2A&0x03)<<COM2A0)
#define	SET_WGM2		(TCCR2A |= (REG_WGM2&0x03)<<WGM20);(TCCR2B = ((REG_WGM2>>2)<<WGM22))

#define START_PWM2		(TCCR2B|=  (REG_CS2<<CS20))
#define STOPT_PWM2		(TCCR2B&= ~(REG_CS2<<CS20))

//#define SET_ICR2		((ICR2 = REG_ICR1>>8);(ICR1L = REG_ICR1&0x0F))
#define	SET_OCR2A( data )	(OCR2A = (((unsigned long)data*OCR2A_MAX)/100))

#endif
