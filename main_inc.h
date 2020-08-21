#ifndef MAIN_INC_H
#define MAIN_INC_H

typedef struct{
	unsigned char	regist;			//�o�^��ԁB�^�X�N���L����������
	unsigned short	currentTime;	//���ݎ��ԁB0�ɂȂ�����^�X�N�N��
	unsigned short	cycleTime;		//�^�X�N�����B
	void			(*func)(void);
}TASK_PARAMETER;


static void powerLed( void );
#define	TASK_NUM	7
TASK_PARAMETER	taskParameter[TASK_NUM]	={
	//���ݎ���(�J�n���I�t�Z�b�g) , ���� , �֐���
	{	true,	0,	10,	drvInMain	},
	{	true,	1,	10,	lnkInMain	},
	{	true,	2,	10,	aplMain		},
	{	true,	3,	10,	lnkOutMain	},
	{	true,	4,	4,	drvOutMain	},
	{	true,	5,	250,powerLed	},
	{	false,	0,	2,	drvUartChangeTx}		//UART�h���C�o����̗v���ɂ��L��������
};

//CS0 �N���b�N�I��0
enum{
	CS0_STOP	= 0,
	CS0_DIV_NO,
	CS0_DIV_8,
	CS0_DIV_64,
	CS0_DIV_256,
	CS0_DIV_1024,
	CS0_DIV_TODOWN,
	CS0_DIV_T0UP
};


//WGM0 �g�`�������
enum{
	WGM_NORMAL	=0,
	WGM_8BIT_PHASE_BASE_PWM,
	WGM_COMP_CTC,
	WGM_8BIT_HIGHT_SPEED_PWM,
	WGM_RESERVE,
	WGM_PHASE_BASE_PWM,
	WGM_RESERVE1,
	WGM_HIGHT_SPEED_PWM
};

//COM0A ��rA�o�͑I��
enum{
	COM0A_NORMAL	= 0,
	COM0A_COMP_TOGLE,
	COM0A_COMP_LOW,
	COM0A_COMP_HIGH
};

//���W�X�^�ݒ�
#define		REG_CS0		(CS0_DIV_64 & 0x07)	//3bit
#define		REG_WGM		(WGM_COMP_CTC & 0x07)		//3bit
#define		REG_COM0A	(COM0A_NORMAL & 0x03)	//2bit
#define		TIMER_TOP	125		//8us*125=1ms,1ms�����荞��
#define		REG_OCIE0A	1		//��r��v�����݋���
#define		REG_CAL		171		//�����N���b�N�Z��(�f�t�H���g171)


//���W�X�^�Z�b�g�p
#define		SET_TCCR0A	(( REG_COM0A << COM0A0 ) | ((REG_WGM & 0x03) << WGM00))	//WGM00,01�̂�
#define		SET_TCCR0B	(( (REG_WGM >> 2) << WGM02) | (REG_CS0 << CS00))
#define		SET_OCR0A	TIMER_TOP
#define		SET_TIMSK0	(( REG_OCIE0A << OCIE0A))
#define		SET_OSCCAL	REG_CAL

#endif
