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
// main_inc.h�ɋL�q�����sim���s���ɒl���Z�b�g����Ȃ��Ȃ������߁A.c�ֈړ�
// IEDver���ς�������߁HAtmelStudio6->7
// �܂��A�ϐ���.h�ɒ�`����̂͗ǂ��Ȃ��Ƃ̂��ƁB

static TASK_PARAMETER	taskParameter[TASK_NUM]	={
	//���ݎ���(�J�n���I�t�Z�b�g) , ���� , �֐���
	{	true,	0,	10,	drvInMain	},
	{	true,	1,	10,	lnkInMain	},
	{	true,	2,	10,	aplMain		},
	{	true,	3,	10,	lnkOutMain	},
	{	true,	4,	10,	drvOutMain	},
	{	false,	5,	250,powerLed	},
	{	false,	0,	2,	drvUartChangeTx}		//UART�h���C�o����̗v���ɂ��L��������
};
//CTRLA CLKSEL
enum{
	CLKSEL_DIV1		= 0x0,
	CLKSEL_DIV2		= 0x1,
	CLKSEL_DIV4		= 0x2,
	CLKSEL_DIV8		= 0x3,
	CLKSEL_DIV16	= 0x4,
	CLKSEL_DIV64	= 0x5,
	CLKSEL_DIV256	= 0x6,
	CLKSEL_DIV1024	= 0x7,
};

//CTRLB WGMODE
enum{
	WGMODE_NORMAL		= 0x0,
	WGMODE_FRQ			= 0x1,
	WGMODE_SINGLESLOPE	= 0x3,
	WGMODE_DSTOP		= 0x5,
	WGMODE_DSBOTH		= 0x6,
	WGMODE_DSBOTTOM		= 0x7,
};

//���W�X�^�ݒ�
#define		OVF_DI		(0)
#define		OVF_EN		(1)

//���W�X�^�Z�b�g�p
#define		CTRLA_REG_START_TASK_TIMER		( TCA0.SINGLE.CTRLA = TCA0.SINGLE.CTRLA | 0x01 )

enum{
	FUSE_OSCCFG_FREQSEL_16MHZ	= 1,
	FUSE_OSCCFG_FREQSEL_20MHZ	= 2
};


#endif
