
#define	ROT_ENC_DEBTIME	(1)	//1�����҂�(�������܂ő҂�10+10x1ms
#define	PUSH_SW_DEBTIME		(60)	//�f�o�E���X�҂�����
#define	PUSH_SW_LONGTIME	(100)	//���������莞��(10x100=1000ms

#define	PORT_ROT_ENC0_A		((PORTB & (1<<PB4))>> PB4)
#define	PORT_ROT_ENC0_B		((PORTB & (1<<PB5))>> PB5)

#define	PORT_PUSHSW_0		((PORTC & (1<<PC4))>> PC4)
#define	PORT_PUSHSW_1		((PORTC & (1<<PC5))>> PC5)


#define	ROT_VECT_FORWARD	((signed char)(4))
#define	ROT_VECT_REVERCE	((signed char)(-4))

static const signed char grayCodeTable[0xF+1] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

//���[�^���[�G���R�[�_�[�������
typedef enum{
	ROT_ENC_STATE_WAIT,		//���͊Ď���
	ROT_ENC_STATE_FORWARD,	//���]���m
	ROT_ENC_STATE_REVERCE,	//�t�]���m
	ROT_ENC_STATE_DEBOUNCE	//�f�o�E���X�^�C���E�F�C�g
}ROT_ENC_STATE;

//�v�b�V���X�C�b�`�������
typedef enum{
	PUSH_SW_STATE_OFF,
	PUSH_SW_STATE_ON,
	PUSH_SW_STATE_DEBOUNCE	//�f�o�E���X�^�C���E�F�C�g
}PUSH_SW_STATE;

//�|�[�g���
typedef enum{
	PORT_OFF	= 0,
	PORT_ON		= 1
}SW_PORT_STATE;

typedef enum{
	NO_SET
}ROT_PORT_NO;

typedef enum{
	NO_0,
	NO_1
}PUSH_SW_PORT_NO;

#define		ROT_ENC_0_POS		(0x03)		//bit0,1������
#define		ROT_ENC_1_POS		(0x0C)		//bit2,3������

//�s���ω����荞��PCINT8-11���g�p����
#define	REG_PCIE2	0		//PCINT16-23pin�̊��荞�݋��ݒ�
#define	REG_PCIE1	1		//PCINT08-14pin�̊��荞�݋��ݒ�
#define	REG_PCIE0	1		//PCINT00-07pin�̊��荞�݋��ݒ�


//�s���ω����荞�ݐ��䃌�W�X�^
#define	SET_PCICR	(PCICR | (REG_PCIE2<<PCIE2) | (REG_PCIE1<<PCIE1) | (REG_PCIE0<<PCIE0))

//�s���ω����荞�݃}�X�N
#define	SET_PCMSK1	(PCMSK1 | 0x30)
#define	SET_PCMSK0	(PCMSK0 | 0x30)





