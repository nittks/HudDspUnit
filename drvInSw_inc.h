#ifndef DRV_IN_SW_INC_H
#define DRV_IN_SW_INC_H

#define	ROT_ENC_DEBTIME	(1)	//1�����҂�(�������܂ő҂�10+10x1ms
#define	PUSH_SW_DEBTIME		(60)	//�f�o�E���X�҂�����
#define	PUSH_SW_LONGTIME	(100)	//���������莞��(10x100=1000ms

#define	PORT_ROT_ENC0_A		((PORTD.IN & >> 0)& 0x01)
#define	PORT_ROT_ENC0_B		((PORTD.IN & >> 1)& 0x01)

#define	PORT_PUSHSW_0		((PORTD.IN & >> 2)& 0x01)


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


#endif
