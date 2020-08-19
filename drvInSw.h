#ifndef DRV_IN_SW_H
#define DRV_IN_SW_H

#define	ROT_ENC_NUM	1		//���[�^���[�G���R�[�_�[��
#define	PUSH_SW_NUM	1		//�v�b�V���X�C�b�`��

//���[�^���[�G���R�[�_�[���
typedef enum{
	DRV_IN_ROT_ENC_STATE_STOP,	//��~�A�ω��Ȃ�
	DRV_IN_ROT_ENC_STATE_FORWARD,	//���]
	DRV_IN_ROT_ENC_STATE_REVERCE	//�t�]
}DRV_IN_ROT_ENC_STATE;

//�v�b�V���X�C�b�`���
typedef enum{
	DRV_IN_PUSH_SW_STATE_OFF,
	DRV_IN_PUSH_SW_STATE_ON,
	DRV_IN_PUSH_SW_STATE_LONGON,
}DRV_IN_PUSH_SW_STATE;

typedef struct{
	DRV_IN_ROT_ENC_STATE	rotEncState[ROT_ENC_NUM];
	DRV_IN_PUSH_SW_STATE	pushSwState[PUSH_SW_NUM];
}DRV_IN_SW;

extern void initDrvInSw( void );
extern DRV_IN_SW *getDrvInSw( void );
extern void drvInSwMain( void );
extern void interSwInput( void );
extern void interPcInt00_07( void );
extern void interPcInt08_14( void );

#endif
