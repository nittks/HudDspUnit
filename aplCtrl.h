#ifndef APL_CTRL_H
#define APL_CTRL_H


//���
typedef enum{
	APL_CTRL_STATE_BOOT,		//�N������
	APL_CTRL_STATE_NOMARL,		//�ʏ�
	APL_CTRL_STATE_TESTDISP,	//�e�X�g�\��
	APL_CTRL_STATE_SETTING		//�ݒ�
}APL_CTRL_STATE;

//���(�e�X�g�\����
typedef enum{
	APL_CTRL_STATE_TEST_AUTO,
	APL_CTRL_STATE_TEST_SPEED,
	APL_CTRL_STATE_TEST_REV
}APL_CTRL_STATE_TEST;

//���(�ݒ��
typedef enum{
	APL_CTRL_STATE_SET_BRIGHT_7SEG,			//����(7�Z�O
	APL_CTRL_STATE_SET_BRIGHT_BARLED,		//����(�o�[LED
	APL_CTRL_STATE_SET_BRIGHT_DIM_7SEG,		//��������(7�Z�O
	APL_CTRL_STATE_SET_BRIGHT_DIM_BARLED,	//��������(�o�[LED
	APL_CTRL_STATE_SET_DISPCYC_7SEG,		//�\���X�V���x(7�Z�O
	APL_CTRL_STATE_SET_DISPCYC_BARLED,		//�\���X�V���x(�o�[LED
	APL_CTRL_STATE_SET_PALSE_SPEED,			//�p���X�d�l�ԑ�
	APL_CTRL_STATE_SET_PALSE_REV,			//�p���X�d�l��]��
	APL_CTRL_STATE_SET_MAX
}APL_CTRL_STATE_SET;


typedef struct {
	APL_CTRL_STATE		state;
	APL_CTRL_STATE_TEST	stateTest;
	APL_CTRL_STATE_SET	stateSet;
}APL_CTRL;

typedef struct {
	unsigned char	bright7seg;			//����(7�Z�O
	unsigned char	brightBarled;		//����(�o�[led
	unsigned char	brightDim7seg;		//��������(7�Z�O
	unsigned char	brightDimBarled;	//��������(�o�[led
	unsigned char	dispcyc7seg;		//�\���X�V���x(7�Z�O
	unsigned char	dispcycBarled;		//�\���X�V���x(�o�[led
}APL_CTRL_SET;

typedef struct {
	unsigned char	speed;			//�p���X�d�l�ԑ�   
	unsigned char	rev;			//�p���X�d�l��]�� 
}APL_CTRL_SET_PALSE;



extern void initAplCtrl( void );
extern void aplCtrlMain( void );
extern APL_CTRL *getAplCtrl( void );
extern APL_CTRL_SET *getAplCtrlSet( void );
extern APL_CTRL_SET_PALSE *getAplCtrlSetPalse( void );
#endif
