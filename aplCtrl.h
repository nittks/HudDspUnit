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
}APL_CTRL_STATE_TEST;

//���(�ݒ��
typedef enum{
	APL_CTRL_STATE_SET_COLOR_7SEG,			//���F
	APL_CTRL_STATE_SET_BRIGHT_7SEG,			//����(7�Z�O
	APL_CTRL_STATE_SET_BRIGHT_DIM_7SEG,		//��������(7�Z�O
	APL_CTRL_STATE_SET_DISPCYC_7SEG,		//�\���X�V���x(7�Z�O
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
	unsigned char	color7seg;			//���F
	unsigned char	bright7seg;			//����(7�Z�O
	unsigned char	brightBarled;		//����(�o�[LED
	unsigned char	brightDim7seg;		//��������(7�Z�O
	unsigned char	brightDimBarled;	//��������(�o�[LED
	unsigned char	dispcyc7seg;		//�\���X�V���x(7�Z�O
	unsigned char	dispcycBarled;		//�\���X�V���x(�o�[LED
}APL_CTRL_SET;

typedef struct {
	unsigned char	speed;			//�p���X�d�l�ԑ�   
	unsigned char	rev;			//�p���X�d�l��]�� 
}APL_CTRL_SET_PALSE;



typedef enum{
	SET_COLOR,
	SET_PALSE_SPEED,
	SET_PALSE_REV,
	SET_ITEM_NO_MAX
}SET_ITEM_NO;

enum{
	SETTING_COLOR_RED,
	SETTING_COLOR_GREEN,
	SETTING_COLOR_BLUE,
	SETTING_COLOR_WHITE,
	SETTING_COLOR_MAX = SETTING_COLOR_WHITE
};

enum{
	SETTING_PALSE_SPEED_04	= 1,
	SETTING_PALSE_SPEED_08,
	SETTING_PALSE_SPEED_16,
	SETTING_PALSE_SPEED_20,
	SETTING_PALSE_SPEED_25,
	SETTING_PALSE_SPEED_MAX = SETTING_PALSE_SPEED_25	//1�n�܂�̂���-1
};
enum{
	SETTING_PALSE_REV_01	= 1,
	SETTING_PALSE_REV_02,
	SETTING_PALSE_REV_03,
	SETTING_PALSE_REV_04,
	SETTING_PALSE_REV_05,
	SETTING_PALSE_REV_06,
	SETTING_PALSE_REV_08,
	SETTING_PALSE_REV_10,
	SETTING_PALSE_REV_MAX = SETTING_PALSE_REV_10	//1�n�܂�̂���-1
};



extern void initAplCtrl( void );
extern void aplCtrlMain( void );
extern APL_CTRL *getAplCtrl( void );
extern APL_CTRL_SET *getAplCtrlSet( void );
extern APL_CTRL_SET_PALSE *getAplCtrlSetPalse( void );


#endif
