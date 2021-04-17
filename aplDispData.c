#include <avr/io.h>
#include <avr/interrupt.h>  //���荞�݂��g�p���邽��
#include "aplDispData_inc.h"
#include "aplDispData.h"
#include "aplData.h"
#include "aplCtrl.h"
#include "hardware.h"

//���J�p
static APL_DISP_DATA	aplDispData;

//���������p
//�ʏ펞
static unsigned short	dispCycSpeed;

//�e�X�g�f�[�^
static unsigned char	testCycSpeed;
static unsigned char	testSpeed;
static TEST_STATE		testStateSpeed;


static void dispSpeed(	unsigned char *retSpeed	, const unsigned char  inSpeed	);
static void disp7seg(	unsigned char *retSpeed	, const unsigned char  inSpeed	);
static unsigned char	makeTestDataSpeed( void );
static unsigned char	makeTestDataSpeedManual( void );
//********************************************************************************
// ������
//********************************************************************************
void initAplDispData( void )
{
	unsigned char	i;
	
	for( i=0 ; i<LED_7SEG_DIGIT_NUM ; i++ ){
		aplDispData.led7Seg[i]	= APL_DSP_DATA_7SEG_0;
	}
	aplDispData.settingMode	= APL_DISP_SETTING_OFF;
	aplDispData.color7seg	= APL_DISP_DATA_WHITE;

	dispCycSpeed	= 0;

	testCycSpeed	= 0;
	testSpeed		= 0;
	testStateSpeed	= TEST_STATE_UP;
	
	//debug
	dispSpeed( &aplDispData.led7Seg[0] , 123);
}
//********************************************************************************
// �擾
//********************************************************************************
APL_DISP_DATA *getAplDispData( void )
{
	return( &aplDispData );
}
//********************************************************************************
// ���C������
//********************************************************************************
void aplDispDataMain( void )
{
	APL_DATA_CAR		*inAplDataCar;
	APL_CTRL			*inAplCtrl;
	APL_CTRL_SET		*inAplCtrlSet;
	APL_CTRL_SET_PALSE	*inAplCtrlSetPalse;
	
	unsigned char		tmpSpeed;

	inAplDataCar		= getAplDataCar();
	inAplCtrl			= getAplCtrl();
	inAplCtrlSet		= getAplCtrlSet();
	inAplCtrlSetPalse	= getAplCtrlSetPalse();
	

	//------------------------------
	// ���F�f�[�^�o��
	//------------------------------
	//aplDispData.color7seg			= inAplCtrlSet->color7seg;

	//------------------------------
	// �P�x�f�[�^�o��
	//------------------------------
	if( inAplDataCar->ill == APL_DATA_ILL_OFF ){
		aplDispData.bright7seg		= inAplCtrlSet->bright7seg/3;
	}else{
		aplDispData.bright7seg		= inAplCtrlSet->brightDim7seg;
	}

	switch( inAplCtrl->state ){
	//****************************************
	// ����N��
	//****************************************
	case APL_CTRL_STATE_BOOT:
		//������
		break;
	//****************************************
	// �ʏ�
	//****************************************
	case APL_CTRL_STATE_NOMARL:		//�ʏ�
		aplDispData.settingMode		= APL_DISP_SETTING_OFF;
		
		//dispSpeed( &aplDispData.led7Seg[0] , inAplDataCar->speed );

		break;

	//****************************************
	// �e�X�g���[�h
	//****************************************
	case APL_CTRL_STATE_TESTDISP:		//�e�X�g���[�h
		aplDispData.settingMode		= APL_DISP_SETTING_OFF;
		
		switch( inAplCtrl->stateTest ){
		case APL_CTRL_STATE_TEST_AUTO:
			tmpSpeed	= makeTestDataSpeed();
			dispSpeed( &aplDispData.led7Seg[0] , tmpSpeed );
			break;
		case APL_CTRL_STATE_TEST_SPEED:
			tmpSpeed	= makeTestDataSpeedManual();
			dispSpeed( &aplDispData.led7Seg[0] , tmpSpeed );
			break;
		}
		break;
	//****************************************
	// �ݒ�
	//****************************************
	case APL_CTRL_STATE_SETTING:		//�ݒ�
		//�ݒ胂�[�h�\��ON
		aplDispData.settingMode		= APL_DISP_SETTING_ON;

		switch( inAplCtrl->stateSet){
		case APL_CTRL_STATE_SET_COLOR_7SEG:		//���F
			disp7seg( &aplDispData.led7Seg[0] , SET_BRIGHT_7SEG_DISP );
			break;
		case APL_CTRL_STATE_SET_BRIGHT_7SEG:		//����(7�Z�O
			disp7seg( &aplDispData.led7Seg[0] , SET_BRIGHT_7SEG_DISP );
			break;
		case APL_CTRL_STATE_SET_BRIGHT_DIM_7SEG:		//��������(7�Z�O
			disp7seg( &aplDispData.led7Seg[0] , SET_BRIGHT_7SEG_DISP );
			break;
		case APL_CTRL_STATE_SET_DISPCYC_7SEG:		//�\���X�V���x(7�Z�O
			disp7seg( &aplDispData.led7Seg[0], inAplCtrlSet->dispcyc7seg * SPEED_DIGIT );
			break;
		case APL_CTRL_STATE_SET_PALSE_SPEED:			//�p���X�d�l�ԑ� 
			disp7seg( &aplDispData.led7Seg[0] , SETTING_PALSE_SPEED[inAplCtrlSetPalse->speed] );
			break;
		case APL_CTRL_STATE_SET_PALSE_REV:			//�p���X�d�l��]�� 
			disp7seg( &aplDispData.led7Seg[0] , SETTING_PALSE_REV[inAplCtrlSetPalse->rev] );
			break;
		default:
			break;
		}
		break;
	}
}

//********************************************************************************
// �ԑ��\��
//********************************************************************************
static void dispSpeed( unsigned char *retSpeed , const unsigned char inSpeed )
{
	APL_CTRL_SET	*inAplCtrlSet;

	inAplCtrlSet	= getAplCtrlSet();

	//�\���X�V�����ҋ@(�ݒ�l[100%]�P��*1%�ӂ�̎��ԁ����Ԃ��o���A��r
	if( dispCycSpeed < ( inAplCtrlSet->dispcyc7seg * DISPCYC_7SEG_DIGIT ) ){
		dispCycSpeed++;
	}else{
		dispCycSpeed = 0;

		disp7seg( retSpeed , inSpeed );
	}
	
}
//********************************************************************************
// 7seg�\��
//********************************************************************************
static void disp7seg( unsigned char *retSpeed , const unsigned char inSpeed )
{
	unsigned char	i;
	unsigned char	tmpSpeed;

	//�X�s�[�h�\������
	tmpSpeed = inSpeed;	// /=�ő�����Ă������߃��[�N�փR�s�[
	for( i=0 ; i<LED_7SEG_DIGIT_NUM ; i++ ){
		//1�����o
		if( tmpSpeed > 0 ){
			//���̌�����v�Z
			retSpeed[i]	= tmpSpeed % 10;
			tmpSpeed	/= 10;
		}else{
			if( i==0 ){
				//1�̈ʂ�0�̎���0�\��
				retSpeed[i]	= 0;
			}else{
				//10�A100�̈ʂ�0�̎��͔�\��
				retSpeed[i]	= APL_DSP_DATA_7SEG_BLANK;
			}
		}
	}	
}
//********************************************************************************
// �ԑ��e�X�g�f�[�^
//********************************************************************************
static unsigned char makeTestDataSpeed( void )
{
	if( testCycSpeed < TEST_CYC_SPEED ){
		testCycSpeed++;
	}else{
		testCycSpeed = 0;
		if( testStateSpeed == TEST_STATE_UP ){
			if( testSpeed < (SPEED_MAX-CHG_VAL_SPEED)){
				//1ms��1rpm����
				testSpeed += CHG_VAL_SPEED;
			}else{
				//��Ԃ�����������
				testSpeed = SPEED_MAX;
				testStateSpeed	= TEST_STATE_DOWN;
			}
		}else if( testStateSpeed == TEST_STATE_DOWN ){
			if( testSpeed > (SPEED_MIN+CHG_VAL_SPEED)){
				//1ms��1rpm����
				testSpeed -= CHG_VAL_SPEED;
			}else{
				//��Ԃ�����������
				testSpeed = SPEED_MIN;
				testStateSpeed	= TEST_STATE_UP;
			}
		}
	}
	return( testSpeed );
}
//********************************************************************************
// �ԑ��e�X�g�f�[�^(�蓮
//********************************************************************************
static unsigned char makeTestDataSpeedManual( void )
{
	APL_DATA_SW			*inAplDataSw;
	inAplDataSw		= getAplDataSw();

	if( inAplDataSw->rotEncSet == APL_DATA_ROT_ENC_UP ){
		if( testSpeed < (SPEED_MAX-CHG_VAL_SPEED_MANUAL) ){
			//1ms��1rpm����
			testSpeed += CHG_VAL_SPEED_MANUAL;
		}else{
			testSpeed = SPEED_MAX;
		}
	}else if( inAplDataSw->rotEncSet == APL_DATA_ROT_ENC_DOWN ){
		if( testSpeed > (SPEED_MIN+CHG_VAL_SPEED_MANUAL)){
			//1ms��1rpm����
			testSpeed -= CHG_VAL_SPEED_MANUAL;
		}else{
			testSpeed = SPEED_MIN;
		}
	}
	return( testSpeed );
}
