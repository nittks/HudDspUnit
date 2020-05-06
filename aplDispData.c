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
static unsigned short	dispCycRev;

//�e�X�g�f�[�^
static unsigned char	testCycSpeed;
static unsigned char	testCycRev;
static unsigned char	testSpeed;
static unsigned short	testRev;
static TEST_STATE		testStateSpeed;
static TEST_STATE		testStateRev;


static void dispSpeed(	unsigned char *retSpeed	, const unsigned char  inSpeed	);
static void dispRev(	unsigned char *retRev	, const unsigned short inRev	);
static void disp7seg(	unsigned char *retSpeed	, const unsigned char  inSpeed	);
static void dispBarLed(	unsigned char *retRev	, const unsigned short inRev	);
static unsigned char	makeTestDataSpeed( void );
static unsigned short	makeTestDataRev( void );
static unsigned char	makeTestDataSpeedManual( void );
static unsigned short	makeTestDataRevManual( void );
//********************************************************************************
// ������
//********************************************************************************
void initAplDispData( void )
{
	unsigned char	i;
	
	for( i=0 ; i<LED_7SEG_NUM ; i++ ){
		aplDispData.led7Seg[i]	= APL_DSP_DATA_7SEG_0;
	}
	aplDispData.barLedOnNum	= 0;
	aplDispData.settingMode	= APL_DISP_SETTING_OFF;
	aplDispData.settingNo	= APL_DISP_SETTING_NO_BRIGHT_7SEG;

	dispCycSpeed	= 0;
	dispCycRev		= 0;

	testCycSpeed	= 0;
	testCycRev		= 0;
	testSpeed		= 0;
	testRev			= 0;
	testStateSpeed	= TEST_STATE_UP;
	testStateRev	= TEST_STATE_UP; 
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
	unsigned short		tmpRev;

	inAplDataCar		= getAplDataCar();
	inAplCtrl			= getAplCtrl();
	inAplCtrlSet		= getAplCtrlSet();
	inAplCtrlSetPalse	= getAplCtrlSetPalse();
	
	//------------------------------
	// �P�x�f�[�^�o��
	//------------------------------
	if( inAplDataCar->ill == APL_DATA_ILL_OFF ){
		aplDispData.bright7seg		= inAplCtrlSet->bright7seg;
		aplDispData.brightBarled	= inAplCtrlSet->brightBarled;
	}else{
		aplDispData.bright7seg		= inAplCtrlSet->brightDim7seg;
		aplDispData.brightBarled	= inAplCtrlSet->brightDimBarled;
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
		
		dispSpeed( &aplDispData.led7Seg[0] , inAplDataCar->speed );
		dispRev( &aplDispData.barLedOnNum , inAplDataCar->rev );
		break;

	//****************************************
	// �e�X�g���[�h
	//****************************************
	case APL_CTRL_STATE_TESTDISP:		//�e�X�g���[�h
		aplDispData.settingMode		= APL_DISP_SETTING_OFF;
		
		switch( inAplCtrl->stateTest ){
		case APL_CTRL_STATE_TEST_AUTO:
			tmpSpeed	= makeTestDataSpeed();
			tmpRev		= makeTestDataRev();
			dispSpeed( &aplDispData.led7Seg[0] , tmpSpeed );
			dispRev( &aplDispData.barLedOnNum , tmpRev );
			break;
		case APL_CTRL_STATE_TEST_SPEED:
			tmpSpeed	= makeTestDataSpeedManual();
			dispSpeed( &aplDispData.led7Seg[0] , tmpSpeed );
			dispRev( &aplDispData.barLedOnNum , testRev );
			break;
		case APL_CTRL_STATE_TEST_REV:
			tmpRev		= makeTestDataRevManual();
			dispSpeed( &aplDispData.led7Seg[0] , testSpeed );
			dispRev( &aplDispData.barLedOnNum , tmpRev );
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
		case APL_CTRL_STATE_SET_BRIGHT_7SEG:		//����(7�Z�O
			aplDispData.settingNo = APL_DISP_SETTING_NO_BRIGHT_7SEG;
			disp7seg( &aplDispData.led7Seg[0] , SET_BRIGHT_7SEG_DISP );
			dispBarLed( &aplDispData.barLedOnNum , inAplCtrlSet->bright7seg * REV_DIGIT );
			break;
		case APL_CTRL_STATE_SET_BRIGHT_BARLED:		//����(�o�[LED
			aplDispData.settingNo = APL_DISP_SETTING_NO_BRIGHT_BARLED;
			disp7seg( &aplDispData.led7Seg[0] , inAplCtrlSet->brightBarled * SPEED_DIGIT );
			dispBarLed( &aplDispData.barLedOnNum , SET_BRIGHT_BARLED_DISP ); 
			break;
		case APL_CTRL_STATE_SET_BRIGHT_DIM_7SEG:		//��������(7�Z�O
			aplDispData.settingNo = APL_DISP_SETTING_NO_BRIGHT_7SEG;
			disp7seg( &aplDispData.led7Seg[0] , SET_BRIGHT_7SEG_DISP );
			dispBarLed( &aplDispData.barLedOnNum , inAplCtrlSet->brightDim7seg * REV_DIGIT );
			break;
		case APL_CTRL_STATE_SET_BRIGHT_DIM_BARLED:	//��������(�o�[LED
			aplDispData.settingNo = APL_DISP_SETTING_NO_BRIGHT_BARLED;
			disp7seg( &aplDispData.led7Seg[0] , inAplCtrlSet->brightDimBarled * SPEED_DIGIT );
			dispBarLed( &aplDispData.barLedOnNum , SET_BRIGHT_BARLED_DISP );
			break;
		case APL_CTRL_STATE_SET_DISPCYC_7SEG:		//�\���X�V���x(7�Z�O
			aplDispData.settingNo = APL_DISP_SETTING_NO_DISP_CYC_7SEG;
			tmpSpeed = makeTestDataSpeed();
			dispSpeed( &aplDispData.led7Seg[0] , tmpSpeed );
			dispBarLed( &aplDispData.barLedOnNum , inAplCtrlSet->dispcyc7seg * REV_DIGIT );
			break;
		case APL_CTRL_STATE_SET_DISPCYC_BARLED:		//�\���X�V���x(�o�[LED
			aplDispData.settingNo = APL_DISP_SETTING_NO_DISP_CYC_BARLED;
			tmpRev = makeTestDataRev();
			disp7seg( &aplDispData.led7Seg[0] , inAplCtrlSet->dispcycBarled * SPEED_DIGIT );
			dispRev( &aplDispData.barLedOnNum , tmpRev ); 
			break;
		case APL_CTRL_STATE_SET_PALSE_SPEED:			//�p���X�d�l�ԑ� 
			aplDispData.settingNo = APL_DISP_SETTING_NO_DISP_PALSE_SPEED;
			disp7seg( &aplDispData.led7Seg[0] , SETTING_PALSE_SPEED[inAplCtrlSetPalse->speed] );
			aplDispData.barLedOnNum	= 0;
			break;
		case APL_CTRL_STATE_SET_PALSE_REV:			//�p���X�d�l��]�� 
			aplDispData.settingNo = APL_DISP_SETTING_NO_DISP_PALSE_REV;
			disp7seg( &aplDispData.led7Seg[0] , SETTING_PALSE_REV[inAplCtrlSetPalse->rev] );
			aplDispData.barLedOnNum	= 0;
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
	for( i=0 ; i<LED_7SEG_NUM ; i++ ){
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
// ��]���\��
//********************************************************************************
static void dispRev( unsigned char *retRev , const unsigned short inRev )
{
	APL_CTRL_SET		*inAplCtrlSet;
	inAplCtrlSet		= getAplCtrlSet();

	//�\���X�V�����ҋ@(�ݒ�l[100%]�P��*1%�ӂ�̎��ԁ����Ԃ��o���A��r
	if( dispCycRev < ( inAplCtrlSet->dispcycBarled * DISPCYC_BARLED_DIGIT ) ){
		dispCycRev++;
	}else{
		dispCycRev = 0;

		dispBarLed( retRev , inRev );
	}
}
//********************************************************************************
// �o�[LED�\��
//********************************************************************************
static void dispBarLed( unsigned char *retRev , const unsigned short inRev )
{
	//��]���\������
	//LED��1�ȏ�_������
	if( inRev > REV_PER_SEG ){
		//�_��������Z�O�����g�� = ��]�� / 1�Z�O�ӂ�̉�]��
		*retRev = inRev / REV_PER_SEG;
	}else{
		*retRev = 0;
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
// ��]���e�X�g�f�[�^
//********************************************************************************
static unsigned short makeTestDataRev( void )
{
	if( testCycRev < TEST_CYC_REV ){
		testCycRev++;
	}else{
		testCycRev = 0;
		if( testStateRev == TEST_STATE_UP ){
			if( testRev < (REV_MAX-CHG_VAL_REV)){
				//1ms��1rpm����
				testRev += CHG_VAL_REV;
			}else{
				//��Ԃ�����������
				testRev = REV_MAX;
				testStateRev	= TEST_STATE_DOWN;
			}
		}else if( testStateRev == TEST_STATE_DOWN ){
			if( testRev > (REV_MIN+CHG_VAL_REV)){
				//1ms��1rpm����
				testRev -= CHG_VAL_REV;
			}else{
				//��Ԃ�����������
				testRev = REV_MIN;
				testStateRev	= TEST_STATE_UP;
			}
		}
	}
	return( testRev );
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
//********************************************************************************
// ��]���e�X�g�f�[�^(�蓮
//********************************************************************************
static unsigned short makeTestDataRevManual( void )
{
	APL_DATA_SW			*inAplDataSw;
	inAplDataSw		= getAplDataSw();

	if( inAplDataSw->rotEncSet == APL_DATA_ROT_ENC_UP ){
		if( testRev < (REV_MAX-CHG_VAL_REV_MANUAL) ){
			//1ms��1rpm����
			testRev += CHG_VAL_REV_MANUAL;
		}else{
			testRev = REV_MAX;
		}
	}else if( inAplDataSw->rotEncSet == APL_DATA_ROT_ENC_DOWN ){
		if( testRev > (REV_MIN+CHG_VAL_REV_MANUAL)){
			//1ms��1rpm����
			testRev -= CHG_VAL_REV_MANUAL;
		}else{
			testRev = REV_MIN;
		}
	}
	return( testRev );
}
