#include <avr/io.h>
#include <stdbool.h>
#include "aplCtrl_inc.h"
#include "aplCtrl.h"
#include "aplData.h"
#include "lnkOutEep.h"
#include "lnkOutCom.h"
#include "hardware.h"

static APL_CTRL				aplCtrl;
static APL_CTRL_SET			aplCtrlSet;
static APL_CTRL_SET			aplCtrlSetBak;		//�ݒ�ۑ�����������Ղɖ߂��A���̒l�ێ��p
static APL_CTRL_SET_PALSE	aplCtrlSetPalse;
static APL_CTRL_SET_PALSE	aplCtrlSetPalseBak;

static void stateJudge( void );
static void procSetting( void );
static void changeSettingVal( unsigned char *val );
static void changeSettingItem( unsigned char *setNo , SET_ITEM_NO no );
static void chkSetPalse( void );
static void apryEep( void );
//********************************************************************************
// ������
//********************************************************************************
void initAplCtrl( void )
{
	aplCtrl.state			= APL_CTRL_STATE_BOOT;
	aplCtrl.stateSet		= APL_CTRL_STATE_SET_BRIGHT_7SEG;
	aplCtrlSetPalse.speed	= INIT_PALSE_SPEED;
	aplCtrlSetPalse.rev		= INIT_PALSE_REV;
	
}
//********************************************************************************
// �����Ԏ擾
//********************************************************************************
APL_CTRL *getAplCtrl( void )
{
	return( &aplCtrl );
}
//********************************************************************************
// �ݒ�l�擾
//********************************************************************************
APL_CTRL_SET *getAplCtrlSet( void )
{
	return( &aplCtrlSet );
}
//********************************************************************************
// �p���X�ݒ�l�擾
//********************************************************************************
APL_CTRL_SET_PALSE *getAplCtrlSetPalse( void )
{
	return( &aplCtrlSetPalse );
}
//********************************************************************************
// ���C������
//********************************************************************************
void aplCtrlMain( void )
{
//	PORTB = (PORTB & 0xFE) | (~(PORTB & 0x01) );

	//��Ԕ���A��ԑJ��
	stateJudge();

	//�ݒ菈��
	procSetting();
}
//********************************************************************************
// ��Ԕ���A��ԑJ��
//********************************************************************************
static void stateJudge( void )
{
	APL_DATA_CAR	*inAplDataCar;
	APL_DATA_SW		*inAplDataSw;
	APL_DATA_EEP	*inAplDataEep;

	inAplDataCar	= getAplDataCar();
	inAplDataSw		= getAplDataSw();
	inAplDataEep	= getAplDataEep();

	switch( aplCtrl.state ){
	//****************************************
	// ����N��
	//****************************************
	case APL_CTRL_STATE_BOOT:		//�N������
		if( inAplDataEep->read != APL_DATA_EEP_STATE_UNREAD ){		//EEPROM�ǂݍ��ݍς�
			apryEep();		//�����EEPROM�̃f�[�^��K�p����
			aplCtrl.state = APL_CTRL_STATE_NOMARL;
		}
		break;

	//****************************************
	// �ʏ�
	//****************************************
	case APL_CTRL_STATE_NOMARL:		//�ʏ�
		if( inAplDataSw->pushSwTest == APL_DATA_PUSH_SW_LONGON ){
			//�ʏ�->�e�X�g�\��
			aplCtrl.state = APL_CTRL_STATE_TESTDISP;
			aplCtrl.stateTest = APL_CTRL_STATE_TEST_AUTO;
		}else if( inAplDataSw->pushSwSet == APL_DATA_PUSH_SW_LONGON ){
			//�ʏ�->�ݒ�
			aplCtrl.state = APL_CTRL_STATE_SETTING;
			if( inAplDataCar->ill == APL_DATA_ILL_ON ){
				aplCtrl.stateSet = APL_CTRL_STATE_SET_BRIGHT_DIM_7SEG;
			}else{
				aplCtrl.stateSet = APL_CTRL_STATE_SET_BRIGHT_7SEG;
			}
			//���̓��j�b�g�����M���Ă���p���X�ݒ�l�����ݐݒ�l�Ƃ��Ď擾
			aplCtrlSetPalse.speed	= inAplDataCar->palseSetSpeed;
			aplCtrlSetPalse.rev		= inAplDataCar->palseSetRev;

			//�ύX�O�̒l��ۑ�(�L�����Z�����߂�����
			//�ݒ�
			aplCtrlSetBak = aplCtrlSet;
			//�p���X�ݒ�
			aplCtrlSetPalse.speed		= inAplDataCar->palseSetSpeed;
			aplCtrlSetPalse.rev			= inAplDataCar->palseSetRev;
			aplCtrlSetPalseBak.speed	= inAplDataCar->palseSetSpeed;
			aplCtrlSetPalseBak.rev		= inAplDataCar->palseSetRev;

		}
		break;

	//****************************************
	// �e�X�g�\��
	//****************************************
	case APL_CTRL_STATE_TESTDISP:		//�e�X�g���[�h
		if( inAplDataSw->pushSwTest == APL_DATA_PUSH_SW_LONGON ){
			aplCtrl.state = APL_CTRL_STATE_NOMARL;
		}
		switch( aplCtrl.stateTest ){
		case APL_CTRL_STATE_TEST_AUTO:
			if( inAplDataSw->pushSwTest == APL_DATA_PUSH_SW_ON ){
				aplCtrl.stateTest = APL_CTRL_STATE_TEST_SPEED;
			}
			break;
		case APL_CTRL_STATE_TEST_SPEED:
			if( inAplDataSw->pushSwTest == APL_DATA_PUSH_SW_ON ){
				aplCtrl.stateTest = APL_CTRL_STATE_TEST_AUTO;
			}
			break;
		}
		break;
		
	//****************************************
	// �ݒ�
	//****************************************
	case APL_CTRL_STATE_SETTING:		//�ݒ�
		switch( aplCtrl.stateSet ){
		case APL_CTRL_STATE_SET_BRIGHT_7SEG:		//����(7�Z�O
			if( inAplDataSw->pushSwSet == APL_DATA_PUSH_SW_ON ){
				aplCtrl.stateSet = APL_CTRL_STATE_SET_DISPCYC_7SEG;
			}else if( inAplDataCar->ill == APL_DATA_ILL_ON ){
				aplCtrl.stateSet = APL_CTRL_STATE_SET_BRIGHT_DIM_7SEG;
			}
			break;
		case APL_CTRL_STATE_SET_BRIGHT_DIM_7SEG:	//��������(7�Z�O
			if( inAplDataSw->pushSwSet == APL_DATA_PUSH_SW_ON ){
				aplCtrl.stateSet = APL_CTRL_STATE_SET_DISPCYC_7SEG;
			}else if( inAplDataCar->ill == APL_DATA_ILL_OFF ){
				aplCtrl.stateSet = APL_CTRL_STATE_SET_DISPCYC_7SEG;
			}
			break;
		case APL_CTRL_STATE_SET_DISPCYC_7SEG:		//�\���X�V���x(7�Z�O
			if( inAplDataSw->pushSwSet == APL_DATA_PUSH_SW_ON ){
				aplCtrl.stateSet = APL_CTRL_STATE_SET_PALSE_SPEED;
			}
			break;
		case APL_CTRL_STATE_SET_PALSE_SPEED:		//�p���X�d�l�ԑ� 
			if( inAplDataSw->pushSwSet == APL_DATA_PUSH_SW_ON ){
				aplCtrl.stateSet = APL_CTRL_STATE_SET_PALSE_REV;
			}
			break;
		case APL_CTRL_STATE_SET_PALSE_REV:			//�p���X�d�l��]�� 
			if( inAplDataSw->pushSwSet == APL_DATA_PUSH_SW_ON ){
				if( inAplDataCar->ill == APL_DATA_ILL_ON ){
					aplCtrl.stateSet = APL_CTRL_STATE_SET_BRIGHT_DIM_7SEG;
				}else{
					aplCtrl.stateSet = APL_CTRL_STATE_SET_BRIGHT_7SEG;
				}
			}
			break;
		default:
			break;
		}

		//�ݒ肩��o��
		if( inAplDataSw->pushSwSet == APL_DATA_PUSH_SW_LONGON ){
			setLnkOutEep();	//EEPROM�����ݗv��
			chkSetPalse();	//�p���X�ݒ�ύX�L���`�F�b�N
			aplCtrl.state	= APL_CTRL_STATE_NOMARL;
		}else if( inAplDataSw->pushSwTest == APL_DATA_PUSH_SW_LONGON ){
			aplCtrl.state	= APL_CTRL_STATE_NOMARL;
			aplCtrlSet		= aplCtrlSetBak;		//�ύX�O�̒l�ɖ߂�
		}
		break;
	}
}
//********************************************************************************
// �ݒ�T�u���[�`��
//********************************************************************************
static void procSetting( void )
{
	switch( aplCtrl.state ){
	case APL_CTRL_STATE_BOOT:		//�N������
		//������
		break;

	case APL_CTRL_STATE_NOMARL:		//�ʏ�
		//������
		break;

	case APL_CTRL_STATE_TESTDISP:		//�e�X�g���[�h
		//������
		break;
		
	case APL_CTRL_STATE_SETTING:		//�ݒ�
		switch( aplCtrl.stateSet ){
		case APL_CTRL_STATE_SET_COLOR_7SEG:			//���F
			changeSettingItem( &aplCtrlSet.color7seg ,SET_COLOR);
			break;
		case APL_CTRL_STATE_SET_BRIGHT_7SEG:			//����(7�Z�O
			changeSettingVal( &aplCtrlSet.bright7seg );
			break;
		case APL_CTRL_STATE_SET_BRIGHT_DIM_7SEG:		//��������(7�Z�O
			changeSettingVal( &aplCtrlSet.brightDim7seg );
			break;
		case APL_CTRL_STATE_SET_DISPCYC_7SEG:		//�\���X�V���x(7�Z�O
			changeSettingVal( &aplCtrlSet.dispcyc7seg );
			break;
		case APL_CTRL_STATE_SET_PALSE_SPEED:			//�p���X�d�l�ԑ� 
			changeSettingItem(	&aplCtrlSetPalse.speed ,SET_PALSE_SPEED);
			break;
		case APL_CTRL_STATE_SET_PALSE_REV:			//�p���X�d�l��]�� 
			changeSettingItem(	&aplCtrlSetPalse.rev,SET_PALSE_REV);
			break;
		default:
			break;
		}
		break;
	}
}

//********************************************************************************
// �ݒ�l�ύX
//********************************************************************************
void changeSettingVal( unsigned char *val )
{
	APL_DATA_SW		*inAplDataSw;
	inAplDataSw		= getAplDataSw();

	if( inAplDataSw->rotEncSet == APL_DATA_ROT_ENC_UP ){
		//����u���b�N
		if( *val < ( SETTING_VAL_MAX - SETTING_VAL_INTERVAL ) ){
			*val += SETTING_VAL_INTERVAL;
		}else{
			*val = SETTING_VAL_MAX;
		}
	}else if( inAplDataSw->rotEncSet == APL_DATA_ROT_ENC_DOWN ){
		//�����u���b�N
		if( *val > ( SETTING_VAL_MIN + SETTING_VAL_INTERVAL ) ){
			*val -= SETTING_VAL_INTERVAL;
		}else{
			*val = SETTING_VAL_MIN;
		}
	}
}
//********************************************************************************
// �p���X�ݒ�l�ύX
//********************************************************************************
void changeSettingItem( unsigned char *setNo , SET_ITEM_NO no )
{
	APL_DATA_SW		*inAplDataSw;
	inAplDataSw		= getAplDataSw();

	if( inAplDataSw->rotEncSet == APL_DATA_ROT_ENC_UP ){
		//����u���b�N
		if( *setNo < PALSE_ITEM_MAX[no] ){
			*setNo += 1;
		}else{
			*setNo = PALSE_ITEM_MIN[no];
		}
	}else if( inAplDataSw->rotEncSet == APL_DATA_ROT_ENC_DOWN ){
		//�����u���b�N
		if( *setNo > PALSE_ITEM_MIN[no] ){
			*setNo -= 1;
		}else{
			*setNo = PALSE_ITEM_MAX[no];
		}
	}
}

//********************************************************************************
// �p���X�d�l�ݒ�ύX�`�F�b�N&�ύX�v��
//********************************************************************************
static void chkSetPalse( void )
{
	//�ύX�l�`�F�b�N
	if(( aplCtrlSetPalse.speed != aplCtrlSetPalseBak.speed ) ||
	   ( aplCtrlSetPalse.rev != aplCtrlSetPalseBak.rev ))
	{
		//���̓��j�b�g�֐ݒ�l�ύX�ʐM���M
		setLnkOutCom();
	}
}
		
//********************************************************************************
// EEPROM�f�[�^�K�p
//********************************************************************************
static void apryEep( void )
{
	APL_DATA_EEP	*inAplDataEep;

	inAplDataEep	= getAplDataEep();

	if( inAplDataEep->read == APL_DATA_EEP_STATE_READED){
		//�Ǎ��ς݂Ȃ甽�f
		aplCtrlSet.color7seg			= inAplDataEep->color7seg;
		aplCtrlSet.bright7seg			= inAplDataEep->bright7seg;
		aplCtrlSet.brightDim7seg		= inAplDataEep->brightDim7seg;
		aplCtrlSet.dispcyc7seg			= inAplDataEep->dispcyc7seg;
	}else if( inAplDataEep->read == APL_DATA_EEP_STATE_SUMERROR){
		//SUM�G���[���̓f�t�H���g�l�Ǎ�
		aplCtrlSet.color7seg			= eepDefault[COLOR_7SEG];
		aplCtrlSet.bright7seg			= eepDefault[BRIGHT_7SEG];
		aplCtrlSet.brightDim7seg		= eepDefault[BRIGHT_DIM_7SEG];
		aplCtrlSet.dispcyc7seg			= eepDefault[DISPCYC_7SEG];
		setLnkOutEep();	//EEPROM�����ݗv��
	}
}
