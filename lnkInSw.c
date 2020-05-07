#include <avr/io.h>

#include "lnkInSw_inc.h"
#include "lnkInSw.h"
#include "aplData.h"
#include "drvInSw.h"

static APL_DATA_ROT_ENC judgeRotEnc( DRV_IN_ROT_ENC_STATE rotEncState );
static APL_DATA_PUSH_SW judgePushSw( DRV_IN_PUSH_SW_STATE pushSwState );
//********************************************************************************
// ������
//********************************************************************************
void initLnkInSw( void )
{
}
//********************************************************************************
// ���C������
//********************************************************************************
void lnkInSwMain( void )
{
	DRV_IN_SW		*inDrvInSw;
	APL_DATA_SW		aplDataSw;

	//SW���̓h���C�o�f�[�^�擾
	inDrvInSw = getDrvInSw();

	//���[�^���[�G���R�[�_�[����
	aplDataSw.rotEncSet		= judgeRotEnc( inDrvInSw->rotEncState[ROT_ENC_SET] );
	//�v�b�V���X�C�b�`����
	aplDataSw.pushSwSet		= judgePushSw( inDrvInSw->pushSwState[PUSH_SW_SET] );
	aplDataSw.pushSwTest	= judgePushSw( inDrvInSw->pushSwState[PUSH_SW_TEST] );

	setAplDataSw( &aplDataSw );
}

//********************************************************************************
// ���[�^���[�G���R�[�_�[���͔���
//********************************************************************************
static APL_DATA_ROT_ENC judgeRotEnc( DRV_IN_ROT_ENC_STATE rotEncState )
{
	APL_DATA_ROT_ENC	ret;

	//���[�^���[�G���R�[�_�[����
	if( rotEncState == DRV_IN_ROT_ENC_STATE_FORWARD ){
		ret	= APL_DATA_ROT_ENC_UP;	
	}else if( rotEncState == DRV_IN_ROT_ENC_STATE_REVERCE ){
		ret	= APL_DATA_ROT_ENC_DOWN;
	}else{
		ret	= APL_DATA_ROT_ENC_STOP;
	}
	return( ret );
}
//********************************************************************************
// �v�b�V���X�C�b�`���͔���
//********************************************************************************
static APL_DATA_PUSH_SW judgePushSw( DRV_IN_PUSH_SW_STATE pushSwState )
{
	APL_DATA_PUSH_SW	ret;

	if( pushSwState == DRV_IN_PUSH_SW_STATE_ON ){
		ret	= APL_DATA_PUSH_SW_ON;
	}else if( pushSwState == DRV_IN_PUSH_SW_STATE_LONGON ){
		ret	= APL_DATA_PUSH_SW_LONGON;
	}else{
		ret	= APL_DATA_PUSH_SW_OFF;
	}
	return( ret );
}
