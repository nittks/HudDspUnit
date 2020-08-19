
#include "hardware.h"
#include "lnkOutLed_inc.h"
#include "lnkOutLed.h"
#include "drvOutShiftReg.h"
#include "aplDispData.h"

//********************************************************************************
// ������
//********************************************************************************
void initLnkOutLed( void )
{
}
//********************************************************************************
// ���C������
//********************************************************************************
void lnkOutLedMain( void )
{
	APL_DISP_DATA		*inAplDispData;
	DRV_LED_7SEG_DATA	outDrvLed7SegData;
	DRV_LED_BAR_DATA	outDrvLedBarData;
	unsigned char		i,j;
	unsigned char		onLedNum;

	inAplDispData = getAplDispData();
	
	
	//----------------------------------
	// 7�Z�O�\������
	//----------------------------------
	//1�̈ʂ���1�Z�O�����g��on,off���Z�b�g
	for( i=0 ; i<LED_7SEG_NUM ; i++ ){
		outDrvLed7SegData.data[i].data = led7SegBit[ inAplDispData->led7Seg[i] ];
	}
	//�ݒ胂�[�h�\��ON
	if( inAplDispData->settingMode == APL_DISP_SETTING_ON ){
		//1000�̈ʂɐݒ�No�\���Bdot�t��
		outDrvLed7SegData.data[LED_7SEG_NUM-1].data = DOT_ON( led7SegBit[ inAplDispData->settingNo ] );
	}

	//�P�x
	outDrvLed7SegData.bright = inAplDispData->bright7seg;
	
	//drv�֏o��
	setDrvOutShiftReg7seg( &outDrvLed7SegData );
	
	//----------------------------------
	// �o�[LED�\������
	//----------------------------------
	for( i=0 ; i<LED_BAR_NUM ; i++ ){
		if( inAplDispData->barLedOnNum <= ( i*LED_BAR_SEG )){
			onLedNum	= 0;
		}else if( inAplDispData->barLedOnNum >= ( (i*LED_BAR_SEG) + LED_BAR_SEG )){
			onLedNum	= 10;
		}else{
			//�Y�����W���[���̓_����
			onLedNum	= inAplDispData->barLedOnNum - ( i*LED_BAR_SEG );
		}
		//ON����bit�̂�1���Z�b�g���邽�߁A������
		outDrvLedBarData.data[i].data = 0;
		for( j=0 ; j<onLedNum ; j++ ){
			//���ʃr�b�g����1���Z�b�g���A�V�t�g�ŕK�v�����߂�
			
			//�V�t�g���Ă���Z�b�g(���T�͈Ӗ��Ȃ�
			outDrvLedBarData.data[i].data <<= 1;
			outDrvLedBarData.data[i].bit.bit0	= 1;

		}
	}
	outDrvLedBarData.bright = inAplDispData->brightBarled;
	setDrvOutShiftRegLedBar( &outDrvLedBarData );
}


