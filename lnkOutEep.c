#include <stdbool.h>
#include "lnkOutEep_inc.h"
#include "lnkOutEep.h"
#include "aplCtrl.h"
#include "drvEep.h"

static unsigned char	eepReq;

//********************************************************************************
// ������
//********************************************************************************
void initLnkOutEep( void )
{
	eepReq = false;
}
//********************************************************************************
// APL���Z�b�g
//********************************************************************************
void setLnkOutEep( void )
{
	eepReq = true;
}
//********************************************************************************
// ���C������
//********************************************************************************
void lnkOutEepMain( void )
{
	APL_CTRL_SET	*inAplCtrlSet;
	DRV_EEP_WRITE	outDrvEepWrite;
	unsigned char	i;
	
	inAplCtrlSet = getAplCtrlSet();

	//�����ݗv���L
	if( eepReq == true ){
		eepReq = false;

		i=0;
		outDrvEepWrite.val[i++]	= inAplCtrlSet->bright7seg;
		outDrvEepWrite.val[i++]	= inAplCtrlSet->brightBarled;
		outDrvEepWrite.val[i++]	= inAplCtrlSet->brightDim7seg;
		outDrvEepWrite.val[i++]	= inAplCtrlSet->brightDimBarled;
		outDrvEepWrite.val[i++]	= inAplCtrlSet->dispcyc7seg;
		outDrvEepWrite.val[i++]	= inAplCtrlSet->dispcycBarled;
		
		//sum�l�v�Z
		outDrvEepWrite.val[DRV_EEP_MAP_SUM] = 0;
		for( i=0 ; i<DRV_EEP_MAP_SUM ; i++ ){
			outDrvEepWrite.val[DRV_EEP_MAP_SUM] += outDrvEepWrite.val[i];
		}
		setDrvEep( &outDrvEepWrite );
	}

}
