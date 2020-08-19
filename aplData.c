
#include "aplData.h"
#include "aplData_inc.h"
#include "hardware.h"

static APL_DATA_CAR		aplDataCar;
static APL_DATA_SW		aplDataSw;
static APL_DATA_EEP		aplDataEep;


//********************************************************************************
// ������
//********************************************************************************
void initAplData( void )
{
	aplDataCar.ig				= OFF;
	aplDataCar.acc				= OFF;
	aplDataCar.ill				= OFF;
	aplDataCar.speed			= 0;
	aplDataCar.rev				= 0;
	
	aplDataSw.rotEncSet			= APL_DATA_ROT_ENC_STOP;
	aplDataSw.pushSwSet			= APL_DATA_PUSH_SW_OFF;
	aplDataSw.pushSwTest		= APL_DATA_PUSH_SW_OFF;

	aplDataEep.read				= APL_DATA_EEP_STATE_UNREAD;
	aplDataEep.bright7seg		= 0;
	aplDataEep.brightDim7seg	= 0;
	aplDataEep.dispcyc7seg		= 0;
}
//********************************************************************************
// ���C������
//********************************************************************************
void aplDataMain( void )
{

}
//********************************************************************************
// LNK����Z�b�g
//********************************************************************************
void setAplDataCar( APL_DATA_CAR *inData )
{
	aplDataCar = *inData;
}
//********************************************************************************
// LNK����Z�b�g
//********************************************************************************
void setAplDataSw( APL_DATA_SW *inData )
{
	aplDataSw = *inData;
}
//********************************************************************************
// LNK����Z�b�g
//********************************************************************************
void setAplDataEep( APL_DATA_EEP *inData )
{
	aplDataEep = *inData;
}
//********************************************************************************
// APL_DATA�擾
//********************************************************************************
APL_DATA_CAR *getAplDataCar(void)
{
	return( &aplDataCar );
}
//********************************************************************************
// APL_DATA�擾
//********************************************************************************
APL_DATA_SW *getAplDataSw(void)
{
	return( &aplDataSw );
}
//********************************************************************************
// APL_DATA�擾
//********************************************************************************
APL_DATA_EEP *getAplDataEep(void)
{
	return( &aplDataEep );
}
