#include <avr/io.h>

#include "lnkInCom_inc.h"
#include "lnkInCom.h"

#include "aplData.h"
#include "drvUart.h"

//������
void initLnkInCom( void )
{
}
//���C������
void lnkInComMain( void )
{
	DRV_UART_RX		*inDrvUartRx;
	APL_DATA_CAR	aplDataCar;
	unsigned char	i;
	unsigned char	sum;
	
	inDrvUartRx = getDrvUartRx();

	sum=0;
	for( i=0 ; i<inDrvUartRx->rxDataNum-1 ; i++ ){
		sum += inDrvUartRx->rxData[i];
	}
	
	if( sum != inDrvUartRx->rxData[inDrvUartRx->rxDataNum-1]){
		//SUM�G���[�B��������
	}else{

		aplDataCar.speed	= inDrvUartRx->rxData[UART_NO_SPEED];
		aplDataCar.rev		= (( (unsigned short)inDrvUartRx->rxData[UART_NO_REV0] << 8) |
												 inDrvUartRx->rxData[UART_NO_REV1]);
		aplDataCar.palseSetSpeed	= inDrvUartRx->rxData[UART_NO_PALSE_SET] & 0x0F;
		aplDataCar.palseSetRev		= inDrvUartRx->rxData[UART_NO_PALSE_SET] >> 4;

		aplDataCar.ig	= (inDrvUartRx->rxData[UART_NO_CAR_SIG] & (1<<POS_IG)) >> POS_IG;
		aplDataCar.acc	= (inDrvUartRx->rxData[UART_NO_CAR_SIG] & (1<<POS_ACC)) >> POS_ACC;
		aplDataCar.ill	= (inDrvUartRx->rxData[UART_NO_CAR_SIG] & (1<<POS_ILL)) >> POS_ILL;
		aplDataCar.vtc	= (inDrvUartRx->rxData[UART_NO_CAR_SIG] & (1<<POS_VTC)) >> POS_VTC;
		
		
		//�͈̓u���b�N
		if( (aplDataCar.palseSetSpeed < UART_PALSE_SET_SPEED_MIN ) || 
			(UART_PALSE_SET_SPEED_MAX < aplDataCar.palseSetSpeed) )
		{
			aplDataCar.palseSetSpeed = UART_PALSE_SET_SPEED_DEFAULT;	
		}

		if( (aplDataCar.palseSetRev < UART_PALSE_SET_REV_MIN ) ||
		(UART_PALSE_SET_REV_MAX < aplDataCar.palseSetRev) )
		{
			aplDataCar.palseSetRev = UART_PALSE_SET_REV_DEFAULT;
		}	
		
		setAplDataCar( &aplDataCar );
	}
}