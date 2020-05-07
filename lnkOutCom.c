#include <avr/io.h>
#include <string.h>
#include <stdbool.h>
#include "lnkOutCom_inc.h"
#include "lnkOutCom.h"

#include "aplData.h"
#include "aplCtrl.h"
#include "drvUart.h"

static unsigned char	txReqFlag;

//********************************************************************************
// ������
//********************************************************************************
void initLnkOutCom( void )
{
}
//********************************************************************************
// ���̓��j�b�g�ݒ�ύX�v���t���[�����M
//********************************************************************************
void setLnkOutCom( void )
{
	txReqFlag = true;
}
//********************************************************************************
// ���C������
//********************************************************************************
void lnkOutComMain( void )
{
	APL_CTRL_SET_PALSE	*inAplCtrlSetPalse;
	DRV_UART_TX			outDrvUartTx;
	unsigned char		i;
	unsigned char		sum;
	
	inAplCtrlSetPalse = getAplCtrlSetPalse();
	
	if( txReqFlag == true ){
		txReqFlag = false;
		
		//�ʐMUART�o�̓f�[�^�Z�b�g
		outDrvUartTx.txData[0]	= UART_TX_ID;
		outDrvUartTx.txData[1]	= UART_TX_LENGTH;
		outDrvUartTx.txData[2]	= (inAplCtrlSetPalse->rev << 4) | (inAplCtrlSetPalse->speed);

		sum = 0;
		for( i=0 ; i<UART_TX_LENGTH-1 ; i++ ){
			sum += outDrvUartTx.txData[i];
		}

		outDrvUartTx.txData[3]	= sum;
		outDrvUartTx.txDataNum	= UART_TX_LENGTH;

		setDrvUartTx( &outDrvUartTx );
	}
}
