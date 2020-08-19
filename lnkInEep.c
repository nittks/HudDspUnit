#include <avr/io.h>
#include <stdbool.h>

#include "lnkInEep_inc.h"
#include "lnkInEep.h"

#include "hardware.h"
#include "aplData.h"
#include "drvEep.h"

static unsigned char	eepReadFlag;		//����EEP�f�[�^���f�ς݃t���O

//********************************************************************************
// ������
//********************************************************************************
void initLnkInEep( void )
{
	eepReadFlag = false;
}
//********************************************************************************
// ���C������
//********************************************************************************
void lnkInEepMain( void )
{
	DRV_EEP_READ	*inDrvEep;
	APL_DATA_EEP	aplDataEep;
	unsigned char	i;
	unsigned char	sum;

	//SW���̓h���C�o�f�[�^�擾
	inDrvEep = getDrvEep();

	if( eepReadFlag == false ){		//�����f
		if( inDrvEep->readState == DRV_EEP_READ_STATE_READED ){
			eepReadFlag = true;		//���f�ς�

			sum = 0;
			for( i=0 ; i<DRV_EEP_MAP_MAX-1 ; i++ ){	//SUM�������f�[�^
				sum += inDrvEep->val[i];
			}
			//SUM����
			if( sum == inDrvEep->val[DRV_EEP_MAP_SUM] ){
				i=0;
				aplDataEep.read				= APL_DATA_EEP_STATE_READED;
				aplDataEep.bright7seg		= inDrvEep->val[i++];
				aplDataEep.brightBarled		= inDrvEep->val[i++];
				aplDataEep.brightDim7seg	= inDrvEep->val[i++];
				aplDataEep.brightDimBarled	= inDrvEep->val[i++];
				aplDataEep.dispcyc7seg		= inDrvEep->val[i++];
				aplDataEep.dispcycBarled	= inDrvEep->val[i++]; 
			}else{
				aplDataEep.read				= APL_DATA_EEP_STATE_SUMERROR;
				aplDataEep.bright7seg		= 0;
				aplDataEep.brightBarled		= 0;
				aplDataEep.brightDim7seg	= 0;
				aplDataEep.brightDimBarled	= 0;
				aplDataEep.dispcyc7seg		= 0;
				aplDataEep.dispcycBarled	= 0;
			}
			setAplDataEep( &aplDataEep );
		}
	}
}
