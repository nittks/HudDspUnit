
#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>  //���荞�݂��g�p���邽��

#include "drvInSw_inc.h"
#include "drvInSw.h"
#include "hardware.h"

//�o�̓f�[�^
static DRV_IN_SW		drvInSwData;		//�X�C�b�`���̓f�[�^

//�������
static ROT_ENC_STATE	rotEncState[ROT_ENC_NUM];		//���[�^���[�G���R�[�_�[���
static PUSH_SW_STATE	pushSwState[PUSH_SW_NUM];		//�v�b�V���X�C�b�`���
static unsigned char	rotEncDebTimeCnt[ROT_ENC_NUM];	//�f�o�E���X�o�ߎ��ԃJ�E���g
static unsigned char	pushSwDebTimeCnt[PUSH_SW_NUM];	//�f�o�E���X�o�ߎ��ԃJ�E���g
static unsigned char	grayCode[ROT_ENC_NUM];			//ROTENC���́B�O��l��ۑ����A����l�ƍ��킹�O���C�R�[�h������
static signed char		rotateVect[ROT_ENC_NUM];		//ROTENC�B��]�����ω��ʃJ�E���g�B�t�����̓}�C�i�X�l�̂���singed���g�p

static void chkRotateVectCnt( unsigned char portNo );
//********************************************************************************//
// ������
//********************************************************************************//
void initDrvInSw( void )
{
	unsigned char	i;

	//�o��
	for( i=0 ; i<ROT_ENC_NUM; i++ ){
		drvInSwData.rotEncState[i]	= DRV_IN_ROT_ENC_STATE_STOP;	//���͖���
		drvInSwData.pushSwState[i]	= DRV_IN_PUSH_SW_STATE_OFF;	//���͖���

		rotEncState[i]	= ROT_ENC_STATE_WAIT;	//�Ď��J�n
		pushSwState[i]	= PUSH_SW_STATE_OFF;
		rotEncDebTimeCnt[i]	= 0;		//�f�o�E���X�o�ߎ��ԃJ�E���g
		pushSwDebTimeCnt[i]	= 0;		//�f�o�E���X�o�ߎ��ԃJ�E���g
		grayCode[i]		= 0;
		rotateVect[i]	= 0;
	}
	
	//�����݃��W�X�^�ݒ�(_inc.h���Œ�`
	PCICR	= SET_PCICR;
	PCMSK0	= SET_PCMSK0;
	PCMSK1	= SET_PCMSK1;
	
}

//********************************************************************************//
// �f�[�^�擾
//********************************************************************************//
DRV_IN_SW *getDrvInSw( void )
{
	return( &drvInSwData );
}

//********************************************************************************//
// ���C������
//********************************************************************************//
void drvInSwMain( void )
{
	unsigned char	i;
	unsigned char	portTmp;

	cli();
	/****************************************/
	// ���[�^���[�G���R�[�_�[���͌��m����
	/****************************************/
	
	for( i=0 ; i<ROT_ENC_NUM; i++ ){
		//���]
		if( rotEncState[i] == ROT_ENC_STATE_FORWARD ){
			rotEncState[i] = ROT_ENC_STATE_DEBOUNCE;
			rotEncDebTimeCnt[i] = 0;
			drvInSwData.rotEncState[i]	= DRV_IN_ROT_ENC_STATE_FORWARD;
		//�t�]
		}else if( rotEncState[i] == ROT_ENC_STATE_REVERCE ){
			rotEncState[i] = ROT_ENC_STATE_DEBOUNCE;
			rotEncDebTimeCnt[i] = 0;
			drvInSwData.rotEncState[i]	= DRV_IN_ROT_ENC_STATE_REVERCE;
		//�f�o�E���X�ҋ@
		}else if( rotEncState[i] == ROT_ENC_STATE_DEBOUNCE ){
			
			drvInSwData.rotEncState[i]	= DRV_IN_ROT_ENC_STATE_STOP;
			//�f�o�E���X�o��
			if( rotEncDebTimeCnt[i] >= ROT_ENC_DEBTIME ){
				rotEncDebTimeCnt[i] = 0;
				rotEncState[i] = ROT_ENC_STATE_WAIT;
			}else{
				rotEncDebTimeCnt[i]++;
			}
		}
	}
	/****************************************/
	// �v�b�V���X�C�b�`���͌��m����
	/****************************************/
	portTmp	= ((~PINB)>>PB4) & 0x03;		//2�|�[�g���ۑ�,ON��LOW�̂��ߔ��]

	for( i=0 ; i<PUSH_SW_NUM; i++ ){
		if( pushSwState[i] == PUSH_SW_STATE_OFF ){
			//�I�t
			drvInSwData.pushSwState[i] = DRV_IN_PUSH_SW_STATE_OFF;
		}else if( pushSwState[i] == PUSH_SW_STATE_ON ){
			//�I��
			pushSwDebTimeCnt[i] = 0;
			pushSwState[i] = PUSH_SW_STATE_DEBOUNCE;
		}else if( pushSwState[i] == PUSH_SW_STATE_DEBOUNCE ){
			//�f�o�E���X�҂�
			pushSwDebTimeCnt[i]++;
			if( ((portTmp & (PORT_ON<<i))>>i) == PORT_OFF ){
				if( pushSwDebTimeCnt[i] >= PUSH_SW_DEBTIME ){
					//�Z��
					pushSwState[i] = PUSH_SW_STATE_OFF;
					drvInSwData.pushSwState[i] = DRV_IN_PUSH_SW_STATE_ON;
				}
			}else{
				if( pushSwDebTimeCnt[i] >= PUSH_SW_LONGTIME ){
					//������
					pushSwState[i] = PUSH_SW_STATE_OFF;
					drvInSwData.pushSwState[i] = DRV_IN_PUSH_SW_STATE_LONGON;
				}
			}
		}
	}
	sei();
}

//********************************************************************************//
// �|�[�g�ω����荞��
//********************************************************************************//
void interPcInt08_14( void )
{
	unsigned char	portTmp;		//���[�^���[�G���R�[�_�[�S4���͈ꎞ�ۑ�

	cli();
	portTmp	= ((~PINC) >>PC4) & 0x03;		//4�|�[�g���ۑ�,ON��LOW�̂��ߔ��]

	if( (portTmp&ROT_ENC_0_POS) != (grayCode[NO_SET]&0x03)){	//�|�[�g�ω�
		grayCode[NO_SET]	= ((grayCode[NO_SET] << 2) | (portTmp & ROT_ENC_0_POS)) & 0x0F;	//4bit�̂ݎg�p
		chkRotateVectCnt( NO_SET );
	}


	sei();
}
//********************************************************************************//
// ��]�ω��ʃ`�F�b�N
//********************************************************************************//
static void chkRotateVectCnt( unsigned char portNo )
{
	//�O���C�R�[�h�e�[�u�������ƂɁA��]�����ω��ʂ����Z���Ă���
	rotateVect[portNo]		+=grayCodeTable[ grayCode[portNo] ];

	//�ω��ʂ�4or-4�ŉ�]��������
	if( rotateVect[portNo] >= ROT_VECT_FORWARD ){
		rotEncState[portNo] = ROT_ENC_STATE_FORWARD;
		rotateVect[portNo]	 = 0;
	}else if( rotateVect[portNo] <= ROT_VECT_REVERCE ){
		rotEncState[portNo] = ROT_ENC_STATE_REVERCE;
		rotateVect[portNo]	 = 0;
	}
}

//********************************************************************************//
// �|�[�g�ω����荞��
//********************************************************************************//
void interPcInt00_07( void )
{
	unsigned char	portTmp;
	unsigned char	i;

	cli();
	portTmp	= ((~PINB)>>PB4) & 0x03;		//2�|�[�g���ۑ�,ON��LOW�̂��ߔ��]

	for( i=0 ; i<PUSH_SW_NUM ; i++ ){
		if((pushSwState[i] == PUSH_SW_STATE_OFF) &&
		   (((portTmp>>i) & (PORT_ON)) == PORT_ON )){
			pushSwState[i] = PUSH_SW_STATE_ON;
		}
	}
	sei();
}
