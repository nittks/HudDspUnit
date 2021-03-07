/*
 * periodTask.c
 *
 * Created: 2016/05/06 8:25:08
 *  Author: sin
 */ 

/*
����^�X�N
MPU		:ATmega328
clock	:����8MHz
*/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdbool.h>

#include "timer.h"
#include "drvInMain.h"
#include "drvOutMain.h"
#include "lnkInMain.h"
#include "lnkOutMain.h"
#include "aplMain.h"

#include "drvUart.h"
#include "main_inc.h"
#include "main.h"


#define INT_CNT_MAX	((unsigned char)125)	//8us*125=1ms�����荞��

static void mainTask( void );
static void initReg( void );
static void USART_Init( unsigned short baud );

//********************************************************************************
// ���C��
//********************************************************************************
int main(void)
{
	initMain();

	while(1)
	{
		//����������X���[�v�B�^�C�}���荞�݂ŋN������ēx���[�v�J�n
		mainTask();
		set_sleep_mode(SLEEP_MODE_IDLE);
	}
}

//********************************************************************************
// ������
//********************************************************************************
void initMain( void )
{
	initReg();

	initDrvIn();
	initLnkIn();
	initApl();
	initLnkOut();
	initDrvOut();
	
}
static void mainTask( void )
{
	unsigned char	i;
	
	//�����^�X�N���s���ԃ`�F�b�N
	for( i=0; i<TASK_MAX ; i++){
		if( taskParameter[i].regist == true ){	//�^�X�N�L��

			//�������Ԍo�߂��Ă����珈��
			if( taskParameter[i].currentTime <= 0 ){
				//���Z�b�g
				taskParameter[i].currentTime = taskParameter[i].cycleTime; 
				//�^�X�N���s
				taskParameter[i].func();
			}
		}
	}
}


//********************************************************************************
// �^�X�N���ԃJ�E���g
//********************************************************************************
void interTaskTime( void )
{
	unsigned char	i;
	
	//�����^�X�N���s���ԃ`�F�b�N
	for( i=0; i<TASK_MAX ; i++){
		if( taskParameter[i].regist == true ){	//�o�^�L���^�X�N�̂�

			//10ms��1�J�E���g�_�E���B
			if( taskParameter[i].currentTime > 0 ){
				taskParameter[i].currentTime--;
			}
		}
	}
}
//********************************************************************************//
// �^�X�N�L����
//********************************************************************************//
void enableTask( unsigned char taskNo )
{
	//�^�X�N�L����
	taskParameter[taskNo].regist	= true;
	//�����Z�b�g
	taskParameter[taskNo].currentTime	= taskParameter[taskNo].cycleTime;
}
//********************************************************************************//
// �^�X�N������
//********************************************************************************//
void disableTask( unsigned char taskNo )
{
	//�^�X�N�L����
	taskParameter[taskNo].regist	= false;
}

//********************************************************************************
// ���W�X�^������
//********************************************************************************
static void initReg(void)
{
	cli();

	//I/O�ݒ�
	PORTA.DIR	-= 0x00;
	PORTC.DIR	-= 0x00;
	PORTD.DIR	-= 0x00;
	PORTE.DIR	-= 0x00;

	PORTA.PIN0CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN1CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN2CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN3CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN4CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN5CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN6CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN7CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;

	PORTC.PIN0CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTC.PIN1CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTC.PIN2CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTC.PIN3CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;

	PORTD.PIN0CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_BOTHEDGES_gc;
	PORTD.PIN1CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_BOTHEDGES_gc;
	PORTD.PIN2CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_BOTHEDGES_gc;
	PORTD.PIN3CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN4CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN5CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN6CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN7CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;

	PORTE.PIN0CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTE.PIN1CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTE.PIN2CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTE.PIN3CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTE.PIN4CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTE.PIN5CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTE.PIN6CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;
	PORTE.PIN7CTRL	= (1<<PORT_PULLUPEN_bp) | PORT_ISC_INPUT_DISABLE_gc;

	//�N���b�N�ݒ�
	CLKCTRL.MCLKCTRLB	= SET_CLKCTRL_MCLKCTRLB( CLKCTRL_PDIV_64X_gc );

	//�^�C�}�ݒ�
	TCA0.SINGLE.CTRLESET	= SET_TCA_CTRLESET( TCA_SINGLE_DIR_UP_gc );
	TCA0.SINGLE.CTRLA		= SET_TCA_CTRLA( TCA_SINGLE_CLKSEL_DIV2_gc );
	TCA0.SINGLE.PER			= (double)0.001/((double)1/(FOSC/64/2));
	TCA0.SINGLE.CTRLB		= SET_TCA_CTRLB( TCA_SINGLE_WGMODE_NORMAL_gc );
	TCA0.SINGLE.INTCTRL		= SET_TCA_INTCTRL( OVF_EN );
	
	//���荞�݋���
	sei();
}

//********************************************************************************
// ����m�F�pLED�_��
//********************************************************************************
static void powerLed( void )
{
//	PORTB ^= (1<<PB1);
}
