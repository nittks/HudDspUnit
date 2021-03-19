#include <avr/interrupt.h>
#include <avr/io.h>
#include "vect_inc.h"
#include "main.h"
#include "drvUart.h"
#include "drvInSw.h"
#include "drvEep.h"
#include "drvOutSerialLed.h"


ISR(TCA0_OVF_LUNF)	//�^�C�}���荞��
{
	interTaskTime();
}
ISR(USART_RX_vect)		//UART��M������
{
	interGetUartRxData();
}
ISR(USART0_DRE_vect)	//UART DataRegisterEmpty���M���W�X�^�󊄍���
{
	interSetTxBuffer();
}

ISR(PCINT0_vect){		//�|�[�g�ω����荞��
	interPcInt00_07();
}
ISR(PCINT1_vect){		//�|�[�g�ω����荞��
	interPcInt08_14();
}
ISR(EE_READY_vect){		//EEPROM�A�N�Z�X�\������
	interEepRedy();
}
