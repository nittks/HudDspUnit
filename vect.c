#include <avr/interrupt.h>
#include <avr/io.h>
#include "vect_inc.h"
#include "main.h"
#include "drvUart.h"
#include "drvInSw.h"
#include "drvEep.h"
#include "drvOutSerialLed.h"


ISR(TCA0_OVF_vect)	//�^�C�}���荞��
{
	interTaskTime();
}
ISR(USART0_RXC_vect)		//UART��M������
{
	interGetUartRxData();
}
ISR(USART0_DRE_vect)	//UART DataRegisterEmpty���M���W�X�^�󊄍���
{
	interSetTxBuffer();
}
ISR(USART0_TXC_vect)
{
	interChangeNextCCLPort();
}

ISR(PCINT0_vect){		//�|�[�g�ω����荞��
	interPcInt00_07();
}
ISR(PCINT1_vect){		//�|�[�g�ω����荞��
	interPcInt08_14();
}
ISR(NVMCTRL_EE_vect){		//EEPROM�A�N�Z�X�\������
	interEepRedy();
}
