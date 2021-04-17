#ifndef MAIN_H
#define MAIN_H

#define F_CPU	((uint32_t)20000000)		// delay.h���g�p����ꍇ�́A���main.h��include����
#define F_PDIV	((uint8_t)1)
	// ��SerialLed�̐M������400ns���^�C�}�ō�邽�߁A�J�E���g���������ɂȂ镪���l�ōł��ᑬ�ɂȂ�{����I���B

enum{
	TASK_DRV_IN_MAIN,
	TASK_LINK_IN,
	TASK_APL,
	TASK_LINK_OUT,
	TASK_DRV_OUT,
	TASK_POWER_LED,
	TASK_UART_CHANGE_TX,
	TASK_MAX
};

extern void interTaskTime( void );
extern void initMain( void );
extern void enableTask( unsigned char taskNo );
extern void disableTask( unsigned char taskNo );

#endif
