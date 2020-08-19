#ifndef DRV_EEP_INC_H
#define DRV_EEP_INC_H


#define	EEP_READY_FAIL		0

#define	INT_EEP_ENABLE			(EECR = EECR | (1<<EERIE))
#define	INT_EEP_DISABLE			(EECR = EECR & (~(1<<EERIE)))

typedef enum{
	EEP_STATE_BOOT,
	EEP_STATE_READY,
	EEP_STATE_READ,
	EEP_STATE_WRITE
}EEP_STATE;
#endif
