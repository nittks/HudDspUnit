#ifndef DRV_UART_INC_H
#define DRV_UART_INC_H

#include "main.h"

#define USEBAUD	((unsigned short)9600)
//----------------------------------------
// �S��
//----------------------------------------
typedef enum{
	UART_STATE_STANDBY,		//��M��~(�I��
	UART_STATE_TRANS,		//���M��
	UART_STATE_RECEIVE		//��M��
}UART_STATE;

#define	RS485_AUTO_XDIR OFF	(0)
#define	RS485_AUTO_XDIR_ON	(1)
#define	RS485_AUTO_TX_OUTPUT_OFF	(0)
#define	RS485_AUTO_TX_OUTPUT_ON		(1)

enum{
	RXMODE_NORMAL	= 0x0,
	RXMODE_CLK2X	= 0x1,
	RXMODE_GENAUTO	= 0x2,
	RXMODE_LINAUTO	= 0x3,
};
#define	SET_CTRLA( xdir , txd )		( (USART1.CTRLA & (~USART_RS485_gm)) | (( xdir << USART_RS4851_bp ) | (txd << USART_RS4850_bp)))
#define	SET_CTRLB( rxmode )			( (USART1.CTRLB & USART_RXMODE_gm) | ( rxmode << USART_RXMODE_gp ))

enum{
	CMODE_ASYNCHRONOUS	= 0x0,
	CMODE_SYNCHRONOUS	= 0x1,
	CMODE_IRCOM			= 0x2,
	CMODE_MSPI			= 0x3,
};

enum{
	PMODE_DISABLED	= 0x0,
	PMODE_EVEN		= 0x2,
	PMODE_ODD		= 0x3,
};

enum{
	SBMODE_1BIT	= 0,
	SBMODE_2BIT	= 1,
};
enum{
	CHSIZE_5BIT		= 0x0,
	CHSIZE_6BIT		= 0x1,
	CHSIZE_7BIT		= 0x2,
	CHSIZE_8BIT		= 0x3,
	CHSIZE_9BITL	= 0x6,
	CHSIZE_9BITH	= 0x7,
};
#define	SET_CTRLC( cmode , pmode , sbmode , chsize )		((cmode<<USART_CMODE_gp)|(pmode<<USART_PMODE_gp)|(sbmode<<USART_SBMODE_bp)|(chsize<<USART_CHSIZE_gp))


//----------------------------------------
// ���M
//----------------------------------------
#define UART_DATA_REG_EMP_FLG			((USART1.STATUS & USART_DREIF_bm) >> USART_DREIF_bp)
#define DREIF_EMPTY		(1)
#define DREIF_NOEMPTY	(0)
#define DI_UART_TX						(USART1.CTRLB = USART1.CTRLB & (~USART_RXEN_bm))	//���M�֎~
#define EN_UART_TX						(USART1.CTRLB = USART1.CTRLB | (USART_RXEN_bm))		//���M����
#define DI_INTER_UART_TX_REG_EMPTY		(USART1.CTRLA = USART1.CTRLA & (~USART_DREIF_bm))	//���M�o�b�t�@�󊄍��݋֎~
#define EN_INTER_UART_TX_REG_EMPTY		(USART1.CTRLA = USART1.CTRLA | (USART_DREIF_bm))	//���M�o�b�t�@�󊄍��݋���
#define DI_INTER_UART_TX_FIN			(USART1.CTRLA = USART1.CTRLA & (~USART_TXCIE_bm))	//���M���������݋���
#define EN_INTER_UART_TX_FIN			(USART1.CTRLA = USART1.CTRLA | (USART_TXCIE_bm))	//���M���������݋���
//----------------------------------------
// ��M
//----------------------------------------
#define UART_REG_RXIC					((USART1.RXDATAH & USART_RXCIF_bm) >> USART_RXCIF_bp)		//UART��M�����t���O
#define RXC_IN_DATA						(1)		//��M�f�[�^�L��
#define RXC_NO_DATA						(0)		//��M�f�[�^����

#define RX_BUF_SIZE		0xF;	//��M�o�b�t�@�T�C�Y

#define UART_DATAPOS_ID			0		//�f�[�^�ʒuID
#define UART_DATAPOS_LENGTH		1		//�f�[�^�ʒu�����O�X
#define UART_FRAME_TIMEOUT		2		//�^�C���A�E�g(10(�t���[������)-7(���M����)=2(�󂫎���

#define UART_ID_CARDATA			0x11		//�t���[��ID

#define DI_UART_RX						(USART1.CTRLB = USART1.CTRLB & (~USART_RXEN_bm))	//��M�֎~
#define EN_UART_RX						(USART1.CTRLB = USART1.CTRLB | (USART_RXEN_bm))	//��M����
#define DI_INTER_UART_RX_COMP			(USART1.CTRLA = USART1.CTRLA & (~USART_RXCIE_bm))	//��M���������݋֎~
#define EN_INTER_UART_RX_COMP			(USART1.CTRLA = USART1.CTRLA | (USART_RXCIE_bm))	//��M���������݋���

#endif
