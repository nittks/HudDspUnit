
#include <avr/io.h>
#include <avr/interrupt.h>  //割り込みを使用するため

#include "main.h"
#include "drvInSw_inc.h"
#include "drvInSw.h"
#include "hardware.h"

//出力データ
static DRV_IN_SW		drvInSwData;		//スイッチ入力データ

//内部状態
static ROT_ENC_CNT		rotEncCnt[ROT_ENC_NUM];
static PORT_PUSH_SW		portPushSw[PUSH_SW_NUM];

volatile static	uint8_t			debugBuf255[255]={0};
volatile static	uint8_t			debugCnt=0;


static void chkRotateVectCnt( unsigned char portNo );
static void inputRotEnc( void );
static void inputPushSw( void );
static void startDebounceTimer( void );
static void stopDebounceTimer( void );


//********************************************************************************//
// 初期化
//********************************************************************************//
void initDrvInSw( void )
{
	unsigned char	i;

	//出力
	for( i=0 ; i<ROT_ENC_NUM; i++ ){
		drvInSwData.rotEncState[i]	= DRV_IN_ROT_ENC_STATE_STOP;	//入力無し
		rotEncCnt[i].rotEncState		= ROT_ENC_STATE_WAIT;	//監視開始
		rotEncCnt[i].rotEncDebTimeCnt	= 0;		//デバウンス経過時間カウント
		rotEncCnt[i].grayCode			= 0;
		rotEncCnt[i].rotateVect			= 0;
	}
	for( i=0 ; i<PUSH_SW_NUM; i++ ){
		drvInSwData.pushSwState[i]	= DRV_IN_PUSH_SW_STATE_OFF;	//入力無し
		portPushSw[i].portIn		= PORT_OFF;
		portPushSw[i].portInLatch	= PORT_OFF;
		portPushSw[i].state			= PUSH_SW_STATE_OFF;
		portPushSw[i].cntTimeDebounce	= 0;
	}
	
	PORTD.DIRCLR	= 0x07;
	PORTD.PIN0CTRL	|= ( PORT_INVEN_bm | PORT_PULLUPEN_bm );	//反転設定で従来動作
	PORTD.PIN1CTRL	|= ( PORT_INVEN_bm | PORT_PULLUPEN_bm );
	PORTD.PIN2CTRL	|= ( PORT_PULLUPEN_bm );
	
	//割込み要求クリア(
	PORTD.INTFLAGS	= 0x03;		//PD0-1

	TCB0.CTRLA		= TCB_CLKSEL_CLKDIV2_gc;		// CLK_PER/2
	TCB0.CTRLB		= TCB_CNTMODE_SINGLE_gc;
	TCB0.CNT		= 0;
	TCB0.CCMP		= ROT_ENC_DEBOUNCE_TIME_CNT;	// TOP値
}

//********************************************************************************//
// データ取得
//********************************************************************************//
DRV_IN_SW *getDrvInSw( void )
{
	return( &drvInSwData );
}

//********************************************************************************//
// メイン処理
//********************************************************************************//
void drvInSwMain( void )
{
	cli();

	inputRotEnc();
	inputPushSw();

	sei();
}

//********************************************************************************//
// ロータリーエンコーダー入力検知処理
//********************************************************************************//
static void inputRotEnc( void )
{
	uint8_t	i;
	
	for( i=0 ; i<ROT_ENC_NUM; i++ ){
		int8_t		rotVectCnt;
		cli();
		rotVectCnt	= rotEncCnt[i].rotCntFwd - rotEncCnt[i].rotCntRev;


		if( rotVectCnt >= 1 ){
			drvInSwData.rotEncState[i]	= DRV_IN_ROT_ENC_STATE_FORWARD;
		}else if( rotVectCnt <= -1 ){
			drvInSwData.rotEncState[i]	= DRV_IN_ROT_ENC_STATE_REVERCE;
		}else{
			drvInSwData.rotEncState[i]	= DRV_IN_ROT_ENC_STATE_STOP;
		}
		rotEncCnt[i].rotCntFwd	= 0;
		rotEncCnt[i].rotCntRev	= 0;
		sei();
	}
	/*
		//正転
		if( rotEncCnt[i].rotEncState == ROT_ENC_STATE_FORWARD ){
			rotEncCnt[i].rotEncState = ROT_ENC_STATE_DEBOUNCE;
			rotEncCnt[i].rotEncDebTimeCnt = 0;
			drvInSwData.rotEncState[i]	= DRV_IN_ROT_ENC_STATE_FORWARD;
		//逆転
		}else if( rotEncCnt[i].rotEncState == ROT_ENC_STATE_REVERCE ){
			rotEncCnt[i].rotEncState = ROT_ENC_STATE_DEBOUNCE;
			rotEncCnt[i].rotEncDebTimeCnt = 0;
			drvInSwData.rotEncState[i]	= DRV_IN_ROT_ENC_STATE_REVERCE;
		//デバウンス待機
		}else if( rotEncCnt[i].rotEncState == ROT_ENC_STATE_DEBOUNCE ){
			
			drvInSwData.rotEncState[i]	= DRV_IN_ROT_ENC_STATE_STOP;
			//デバウンス経過
			if( rotEncCnt[i].rotEncDebTimeCnt >= ROT_ENC_DEBTIME ){
				rotEncCnt[i].rotEncDebTimeCnt = 0;
				rotEncCnt[i].rotEncState = ROT_ENC_STATE_WAIT;
			}else{
				rotEncCnt[i].rotEncDebTimeCnt++;
			}
		}
	}
	*/
}

//********************************************************************************//
// プッシュスイッチ入力検知処理
//********************************************************************************//
static void inputPushSw( void )
{
	uint8_t			i;
	PORT_PUSH_SW*	pSw;

	//ポート入力
	portPushSw[NO_0].portIn	= (PORTD.IN>>PIN2_bp) & 0x01;
	
	//チャタリングキャンセル(トリガで拾って、ラッチで止める
	for( i=0 ; i<PUSH_SW_NUM; i++ ){
		pSw	= &portPushSw[i];		//記述が長いので短くする
		if( pSw->cntTimeDebounce <= PUSH_SW_DEBOUNCE_10MS ){
			pSw->cntTimeDebounce++;
		}else{
			if( pSw->portInLatch	!= pSw->portIn ){
				pSw->portInLatch		= pSw->portIn;
				pSw->cntTimeDebounce	= 0;
			}
		}
	}

	//判定状態遷移(わかりにくい、読みにくい
	for( i=0 ; i<PUSH_SW_NUM; i++ ){
		pSw	= &portPushSw[i];		//記述が長いので短くする
		switch( pSw->state ){
			case PUSH_SW_STATE_OFF:
				if( pSw->portInLatch == PORT_ON ){
					pSw->state	= PUSH_SW_STATE_ON;
				}
				drvInSwData.pushSwState[i] = DRV_IN_PUSH_SW_STATE_OFF;
				pSw->cntTimeJudge	= 0;
				break;

			case PUSH_SW_STATE_ON:
				if( pSw->portInLatch == PORT_ON ){
					if( pSw->cntTimeJudge < PUSH_SW_LONGON_10MS ){
						pSw->cntTimeJudge++;
						if( pSw->cntTimeJudge == PUSH_SW_LONGON_10MS ){	//確定時、1発だけ送る
							drvInSwData.pushSwState[i] = DRV_IN_PUSH_SW_STATE_LONGON;
						}
					}else{
						drvInSwData.pushSwState[i] = DRV_IN_PUSH_SW_STATE_OFF;
					}
				}else{
					if( pSw->cntTimeJudge >= PUSH_SW_LONGON_10MS ){
						pSw->state	= PUSH_SW_STATE_OFF;
						drvInSwData.pushSwState[i] = DRV_IN_PUSH_SW_STATE_OFF;
					}else{
						pSw->state	= PUSH_SW_STATE_WAIT_DOUBLE;
						pSw->cntTimeJudge	= 0;
					}
				}
				break;

			case PUSH_SW_STATE_WAIT_DOUBLE:
				if( pSw->portInLatch == PORT_OFF ){
					if( pSw->cntTimeJudge <= PUSH_SW_DOUBLE_TIMEOUT_10MS ){
						pSw->cntTimeJudge++;
					}else{
						pSw->state	= PUSH_SW_STATE_OFF;
						drvInSwData.pushSwState[i] = DRV_IN_PUSH_SW_STATE_ON;	//短押し確定
					}
				}else{
						pSw->state	= PUSH_SW_STATE_DOUBLE;
						drvInSwData.pushSwState[i] = DRV_IN_PUSH_SW_STATE_DOUBLEON;
						pSw->cntTimeJudge	= 0;
				}
				break;
			case PUSH_SW_STATE_DOUBLE:
				drvInSwData.pushSwState[i] = DRV_IN_PUSH_SW_STATE_OFF;	//遷移時単発送信のため、遷移後OFF
				if( pSw->portInLatch == PORT_OFF ){
					pSw->state	= PUSH_SW_STATE_OFF;
				}
				break;
			default:
				break;
		}
	}
}

//********************************************************************************//
// ポート変化割り込み
// pushswも割り込み使ってないから、割り込みしなくても良いかもしれない
// ->ロータリーエンコーダを早く回したときの周期が3msほどなので、割り込みしたい
// A相B相合わせた状態の切り替わりまでの時間は0.5msほど
// ->だるすぎると遅い
// バタつく期間は長くて4ms。バタつき中にレベルが一定の時間は長くて0.4msほど
// ->個々のポートに対し、チャタリングキャンセルフィルタをかける
//   0.5ms同じレベルを継続でレベル確定。
// ->タイマが残ってないので、500usポート割り込み停止
//********************************************************************************//
void interPortD( void )
{
	unsigned char	portTmp;		//ロータリーエンコーダー全4入力一時保存
	uint8_t			portTmpPre	= 0;

	cli();
	PORTD.INTFLAGS	= /*PORTD.INTFLAGS &*/ 0x03;	// 割り込み要求クリア

	// デバウンスタイム経過まで、ポート割り込み停止
	PORTD.PIN0CTRL	&= ( ~PORT_INVEN_bm );
	PORTD.PIN1CTRL	&= ( ~PORT_INVEN_bm );
	startDebounceTimer();

	portTmp	= (~PORTD.IN) & 0x03;

	if( portTmp != (rotEncCnt[NO_SET].grayCode & 0x03)){	//ポート変化
		rotEncCnt[NO_SET].grayCode	= (((rotEncCnt[NO_SET].grayCode << 2) | portTmp) & 0x0F);
		chkRotateVectCnt( NO_SET );
	}

	sei();
}
//********************************************************************************//
// 回転変化量チェック
//********************************************************************************//
static void chkRotateVectCnt( unsigned char portNo )
{
	//グレイコードテーブルをもとに、回転方向変化量を加算していく
	rotEncCnt[portNo].rotateVect		+=grayCodeTable[ rotEncCnt[portNo].grayCode ];

	//変化量が4or-4で回転方向決定
	if( rotEncCnt[portNo].rotateVect <= ROT_VECT_FORWARD ){
		rotEncCnt[portNo].rotEncState = ROT_ENC_STATE_FORWARD;
		rotEncCnt[portNo].rotCntFwd++;
		rotEncCnt[portNo].rotateVect	 = 0;
	}else if( rotEncCnt[portNo].rotateVect >= ROT_VECT_REVERCE ){
		rotEncCnt[portNo].rotEncState = ROT_ENC_STATE_REVERCE;
		rotEncCnt[portNo].rotCntRev++;
		rotEncCnt[portNo].rotateVect	 = 0;
	}

}

static void startDebounceTimer( void )
{
	TCB0.CNT		= 0;
	TCB0.INTCTRL	= TCB_CAPT_bm;
	TCB0.INTFLAGS	= TCB_CAPT_bm;

	TCB0.CTRLA		= TCB_ENABLE_bm;
}

static void stopDebounceTimer( void )
{
	TCB0.CTRLA		= TCB_ENABLE_bm;

	TCB0.CNT		= 0;
	TCB0.INTCTRL	= ~TCB_CAPT_bm;
	TCB0.INTFLAGS	= TCB_CAPT_bm;
}
void interDebounceTime( void )
{
	cli();

	stopDebounceTimer();

	PORTD.PIN0CTRL	|= ( PORT_INVEN_bm );
	PORTD.PIN1CTRL	|= ( PORT_INVEN_bm );

	sei();
}