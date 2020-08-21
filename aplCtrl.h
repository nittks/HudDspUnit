#ifndef APL_CTRL_H
#define APL_CTRL_H


//状態
typedef enum{
	APL_CTRL_STATE_BOOT,		//起動初回
	APL_CTRL_STATE_NOMARL,		//通常
	APL_CTRL_STATE_TESTDISP,	//テスト表示
	APL_CTRL_STATE_SETTING		//設定
}APL_CTRL_STATE;

//状態(テスト表示内
typedef enum{
	APL_CTRL_STATE_TEST_AUTO,
	APL_CTRL_STATE_TEST_SPEED,
	APL_CTRL_STATE_TEST_REV
}APL_CTRL_STATE_TEST;

//状態(設定内
typedef enum{
	APL_CTRL_STATE_SET_BRIGHT_7SEG,			//調光(7セグ
	APL_CTRL_STATE_SET_BRIGHT_BARLED,		//調光(バーLED
	APL_CTRL_STATE_SET_BRIGHT_DIM_7SEG,		//調光減光(7セグ
	APL_CTRL_STATE_SET_BRIGHT_DIM_BARLED,	//調光減光(バーLED
	APL_CTRL_STATE_SET_DISPCYC_7SEG,		//表示更新速度(7セグ
	APL_CTRL_STATE_SET_DISPCYC_BARLED,		//表示更新速度(バーLED
	APL_CTRL_STATE_SET_PALSE_SPEED,			//パルス仕様車速
	APL_CTRL_STATE_SET_PALSE_REV,			//パルス仕様回転数
	APL_CTRL_STATE_SET_MAX
}APL_CTRL_STATE_SET;


typedef struct {
	APL_CTRL_STATE		state;
	APL_CTRL_STATE_TEST	stateTest;
	APL_CTRL_STATE_SET	stateSet;
}APL_CTRL;

typedef struct {
	unsigned char	bright7seg;			//調光(7セグ
	unsigned char	brightBarled;		//調光(バーled
	unsigned char	brightDim7seg;		//調光減光(7セグ
	unsigned char	brightDimBarled;	//調光減光(バーled
	unsigned char	dispcyc7seg;		//表示更新速度(7セグ
	unsigned char	dispcycBarled;		//表示更新速度(バーled
}APL_CTRL_SET;

typedef struct {
	unsigned char	speed;			//パルス仕様車速   
	unsigned char	rev;			//パルス仕様回転数 
}APL_CTRL_SET_PALSE;



extern void initAplCtrl( void );
extern void aplCtrlMain( void );
extern APL_CTRL *getAplCtrl( void );
extern APL_CTRL_SET *getAplCtrlSet( void );
extern APL_CTRL_SET_PALSE *getAplCtrlSetPalse( void );
#endif
