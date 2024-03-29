#ifndef APL_CTRL_H
#define APL_CTRL_H

#include <stdbool.h>
#include <stdint.h>

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
}APL_CTRL_STATE_TEST;

//状態(設定内
typedef enum{
	APL_CTRL_STATE_SETTING_NON,
	APL_CTRL_STATE_SETTING_ROOT,
	APL_CTRL_STATE_SETTING_BRIGHT,
	APL_CTRL_STATE_SETTING_BRIGHT_DIM,
	APL_CTRL_STATE_SETTING_COLOR,
	APL_CTRL_STATE_SETTING_DSPCYC,
	APL_CTRL_STATE_SETTING_PALSE_SPEED,
	APL_CTRL_STATE_SETTING_PALSE_REV,
	APL_CTRL_STATE_SETTING_COLOR_RGB,
	APL_CTRL_STATE_SETTING_COLOR_R,
	APL_CTRL_STATE_SETTING_COLOR_G,
	APL_CTRL_STATE_SETTING_COLOR_B,
	APL_CTRL_STATE_SETTING_MAX
}APL_CTRL_STATE_SET;

typedef enum{
	APL_CTRL_STATE_SET_COLOR_USER_UNSELECT,
	APL_CTRL_STATE_SET_COLOR_USER_RED,
	APL_CTRL_STATE_SET_COLOR_USER_GREEN,
	APL_CTRL_STATE_SET_COLOR_USER_BLUE,
	APL_CTRL_STATE_SET_COLOR_USER_MAX
}APL_CTRL_STATE_SET_COLOR_USER;


typedef struct {
	uint8_t				settginHierarchy;
	APL_CTRL_STATE		state;
	APL_CTRL_STATE_TEST	stateTest;
	APL_CTRL_STATE_SET	stateSet;
	APL_CTRL_STATE_SET_COLOR_USER	stateColorUser;		//型名が長い
}APL_CTRL;


typedef struct{
	uint8_t		h;
	uint8_t		s;
	uint8_t		v;
}COLOR;

typedef struct {
	uint8_t			dspVal;				//設定画面表示値
	uint8_t			colorNo;
	COLOR			color;
	unsigned char	bright7seg;			//調光(7セグ
	unsigned char	brightDim7seg;		//調光減光(7セグ
	unsigned char	dispcyc7seg;		//表示更新速度(7セグ
}APL_CTRL_SET;

typedef struct {
	unsigned char	speed;			//パルス仕様車速   
	unsigned char	rev;			//パルス仕様回転数 
}APL_CTRL_SET_PALSE;



typedef enum{
	SET_COLOR,
	SET_PALSE_SPEED,
	SET_PALSE_REV,
	SET_ITEM_NO_MAX
}SET_ITEM_NO;

enum{
	SETTING_COLOR_WHITE,
	SETTING_COLOR_RED,
	SETTING_COLOR_YELLOW,
	SETTING_COLOR_GREEN,
	SETTING_COLOR_AQUA,
	SETTING_COLOR_BLUE,
	SETTING_COLOR_PERPLE,
	SETTING_COLOR_USER,
	SETTING_COLOR_MAX
};

enum{
	SETTING_PALSE_SPEED_04	= 1,
	SETTING_PALSE_SPEED_08,
	SETTING_PALSE_SPEED_16,
	SETTING_PALSE_SPEED_20,
	SETTING_PALSE_SPEED_25,
	SETTING_PALSE_SPEED_MAX = SETTING_PALSE_SPEED_25	//1始まりのため-1
};
enum{
	SETTING_PALSE_REV_01	= 1,
	SETTING_PALSE_REV_02,
	SETTING_PALSE_REV_03,
	SETTING_PALSE_REV_04,
	SETTING_PALSE_REV_05,
	SETTING_PALSE_REV_06,
	SETTING_PALSE_REV_08,
	SETTING_PALSE_REV_10,
	SETTING_PALSE_REV_MAX = SETTING_PALSE_REV_10	//1始まりのため-1
};


// 異常
typedef struct
{
	bool			rx;
	bool			sum;
}APL_CTRL_ERR_FLAG;

extern void initAplCtrl( void );
extern void aplCtrlMain( void );
extern APL_CTRL *getAplCtrl( void );
extern APL_CTRL_SET *getAplCtrlSet( void );
extern APL_CTRL_SET_PALSE *getAplCtrlSetPalse( void );
extern APL_CTRL_ERR_FLAG *getAplCtrlErrFlag( void );



#endif
