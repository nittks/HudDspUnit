#ifndef APL_CTRL_INC_H
#define APL_CTRL_INC_H

#include "aplCtrl.h"

#define		SETTING_VAL_MAX			((unsigned char)100)
#define		SETTING_VAL_MIN			((unsigned char)0)
#define		SETTING_VAL_DEF			((unsigned char)50)		//デフォルト値(EEPROM読み込みエラー時
#define		SETTING_VAL_INTERVAL	((unsigned char)5)	//変化間隔(%)

typedef enum{
	COLOR_7SEG,
	BRIGHT_7SEG,
	BRIGHT_DIM_7SEG,
	DISPCYC_7SEG,
	SETTING_ITEM_MAX
}SETTING_ITEM;

enum{
	COLOR_7SEG_WHITE,
	COLOR_7SEG_RED,
	COLOR_7SEG_YELLOW,
	COLOR_7SEG_GREEN,
	COLOR_7SEG_AQUA,
	COLOR_7SEG_BLUE,
	COLOR_7SEG_PURPLE,
	COLOR_7SEG_MAX
};

COLOR_RGB COLOR_TABLE[COLOR_7SEG_MAX] ={
	{  30 ,  30 ,  30 },
	{ 100 ,   0 ,   0 },
	{  50 ,  50 ,   0 },
	{   0 , 100 ,   0 },
	{   0 ,  50 ,  50 },
	{   0 ,   0 , 100 },
	{  50 ,   0 ,  50 }};

unsigned char eepDefault[SETTING_ITEM_MAX] = {
	COLOR_7SEG_WHITE,
	80,		//7セグ輝度
	20,		//7セグ輝度(減光
	20,		//7セグ表示更新速度
};

const unsigned char PALSE_ITEM_MIN[]={SETTING_COLOR_WHITE,SETTING_PALSE_SPEED_04,SETTING_PALSE_REV_01};
const unsigned char PALSE_ITEM_MAX[]={SETTING_COLOR_MAX,SETTING_PALSE_SPEED_MAX,SETTING_PALSE_REV_MAX};
#define		INIT_PALSE_SPEED	SETTING_PALSE_SPEED_04
#define		INIT_PALSE_REV		SETTING_PALSE_REV_04

#endif
