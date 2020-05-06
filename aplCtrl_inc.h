

#define		SETTING_VAL_MAX			((unsigned char)100)
#define		SETTING_VAL_MIN			((unsigned char)0)
#define		SETTING_VAL_DEF			((unsigned char)50)		//デフォルト値(EEPROM読み込みエラー時
#define		SETTING_VAL_INTERVAL	((unsigned char)5)	//変化間隔(%)

typedef enum{
	BRIGHT_7SEG,
	BRIGHT_BARLED,
	BRIGHT_DIM_7SEG,
	BRIGHT_DIM_BARLED,
	DISPCYC_7SEG,
	DISPCYC_BARLED,
	SETTING_ITEM_MAX
}SETTING_ITEM;

unsigned char eepDefault[SETTING_ITEM_MAX] = {
	80,		//7セグ輝度
	90,		//バーLED輝度
	20,		//7セグ輝度(減光
	20,		//バーLED輝度(減光
	20,		//7セグ表示更新速度
	0		//バーLED表示更新速度
};

typedef enum{
	SET_PALSE_SPEED,
	SET_PALSE_REV,
	SET_PALSE_NO_MAX
}SET_PALSE_NO;

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

const unsigned char PALSE_ITEM_MIN[]={SETTING_PALSE_SPEED_04,SETTING_PALSE_REV_01};
const unsigned char PALSE_ITEM_MAX[]={SETTING_PALSE_SPEED_MAX,SETTING_PALSE_REV_MAX};
#define		INIT_PALSE_SPEED	SETTING_PALSE_SPEED_04
#define		INIT_PALSE_REV		SETTING_PALSE_REV_04
