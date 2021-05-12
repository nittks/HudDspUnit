#ifndef APL_SETTING_H
#define APL_SETTING_H
typedef enum{
	TYPE_NON,
	TYPE_VAL,
	TYPE_ITEM,
	TYPE_MAX
}SETTING_LIST_TYPE;

typedef struct setting_contens_struct SETTING_CONTENTS;
struct setting_contens_struct{
	uint8_t				name;
	uint8_t				max;
	SETTING_LIST_TYPE	type;
	uint8_t				oneDigitVal;	//�f�W�^���l1�ӂ�̒l
	uint8_t*			itemList;		//���e�����ڂ̎��A�\������l�̃��X�g
	SETTING_CONTENTS*	nexthierarchy;	//���̊K�w�ւ̃|�C���^
	uint8_t*			val;			//�Ή�����ϐ��ւ̃|�C���^
	uint8_t*			valBak;			//�ݒ��K�p���ɖ߂��l
};

typedef struct aplCtrlSetting*	APL_CTRL_SETTING;
struct aplCtrlSetting{
	uint8_t	hierarchy;
	uint8_t	itemNo;
	uint8_t	dspVal;

	SETTING_CONTENTS*	settingContensNow;
	SETTING_CONTENTS*	settingContensRoot;
	SETTING_CONTENTS**	settingContensRoute;
	uint8_t*			settingContensRouteItemNo;

	uint8_t	(*getNowHierarchy)		( APL_CTRL_SETTING this );	//�֐����̊��ʂ������ƃG���[�B�Ȃ����킩��Ȃ�
	bool	(*changeHierarchyUp)	( APL_CTRL_SETTING this );
	bool	(*changeHierarchyDown)	( APL_CTRL_SETTING this );
	void	(*exitSetting)			( APL_CTRL_SETTING this );
	void	(*selectUp)				( APL_CTRL_SETTING this );
	void	(*selectDown)			( APL_CTRL_SETTING this );
	uint8_t (*getDspVal)			( APL_CTRL_SETTING this );
	bool	(*isNextHierarchy)		( APL_CTRL_SETTING this );
	void	(*preSettingRestore)	( APL_CTRL_SETTING this );
	void	(*saveSetting)			( APL_CTRL_SETTING this );
	uint8_t	(*getState)				( APL_CTRL_SETTING this );
};

extern APL_CTRL_SETTING aplCtrlSetting_new	( SETTING_CONTENTS* root , uint8_t hierarchyMax );
extern uint8_t getNowHierarchyImpl			( APL_CTRL_SETTING this );
extern bool changeHierarchyUpImpl			( APL_CTRL_SETTING this );
extern bool changeHierarchyDownImpl			( APL_CTRL_SETTING this );
extern void exitSettingImpl					( APL_CTRL_SETTING this );
extern void selectUpImpl					( APL_CTRL_SETTING this );
extern void selectDownImpl					( APL_CTRL_SETTING this );
extern uint8_t getDspValImpl				( APL_CTRL_SETTING this );
extern uint8_t getStateImpl					( APL_CTRL_SETTING this );
extern bool	isNextHierarchyImpl				( APL_CTRL_SETTING this );
extern void preSettingRestoreImpl			( APL_CTRL_SETTING this );
extern void	saveSettingImpl					( APL_CTRL_SETTING this );
#endif
