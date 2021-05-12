//********************************************************************************
/*
	aplCtrl����Z�b�e�B���O�̏����𕪗��B
	aplCtrl����ׂ͍����Ƃ��낪�����Ȃ��悤�ɉB�������������B
	�����I�ɁAOOP���ۂ��쐬�B
	�Q�l
	https://qiita.com/qiita_kuru/items/19d501b364c7e32b3576
	https://qiita.com/qiita_kuru/items/8f3441bce9b2f53d1d62
	https://qiita.com/developer-kikikaikai/items/9715b1282f473463cbb5
	https://qiita.com/developer-kikikaikai/items/bf5a4ae32c2158ffbf7a	
*/	
//********************************************************************************

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "aplCtrlSetting.h"

static uint8_t addVal( uint8_t val , SETTING_CONTENTS* selectItem );
static uint8_t subVal( uint8_t val , SETTING_CONTENTS* selectItem );
static uint8_t assignValRangeBlock( uint8_t val , uint8_t min , uint8_t max);
//********************************************************************************
// �R���X�g���N�^
//********************************************************************************
APL_CTRL_SETTING aplCtrlSetting_new( SETTING_CONTENTS* root , uint8_t hierarchyMax)
{
	APL_CTRL_SETTING	instance = calloc(1, sizeof(*instance));
	
	instance->settingContensNow		= root;
	instance->settingContensRoot	= root;
	
	instance->settingContensRoute	= (SETTING_CONTENTS *)malloc( sizeof(SETTING_CONTENTS *) * hierarchyMax);
	instance->settingContensRouteItemNo 
									= (uint8_t*)malloc( sizeof(uint8_t) * hierarchyMax);
	
	instance->getNowHierarchy		= getNowHierarchyImpl;
	instance->changeHierarchyUp		= changeHierarchyUpImpl;
	instance->changeHierarchyDown	= changeHierarchyDownImpl;
	instance->exitSetting			= exitSettingImpl;
	instance->selectUp				= selectUpImpl;
	instance->selectDown			= selectDownImpl;
	instance->getDspVal				= getDspValImpl;
	instance->getState				= getStateImpl;
	instance->isNextHierarchy		= isNextHierarchyImpl;
	instance->preSettingRestore		= preSettingRestoreImpl;
	instance->saveSetting			= saveSettingImpl;
	
	instance->dspVal	= 0;
	instance->itemNo	= 0;
	instance->hierarchy	= 0;
	
	return( instance );
}
//********************************************************************************
// ���݊K�w�擾
//********************************************************************************
uint8_t getNowHierarchyImpl( APL_CTRL_SETTING this )
{
	return( this->hierarchy );
}
//********************************************************************************
// �K�w�ύX�@���
//********************************************************************************
bool changeHierarchyUpImpl( APL_CTRL_SETTING this )
{
	bool	result = false;
	
	if( this->hierarchy > 0 ){
		this->hierarchy--;
		this->settingContensNow	= this->settingContensRoute[this->hierarchy];
		this->itemNo			= this->settingContensRouteItemNo[this->hierarchy];
		result = true;
	}
	
	return( result );
}
//********************************************************************************
// �K�w�ύX�@����
//********************************************************************************
bool changeHierarchyDownImpl( APL_CTRL_SETTING this )
{
	bool	result = false;
	
	if( this->isNextHierarchy( this ) ){
		this->settingContensRoute[this->hierarchy] = this->settingContensNow;		//�߂邽�߂̌o�H�ۑ�
		this->settingContensRouteItemNo[this->hierarchy] = this->itemNo;
		
		this->settingContensNow = &this->settingContensNow->nexthierarchy[this->itemNo];
		this->hierarchy++;
		
		if( (this->settingContensNow->type == TYPE_ITEM) &&
			(this->settingContensNow->val  != NULL)
		){
			this->itemNo	= assignValRangeBlock( *this->settingContensNow->val , 0 , this->settingContensNow->max );
		}else{
			this->itemNo	= 0;
		}
		result = true;
	}
	return( result );
}
//********************************************************************************
// �������l��͈͂Ńu���b�N����
// ��ʓI�Ȗ��̂���܂���ł��������H
//********************************************************************************
static uint8_t assignValRangeBlock( uint8_t val , uint8_t min , uint8_t max)
{
	if( val < min ){
		return( min );
	}else if( val > max ){
		return( max );
	}else{
		return( val );
	}
}
//********************************************************************************
// �Z�b�e�B���O���j���[���o��
//********************************************************************************
void exitSettingImpl( APL_CTRL_SETTING this )
{
	this->hierarchy			= 0;
	this->itemNo			= 0;
	this->settingContensNow	= this->settingContensRoot;
}
//********************************************************************************
// �I�����ځ@����
//********************************************************************************
void selectUpImpl( APL_CTRL_SETTING this )
{
	SETTING_CONTENTS*	selectItem;
	selectItem = this->settingContensNow;
	
	//�l�p�ϐ����R�t���L��@���@type�����ځ@�̎��͗������Z����
	if( selectItem->val != NULL ){
		*selectItem->val = addVal( *selectItem->val	, selectItem );
	}
	
	if( selectItem->type == TYPE_ITEM ){
		this->itemNo = addVal( this->itemNo	, selectItem );
	}
}
//********************************************************************************
// ���Z(���MAX)
//********************************************************************************
static uint8_t addVal( uint8_t val , SETTING_CONTENTS* selectItem )
{
	uint8_t	result;
	
	if( (val + selectItem->oneDigitVal) < selectItem->max ){
		result = val + selectItem->oneDigitVal;
	}else{
		result = selectItem->max;
	}
	return( result );
}
//********************************************************************************
// �I�����ځ@����
//********************************************************************************
void selectDownImpl( APL_CTRL_SETTING this )
{
	SETTING_CONTENTS*	selectItem;
	selectItem = this->settingContensNow; 
	
	//�l�p�ϐ����R�t���L��@���@type�����ځ@�̎��͗������Z����
	if( selectItem->val != NULL ){
		*selectItem->val = subVal( *selectItem->val	, selectItem );
	}
	
	if( selectItem->type == TYPE_ITEM ){
		this->itemNo = subVal( this->itemNo	, selectItem );
	}
}
//********************************************************************************
// ���Z(����0
//********************************************************************************
static uint8_t subVal( uint8_t val , SETTING_CONTENTS* selectItem )
{
	uint8_t	result;
	
	if( val > selectItem->oneDigitVal ){
		result = val - selectItem->oneDigitVal;
	}else{
		result = 0;
	}
	
	return( result );
}
//********************************************************************************
// �\���l�擾
//********************************************************************************
uint8_t getDspValImpl( APL_CTRL_SETTING this )
{
	SETTING_CONTENTS*	selectItem;
	selectItem = this->settingContensNow;
	
	uint8_t		dspVal;
	
	if( selectItem->val == NULL ){
		dspVal = this->itemNo;
	}else{
		if( selectItem->type == TYPE_ITEM ){
			dspVal = selectItem->itemList[*selectItem->val];
		}else{
			dspVal = *selectItem->val;		// NON��VAL�̍���
		}
	}
	
	return( dspVal );
}
//********************************************************************************
//********************************************************************************
uint8_t getStateImpl( APL_CTRL_SETTING this )
{
	return( this->settingContensNow->name );
}

//********************************************************************************
// ���̊K�w�͂���܂����H
//********************************************************************************
bool isNextHierarchyImpl( APL_CTRL_SETTING this )
{
	SETTING_CONTENTS*	nextContents;
	
	if( this->settingContensNow->nexthierarchy == NULL ){
		return( false );
	}
	
	nextContents = &this->settingContensNow->nexthierarchy[this->itemNo];
	
	if( nextContents->name != 0 ){		// 0:APL_CTRL_STATE_SETTING_NON
		return( true );
	}else{
		return( false );
	}
}
//********************************************************************************
//********************************************************************************
void preSettingRestoreImpl( APL_CTRL_SETTING this )
{
	*this->settingContensNow->val	= *this->settingContensNow->valBak;
}
//********************************************************************************
//********************************************************************************
void saveSettingImpl( APL_CTRL_SETTING this )
{
	*this->settingContensNow->valBak	= *this->settingContensNow->val;
}
