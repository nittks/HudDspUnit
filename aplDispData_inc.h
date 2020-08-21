#ifndef APL_DISP_DATA_INC_H
#define APL_DISP_DATA_INC_H

//********************************************************************************
// �W������p�萔
//********************************************************************************
#define		SPEED_MAX		((unsigned char)200)
#define		SPEED_MIN		((unsigned char)0)
#define		DISPCYC_7SEG_DIGIT			((unsigned short)1)	//1%�ӂ�̎���[10ms](100%��1000ms)
#define		SPEED_DIGIT		((unsigned char)1)		//1%�ӂ�̕\���l

//********************************************************************************
// �e�X�g���[�h
//********************************************************************************
//�e�X�g�f�[�^�v�Z����
#define		TEST_CYC_SPEED	((unsigned char)10-1)	//10x10ms=100ms������
//�v�Z���̕ω��l
#define		CHG_VAL_SPEED	((unsigned char)2)
//���[�^���[�N���b�N���̕ω��l
#define		CHG_VAL_SPEED_MANUAL	((unsigned char)5)


typedef enum{
	TEST_STATE_UP,
	TEST_STATE_DOWN
}TEST_STATE;


//********************************************************************************
// �ݒ�@�p���X�ݒ�@�\���l
//********************************************************************************
const unsigned char SETTING_PALSE_SPEED[]=	{0,4,8,16,20,25};		//�ԑ��p���X�d�l
const unsigned char SETTING_PALSE_REV[]={0,1,2,3,4,5,6,8,10,12};	//��]���p���X�d�l(�C����

#define SET_BRIGHT_7SEG_DISP	((unsigned char)180)


#endif
