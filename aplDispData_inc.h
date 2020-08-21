#ifndef APL_DISP_DATA_INC_H
#define APL_DISP_DATA_INC_H

//********************************************************************************
// �W������p�萔
//********************************************************************************
#define		SPEED_MAX		((unsigned char)200)
#define		SPEED_MIN		((unsigned char)0)
#define		REV_MAX			((unsigned short)8000)
#define		REV_MIN			((unsigned short)0)
#define		REV_PER_SEG		(200)					//1�Z�O�����g������̉�]��
#define		DISPCYC_7SEG_DIGIT			((unsigned short)1)	//1%�ӂ�̎���[10ms](100%��1000ms)
#define		DISPCYC_BARLED_DIGIT		((unsigned short)1)	//1%�ӂ�̎���[10ms](100%��1000ms)
#define		SPEED_DIGIT		((unsigned char)1)		//1%�ӂ�̕\���l
#define		REV_DIGIT		((unsigned short)80)	//1%�ӂ�̕\���l(100%��8000rpm�\��

//********************************************************************************
// �e�X�g���[�h
//********************************************************************************
//�e�X�g�f�[�^�v�Z����
#define		TEST_CYC_SPEED	((unsigned char)10-1)	//10x10ms=100ms������
#define		TEST_CYC_REV	((unsigned char)2-1)	//10x10ms=100ms������
//�v�Z���̕ω��l
#define		CHG_VAL_SPEED	((unsigned char)2)
#define		CHG_VAL_REV		((unsigned char)200)
//���[�^���[�N���b�N���̕ω��l
#define		CHG_VAL_SPEED_MANUAL	((unsigned char)5)
#define		CHG_VAL_REV_MANUAL		((unsigned char)200)


typedef enum{
	TEST_STATE_UP,
	TEST_STATE_DOWN
}TEST_STATE;


//********************************************************************************
// �ݒ�@�p���X�ݒ�@�\���l
//********************************************************************************
const unsigned char SETTING_PALSE_SPEED[6]=	{0,4,8,16,20,25};		//�ԑ��p���X�d�l
const unsigned char SETTING_PALSE_REV[9]={0,1,2,3,4,5,6,8,10,12};	//��]���p���X�d�l(�C����

#define SET_BRIGHT_7SEG_DISP	((unsigned char)180)
#define SET_BRIGHT_BARLED_DISP	((unsigned short)8000)




#endif
