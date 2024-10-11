/**
*****************************************************************************
*	@file	f_TransPose.hpp
*	@brief	��̬��ʾ������ ���� ��ת���� ��Ԫ�� ŷ���ǻ��� transformation of 3dPose description
*
*	@author		HuangMinYu
*	@date		2024.01.04
*	@version	V2.0
*
*****************************************************************************
*/

#ifndef _6DOFROBOTICS_TRANSPOSE_HPP_
#define _6DOFROBOTICS_TRANSPOSE_HPP_

#include "../include/t_Matrix.hpp"
#include "../include/t_3DStruct.hpp"

class TranPose
{
public:
	TranPose();
	~TranPose();

	// ��ת���� <-> ŷ����
	// ��X��ѡ��N��
	static KMat<double>  EulToRot_X(double &_Radian);
	// ��y����תN��
	static KMat<double>  EulToRot_Y(double& _Radian);
	// ��Z����ת
	static KMat<double>  EulToRot_Z(double& _Radian);



	// ŷ����ת��ת����
	static KMat<double> EulToRot_ZXZ(KMat<double> &_EulAngle);
	static KMat<double> EulToRot_ZXZ_T(KMat<double> &_EulAngle);

	static KMat<double> EulToRot_ZYX(KMat<double>& _EulAngle);
	static KMat<double> EulToRot_ZYX_T(KMat<double>& _EulAngle);

	static KMat<double> EulToRot_XYZ(KMat<double>& _EulAngle);
	static KMat<double> EulToRot_XYZ_T(KMat<double>& _EulAngle);

	// ��ת���� <-> ��Ԫ��
	static KMat<double> RotToQ4(KMat<double>& _RotateMat);
	static KMat<double> Q4ToRot(KMat<double>& _RotateMat);

	// ŷ���� -> ��Ԫ��
	static KMat<double> EulToQ4_ZXZ(KMat<double>& _EulAngle);

	static KMat<double> EulToQ4_ZYX(KMat<double>& _EulAngle);
	static KMat<double> EulToQ4_XYZ(KMat<double>& _EulAngle);

	// ��ת���� -> ��ת����
	static KMat<double>  RotToVector(KMat<double>& _RotateMat);

	// ��ת���� -> ��ת����
	static KMat<double> VectorToRot(KMat<double>& _Vector1_3);

	// ��������ת������VectorX,VectorY,VectorZ) -> ��ת����Tcp��������ϵ����ڻ�����ϵ����ת����
	static KMat<double> VectorXYZToRotTcp(KMat<double>& _VectorX, KMat<double>& _VectorY, KMat<double>& _VectorZ);




private:

};


#endif