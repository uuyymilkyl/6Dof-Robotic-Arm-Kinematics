/**
*****************************************************************************
*	@file	f_TransPose.hpp
*	@brief	姿态表示功能类 向量 旋转矩阵 四元数 欧拉角互换 transformation of 3dPose description
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

	// 旋转矩阵 <-> 欧拉角
	// 绕X轴选举N度
	static KMat<double>  EulToRot_X(double &_Radian);
	// 绕y轴旋转N度
	static KMat<double>  EulToRot_Y(double& _Radian);
	// 绕Z轴旋转
	static KMat<double>  EulToRot_Z(double& _Radian);



	// 欧拉角转旋转矩阵
	static KMat<double> EulToRot_ZXZ(KMat<double> &_EulAngle);
	static KMat<double> EulToRot_ZXZ_T(KMat<double> &_EulAngle);

	static KMat<double> EulToRot_ZYX(KMat<double>& _EulAngle);
	static KMat<double> EulToRot_ZYX_T(KMat<double>& _EulAngle);

	static KMat<double> EulToRot_XYZ(KMat<double>& _EulAngle);
	static KMat<double> EulToRot_XYZ_T(KMat<double>& _EulAngle);

	// 旋转矩阵 <-> 四元数
	static KMat<double> RotToQ4(KMat<double>& _RotateMat);
	static KMat<double> Q4ToRot(KMat<double>& _RotateMat);

	// 欧拉角 -> 四元数
	static KMat<double> EulToQ4_ZXZ(KMat<double>& _EulAngle);

	static KMat<double> EulToQ4_ZYX(KMat<double>& _EulAngle);
	static KMat<double> EulToQ4_XYZ(KMat<double>& _EulAngle);

	// 旋转矩阵 -> 旋转向量
	static KMat<double>  RotToVector(KMat<double>& _RotateMat);

	// 旋转向量 -> 旋转矩阵
	static KMat<double> VectorToRot(KMat<double>& _Vector1_3);

	// 三正交旋转向量（VectorX,VectorY,VectorZ) -> 旋转矩阵（Tcp工具坐标系相对于基坐标系的旋转矩阵）
	static KMat<double> VectorXYZToRotTcp(KMat<double>& _VectorX, KMat<double>& _VectorY, KMat<double>& _VectorZ);




private:

};


#endif