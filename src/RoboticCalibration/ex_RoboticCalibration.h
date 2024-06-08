//************ extra function **********//
#ifndef _EX_ROBOTIC_CALIBRATION_H_
#define _EX_ROBOTIC_CALIBRATION_H_

#include <iostream>
#include <vector>
#include "../../src/f_TransPose.hpp"

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class MCalibration
{
public:
    MCalibration();
    ~MCalibration();

public:

    /**
    *****************************************************************************
    *  @brief  TCP标定 优化标定数据
    *
    *  @param   _vInputPoseMat  [输入追踪器原始姿态）]
    *  @param   _OutputMat      [输出求解矩阵结果 ]

    *****************************************************************************
    */

    static void Calibration_Correct_TCP_Data(std::vector < KMat<double>>& _vInputMats,  std::vector<KMat<double>> &_vOutputMats);

    /**
    *****************************************************************************
    *  @brief  TCP标定（追踪器到笔尖) QR分解解AX=B
    *
    *  @param   _vInputPoseMat  [输入追踪器原始姿态）]
    *  @param   _OutputMat      [输出求解矩阵结果 ]

    *****************************************************************************
    */
    static void Calibration_OpenCV_TCP(std::vector<KMat<double>>& _vInputPoseMat, KMat<double>& _OutputMat);

    /**
    *****************************************************************************
    *  @brief  TCF标定坐标系旋转矩阵
    *
    *  @param   _vInputPoseMat  [输入顺序 原点-X方向-Z方向 (只有三个)]
    *  @param   _OutputMat      [输出求解矩阵结果 ]
    *
    *
    *****************************************************************************
    */
    static void Calibration_OpenCV_TCF(std::vector<KMat<double>>& _vInputPoseMat, KMat<double>& _OutputMat);

    /**
    *****************************************************************************
    *  @brief  TsaiLenz解AX=XB  第一个输入的矩阵为A 第二个为B
    *
    *  @param   _vInputRobotPoseMat  [输入机器人的点]
    *  @param   _vInputTrackerPoseMat [输入追踪器的点]
    *  @param   _OutputMat      [输出求解矩阵结果 ]
    *  @param   numPoses        [姿态的个数]

    *
    *----------------------------------------------------------------------------
    *  @note 历史版本   修改人员     修改日期    修改内容
    *  @note V1.0.0   HuangMinyu   2024/02/23	 创建
    *
    *****************************************************************************
    */
    static int Calibration_OpenCV_TsaiLenz(std::vector<KMat<double >>& _vInputRobotPoseMat, std::vector<KMat<double>>& _vInputTrackerPoseMat, KMat<double>& _OutputMat, int numPoses);


    /**
    *****************************************************************************
    *  @brief  手眼标定
    *
    *  @param   _vInputRobotPoseMat  [输入机器人的点]
    *  @param   _vObCamMatR [输入相机外参的R (3x3矩阵)]
    *  @param   _vObCamMatT [输入相机的内参T (3x1矩阵)]
    *  @param   _OutputMat      [输出求解矩阵结果]
    *
    *  @return  无返回值
    *  @author  HuangMinyu
    *  @version V1.0.0
    *
    *----------------------------------------------------------------------------
    *  @note 历史版本   修改人员     修改日期    修改内容
    *  @note V1.0.0   HuangMinyu   2024/02/23	 创建
    *
    *****************************************************************************
    */
    static int Calibration_HandEye(std::vector<KMat<double>>& _vInputRobotPoseMat, std::vector<cv::Mat>& _vObCamMatR, std::vector<cv::Mat>& _vObCamMatT, KMat<double>& _OutputMat);

    /* KMat转cvMat */
    static cv::Mat TransKMatToMat(KMat<double>& _input_vmat);

    /* cvMat转KMat */
    static KMat<double> TramsMatToKMat(cv::Mat& _input_cvmat);

    /*最小二乘解TCP*/
    static void LeastSquareSolveTCP(std::vector<KMat<double>>& _vec_inputPose, KMat<double>& _matOut);


private:

};

static inline Eigen::MatrixXd KMat2Eigen(KMat<double> &_vmat){


    int rows = _vmat.m_nRows;
	int cols = _vmat.m_nCols;
	Eigen::MatrixXd eigenMat(rows,cols);
	for(int i= 0; i < rows; i++)
	{
		for(int j=0; j< cols; j++)
		{
			eigenMat(i,j) = _vmat(i,j);
		}
	}

	return eigenMat;

}


static inline KMat<double> Eigen2KMat(Eigen::MatrixXd &_eigenMat)
{

	KMat<double> Vmat(_eigenMat.rows(), _eigenMat.cols());

	for(int i = 0;i< _eigenMat.rows(); i++)
	{

		for(int j = 0; j < _eigenMat.cols(); j++)
		{
			Vmat(i,j) = _eigenMat(i,j);
		}

	}
	return Vmat;

}




#endif // !_MJTRACKER_MCALIBRATION_H_
