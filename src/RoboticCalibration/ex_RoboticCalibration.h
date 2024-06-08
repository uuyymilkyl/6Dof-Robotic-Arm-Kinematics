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
    *  @brief  TCP�궨 �Ż��궨����
    *
    *  @param   _vInputPoseMat  [����׷����ԭʼ��̬��]
    *  @param   _OutputMat      [����������� ]

    *****************************************************************************
    */

    static void Calibration_Correct_TCP_Data(std::vector < KMat<double>>& _vInputMats,  std::vector<KMat<double>> &_vOutputMats);

    /**
    *****************************************************************************
    *  @brief  TCP�궨��׷�������ʼ�) QR�ֽ��AX=B
    *
    *  @param   _vInputPoseMat  [����׷����ԭʼ��̬��]
    *  @param   _OutputMat      [����������� ]

    *****************************************************************************
    */
    static void Calibration_OpenCV_TCP(std::vector<KMat<double>>& _vInputPoseMat, KMat<double>& _OutputMat);

    /**
    *****************************************************************************
    *  @brief  TCF�궨����ϵ��ת����
    *
    *  @param   _vInputPoseMat  [����˳�� ԭ��-X����-Z���� (ֻ������)]
    *  @param   _OutputMat      [����������� ]
    *
    *
    *****************************************************************************
    */
    static void Calibration_OpenCV_TCF(std::vector<KMat<double>>& _vInputPoseMat, KMat<double>& _OutputMat);

    /**
    *****************************************************************************
    *  @brief  TsaiLenz��AX=XB  ��һ������ľ���ΪA �ڶ���ΪB
    *
    *  @param   _vInputRobotPoseMat  [��������˵ĵ�]
    *  @param   _vInputTrackerPoseMat [����׷�����ĵ�]
    *  @param   _OutputMat      [����������� ]
    *  @param   numPoses        [��̬�ĸ���]

    *
    *----------------------------------------------------------------------------
    *  @note ��ʷ�汾   �޸���Ա     �޸�����    �޸�����
    *  @note V1.0.0   HuangMinyu   2024/02/23	 ����
    *
    *****************************************************************************
    */
    static int Calibration_OpenCV_TsaiLenz(std::vector<KMat<double >>& _vInputRobotPoseMat, std::vector<KMat<double>>& _vInputTrackerPoseMat, KMat<double>& _OutputMat, int numPoses);


    /**
    *****************************************************************************
    *  @brief  ���۱궨
    *
    *  @param   _vInputRobotPoseMat  [��������˵ĵ�]
    *  @param   _vObCamMatR [���������ε�R (3x3����)]
    *  @param   _vObCamMatT [����������ڲ�T (3x1����)]
    *  @param   _OutputMat      [�����������]
    *
    *  @return  �޷���ֵ
    *  @author  HuangMinyu
    *  @version V1.0.0
    *
    *----------------------------------------------------------------------------
    *  @note ��ʷ�汾   �޸���Ա     �޸�����    �޸�����
    *  @note V1.0.0   HuangMinyu   2024/02/23	 ����
    *
    *****************************************************************************
    */
    static int Calibration_HandEye(std::vector<KMat<double>>& _vInputRobotPoseMat, std::vector<cv::Mat>& _vObCamMatR, std::vector<cv::Mat>& _vObCamMatT, KMat<double>& _OutputMat);

    /* KMatתcvMat */
    static cv::Mat TransKMatToMat(KMat<double>& _input_vmat);

    /* cvMatתKMat */
    static KMat<double> TramsMatToKMat(cv::Mat& _input_cvmat);

    /*��С���˽�TCP*/
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
