//************ extra function **********//
#ifndef _EX_ROBOTIC_CALIBRATION_H_
#define _EX_ROBOTIC_CALIBRATION_H_

#include <iostream>
#include <vector>
#include "../../src/f_TransPose.hpp"
#include <fstream>
#include <string>

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

    static void Calibration_OpenCV_TCP(std::vector<KMat<double>>& _vInputPoseMat, KMat<double>& _OutputMat);

    static void Calibration_OpenCV_TCF(KMat<double>& _ER,std::vector<KMat<double>>& _vInputPoseMat, KMat<double>& _OutputMat);

    static int Calibration_OpenCV_TsaiLenz(std::vector<KMat<double >>& _vInputRobotPoseMat, std::vector<KMat<double>>& _vInputTrackerPoseMat, KMat<double>& _OutputMat);


    static double Calculate_AXXB_RandomError(cv::Mat& _CaliTop_R, cv::Mat& _CaliTop_T, cv::Mat& _CaliBot_R, cv::Mat& _CaliBot_T, std::vector<KMat<double>>& _RobotData, std::vector<KMat<double>>& _TrackerData);

    static int Calibration_HandEye(std::vector<KMat<double>>& _vInputRobotPoseMat, std::vector<cv::Mat>& _vObCamMatR, std::vector<cv::Mat>& _vObCamMatT, KMat<double>& _OutputMat);

    /* KMatתcvMat */
    static cv::Mat TransKMatToMat(KMat<double>& _input_vmat);

    /* cvMatתKMat */
    static KMat<double> TramsMatToKMat(cv::Mat& _input_cvmat);

    /*��С���˽�TCP*/
    static void LeastSquareSolveTCP(std::vector<KMat<double>>& _vec_inputPose, KMat<double>& _matOut);

    /*ͨ��txt����RT�궨������ */
    static void LoadRTCaliDataFromTxt(std::string& _inputDir, std::vector<KMat<double>>& _TrackerPoints, std::vector<KMat<double>> & _RobotPoints);

    /*ͨ��txt����tcp�궨������ */

    static void LoadTcpCaliDataFromTxt(std::string& _inputDir, std::vector<KMat<double>>& _TcpPoints);
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


// ������������֮��ļн�
static inline double calculateAngleBetweenVectors(const cv::Mat& vec1, const cv::Mat& vec2) {

    double dot_product = vec1.dot(vec2);
    double norm_vec1 = norm(vec1);
    double norm_vec2 = norm(vec2);
    double cos_theta = dot_product / (norm_vec1 * norm_vec2);
    double angle = acos(cos_theta);
    return angle * 180.0 / CV_PI; // �Ƕ�
}

#endif // !_MJTRACKER_MCALIBRATION_H_
