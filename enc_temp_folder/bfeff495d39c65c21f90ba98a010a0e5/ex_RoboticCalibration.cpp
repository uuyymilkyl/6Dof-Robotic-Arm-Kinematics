#include "ex_RoboticCalibration.h"


MCalibration::MCalibration()
{
}

MCalibration::~MCalibration()
{
}

void MCalibration::Calibration_OpenCV_TCP(std::vector<KMat<double>>& _vec_inputPoseMat, KMat<double>& _outputMat)
{
	std::vector<KMat<double>> vDeRotateMat;
	std::vector<KMat<double>> vDePoseMat;

	// 前后做差求 差值旋转矩阵 和 差值平移矩阵
	for (int i = 0; i < _vec_inputPoseMat.size() - 1; i++)
	{
		KMat<double> DeRotateMat(3, 3);
		DeRotateMat={
			{ _vec_inputPoseMat[i](0, 0) - _vec_inputPoseMat[i + 1](0, 0), _vec_inputPoseMat[i](0, 1) - _vec_inputPoseMat[i + 1](0, 1), _vec_inputPoseMat[i](0, 2) - _vec_inputPoseMat[i + 1](0, 2)},
			{ _vec_inputPoseMat[i](1, 0) - _vec_inputPoseMat[i + 1](1, 0), _vec_inputPoseMat[i](1, 1) - _vec_inputPoseMat[i + 1](1, 1), _vec_inputPoseMat[i](1, 2) - _vec_inputPoseMat[i + 1](1, 2)},
			{ _vec_inputPoseMat[i](2, 0) - _vec_inputPoseMat[i + 1](2, 0), _vec_inputPoseMat[i](2, 1) - _vec_inputPoseMat[i + 1](2, 1), _vec_inputPoseMat[i](2, 2) - _vec_inputPoseMat[i + 1](2, 2)}
			};

		//DeRotateMat = DeRotateMat._Orthogonal(DeRotateMat);

		KMat<double> DeTranslateMat(3, 1);
		DeTranslateMat={
			{_vec_inputPoseMat[i+1](0, 3) - _vec_inputPoseMat[i ](0, 3)},
			{_vec_inputPoseMat[i+1](1, 3) - _vec_inputPoseMat[i ](1, 3)},
			{_vec_inputPoseMat[i+1](2, 3) - _vec_inputPoseMat[i ](2, 3)}
			};
		vDeRotateMat.push_back(DeRotateMat);
		vDePoseMat.push_back(DeTranslateMat);

		std::cout << " DeRotate " << std::endl;
		DeRotateMat._Print();
		std::cout << " DePost " << std::endl;
		DeTranslateMat._Print();
	}

	// 将做差后的矩阵赋值到OpenCV的同一个Mat中，组成9*3 R矩阵和9*1 T矩阵

	cv::Mat R_Mat((_vec_inputPoseMat.size() - 1) * 3, 3, CV_64F);
	cv::Mat P_Mat((_vec_inputPoseMat.size() - 1) * 3, 1, CV_64F);
	for (int i = 0; i < _vec_inputPoseMat.size() - 1; i++)
	{
		for (int rows = 1; rows < 4; rows++)
		{
			int row = (rows + (i * 3)) - 1;
			P_Mat.at<double>(row, 0) = vDePoseMat[i](rows - 1, 0);

			for (int cols = 1; cols < 4; cols++)
			{
				R_Mat.at<double>(row, cols - 1) = vDeRotateMat[i](rows - 1, cols - 1);

			}
		}
	}




	// QR分解 求解AX=B
	cv::Mat Solve_Mat;
	cv::solve(R_Mat, P_Mat, Solve_Mat, cv::DECOMP_QR);

	KMat<double> Solve_Result(3, 1);
	Solve_Result = {
		{Solve_Mat.at<double>(0,0)},
		{Solve_Mat.at<double>(1,0)},
		{Solve_Mat.at<double>(2,0)}
		};

	// 得到TCP矩阵
	_outputMat = Solve_Result;
}

void MCalibration::Calibration_OpenCV_TCF(std::vector<KMat<double>>& _vec_inputPoseMat, KMat<double>& _outputMat)
{
	//0是原点
	//求X方向
	KMat<double> vecX(3, 1);
	vecX = {{_vec_inputPoseMat[1](0,3) - _vec_inputPoseMat[0](0,3) },
			{_vec_inputPoseMat[1](1,3) - _vec_inputPoseMat[0](1,3) },
			{_vec_inputPoseMat[1](2,3) - _vec_inputPoseMat[0](2,3) }

		};
	//X方向向量归一化
	KMat<double> vecUnitX(3, 1);
	vecUnitX = vecX / KMat<double>::_GetModuleLength(vecX);

	//求Z方向
	KMat<double> vecZ(3, 1);
	vecZ = { {_vec_inputPoseMat[2](0,3) - _vec_inputPoseMat[0](0,3) },
		     {_vec_inputPoseMat[2](1,3) - _vec_inputPoseMat[0](1,3) },
			 {_vec_inputPoseMat[2](2,3) - _vec_inputPoseMat[0](2,3) }

		};
	//Z方向向量归一化
	KMat<double> vecUnitZ(3, 1);
	vecUnitZ = vecZ / KMat<double>::_GetModuleLength(vecZ);

	//叉乘求Y
	KMat<double> vecY(3, 1);
	vecY = KMat<double>::_Cross(vecUnitX, vecUnitZ);
	KMat<double> vecUnitY(3, 1);
	vecUnitY = vecY / KMat<double>::_GetModuleLength(vecY);

	//放进旋转矩阵
	KMat<double> TCFRotateMat(3, 3);

	//由各轴旋转向量组成旋转矩阵
	TCFRotateMat = {
		{vecUnitX(0,0),vecUnitY(0,0),vecUnitZ(0,0)},
		{vecUnitX(1,0),vecUnitY(1,0),vecUnitZ(1,0)},
		{vecUnitX(2,0),vecUnitY(2,0),vecUnitZ(2,0)}
		};

	_outputMat = TCFRotateMat;
}

int MCalibration::Calibration_OpenCV_TsaiLenz(std::vector<KMat<double>>& _vec_inputRobotPoseMat, std::vector<KMat<double>>& _vec_inputTrackerPoseMat, KMat<double>& _outputMat, int numPoses)
{
	// TsaiLenz N(N>4)点标定
	int PoseSize = numPoses;
	/*
	std::vector< KMat<double>> vRobotPoses_R;
	std::vector< KMat<double>> vRobotPoses_T; // 左矩阵 AX=XB的A

	std::vector< KMat<double>> vTrackerPoses_R;  // 右矩阵 AX=BX的B
	std::vector< KMat<double>> vTrackerPoses_T;


	// 以KMat形式
	for (int i = 0; i < PoseSize; i++)
	{
		KMat<double> RobotPose_R = _vec_inputRobotPoseMat[i]._GetR();
		KMat<double> RobotPose_T = _vec_inputRobotPoseMat[i]._GetT();

		KMat<double> TrackerPose_R = _vec_inputTrackerPoseMat[i]._GetR();
		KMat<double> TrackerPose_T = _vec_inputTrackerPoseMat[i]._GetT();

		vRobotPoses_R.push_back(RobotPose_R);
		vRobotPoses_T.push_back(RobotPose_T);
		vRobotPoses_T.push_back(TrackerPose_R);
		vRobotPoses_T.push_back(TrackerPose_T);
	}
	*/
	//换成OpenCV的Mat形式
	std::vector< cv::Mat> vMat_RobotPoses_R; // 左矩阵 AX=XB的A
	std::vector< cv::Mat> vMat_RobotPoses_T;
	std::vector< cv::Mat> vMat_TrackerPoses_R;  // 右矩阵 AX=BX的B
	std::vector< cv::Mat> vMat_TrackerPoses_T;
	for (int i = 0; i < PoseSize; i++)
	{

		KMat<double> KMat_RobotPoseR = _vec_inputRobotPoseMat[i]._GetR();
		KMat_RobotPoseR._Print();
		KMat<double> KMat_RobotPoseT = _vec_inputRobotPoseMat[i]._GetT();
		KMat_RobotPoseT._Print();

		cv::Mat RobotPose_R = TransKMatToMat(KMat_RobotPoseR);
		cv::Mat RobotPose_T = TransKMatToMat(KMat_RobotPoseT);

		KMat<double> KMat_TrackerPoseR = _vec_inputRobotPoseMat[i]._GetR();
		KMat<double> KMat_TrackerPoseT = _vec_inputRobotPoseMat[i]._GetT();
		cv::Mat TrackerPose_R = TransKMatToMat(KMat_TrackerPoseR);
		cv::Mat TrackerPose_T = TransKMatToMat(KMat_TrackerPoseT);

		vMat_RobotPoses_R.push_back(RobotPose_R);
		vMat_RobotPoses_T.push_back(RobotPose_T);
		vMat_TrackerPoses_R.push_back(TrackerPose_R);
		vMat_TrackerPoses_T.push_back(TrackerPose_T);
	}
	cv::Mat CaliResult_R;
	cv::Mat CaliResult_T;

	cv::calibrateHandEye(vMat_RobotPoses_R, vMat_RobotPoses_T, vMat_RobotPoses_T, vMat_RobotPoses_T, CaliResult_R, CaliResult_T, cv::CALIB_HAND_EYE_TSAI);

	return 0;
}

int MCalibration::Calibration_HandEye(std::vector<KMat<double>>& _vec_inputRobotPoseMat, std::vector<cv::Mat>& _vec_obCamMatR, std::vector<cv::Mat>& _vec_obCamMatT, KMat<double>& _outputMat)
{
	std::vector<cv::Mat> vRobotMat_R;
	std::vector<cv::Mat> vRobotMat_T;
	std::vector<cv::Mat> vObCamMat_R = _vec_obCamMatR;
	std::vector<cv::Mat> vObCamMat_T = _vec_obCamMatT;

	for (int i = 0; i < _vec_inputRobotPoseMat.size(); i++)
	{
		cv::Mat RobotMat_R;

		KMat<double> vmatRobotR = _vec_inputRobotPoseMat[i]._GetR();
		RobotMat_R = TransKMatToMat(vmatRobotR);

		vRobotMat_R.push_back(RobotMat_R);

		KMat<double> vmatRobotT = _vec_inputRobotPoseMat[i]._GetT();
		cv::Mat RobotMat_T = TransKMatToMat(vmatRobotT);

		RobotMat_T.push_back(RobotMat_T);
	}
	cv::Mat outR;
	cv::Mat outT;

	cv::calibrateHandEye(vRobotMat_R, vRobotMat_T, vObCamMat_R, vObCamMat_T, outR, outT);

	return 0;
}

cv::Mat MCalibration::TransKMatToMat(KMat<double>& _input_vmat)
{
	cv::Mat Result(_input_vmat.m_nRows, _input_vmat.m_nCols, CV_64F);

	for (int i = 0; i < _input_vmat.m_nRows; i++)
	{
		for (int j = 0; j < _input_vmat.m_nCols; j++)
		{
			Result.at<double>(i, j) = _input_vmat(i, j);
		}
	}
	return Result;
}

KMat<double> MCalibration::TramsMatToKMat(cv::Mat& _input_cvmat)
{
	KMat<double> Result(_input_cvmat.rows, _input_cvmat.cols);
	for (int i = 0; i < _input_cvmat.rows; i++)
	{
		for (int j = 0; j < _input_cvmat.cols; j++)
		{
			Result(i, j) = _input_cvmat.at<double>(i, j);
		}
	}
	return Result;
}


void MCalibration::LeastSquareSolveTCP(std::vector<KMat<double>>& _vec_inputPose, KMat<double>& _matOut)
{
	std::vector<Eigen::MatrixXd> vec_EigenMat;

	//  把数据转换成eigen形式
	for (int i = 0; i < _vec_inputPose.size(); i++)
	{
		KMat<double> VMatInputPose = _vec_inputPose[i];
		Eigen::MatrixXd EigenMat = KMat2Eigen(VMatInputPose);
		vec_EigenMat.push_back(EigenMat);
		std::cout << " Each  matrix : --- " << std::endl << EigenMat << std::endl;
	}
	// 构造矩阵AE
	// 构造矩阵 A 和向量 b
	//Eigen::MatrixXd A(16, 4); // 4 * 4 = 16
	//Eigen::VectorXd b(16);
	//for (int i = 0; i < 4; ++i) {
	//	A.block<4, 4>(i * 4, 0) = vec_EigenMat[i].block<4, 4>(0, 0);
	//	b.segment<4>(i * 4) = vec_EigenMat[i].block<4, 1>(0, 0);
	//}

	// 求解最小二乘法问题
	//Eigen::VectorXd x = A.fullPivHouseholderQr().solve(b);
	//std::cout << x << " : result" << std::endl;
	//// 将结果转换回 KMat
	//Eigen::MatrixXd eMatResultMap = Eigen::Map<Eigen::MatrixXd>(x.data(), 4, 4);
	//KMat<double> vMatOut = Eigen2KMat(eMatResultMap);

	Eigen::MatrixXd A(3 * _vec_inputPose.size(), 6);
	Eigen::VectorXd b(3 * _vec_inputPose.size());

	// Construct the least squares system
	for (size_t i = 0; i < vec_EigenMat.size(); ++i) {
		Eigen::Matrix3d R = vec_EigenMat[i].block<3, 3>(0, 0);
		Eigen::Vector3d t = vec_EigenMat[i].block<3, 1>(0, 3);

		A.block<3, 3>(3 * i, 0) = Eigen::Matrix3d::Identity() - R;
		A.block<3, 3>(3 * i, 3) = R;
		b.segment<3>(3 * i) = t;

		std::cout << " A  matrix : --- " << std::endl << A << std::endl;
		std::cout << " B matrix : --- " << std::endl << b << std::endl;
	}



	// Solve the least squares problem
	Eigen::VectorXd x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	std::cout << " ---x :" << x << std::endl;
	// Construct the TCP transformation matrix
	Eigen::Matrix4d TCP = Eigen::Matrix4d::Identity();
	TCP.block<3, 1>(0, 3) = x.head<3>();
	Eigen::AngleAxisd rot(x.tail<3>().norm(), x.tail<3>().normalized());
	TCP.block<3, 3>(0, 0) = rot.matrix();

	// 将结果转换回 KMat
	std::cout << " tcp  matrix : --- " << TCP << std::endl;

}

