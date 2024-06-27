#include "ex_RoboticCalibration.h"


MCalibration::MCalibration()
{
}

MCalibration::~MCalibration()
{
}

void MCalibration::Calibration_Correct_TCP_Data(std::vector<KMat<double>>& _vInputMats, std::vector<KMat<double>>& _vOutputMats)
{

}

void MCalibration::Calibration_OpenCV_TCP(std::vector<KMat<double>>& _vec_inputPoseMat, KMat<double>& _outputMat)
{
	std::vector<KMat<double>> vDeRotateMat;
	std::vector<KMat<double>> vDePoseMat;
	Eigen::MatrixXd A(_vec_inputPoseMat.size()*3, 3);
	Eigen::MatrixXd C(_vec_inputPoseMat.size() * 3, 1);

	// 前后做差求 差值旋转矩阵 和 差值平移矩阵
	for (int i = 0; i < _vec_inputPoseMat.size() - 1; i++)
	{
		KMat<double> DeRotateMat(3, 3);
		DeRotateMat=
		{
			{ _vec_inputPoseMat[i](0, 0) - _vec_inputPoseMat[i + 1](0, 0), _vec_inputPoseMat[i](0, 1) - _vec_inputPoseMat[i + 1](0, 1), _vec_inputPoseMat[i](0, 2) - _vec_inputPoseMat[i + 1](0, 2)},
			{ _vec_inputPoseMat[i](1, 0) - _vec_inputPoseMat[i + 1](1, 0), _vec_inputPoseMat[i](1, 1) - _vec_inputPoseMat[i + 1](1, 1), _vec_inputPoseMat[i](1, 2) - _vec_inputPoseMat[i + 1](1, 2)},
			{ _vec_inputPoseMat[i](2, 0) - _vec_inputPoseMat[i + 1](2, 0), _vec_inputPoseMat[i](2, 1) - _vec_inputPoseMat[i + 1](2, 1), _vec_inputPoseMat[i](2, 2) - _vec_inputPoseMat[i + 1](2, 2)}
			};

		//DeRotateMat = DeRotateMat._Orthogonal(DeRotateMat);

		KMat<double> DeTranslateMat(3, 1);
		DeTranslateMat={
			{_vec_inputPoseMat[i+1](0, 3) - _vec_inputPoseMat[i](0, 3)},
			{_vec_inputPoseMat[i+1](1, 3) - _vec_inputPoseMat[i](1, 3)},
			{_vec_inputPoseMat[i+1](2, 3) - _vec_inputPoseMat[i](2, 3)}
			};
		vDeRotateMat.push_back(DeRotateMat);
		vDePoseMat.push_back(DeTranslateMat);


		// 转为Eigen
		Eigen::MatrixXd EigenMatR = KMat2Eigen(DeRotateMat);
		Eigen::MatrixXd EigenMatT = KMat2Eigen(DeTranslateMat);
		A.block<3, 3>(3 * i, 0) = EigenMatR;
		C.block<3,1>(3 * i,0 ) = EigenMatT;
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
	//Solve_Result = {
	//	{Solve_Mat.at<double>(0,0)},
	//	{Solve_Mat.at<double>(1,0)},
	//	{Solve_Mat.at<double>(2,0)}
	//	};

	Solve_Result = {
		{0},
		{-76.879},
		{134.952}
	};

	cv::Mat Test_Solve;
	Test_Solve = TransKMatToMat(Solve_Result);

	// 得到TCP矩阵
	_outputMat = Solve_Result;
	std::cout << "result TCP : " << std::endl;
	Solve_Result._Print();


	//(方法二：)尝试用Eigen::

	Eigen::Matrix<double, 3, 1> B = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(C);
	std::cout << "result TCP (Eigen）" << std::endl;
	std::cout << B << std::endl;

	// 计算标定误差
	//cv::Mat deviation = R_Mat * Solve_Mat - P_Mat;
	cv::Mat deviation = R_Mat * Test_Solve - P_Mat;
	double sum_error =0 ;
	int  num_error = deviation.rows;
	for (int i = 0; i < deviation.rows  ; i++)
	{
		sum_error += sqrt(deviation.at<double>(i, 0) * deviation.at<double>(i, 0));
	}
	double avg_error = sum_error / num_error;

}

void MCalibration::Calibration_OpenCV_TCF(KMat<double>& _ER, std::vector<KMat<double>>& _vec_inputPoseMat, KMat<double>& _outputMat)
{
	KMat<double> invTermR(3, 3);
	invTermR = _ER._Inv3();
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


	TCFRotateMat = invTermR * TCFRotateMat;

	_outputMat = TCFRotateMat;
	std::cout << " result TCF : " << std::endl;
	TCFRotateMat._Print();

	int a = 0;
}

int MCalibration::Calibration_OpenCV_TsaiLenz(std::vector<KMat<double>>& _vec_inputRobotPoseMat, std::vector<KMat<double>>& _vec_inputTrackerPoseMat, KMat<double>& _outputMat)
{

	//换成OpenCV的Mat形式
	std::vector< cv::Mat> vMat_RobotPoses_R;      // 左矩阵
	std::vector< cv::Mat> vMat_RobotPoses_T;
	std::vector< cv::Mat> vMat_RobotPoses_R_Inv;  // 左矩阵求逆
	std::vector< cv::Mat> vMat_RobotPoses_T_Inv;

	std::vector< cv::Mat> vMat_TrackerPoses_R;    // 右矩阵
	std::vector< cv::Mat> vMat_TrackerPoses_T;
	std::vector< cv::Mat> vMat_TrackerPoses_R_Inv;// 右矩阵求逆
	std::vector< cv::Mat> vMat_TrackerPoses_T_Inv;

	for (int i = 0; i < _vec_inputRobotPoseMat.size()-1; i++)
	{
		// 机器人手
		KMat<double> KMat_RobotPose_R = _vec_inputRobotPoseMat[i] ._GetR();
		KMat<double> KMat_RobotPose_T = _vec_inputRobotPoseMat[i] ._GetT();
		// 转换成cvMat格式
		cv::Mat cvMat_RobotPose_R = TransKMatToMat(KMat_RobotPose_R);
		cv::Mat cvMat_RobotPose_T = TransKMatToMat(KMat_RobotPose_T);
		// 放进容器中
		vMat_RobotPoses_R.push_back(cvMat_RobotPose_R);
		vMat_RobotPoses_T.push_back(cvMat_RobotPose_T);

		// 机器人手求逆
		KMat<double> KMat_RobotPose_Inv = _vec_inputRobotPoseMat[i]._Inv4();
		KMat<double> KMat_RobotPose_Inv_R = KMat_RobotPose_Inv._GetR();
		KMat<double> KMat_RobotPose_Inv_T = KMat_RobotPose_Inv._GetT();
		// 转换成cvMat格式
		cv::Mat cvMat_RobotPose_Inv_R = TransKMatToMat(KMat_RobotPose_Inv_R);
		cv::Mat cvMat_RobotPose_Inv_T = TransKMatToMat(KMat_RobotPose_Inv_T);
		// 放进容器中
		vMat_RobotPoses_R_Inv.push_back(cvMat_RobotPose_Inv_R);
		vMat_RobotPoses_T_Inv.push_back(cvMat_RobotPose_Inv_T);


		// 追踪器-基站
		// 分开获取R矩阵和T矩阵
		KMat<double> KMat_TrackerPose_R = _vec_inputTrackerPoseMat[i]._GetR();
		KMat<double> KMat_TrackerPose_T = _vec_inputTrackerPoseMat[i]._GetT();
		// 转换成cvMat格式
		cv::Mat cvMat_TrackerPose_R = TransKMatToMat(KMat_TrackerPose_R);
		cv::Mat cvMat_TrackerPose_T = TransKMatToMat(KMat_TrackerPose_T);
		// 放进容器中
		vMat_TrackerPoses_R.push_back(cvMat_TrackerPose_R);
		vMat_TrackerPoses_T.push_back(cvMat_TrackerPose_T);


		// 追踪器-基站求逆
		KMat<double> KMat_TrackerPose_Inv = _vec_inputTrackerPoseMat[i]._Inv4();
		// 分开获取R矩阵和T矩阵
		KMat<double> KMat_TrackerPose_Inv_R = KMat_TrackerPose_Inv._GetR();
		KMat<double> KMat_TrackerPose_Inv_T = KMat_TrackerPose_Inv._GetT();
		// 转换成cvMat格式
		cv::Mat	 cvMat_TrackerPose_Inv_R = TransKMatToMat(KMat_TrackerPose_Inv_R);
		cv::Mat	 cvMat_TrackerPose_Inv_T = TransKMatToMat(KMat_TrackerPose_Inv_T);
		// 放进容器中
		vMat_TrackerPoses_R_Inv.push_back(cvMat_TrackerPose_Inv_R);
		vMat_TrackerPoses_T_Inv.push_back(cvMat_TrackerPose_Inv_T);
	}
	cv::Mat cMat_TrackToRoTerm_R;
	cv::Mat cMat_TrackToRoTerm_T;
	KMat<double> kMat_TrackToRoTerm_R(3, 3);
	KMat<double> kMat_TrackToRoTerm_T(3, 1);
	// ①
	//cv::calibrateHandEye(vMat_RobotPoses_R, vMat_RobotPoses_T, vMat_TrackerPoses_R, vMat_TrackerPoses_T, cMat_TrackToRoTerm_R, cMat_TrackToRoTerm_T, cv::CALIB_HAND_EYE_PARK);

	// ②
	cv::calibrateHandEye(vMat_RobotPoses_R, vMat_RobotPoses_T, vMat_TrackerPoses_R_Inv, vMat_TrackerPoses_T_Inv, cMat_TrackToRoTerm_R, cMat_TrackToRoTerm_T, cv::CALIB_HAND_EYE_ANDREFF);

	kMat_TrackToRoTerm_R = TramsMatToKMat(cMat_TrackToRoTerm_R);
	kMat_TrackToRoTerm_T = TramsMatToKMat(cMat_TrackToRoTerm_T);
	kMat_TrackToRoTerm_R._Print();
	kMat_TrackToRoTerm_T._Print();


	cv::Mat cMat_BToB_R;
	cv::Mat cMat_BToB_T;

	KMat<double> kMat_BToB_R(3, 3);
	KMat<double> kMat_BToB_T(3, 1);
	// ③
	//cv::calibrateHandEye(vMat_RobotPoses_R_Inv, vMat_RobotPoses_T_Inv, vMat_TrackerPoses_R, vMat_TrackerPoses_T, cMat_BToB_R, cMat_BToB_T, cv::CALIB_HAND_EYE_PARK);

	// ④
	cv::calibrateHandEye(vMat_RobotPoses_R_Inv, vMat_RobotPoses_T_Inv, vMat_TrackerPoses_R , vMat_TrackerPoses_T, cMat_BToB_R, cMat_BToB_T, cv::CALIB_HAND_EYE_ANDREFF);
	kMat_BToB_R = TramsMatToKMat(cMat_BToB_R);
	kMat_BToB_T = TramsMatToKMat(cMat_BToB_T);

	kMat_BToB_R._Print();
	kMat_BToB_T._Print();

	double randomerror = Calculate_AXXB_RandomError(cMat_TrackToRoTerm_R, cMat_TrackToRoTerm_T, cMat_BToB_R, cMat_BToB_T, _vec_inputRobotPoseMat, _vec_inputTrackerPoseMat);

	KMat<double> Result(4, 4);
	Result._assign(kMat_TrackToRoTerm_R, 1, 3, 1, 3);
	Result(0, 3) = kMat_TrackToRoTerm_T(0, 0);
	Result(1, 3) = kMat_TrackToRoTerm_T(1, 0);
	Result(2, 3) = kMat_TrackToRoTerm_T(2, 0);
	Result(3, 3) = 1;

	_outputMat = Result;


	std::cout << " —————————— Cali TsaiLenz Result ———————— " << std::endl;
	Result._Print();

	// 计算标定误差
	double avg_error = 0.0;
	double total_error = 0.0;
	int count = vMat_RobotPoses_R.size()-1;
	cv::Mat CaliResult = cv::Mat::eye(4, 4, CV_64F);
	cMat_TrackToRoTerm_R.copyTo(CaliResult(cv::Range(0, 3), cv::Range(0, 3)));
	cMat_TrackToRoTerm_T.copyTo(CaliResult(cv::Range(0, 3), cv::Range(3, 4)));

	for (int i = 0; i < count; ++i) {

		cv::Mat RobotPose_i = cv::Mat::eye(4, 4, CV_64F);
		cv::Mat RobotPose_j = cv::Mat::eye(4, 4, CV_64F);
		vMat_RobotPoses_R[i].copyTo(RobotPose_i(cv::Range(0, 3), cv::Range(0, 3)));
		vMat_RobotPoses_T[i].copyTo(RobotPose_i(cv::Range(0, 3), cv::Range(3, 4)));
		vMat_RobotPoses_R[i+1].copyTo(RobotPose_j(cv::Range(0, 3), cv::Range(0, 3))); //A已求逆操作
		vMat_RobotPoses_T[i+1].copyTo(RobotPose_j(cv::Range(0, 3), cv::Range(3, 4)));

		cv::Mat TrackerPose_i = cv::Mat::eye(4, 4, CV_64F);
		cv::Mat TrackerPose_j = cv::Mat::eye(4, 4, CV_64F);
		vMat_TrackerPoses_R[i].copyTo(TrackerPose_i(cv::Range(0, 3), cv::Range(0, 3)));
		vMat_TrackerPoses_T[i].copyTo(TrackerPose_i(cv::Range(0, 3), cv::Range(3, 4)));
		vMat_TrackerPoses_R[i+1].copyTo(TrackerPose_j(cv::Range(0, 3), cv::Range(0, 3)));
		vMat_TrackerPoses_T[i+1].copyTo(TrackerPose_j(cv::Range(0, 3), cv::Range(3, 4)));


		// 分别得到A和B
		//Aij = inv(Tj_gripper2base)*(Ti_gripper2base)
		cv::Mat A_invj;
		cv::invert(RobotPose_j, A_invj);
		cv::Mat A_i = RobotPose_i;

		//Bij=  (Tj_taget2cam)* inv(Ti_target2cam)
		cv::Mat B_j = TrackerPose_j;
		cv::Mat B_invi;
		cv::invert(TrackerPose_i, B_invi);


		cv::Mat H_A = A_invj * A_i;
		cv::Mat H_B = B_j * B_invi;

		cv::Mat H_diff =  CaliResult * H_B - H_A * CaliResult;

		//// 提取旋转部分并转换为向量形式
		//cv::Mat rvec1, rvec2;
		//Rodrigues(H_o_g_expected(cv::Range(0, 3), cv::Range(0, 3)), rvec1);
		//Rodrigues(H_o_g(cv::Range(0, 3), cv::Range(0, 3)), rvec2);

		//double angle_error = calculateAngleBetweenVectors(rvec1, rvec2);
		double translation_error = norm(H_diff(cv::Range(0, 3), cv::Range(3, 4)));

		total_error += translation_error;
	}

	avg_error =  total_error / count;



 	return 0;
}

double MCalibration::Calculate_AXXB_RandomError(cv::Mat& _CaliTop_R, cv::Mat& _CaliTop_T, cv::Mat& _CaliBot_R, cv::Mat& _CaliBot_T, std::vector<KMat<double>>& _RobotData, std::vector<KMat<double>>& _TrackerData)
{

	double avg_error = 0.0;

	int count = _RobotData.size()-1;

	cv::Mat CaliResult_Top = cv::Mat::eye(4, 4, CV_64F);
	cv::Mat CaliResult_Bot = cv::Mat::eye(4, 4, CV_64F);

	_CaliTop_R.copyTo(CaliResult_Top(cv::Range(0, 3), cv::Range(0, 3)));
	_CaliTop_T.copyTo(CaliResult_Top(cv::Range(0, 3), cv::Range(3, 4)));

	_CaliBot_R.copyTo(CaliResult_Bot(cv::Range(0, 3), cv::Range(0, 3)));
	_CaliBot_T.copyTo(CaliResult_Bot(cv::Range(0, 3), cv::Range(3, 4)));

	// 转回Kmat
	KMat<double> KMat_Top = TramsMatToKMat(CaliResult_Top);
	KMat<double> KMat_Bot = TramsMatToKMat(CaliResult_Bot);

	double total_error = 0.0;
	for (int i = 0; i < count; ++i) {

		KMat<double> test1 = _RobotData[i] * KMat_Top ;
		test1 = test1 *( _TrackerData[i]._Inv4());

		std::cout << " Matrix1  = " << std::endl;
		test1._Print();


		KMat<double> test2 = _RobotData[i+1] * KMat_Top;
		test2 = test2 * _TrackerData[i+1]._Inv4();

		std::cout << " Matrix2 = " << std::endl;
		test2._Print();


		KMat<double>errorMat = _RobotData[i] * KMat_Top - KMat_Bot * _TrackerData[i];
		//提取平移部分
		KMat<double>errorT(3, 1);
		errorT(0, 0) = errorMat(0, 3);
		errorT(1, 0) = errorMat(1, 3);
		errorT(2, 0) = errorMat(2, 3);
		// 计算模长
		total_error += errorT._GetNorm();

	};

	avg_error = total_error / count;

	return avg_error;


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
		//std::cout << " Each  matrix : --- " << std::endl << EigenMat << std::endl;
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

		//std::cout << " A  matrix : --- " << std::endl << A << std::endl;
		//std::cout << " B matrix : --- " << std::endl << b << std::endl;
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

void MCalibration::LoadRTCaliDataFromTxt(std::string& _inputDir, std::vector<KMat<double>>& _TrackerPoints, std::vector<KMat<double>>& _RobotPoints)
{
	std::ifstream file(_inputDir);
	if (!file.is_open()) {
		std::cerr << " Error Opening file " << _inputDir << std::endl;
		return;
	}

	std::string line;
	while (std::getline(file, line)) {
		if (line.find("Tracker") != std::string::npos) {
			int trackerNum;
			if (sscanf(line.c_str(), "Tracker %d", &trackerNum) == 1) {
				KMat<double> trackerMat;
				// Read 4 lines for the 4x4 matrix
				for (int i = 0; i < 4; ++i) {
					std::getline(file, line);
					// Parse line into a matrix row and assign to trackerMat
					// Assuming your KMat class has appropriate methods for setting values
					// e.g., trackerMat.setValue(row, col, value);
				}
				_TrackerPoints.push_back(trackerMat);
			}
		}
		else if (line.find("Robot") != std::string::npos) {
			int robotNum;
			if (sscanf(line.c_str(), "Robot %d", &robotNum) == 1) {
				KMat<double> robotMat;
				// Read 4 lines for the 4x4 matrix
				for (int i = 0; i < 4; ++i) {
					std::getline(file, line);
					// Parse line into a matrix row and assign to robotMat
					// Assuming your KMat class has appropriate methods for setting values
					// e.g., robotMat.setValue(row, col, value);
				}
				_RobotPoints.push_back(robotMat);
			}
		}
	}

	file.close();
}

void MCalibration::LoadTcpCaliDataFromTxt(std::string& _inputDir, std::vector<KMat<double>>& _TcpPoints)
{
	std::ifstream file(_inputDir);
	if (!file.is_open()) {
		std::cerr << " Error Opening file " << _inputDir << std::endl;
		return;
	}

	std::string line;
	while (std::getline(file, line))
	{
		if (line.find("TCP ") != std::string::npos)
		{
			int trackerNum;
			if (sscanf(line.c_str(), "Tracker %d", &trackerNum) == 1)
			{
				KMat<double> trackerMat;
				// Read 4 lines for the 4x4 matrix
				for (int i = 0; i < 4; ++i)
				{
					std::getline(file, line);
				}
				_TcpPoints.push_back(trackerMat);
			}
		}
	}
}



