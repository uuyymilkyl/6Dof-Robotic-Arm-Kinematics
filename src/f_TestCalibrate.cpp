#include "f_TestCalibrate.hpp"

tRoboticCaliTest::tRoboticCaliTest()
{
	KMat<double> zeroMat3(3, 3);
	m_TermR = zeroMat3;

	// 校对标定情况
	KMat<double>BaseToTrackBase, OriginalDataPoint, TrackToTerm_TR, TrackToTerm_R, ResultDataPoint_Cal1,Cal2,Cal3(4, 4);
	KMat<double>ResultDataPoint(1,3);
	/*OriginalDataPoint = {{-343.867,   444.850,  442.775, 166.825,  34.407,-66.928}};*/
	ResultDataPoint =   { {685.801,559.705,588.637,174.598,0.000,-179.997} };

	m_ResultDataMat = TranPose::EulToRot_XYZ_T(ResultDataPoint);
	m_ResultDataMat._Print();

	BaseToTrackBase = {
		              {0.9603, -0.2788, -0.00045949 ,1019.4},
					  {0.286337, 0.9603 ,-0.0051715, -156.318},
					  {0.0002965, 0.000762, 0.99999, 616.76},
					  {0,0,0,1} };


	OriginalDataPoint = { {0.331347, 0.770632, 0.544367 ,- 133.485},
		                  {0.937035, - 0.201304, - 0.285381, 807.88},
		                  {-0.11034, 0.604651, - 0.788811, 1434.12},
		 {0,0,0, 1} };

	TrackToTerm_TR = { {0.0626684, -0.997985, -0.010281 ,2.5216},
		{0.791415,  0.0435895, 0.606124,-82.336},
		{0.610737 ,0.000186942, -0.79406,133.55},
		{0,0,0,1}
	};

	TrackToTerm_R= {   {0.0626684, - 0.997985, - 0.010281 ,0},
					  {0.791415,   0.0435895, 0.606124, 0},
					  {0.610737 ,0.000186942, - 0.79406, 0},
					  {0,0,0,1} };

	ResultDataPoint_Cal1 = BaseToTrackBase * OriginalDataPoint;
	Cal2 = ResultDataPoint_Cal1 * TrackToTerm_TR;
	//m_OriginalDataMat = TranPose::EulToRot_XYZ_T(OriginalDataPoint);
	//std::cout << " example :" << std::endl;
	Cal2._Print();

	//CaliTCP();
	//CaliTCF();

	CaliTsaiLenz();

	KMat<double> r1(3, 3);
	r1 = { {0.374996,-0.791720,-0.483979 },
		{-0.887216,-0.457312,0.065059},
		{-0.268750,0.404997,-0.872658} };

	KMat<double> Angle(1, 3);

	KMat<double> r2(3, 3);
	r2 = { {0.956815,0.289983,0.015828 },
		{-0.290063,0.956997,0.002264},
		{-0.019174,-0.006758,0.999872} };




}

tRoboticCaliTest::~tRoboticCaliTest()
{
}

void tRoboticCaliTest::CaliTCP()
{
	KMat<double> TCP_Kmat1, TCP_Kmat2, TCP_Kmat3, TCP_Kmat4, TCP_Kmat5, TCP_Kmat6(4, 4);

	TCP_Kmat1 = {
		{0.394525,  0.55654 ,0.731172, -56.1792},
		{0.805097, -0.592903,0.0168806, 579.16},
		{0.442909,  0.582005,-0.681984, 200.55},
		{0, 0, 0, 1 }
	};
	TCP_Kmat2 = {
		{0.132378, 0.53522 , 0.834276 , -69.6368},
		{0.973429,-0.228861,-0.00763469,612.503},
		{0.186847, 0.813119,-0.551295,  202.039},
		{0,0,0,1} };

	TCP_Kmat3 = {
		{0.732232, 0.462192, 0.500215, -32.1872},
		{0.613321,-0.766822,-0.189266, 594.011},
		{0.296099, 0.445379,-0.844963, 210.372},
		{0,0,0,1} };
	TCP_Kmat4 = {
		{0.110109, 0.441541,  0.890459, -85.1218},
		{0.975008,-0.221919, -0.010523, 613.638},
		{0.192963, 0.869364, -0.454942, 194.608},
		{0,0,0,1} };

	TCP_Kmat5 = {
		{ 0.798567,  0.372671,  0.472661, -31.4279},
		{ 0.592018, -0.344553, -0.728559, 701.008},
		{-0.108655,  0.861626,  0.495776, 198.91},
		{0,0,0,1 } };



	TCP_Kmats.push_back(TCP_Kmat1);
	TCP_Kmats.push_back(TCP_Kmat2);
	TCP_Kmats.push_back(TCP_Kmat3);
	TCP_Kmats.push_back(TCP_Kmat4);
	//TCP_Kmats.push_back(TCP_Kmat5);
	//TCP_Kmats.push_back(TCP_Kmat6);

	KMat<double> TCP_Kmat_Test1, TCP_Kmat_Test2, TCP_Kmat_Test3(4,4);
	TCP_Kmat_Test1 = {
	{ 0.314927, 0.777234, 0.544728 ,- 7.59921},
	{ 0.592018, -0.344553, -0.728559, 701.008},
	{-0.108655,  0.861626,  0.495776, 198.91},
	{0,0,0,1 } };


	TestCalculateTcp();

}

void tRoboticCaliTest::CaliTCF()
{
	KMat<double>TCF_TrackerKmat1, TCF_TrackerKmat2, TCF_TrackerKmat3,TCF_B_ToBase(4, 4);

	//------------------ TCF 测试数据 -----------------
	TCF_TrackerKmat1 = { {  0.346517  ,0.753031,   0.559349 ,- 342.638},
						  { 0.937857 ,-0.266233 ,- 0.222584 , 438.24},
						  { -0.0186955, 0.601718, - 0.79849 , 358.239},
						  { 0, 0, 0, 1} };
	TCF_TrackerKmat2 = { { 0.342549 ,   0.752949 , 0.561896,- 150.286},
						  { 0.93933 , - 0.263095 ,- 0.220092,379.608},
						  {-0.0178863,  0.603198, - 0.797391, 355.615},
						  {0,0,0,1} };
	TCF_TrackerKmat3 = { {0.35064 , 0.753558, 0.556059 ,- 340.483},
						  {0.93629, - 0.269206, - 0.225585, 439.546},
						  {-0.0202967, 0.599732 ,- 0.799943, 555.783},
						  {0,0,0,1} };

	TCF_B_ToBase = { {0.9603, - 0.286397, - 0.000145949 ,1025.4},
					  {0.2789, 0.9603 ,- 0.0061715, -156.318},
					  {0.00602965, 0.0150762, 0.999868, 616.76},
					  {0,0,0,1} };

	TCF_TrackerKmat1 = TCF_B_ToBase * TCF_TrackerKmat1;
	TCF_TrackerKmat2 = TCF_B_ToBase * TCF_TrackerKmat2;
	TCF_TrackerKmat3 = TCF_B_ToBase * TCF_TrackerKmat3;
	TCF_TrackerKmat1._Print();
	TCF_TrackerKmat2._Print();
	TCF_TrackerKmat3._Print();

	m_TermR = TCF_TrackerKmat1._GetR();

	TCF_TrackerKmats.push_back(TCF_TrackerKmat1);
	TCF_TrackerKmats.push_back(TCF_TrackerKmat2);
	TCF_TrackerKmats.push_back(TCF_TrackerKmat3);

	TestCalculateTcf();
}

void tRoboticCaliTest::CaliTsaiLenz()
{

	KMat<double> zeroMat(4, 4);

	m_OriginalDataMat = zeroMat;
	m_ResultDataMat = zeroMat;
	m_RobotBaseToTrackerBase = zeroMat;
	m_TrackerToTerminalMat = zeroMat;

	std::string dirname = "CaliData.txt";
	MCalibration::LoadRTCaliDataFromTxt(dirname, Tsai_TrackerKmats, Tsai_RobotKmats);

	TestCalculateTsaiLenzForRobot();

}

void tRoboticCaliTest::TestTCP()
{
	m_SetTcp = { {0.0000},
				 {-76.879},
				 {134.952} };

	std::string strFile = "E:/HMProject/Data/TcpPoints627.txt ";
	m_TestTcpDatas = MCalibration::LoadTcpCaliDataFromTxt(strFile,);
}

void tRoboticCaliTest::TestCalculateTcp()
{
	//KMat<double> outputMat1;
	//MCalibration::LeastSquareSolveTCP(TCP_Kmats, outputMat1);
	std::vector<KMat<double>> filter_mats;
	SphericalDataFitting(TCP_Kmats, filter_mats);
	KMat<double> outputMat2;
	MCalibration::Calibration_OpenCV_TCP(filter_mats, outputMat2);
	m_TrackerToTerminalMat(0, 3) = outputMat2(0, 0);
	m_TrackerToTerminalMat(1, 3) = outputMat2(1, 0);
	m_TrackerToTerminalMat(2, 3) = outputMat2(2, 0);


}
void tRoboticCaliTest::TestCalculateTcf()
{
	KMat<double> outputMat;
	MCalibration::Calibration_OpenCV_TCF(m_TermR,TCF_TrackerKmats, outputMat);
	m_TrackerToTerminalMat._assign(outputMat, 1, 3, 1, 3);
}

void tRoboticCaliTest::TestCalculateTsaiLenzForRobot()
{
	KMat<double> result;
	MCalibration::Calibration_OpenCV_TsaiLenz(Tsai_RobotKmats, Tsai_TrackerKmats, result);
	result._Print();
	m_RobotBaseToTrackerBase = result;
}

void tRoboticCaliTest::CalculateTraslate()
{
	m_ResultDataMat_ByCal = m_RobotBaseToTrackerBase * m_OriginalDataMat * m_TrackerToTerminalMat;
	std::cout << "ResultMat :" << std::endl;
	m_ResultDataMat_ByCal._Print();
	std::cout << "CompareMat : " << std::endl;
	m_ResultDataMat._Print();


}

static inline void generateCombinations(const std::vector<KMat<double>>& _inputData,
	                                     int startIndex,
	                                     int currentSize,
	                                     int targetSize,
	                                     std::vector<KMat<double>>& combination,
	                                     std::vector<std::vector<KMat<double>>>& result)
{
	if (currentSize == targetSize) {
		result.push_back(combination);
		return;
	}

	for (int i = startIndex; i < _inputData.size(); ++i) {
		combination.push_back(_inputData[i]);
		generateCombinations(_inputData, i + 1, currentSize + 1, targetSize, combination, result);
		combination.pop_back();
	}
}


void tRoboticCaliTest::SphericalDataFitting(std::vector<KMat<double>>& _inputData, std::vector<KMat<double>>& _outputData)
{
	if (_inputData.size() <= 4)
	{
		_outputData = _inputData;
		return;
	}
	else
	{
		std::vector<KMat<double>> TemporaryContainer = _inputData;

		// 声明排列组合结果的容器
		std::vector<std::vector<KMat<double>>> vecCombineContainer;

		// 声明可能的组合
		std::vector<KMat<double>> combination;
		// 排列组合
		generateCombinations(_inputData, 0, 0, 4, combination, vecCombineContainer);


		// 声明点组合的圆心差
		double dDivToFittingCenter = 100;
		// 声明最终被保存下来的矩阵组合
		std::vector< KMat<double>> vecSaveGroup;
		// 寻找圆心差最小的一组数据
		for (int i = 0; i < vecCombineContainer.size(); i++)
		{
			std::vector <KMat<double>> vecKmatData = vecCombineContainer[i];
			std::vector<Point3D<double>> vecPoint3dData;
			for (int j = 0; j < vecKmatData.size(); j++)
			{
				Point3D<double> pt3TransFromKmat(0, 0, 0);
				// 逐个点转成Point3D形式
				pt3TransFromKmat = pt3TransFromKmat._fromKMat(vecKmatData[j]);
				vecPoint3dData.push_back(pt3TransFromKmat);
 			}

			// 声明拟合结果
			double dFitSphereR;
			Point3D<double> pt3FitSphereCenter(0,0,0);
			// 拟合球形
			fitSphere(vecPoint3dData, pt3FitSphereCenter, dFitSphereR);
			// 求点到坐标圆心的平均差
			double dAvgDis = 0;
			fitAvgDeviation(vecPoint3dData, pt3FitSphereCenter, dFitSphereR, dAvgDis);
			// 更新最终被保存下来的组合
			if (dAvgDis < dDivToFittingCenter)
			{
				dDivToFittingCenter = dAvgDis;
				vecSaveGroup = vecKmatData;
			}

		}
		_outputData = vecSaveGroup;
	}

}


