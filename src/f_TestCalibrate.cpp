#include "f_TestCalibrate.hpp"

tRoboticCaliTest::tRoboticCaliTest()
{
	KMat<double> TCP_Kmat1, TCP_Kmat2, TCP_Kmat3, TCP_Kmat4,TCP_Kmat5,TCP_Kmat6,
		TCF_TrackerKmat1, TCF_TrackerKmat2, TCF_TrackerKmat3,
		TCF_RobotKmat1, TCF_RobotKmat2, TCF_RobotKmat3,
		Tsai_TrackerKmat1, Tsai_TrackerKmat2, Tsai_TrackerKmat3, Tsai_TrackerKmat4, Tsai_TrackerKmat5, Tsai_TrackerKmat6,
		Tsai_RobotKmat1, Tsai_RobotKmat2, Tsai_RobotKmat3, Tsai_RobotKmat4, Tsai_RobotKmat5, Tsai_RobotKmat6(4, 4);
	TCP_Kmat1 = {
		{0.694019 ,0.442048, 0.568271 ,- 23.7695},
		{0.68138, - 0.148386, - 0.71673, 686.213},
		{-0.232506 ,0.884633, - 0.404185, 179.461},
		{0, 0, 0, 1 }
	};
	TCP_Kmat2 = {
		{0.225718 ,0.621845 ,0.749907 ,- 37.2059},
		{0.929632 ,0.0926582, - 0.35665 ,658.867},
		{-0.291266, 0.77764 ,- 0.557172, 190.716},
		{0,0,0,1} };
	TCP_Kmat3 = {
		{0.574796, 0.558166 ,0.598381 ,- 21.4239},
		{0.792364,-0.562282 ,- 0.23664, 589.992},
		{0.204374, 0.610155, - 0.765468, 204.884},
		{0,0,0,1} };
	TCP_Kmat4 = {
		{0.738743 ,0.329973 ,0.587687 ,- 36.4214},
		{0.529158 ,- 0.824003 ,- 0.202511 ,566.041},
		{0.417433 ,0.460583 ,- 0.783335, 196.827},
		{0,0,0,1} };

	TCP_Kmat5 = {
		{0.741537 ,0.657712 ,0.132428 ,49.6246},
		{ 0.653307 ,- 0.752793, 0.0805688, 521.599},
		{0.152682 ,0.0267715, - 0.987913 ,166.411},
		{0,0,0,1 }};

	TCP_Kmat6 = {
	{ 0.567723,  0.406011,   0.716132, -49.1352},
	{ 0.790577, - 0.511436, - 0.336782, 606.48},
	{ 0.229518, 0.757356 ,- 0.611337, 197.248},
	{ 0,0,0,1 } };


	TCP_Kmats.push_back(TCP_Kmat1);
	TCP_Kmats.push_back(TCP_Kmat2);
	TCP_Kmats.push_back(TCP_Kmat3);
	TCP_Kmats.push_back(TCP_Kmat4);
	TCP_Kmats.push_back(TCP_Kmat5);
	//TCP_Kmats.push_back(TCP_Kmat6);


	//------------------ TCF 测试数据
	TCF_TrackerKmat1 = { {  0.386453, 0.785187 ,0.483876 ,-343.987},
						  { 0.921529,-0.307146,-0.237584, 430.377},
						  {-0.0379275,0.537721,-0.842269, 341.704},
						  { 0, 0, 0, 1} };
	TCF_TrackerKmat2 = {  { 0.39047 ,  0.783225,  0.483831,-199.838},
						  { 0.920165, -0.315594, -0.231726, 384.86},
						  {-0.0287993, 0.535686, -0.843926, 339.219},
						  {0,0,0,1} };
	TCF_TrackerKmat3 = {  { 0.386351,  0.785949,  0.482719 , -341.528},
						  { 0.921305, -0.303916, -0.242552  , 431.547},
						  {-0.0439275 ,0.538442, -0.841517 , 484.882},
						  {0,0,0,1} };
	KMat<double> TCF_RobotPoint1, TCF_RobotPoint2, TCF_RobotPoint3;

	TCF_RobotPoint1 = { {600.000,0.000,800.000,180.0,0.000,180.0} };
	TCF_RobotPoint2 = { {750.000,0.000,800.000,180.0,0.000,180.0} };
	TCF_RobotPoint3 = { {600.000,0.000,950.000,180.0,0.000,180.0} };
	TCF_RobotKmat1 = TranPose::EulToRot_ZYX_T(TCF_RobotPoint1);
	TCF_RobotKmat2 = TranPose::EulToRot_ZYX_T(TCF_RobotPoint2);
	TCF_RobotKmat3 = TranPose::EulToRot_ZYX_T(TCF_RobotPoint2);

	TCF_TrackerKmats.push_back(TCF_TrackerKmat1);
	TCF_TrackerKmats.push_back(TCF_TrackerKmat2);
	TCF_TrackerKmats.push_back(TCF_TrackerKmat3);

	// --------------- TsaiLenz数据 ---------------
	KMat<double> Tsai_RobotPoint1, Tsai_RobotPoint2, Tsai_RobotPoint3, Tsai_RobotPoint4, Tsai_RobotPoint5, Tsai_RobotPoint6(6, 1);
	Tsai_RobotPoint1 = { {464.800,130.240,800.000,-161.588,  0.000,-179.982} };
	Tsai_RobotPoint2 = { {450.000,100.000,800.000, 175.543,-11.370,179.139} };
	Tsai_RobotPoint3 = { {547.000, 50.000,830.000,-175.529,  9.309,149.469} };
	Tsai_RobotPoint4 = { {547.000,130.200,745.000, 182.706, 23.830,151.330} };
	Tsai_RobotPoint5 = { {574.000,130.000,776.300, 167.332,-16.747,179.139} };
	Tsai_RobotPoint6 = { {703.000,222.250,821.250,-152.047,  15.760,-154.952} };

	Tsai_RobotKmat1 = TranPose::EulToRot_XYZ_T(Tsai_RobotPoint1);
	Tsai_RobotKmat2 = TranPose::EulToRot_XYZ_T(Tsai_RobotPoint2);
	Tsai_RobotKmat3 = TranPose::EulToRot_XYZ_T(Tsai_RobotPoint3);
	Tsai_RobotKmat4 = TranPose::EulToRot_XYZ_T(Tsai_RobotPoint4);
	Tsai_RobotKmat5 = TranPose::EulToRot_XYZ_T(Tsai_RobotPoint5);
	Tsai_RobotKmat6 = TranPose::EulToRot_XYZ_T(Tsai_RobotPoint6);

	Tsai_RobotKmats.push_back(Tsai_RobotKmat1);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat2);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat3);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat4);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat5);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat6);

	Tsai_TrackerKmat1 = {
	{ 0.321464, 0.698777, 0.63904 ,-453.687 },
	{ 0.884682, - 0.46226, 0.0604391, 553.284},
	{ 0.337636 ,0.545918 ,- 0.766795, 486.37},
	{ 0,0,0,1 }
	};
	Tsai_TrackerKmat2 = {
	{0.324365, 0.865717, 0.381211 ,-423.263 },
	{0.943211, -0.265454, - 0.199722, 592.262 },
	{-0.0717091,0.424345 ,- 0.902657 ,433.012},
	{ 0,0,0,1 }
	};
	Tsai_TrackerKmat3 = {
	{-0.179502, 0.679148, 0.711714, - 498.249 },
	{ 0.983621, 0.111867, 0.141332 ,495.151 },
	{ 0.0163677, 0.725426, - 0.688105, 465.088 },
	{ 0,0,0,1 }
	};
	Tsai_TrackerKmat4 = {
	{-0.112849, 0.496912, 0.860432, - 503.57 },
	{ 0.986601, 0.15873, 0.0377281, 590.179},
	{-0.117829, 0.853161, - 0.508166 ,345.184 },
	{ 0.00   ,  0.00    ,  0.00    ,    1.00}
	};
	Tsai_TrackerKmat5 = {
	{-0.205428, 0.941319, 0.267803, - 364.096 },
	{0.977649, 0.209888, 0.012191, 583.304},
	{-0.0447328, 0.264321, - 0.963397, 407.713},
	{ 0,0,0,1 }
	};
	Tsai_TrackerKmat6 = {
	{ 0.619068 ,0.308234 ,0.72232, - 185.987 },
	{ 0.565417 ,-0.813254, - 0.137556, 521.775},
	{ 0.54503,  0.493568 , - 0.67774,  518.086},
	{ 0,0,0,1 }
	};

	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat1);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat2);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat3);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat4);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat5);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat6);

	/*for (int i = 0; i < TCP_Kmats.size(); i++)
	{
		TCP_Kmats[i] = TCP_Kmats[i]._Orthogonal(TCP_Kmats[i]);
	}
	for (int i = 0; i < TCF_TrackerKmats.size(); i++)
	{
		TCF_TrackerKmats[i] = TCF_TrackerKmats[i]._Orthogonal(TCF_TrackerKmats[i]);
	}
	for (int i = 0; i < Tsai_RobotKmats.size(); i++)
	{
		Tsai_RobotKmats[i] = Tsai_RobotKmats[i]._Orthogonal(Tsai_RobotKmats[i]);
		Tsai_TrackerKmats[i] = Tsai_TrackerKmats[i]._Orthogonal(Tsai_TrackerKmats[i]);
	}*/
	KMat<double> zeroMat(4, 4);
	m_OriginalDataMat = zeroMat;
	m_ResultDataMat = zeroMat;
	m_RobotBaseToTrackerBase = zeroMat;
	m_TrackerToTerminalMat = zeroMat;

	//TestCalculateTcp();
	//TestCalculateTcf();
	TestCalculateTsaiLenzForRobot();

	// 校对标定情况
	// 记录原始数据点
	KMat<double>OriginalDataPoint,ResultDataPoint(6, 1);
	OriginalDataPoint = { {  -7.815,   9.259,  59.485,146.404,  0.924,-103.743} };
	ResultDataPoint =   { {1022.202,-137.347, 529.019,147.840,-22.749 ,172.770} };

	m_OriginalDataMat = TranPose::EulToRot_ZYX_T(OriginalDataPoint);
	m_ResultDataMat = TranPose::EulToRot_ZYX_T(ResultDataPoint);

	CalculateTraslate();
}

tRoboticCaliTest::~tRoboticCaliTest()
{
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
	MCalibration::Calibration_OpenCV_TCF(TCF_TrackerKmats, outputMat);
	m_TrackerToTerminalMat._assign(outputMat, 1, 3, 1, 3);
}

void tRoboticCaliTest::TestCalculateTsaiLenzForRobot()
{
	KMat<double> result;
	MCalibration::Calibration_OpenCV_TsaiLenz(Tsai_RobotKmats, Tsai_TrackerKmats, result, 6);
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


