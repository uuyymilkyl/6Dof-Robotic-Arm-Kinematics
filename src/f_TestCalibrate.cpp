#include "f_TestCalibrate.hpp"

tRoboticCaliTest::tRoboticCaliTest()
{

	// 校对标定情况
	KMat<double>OriginalDataPoint, ResultDataPoint(6, 1);
	/*OriginalDataPoint = {{-343.867,   444.850,  442.775, 166.825,  34.407,-66.928}};
	ResultDataPoint =   { {593.7, 169.5, 809.3,179.5,6.4 ,-3.9} };


	m_ResultDataMat = TranPose::EulToRot_XYZ_T(ResultDataPoint);
	m_ResultDataMat._Print();

	CalculateTraslate();*/

	OriginalDataPoint = { {1023.8,  -138.6,  520.8, 172.5,  -3.2,-73.7} };
	m_OriginalDataMat = TranPose::EulToRot_XYZ_T(OriginalDataPoint);
	std::cout << " example :" << std::endl;
	m_OriginalDataMat._Print();
	//CaliTCP();
	//CaliTCF();
	CaliTsaiLenz();


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

	TestCalculateTcp();

}

void tRoboticCaliTest::CaliTCF()
{
	KMat<double>TCF_TrackerKmat1, TCF_TrackerKmat2, TCF_TrackerKmat3,TCF_RobotKmat1, TCF_RobotKmat2, TCF_RobotKmat3(4, 4);

	//------------------ TCF 测试数据 -----------------
	TCF_TrackerKmat1 = { {  0.3538, 0.728994 ,0.585997 ,-335.635},
						  { 0.935168 ,-0.287053 ,-0.207515,431.859},
						  { 0.016935 ,  0.621424 ,-0.783291,436.962},
						  { 0, 0, 0, 1} };
	TCF_TrackerKmat2 = { { 0.358188 , 0.724568 , 0.588815 ,-194.48},
						  { 0.93346 ,-0.290625 ,-0.210214, 385.777},
						  { 0.01881 ,  0.624931 ,-0.780453, 438.323},
						  {0,0,0,1} };
	TCF_TrackerKmat3 = { {0.347378 ,0.733421 ,0.584313 ,-334.891},
						  {0.937584 ,-0.282483, -0.202831, 434.942},
						  {0.0162981, 0.618302, -0.785772, 583.212},
						  {0,0,0,1} };


	TCF_TrackerKmats.push_back(TCF_TrackerKmat1);
	TCF_TrackerKmats.push_back(TCF_TrackerKmat2);
	TCF_TrackerKmats.push_back(TCF_TrackerKmat3);

	TestCalculateTcf();
}

void tRoboticCaliTest::CaliTsaiLenz()
{
	KMat<double>
		Tsai_TrackerKmat1, Tsai_TrackerKmat2, Tsai_TrackerKmat3, Tsai_TrackerKmat4, Tsai_TrackerKmat5, Tsai_TrackerKmat6, Tsai_TrackerKmat7, Tsai_TrackerKmat8, Tsai_TrackerKmat9, Tsai_TrackerKmat10,Tsai_TrackerKmat11, Tsai_TrackerKmat12,
		Tsai_RobotKmat1, Tsai_RobotKmat2, Tsai_RobotKmat3, Tsai_RobotKmat4, Tsai_RobotKmat5, Tsai_RobotKmat6,Tsai_RobotKmat7,Tsai_RobotKmat8, Tsai_RobotKmat9, Tsai_RobotKmat10,Tsai_RobotKmat11,Tsai_RobotKmat12 (4, 4);


	// --------------- TsaiLenz数据 ---------------
	// 处理机器人信息
	// KMat<double> Tsai_RobotPoint1, Tsai_RobotPoint2, Tsai_RobotPoint3, Tsai_RobotPoint4, Tsai_RobotPoint5, Tsai_RobotPoint6(6, 1);

	// 1 ---------------
	Tsai_TrackerKmat1 = {
	{0.102182,  0.465705, 0.879021 ,-253.679},
	{0.878413, - 0.455153, 0.14569, 578.768},
	{0.468602, 0.760474, - 0.449545, 383.555},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat1 = {
	{ -0.934294, -0.214328, 0.284883, 731.531 },
	{ -0.0493643, 0.869181, 0.492024, 273.758},
	{ -0.353069,  0.445632,- 0.822651,799.878},
	{ 0,0,0,1 }
	};
	// 2 ---------------
	Tsai_TrackerKmat2 = {
	{0.712843, 0.451156, 0.536949, - 54.6384},
	{0.701141, - 0.440952, - 0.560324, 621.381},
	{-0.0160247, 0.775899, - 0.630653, 171.259},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat2 = {
	{-0.873376, 0.44687, 0.193703, 783.041},
	{ 0.431304, 0.894374, - 0.118626, 241.585},
	{-0.226253, - 0.0200603, - 0.973862, 633.949},
	{ 0,0,0,1 }
	};
	// 3 ---------------
	Tsai_TrackerKmat3 = {
	{0.0746504, 0.404345 ,0.911555, - 301.09 },
	{0.97714 ,- 0.212208, 0.0128751, 408.026 },
	{0.198855, 0.890879, - 0.408401, 338.253},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat3 = {
	{-0.869773, -0.259044 , 0.419989, 766.725 },
	{-0.150072,  0.949672 , 0.274956, 48.7847 },
	{-0.470078,  0.176121 ,-0.864875, 791.844},
	{ 0,0,0,1 }
	};
	// 4 -----------------
	Tsai_TrackerKmat4 = {
	{ 0.799715, - 0.146518, 0.582228, - 268.216},
	{ 0.140151, - 0.897414, - 0.418338, 21.5188},
	{ 0.583793,   0.41615, - 0.69714, 381.101},
	{ 0 ,  0  ,  0  ,   1}
	};
	Tsai_RobotKmat4 = {
	{-0.547889, 0.695814, 0.464393 ,748.662},
	{ 0.83464 , 0.417241, 0.359563, -233.087},
	{ 0.0564377, 0.584608, - 0.809351, 756.477},
	{ 0 ,  0  ,  0  ,   1}
	};
	// 5 ----------------

	Tsai_TrackerKmat5 = {
	{ 0.575991, 0.289486, 0.764482, - 93.2743 },
	{ 0.73527, - 0.592149, - 0.329753 ,343.201},
	{ 0.357228, 0.752035, - 0.553923, 436.597},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat5 = {
	{ -0.874529, 0.294804, 0.385085, 883.745 },
	{ 0.396438 , 0.891927, 0.217493, 28.4085},
	{ -0.27935 , 0.342868,-0.896887, 855.462},
	{ 0,0,0,1 }
	};
	// 6 ---------------

	Tsai_TrackerKmat6 = {
	{ 0.740773, 0.56693, 0.36034, - 41.8169},
	{ 0.606015, - 0.795433, 0.00564976, 497.283},
	{ 0.289829, 0.214186, - 0.932804, 307.981},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat6 = {
	{ -0.851316 , 0.492557, -0.180687,749.932},
	{  0.373208 , 0.810588,  0.45129, 248.454 },
	{  0.368749 , 0.316757, -0.873893, 716.921},
	{ 0,0,0,1 }
	};

	// 7  ---------------

	Tsai_TrackerKmat7 = {
	{ 0.826743, 0.183496, 0.531813, - 247.438},
	{ 0.362014, - 0.897116, - 0.253238 ,184.215 },
	{ 0.430629,  0.401886,  - 0.808113,  340.58},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat7 = {
	{-0.738969 , 0.643714,  0.198889, 686.164},
	{ 0.658255 , 0.626884, 0.416794,  -91.0056 },
	{ 0.143616 , 0.438917, - 0.886976, 726.002},
	{ 0,0,0,1 }
	};

	// 8  ---------------

	Tsai_TrackerKmat8 = {
	{ 0.139826, 0.517739, 0.844035, - 355.728},
	{ 0.910173, - 0.40287, 0.0963416, 669.323 },
	{ 0.389916, 0.754746, - 0.527564, 384.14},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat8 = {
	{ 0.950692  ,-0.182181, 0.250988, 597.252},
	{ -0.0647175, 0.908002, 0.413938, 309.412},
	{ -0.303309, 0.377284, - 0.87502, 800},
	{ 0,0,0,1 }
	};

	// 9  ---------------

	Tsai_TrackerKmat9 = {
	{ 0.697688, 0.541589 , 0.468949, - 412.132},
	{ 0.702978, - 0.643683, - 0.30248, 351.442 },
	{ 0.138035,  0.540698, - 0.829815, 314.314},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat9 = {
	{ -0.911776, 0.410685, 0.001535 ,488.501},
	{  0.405582, 0.899848, 0.160553, - 65.9095 },
	{  0.0645553, 0.147011, - 0.987026, 731.221},
	{ 0,0,0,1 }
	};

	// 10  ---------------

	Tsai_TrackerKmat10 = {
	{ 0.70582,   0.528173,  0.472071,  - 239.959},
	{ 0.623938, - 0.779072 ,- 0.0612273, 514.81 },
	{ 0.335438,  0.337758 , - 0.879432, 324.94},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat10 = {
	{ -0.892489, 0.447237, - 0.0586803, 588},
	{  0.38378, 0.821253,   0.422205, 199.999},
	{  0.237017, 0.354293, - 0.904599, 723},
	{ 0,0,0,1 }
	};

	// 11 -------------
	Tsai_TrackerKmat11 = {
		{0.706591, 0.526781, 0.472473 ,- 239.962},
		{0.622918, - 0.779816, - 0.0621342, 514.621},
		{0.335711, 0.338216, - 0.879152, 324.859},
		{0, 0, 0 ,1}
	};

	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat1);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat2);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat3);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat4);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat5);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat6);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat7);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat8);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat9);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat10);
	Tsai_TrackerKmats.push_back(Tsai_TrackerKmat11);

	Tsai_RobotKmats.push_back(Tsai_RobotKmat1);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat2);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat3);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat4);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat5);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat6);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat7);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat8);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat9);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat10);
	Tsai_RobotKmats.push_back(Tsai_RobotKmat11);




	KMat<double> zeroMat(4, 4);
	m_OriginalDataMat = zeroMat;
	m_ResultDataMat = zeroMat;
	m_RobotBaseToTrackerBase = zeroMat;
	m_TrackerToTerminalMat = zeroMat;


	TestCalculateTsaiLenzForRobot();

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
	// 使用TsaiLenz算法计算相对手眼矩阵 输入：机器人矩阵，追踪器矩阵，结果，数据量
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


