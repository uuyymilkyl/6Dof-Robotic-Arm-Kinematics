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
	KMat<double>
		Tsai_TrackerKmat1, Tsai_TrackerKmat2, Tsai_TrackerKmat3, Tsai_TrackerKmat4, Tsai_TrackerKmat5, Tsai_TrackerKmat6, Tsai_TrackerKmat7, Tsai_TrackerKmat8, Tsai_TrackerKmat9, Tsai_TrackerKmat10, Tsai_TrackerKmat11, Tsai_TrackerKmat12, Tsai_TrackerKmat13, Tsai_TrackerKmat14,
		Tsai_RobotKmat1, Tsai_RobotKmat2, Tsai_RobotKmat3, Tsai_RobotKmat4, Tsai_RobotKmat5, Tsai_RobotKmat6, Tsai_RobotKmat7, Tsai_RobotKmat8, Tsai_RobotKmat9, Tsai_RobotKmat10, Tsai_RobotKmat11, Tsai_RobotKmat12, Tsai_RobotKmat13, Tsai_RobotKmat14(4, 4);


	// --------------- TsaiLenz数据 ---------------
	// 处理机器人信息
	// KMat<double> Tsai_RobotPoint1, Tsai_RobotPoint2, Tsai_RobotPoint3, Tsai_RobotPoint4, Tsai_RobotPoint5, Tsai_RobotPoint6(6, 1);

	// 1 ---------------
	Tsai_TrackerKmat1 = {
	{0.777891, 0.319822, 0.540925 , -32.6978},
	{0.613609, - 0.572245, -0.544077, 617.254},
	{0.135344, 0.755438, - 0.641089 ,1524.8},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat1 = {
	{ -0.805782, 0.536068 ,0.251686, 799.019 },
	{  0.550728, 0.834561 ,- 0.0143643, 268.402},
	{ -0.217748, 0.127036 ,- 0.967702 ,636.809},
	{ 0,0,0,1 }
	};
	// 2 ---------------
	Tsai_TrackerKmat2 = {
	{0.622345, 0.107215, 0.775366, - 192.871},
	{0.442207, - 0.86551 ,- 0.235256, 399.896},
	{0.645863, 0.489283, - 0.586057, 1827.73},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat2 = {
	{-0.798359, 0.416646, 0.434776, 760.184},
	{ 0.5968 , 0.643762,  0.478958, 137.513},
	{-0.0803363 , 0.641854 , - 0.762607, 888.151},
	{ 0,0,0,1 }
	};
	// 3 ---------------
	Tsai_TrackerKmat3 = {
	{0.542906, 0.233827, 0.806584, - 201.856 },
	{0.604823, - 0.775196, - 0.182374 ,521.819 },
	{0.582617, 0.586853, - 0.562283, 1828.23},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat3 = {
	{-0.875388, 0.290407, 0.386471, 731.043},
	{0.454396 ,0.767141, 0.45279 ,228.365},
	{-0.164984, 0.571978, - 0.803506, 896.577},
	{ 0,0,0,1 }
	};
	// 4 -----------------
	Tsai_TrackerKmat4 = {
	{ 0.527681, 0.368022, 0.76558 ,- 157.577},
	{ 0.631395 ,- 0.772842 ,- 0.0636795, 610.523},
	{ 0.568237 ,0.516986, - 0.640182 ,1766.27},
	{ 0 ,  0  ,  0  ,   1}
	};
	Tsai_RobotKmat4 = {
	{-0.930342, 0.266718, 0.251646, 728.4132},
	{ 0.361386 ,0.783241, 0.505899, 335.792},
	{ -0.0621673, 0.561601, - 0.82507, 829.641},
	{ 0 ,  0  ,  0  ,   1}
	};
	// 5 ----------------

	Tsai_TrackerKmat5 = {
	{ 0.678469, 0.373677, 0.632491, - 52.4438 },
	{ 0.539488, - 0.837821, - 0.0837185, 600.315},
	{ 0.498631, 0.398022, - 0.77003, 1614.39},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat5 = {
	{ -0.881603, 0.447417, 0.150312 ,782.395 },
	{  0.459094, 0.73893, 0.493168 ,360.573},
	{  0.109582, 0.503786, - 0.85685, 675.231},
	{ 0,0,0,1 }
	};
	// 6 ---------------

	Tsai_TrackerKmat6 = {
	{ 0.680039, 0.314528 ,0.662284, - 38.5049},
	{ 0.573363, - 0.791125, - 0.213017, 518.66},
	{ 0.456949, 0.524588, - 0.718334, 1628.67},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat6 = {
	{ -0.864111, 0.44251 ,0.239788 ,835.807},
	{ 0.503069, 0.773878, 0.384752, 259.488 },
	{  -0.0153099 ,0.453098, - 0.891329, 695.583},
	{ 0,0,0,1 }
	};

	// 7  ---------------

	Tsai_TrackerKmat7 = {
	{0.919262  ,0.236327 ,0.314812 ,- 84.1842},
	{ 0.326457 ,- 0.90456, - 0.274219 ,410.279},
	{ 0.219961,  0.354852 ,- 0.908679, 1480.76},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat7 = {
	{-0.660702 , 0.750446 ,0.0174171 ,724.592},
	{ 0.709874 , 0.617102 , 0.339506 ,157.177 },
	{ 0.244033 , 0.236676 ,- 0.940443, 562.434},
	{ 0,0,0,1 }
	};

	// 8  ---------------

	Tsai_TrackerKmat8 = {
	{ 0.921517, 0.114542, 0.371062, - 78.7954},
	{ 0.333673, - 0.72237, - 0.605677, 190.87 },
	{ 0.198669, 0.681955, - 0.703895, 1572.96},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat8 = {
	{-0.619235, 0.749727, 0.23336, 831.837},
	{ 0.77462, 0.63192, 0.0253013, - 114.282},
	{ -0.128496 , 0.196433, - 0.972061, 665.579},
	{ 0,0,0,1 }
	};

	// 9  ---------------

	Tsai_TrackerKmat9 = {
	{ 0.931199 ,0.189579, 0.311333, - 133.865},
	{ 0.358933, - 0.625748 ,- 0.692536, 189.029},
	{ 0.0635256,  0.756637, - 0.650742, 1552.22},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat9 = {
	{ -0.635976, 0.75034, 0.180346, 770.347},
	{ 0.743155, 0.658468 ,- 0.118918 ,- 162.416 },
	{  -0.207981, 0.0583959, - 0.976388, 667.859},
	{ 0,0,0,1 }
	};

	// 10  ---------------

	Tsai_TrackerKmat10 = {
	{ 0.95973,  0.137645 , 0.244891,  - 185.163},
	{ 0.275216, - 0.635513, - 0.721373 ,149.424 },
	{ 0.0563382, 0.759721, - 0.647803, 1572.69},
	{ 0,0,0,1 }
	};
	Tsai_RobotKmat10 = {
	{ -0.571244, 0.803253, 0.168714, 719.671},
	{ 0.793142, 0.593113, - 0.138354, - 209.211},
	{ -0.2112, 0.0547802 ,- 0.975907, 687.884},
	{ 0,0,0,1 }
	};

	// 11 -------------
	Tsai_TrackerKmat11 = {
		{0.925584, 0.151899, 0.34673 ,- 260.43},
		{0.374906, - 0.494519, - 0.784154, 135.59},
		{0.0523527, 0.855791 ,- 0.514666, 1654.28},
		{0, 0, 0 ,1}
	};

	Tsai_RobotKmat11 = {
		{-0.617127 ,0.737606 ,0.274029, 683.537},
		{ 0.696773, 0.674069 ,- 0.24523 ,- 275.613},
		{ -0.365597, 0.0395983, - 0.92993 ,785.013},
		{0, 0, 0 ,1}
	};

	// 12 -------------
	Tsai_TrackerKmat12 = {
		{0.981011, - 0.119016 ,0.153142 ,- 285.958},
		{0.0313685, - 0.68183 ,- 0.730838, 88.7638},
		{0.191398, 0.721764 ,- 0.66515, 1633.43},
		{0, 0, 0 ,1}
	};

	Tsai_RobotKmat12 = {
		{-0.346107, 0.906418, 0.242108 ,630.498},
		{ 0.920692, 0.37776 ,- 0.0981005 ,- 256.515},
		{ -0.180379, 0.188954, - 0.965277 ,725.343},
		{0, 0, 0 ,1}
	};

	// 13 -------------
	Tsai_TrackerKmat13 = {
		{0.943642,   0.163742, 0.287626 ,- 290.66},
		{0.327253, - 0.591459 ,- 0.736941 ,181.806},
		{0.0494505, 0.789534, - 0.611711 ,1665.3},
		{0, 0, 0 ,1}
	};

	Tsai_RobotKmat13 = {
		{-0.608743, 0.769089,  0.194766 ,620.624},
		{ 0.751407, 0.637684 , - 0.169547, - 221.639},
		{-0.254596, 0.0431382, - 0.966085, 785.525},
		{0, 0, 0 ,1}
	};

	// 14 -------------
	Tsai_TrackerKmat14 = {
		{0.881764,  0.240776, 0.40561, - 266.607},
		{0.464314, - 0.594543 ,- 0.656453 ,266.765},
		{0.0830941, 0.767167 ,- 0.636043, 1700.89},
		{0, 0, 0 ,1}
	};

	Tsai_RobotKmat14 = {
		{-0.714317, 0.663691, 0.221958, 644.913},
	    { 0.662275, 0.743583, -0.0920666, - 134.988},
	    {-0.226148, 0.0812324 ,- 0.9707, 815.241},
		{0, 0, 0, 1}
	};


	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat1);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat2);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat3);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat4);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat5);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat6);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat7);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat8);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat9);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat10);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat11);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat12);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat13);
	//Tsai_TrackerKmats.push_back(Tsai_TrackerKmat14);

	//Tsai_RobotKmats.push_back(Tsai_RobotKmat1);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat2);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat3);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat4);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat5);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat6);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat7);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat8);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat9);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat10);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat11);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat12);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat13);
	//Tsai_RobotKmats.push_back(Tsai_RobotKmat14);



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
	m_TestTcpDatas = MCalibration::LoadTcpCaliDataFromTxt();
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


