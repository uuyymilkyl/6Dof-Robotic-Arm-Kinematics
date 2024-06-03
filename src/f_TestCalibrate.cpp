#include "f_TestCalibrate.hpp"

tRoboticCaliTest::tRoboticCaliTest()
{
	KMat<double> TCP_Kmat1, TCP_Kmat2, TCP_Kmat3, TCP_Kmat4,
		TCF_TrackerKmat1, TCF_TrackerKmat2, TCF_TrackerKmat3,
		TCF_RobotKmat1, TCF_RobotKmat2, TCF_RobotKmat3,
		Tsai_TrackerKmat1, Tsai_TrackerKmat2, Tsai_TrackerKmat3, Tsai_TrackerKmat4, Tsai_TrackerKmat5, Tsai_TrackerKmat6,
		Tsai_RobotKmat1, Tsai_RobotKmat2, Tsai_RobotKmat3, Tsai_RobotKmat4, Tsai_RobotKmat5, Tsai_RobotKmat6(4, 4);
	TCP_Kmat1 = {
		{-0.290682,0.950913,0.10615,205.877},
		{0.936558,0.260060,0.235007,2796.02},
		{0.195866,0.167728,-0.96618,907.116},
		{0,0,0,1} };
	TCP_Kmat2 = {
		{-0.123936, 0.643885,0.755018,290.423},
		{ 0.840981,0.472013,-0.264489,2734.71},
		{ -0.526679, 0.602176,-0.599994,849.983},
		{0,0,0,1} };
	TCP_Kmat3 = {
		{-0.28978,0.539335,-0.790661,422.686},
		{0.383489,-0.691464,-0.61222,2660.19},
		{-0.876905,-0.48062,-0.00645646,920.816},
		{0,0,0,1} };
	TCP_Kmat4 = {
		{-0.809748,0.29375,-0.507956,306.23},
		{0.114287, 0.928046,0.354498,5898.27},
		{0.575541, 0.229001, -0.785055,887.644},
		{0,0,0,1} };

	TCP_Kmats.push_back(TCP_Kmat1);
	TCP_Kmats.push_back(TCP_Kmat2);
	TCP_Kmats.push_back(TCP_Kmat3);
	TCP_Kmats.push_back(TCP_Kmat4);


	//------------------ TCF ≤‚ ‘ ˝æ›
	TCF_TrackerKmat1 = { { 0.2884802,-0.958566,0.0062955,-337.903},
						  {-0.822675,-0.247788,-0.511671,441.618},
						  { 0.492031,0.140546,-0.859158,1104.11},
						  {0,0,0,1} };
	TCF_TrackerKmat2 = { {0.27198,-0.962226,0.00847161,-193.486},
						  {-0.821957,-0.236882,-0.517951,396.496},
						  {0.500413,0.133909,-0.855368,1101.87},
						  {0,0,0,1} };
	TCF_TrackerKmat3 = { {0.954393,-0.96031,0.00797468,-335.152},
						  {-0.820383,-0.242495,-0.517849,442.648},
						  {0.49923,0.137845,-855435,1249.52},
						  {0,0,0,1} };
	KMat<double> TCF_RobotPoint1, TCF_RobotPoint2, TCF_RobotPoint3;

	TCF_RobotPoint1 = { {600.000,0.000,800.000,180.0,0.000,180.0} };
	TCF_RobotPoint2 = { {750.000,0.000,800.000,180.0,0.000,180.0} };
	TCF_RobotPoint3 = { {600.000,0.000,950.000,180.0,0.000,180.0} };
	TCF_RobotKmat1 = TranPose::EulToRot_XYZ_T(TCF_RobotPoint1);
	TCF_RobotKmat2 = TranPose::EulToRot_XYZ_T(TCF_RobotPoint2);
	TCF_RobotKmat3 = TranPose::EulToRot_XYZ_T(TCF_RobotPoint2);



	KMat<double> Tsai_RobotPoint1, Tsai_RobotPoint2, Tsai_RobotPoint3, Tsai_RobotPoint4, Tsai_RobotPoint5, Tsai_RobotPoint6(6, 1);
	Tsai_RobotPoint1 = { {464.800,130.240,800.000,-161.588,  0.000,-179.982} };
	Tsai_RobotPoint2 = { {450.000,100.000,800.000, 175.543,-11.370,179.139} };
	Tsai_RobotPoint3 = { {547.000, 50.000,830.000,-175.529,  9.309,149.469} };
	Tsai_RobotPoint4 = { {547.000,130.200,745.000, 182.706, 23.830,151.330} };
	Tsai_RobotPoint5 = { {574.000,130.000,776.300, 167.332,-16.747,179.139} };
	Tsai_RobotPoint6 = { {574.000,130.250,744.250, 140.780,-16.145,146.881} };

	Tsai_RobotKmat1 = TranPose::EulToRot_XYZ_T(Tsai_RobotPoint1);
	Tsai_RobotKmat2 = TranPose::EulToRot_XYZ_T(Tsai_RobotPoint2);
	Tsai_RobotKmat3 = TranPose::EulToRot_XYZ_T(Tsai_RobotPoint3);
	Tsai_RobotKmat4 = TranPose::EulToRot_XYZ_T(Tsai_RobotPoint4);
	Tsai_RobotKmat5 = TranPose::EulToRot_XYZ_T(Tsai_RobotPoint5);
	Tsai_RobotKmat6 = TranPose::EulToRot_XYZ_T(Tsai_RobotPoint6);

	Tsai_TrackerKmat1 = {
	{ 0.273061,-0.902117,-0.3341,-449.875 },
	{ -0.874216,-0.087758,-0.47754,534.165 },
	{ 0.401477,0.422474,-0.812608,1179.91 },
	{ 0,0,0,1 }
	};
	Tsai_TrackerKmat2 = {
	{ 0.284307,-0.95581,0.0748141,-419.604 },
	{ -0.900125,-0.292978,-0.322394,586.596 },
	{ 0.330067,0.0243169,-0.943644,1095.86 },
	{ 0,0,0,1 }
	};
	Tsai_TrackerKmat3 = {
	{ -0.212551,-0.977002, 0.0170128,-490.68 },
	{ -0.747643, 0.151393,-0.646614,491.076 },
	{  0.629168,-0.150158,-0.762628,1127.87 },
	{ 0,0,0,1 }
	};
	Tsai_TrackerKmat4 = {
	{ -0.129889,-0.9776819, 0.0170159,-493.571 },
	{ -0.623823,-0.0528835,-0.779774,584.921 },
	{  0.770697,-0.2074340,-0.602493,1009.72 },
	{  0,0,0,1}
	};
	Tsai_TrackerKmat5 = {
	{ -0.256141,-0.966320, 0.0248343,-421.486 },
	{ -0.955037, 0.249014,-0.1609210,573.988 },
	{  0.149317,-0.0649361,-0.986655,1076.18 },
	{ 0,0,0,1 }
	};
	Tsai_TrackerKmat6 = {
	{ -0.263660,-0.8710551,0.414422,-344.761 },
	{ -0.942873,-0.0528835,0.0799237,636.114 },
	{ -0.203652,-0.369675,-0.906568,958.562 },
	{ 0,0,0,1 }
	};





	TestCalculateTcp();
	TestCalculateTcf();
	TestCalculateTsaiLenzForRobot();
}

tRoboticCaliTest::~tRoboticCaliTest()
{
}

void tRoboticCaliTest::TestCalculateTcp()
{
}
void tRoboticCaliTest::TestCalculateTcf()
{
}

void tRoboticCaliTest::TestCalculateTsaiLenzForRobot()
{
}
