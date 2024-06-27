#ifndef _MTESTCALIBRATE_HPP_
#define _MSTETCALIBRATE_HPP

#include "../src/RoboticCalibration/ex_RoboticCalibration.h"

class tRoboticCaliTest
{
public:
	tRoboticCaliTest();
	~tRoboticCaliTest();


	void CaliTCP();
	void CaliTCF();
	void CaliTsaiLenz();
	void TestTCP();

public :
		std::vector<KMat<double>> TCP_Kmats;
		std::vector<KMat<double>> TCF_TrackerKmats;
		std::vector<KMat<double>> Tsai_TrackerKmats;
		std::vector<KMat<double>> Tsai_RobotKmats;

		std::vector<KMat<double>> m_TestTcpDatas;
		KMat<double> m_SetTcp;
		KMat<double> m_CalTcp;
		// 结果
		KMat<double>  m_TrackerToTerminalMat;
		KMat<double>  m_RobotBaseToTrackerBase;

		// 原始数据对比
		KMat<double> m_OriginalDataMat;
		KMat<double> m_ResultDataMat;
		KMat<double> m_ResultDataMat_ByCal;
		KMat<double> m_TermR;

public:
	void TestCalculateTcp();
	void TestCalculateTcf();
	void TestCalculateTsaiLenzForRobot();

	void CalculateTraslate();

public:
	void SphericalDataFitting(std::vector<KMat<double>>& _inputData, std::vector<KMat<double>> &_outputData);

private:



};


#endif // !_MTESTCALIBRATE_HPP_
