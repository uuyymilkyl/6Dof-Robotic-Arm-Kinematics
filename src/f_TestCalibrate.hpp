#ifndef _MTESTCALIBRATE_HPP_
#define _MSTETCALIBRATE_HPP

#include"../include/t_Matrix.hpp"
#include "./f_TransPose.hpp"
class tRoboticCaliTest
{
public:
	tRoboticCaliTest();
	~tRoboticCaliTest();

public :
		std::vector<KMat<double>> TCP_Kmats;
		std::vector<KMat<double>> TCF_TrackerKmats;
		std::vector<KMat<double>> TCF_RobotKmats;
		std::vector<KMat<double>> Tsai_TrackerKmats;
		std::vector<KMat<double>> Tsai_RobotKmats;

public:
	void TestCalculateTcp();
	void TestCalculateTcf();
	void TestCalculateTsaiLenzForRobot();

private:



};


#endif // !_MTESTCALIBRATE_HPP_
