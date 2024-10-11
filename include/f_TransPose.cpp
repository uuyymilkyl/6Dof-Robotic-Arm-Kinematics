#include"../src/f_TransPose.hpp"

TranPose::TranPose()
{
}

TranPose::~TranPose()
{
}

KMat<double> TranPose::EulToRot_X(double& _Radian)
{
    double Rad = _Radian;
    KMat<double> RotateMat(3, 3);

    RotateMat = {
        {1 ,  0 ,  0 },
        {0 , cos(Rad), -sin(Rad)},
        {0 , sin(Rad),  cos(Rad)}
    };
    return RotateMat;
}

KMat<double> TranPose::EulToRot_Y(double& _Radian)
{
    double Rad = _Radian;
    KMat<double> RotateMat(3, 3);

    RotateMat = {
        {cos(Rad) ,  0 ,  sin(Rad)},
        {0 ,         0 ,         0},
        {-sin(Rad),  0 ,  cos(Rad)}
    };
    return RotateMat;
}

KMat<double> TranPose::EulToRot_Z(double& _Radian)
{
    double Rad = _Radian;
    KMat<double> RotateMat(3, 3);

    RotateMat = {
        {cos(Rad) , -sin(Rad) ,  0},
        {sin(Rad)  , cos(Rad), 0},
        {0  ,  0 ,  0}
    };
    return RotateMat;
}


KMat<double> TranPose::EulToRot_XYZ(KMat<double>& _EulAngle)
{
    KMat<double> Mat3_3(3, 3);

    // 角度转弧度
    double a1 = _EulAngle(0, 0) / 180.0f * PI;
    double a2 = _EulAngle(0, 1) / 180.0f * PI;
    double a3 = _EulAngle(0, 2) / 180.0f * PI;
    Mat3_3 = {
        {cos(a2) * cos(a3),                               -cos(a2) * sin(a3)                              ,            sin(a2)},
        {sin(a1) * sin(a2) * cos(a3) + cos(a1) * sin(a3), -sin(a1) * sin(a2) * sin(a3) + cos(a1) * cos(a3), -sin(a1) * cos(a2)},
        {-cos(a1) * sin(a2) * cos(a3) + sin(a1) * sin(a3),  cos(a1) * sin(a2) * sin(a3) + sin(a1) * cos(a3),  cos(a1) * cos(a2)}

    };
    return Mat3_3;
}

KMat<double> TranPose::EulToRot_XYZ_T(KMat<double>& _EulAngle)
{
    KMat<double> Mat4_4(4, 4);

    // 角度转弧度
    double a1 = _EulAngle(0, 3) /180.0f * PI ;
    double a2 = _EulAngle(0, 4) / 180.0f * PI;
    double a3 = _EulAngle(0, 5) / 180.0f * PI;

    Mat4_4 = {
        {cos(a2) * cos(a3),                               -cos(a2) * sin(a3)                              ,            sin(a2), _EulAngle(0, 0)},
        {sin(a1) * sin(a2) * cos(a3) + cos(a1) * sin(a3), -sin(a1) * sin(a2) * sin(a3) + cos(a1) * cos(a3), -sin(a1) * cos(a2), _EulAngle(0, 1)},
        {-cos(a1)* sin(a2) * cos(a3) + sin(a1) * sin(a3),  cos(a1) * sin(a2) * sin(a3) + sin(a1) * cos(a3),  cos(a1) * cos(a2), _EulAngle(0, 2)},
        {0,0,0,1}

    };
    return Mat4_4;
}



KMat<double> TranPose::RotToQ4(KMat<double>& _RotateMat)
{
    return KMat<double>();
}

KMat<double> TranPose::Q4ToRot(KMat<double>& _RotateMat)
{
    return KMat<double>();
}

KMat<double> TranPose::EulToQ4_ZXZ(KMat<double>& _EulAngle)
{

    KMat<double> Q4(1, 4);

    // 角度转弧度
    double a1 = _EulAngle(0, 0) / 180.0f * PI;
    double a2 = _EulAngle(0, 1) / 180.0f * PI;
    double a3 = _EulAngle(0, 2) / 180.0f * PI;

    double q1 = +cos(a2 / 2) * cos((a1+a3)/2);
    double q2 = -sin(a2 / 2) * cos((a1-a3)/2);
    double q3 = +sin(a2 / 2) * sin((a1-a3)/2);
    double q4 = +cos(a2 / 2) * sin((a1+a3)/2);

    Q4 = { {q1,q2,q3,q4} };

    return Q4;
}

KMat<double> TranPose::EulToQ4_ZYX(KMat<double>& _EulAngle)
{
    KMat<double> Q4(1, 4);

    // 角度转弧度
    double a1 = _EulAngle(0, 0) / 180.0f * PI;
    double a2 = _EulAngle(0, 1) / 180.0f * PI;
    double a3 = _EulAngle(0, 2) / 180.0f * PI;

    double q1 = +sin(a1 / 2) * sin(a2 / 2) * sin(a3 / 2) + cos(a1 / 2) * cos(a2 / 2) * cos(a3 / 2);
    double q2 = -sin(a1 / 2) * sin(a2 / 2) * cos(a3 / 2) + sin(a3 / 2) * cos(a1 / 2) * cos(a2 / 2);
    double q3 = +sin(a1 / 2) * sin(a3 / 2) * cos(a2 / 2) + sin(a2 / 2) * cos(a1 / 2) * cos(a3 / 2);
    double q4 = +sin(a1 / 2) * cos(a2 / 2) * cos(a3 / 2) - sin(a2 / 2) * sin(a3 / 2) * cos(a1 / 2);

    Q4 = { {q1,q2,q3,q4} };

    return Q4;
}

KMat<double> TranPose::EulToQ4_XYZ(KMat<double>& _EulAngle)
{
    KMat<double> Q4(1, 4);

    // 角度转弧度
    double a1 = _EulAngle(0, 0) / 180.0f * PI;
    double a2 = _EulAngle(0, 1) / 180.0f * PI;
    double a3 = _EulAngle(0, 2) / 180.0f * PI;

    double q1 = -sin(a1 / 2) * sin(a2 / 2) * sin(a3 / 2) + cos(a1 / 2) * cos(a2 / 2) * cos(a3 / 2);
    double q2 =  sin(a1 / 2) * cos(a2 / 2) * cos(a3 / 2) + sin(a2 / 2) * sin(a3 / 2) * cos(a1 / 2);
    double q3 = -sin(a1 / 2) * sin(a3 / 2) * cos(a2 / 2) + sin(a2 / 2) * cos(a1 / 2) * cos(a3 / 2);
    double q4 = sin(a1 / 2) * sin(a2 / 2) * cos(a3 / 2) + sin(a3 / 2) * cos(a1 / 2) * cos(a2 / 2);

    Q4 = { {q1,q2,q3,q4} };

    return Q4;
}

KMat<double> TranPose::RotToVector(KMat<double>& _RotateMat)
{
    return KMat<double>();
}

KMat<double> TranPose::VectorToRot(KMat<double>& _Vector1_3)
{
    KMat<double> Mat3_3(3, 3);

    return Mat3_3;
}

KMat<double> TranPose::VectorXYZToRotTcp(KMat<double>& _VectorX, KMat<double>& _VectorY, KMat<double>& _VectorZ)
{
    KMat<double> Mat3_3(3, 3);

    return Mat3_3;
}

KMat<double> TranPose::EulToRot_ZXZ(KMat<double>& _EulAngle)
{
    KMat<double> Mat3_3(3, 3);

    // 角度转弧度
    double a1 = _EulAngle(0, 0) / 180.0f * PI;
    double a2 = _EulAngle(0, 1) / 180.0f * PI;
    double a3 = _EulAngle(0, 2) / 180.0f * PI;
    Mat3_3 = {
        {-sin(a1) * cos(a2) * sin(a3) + cos(a1) * cos(a3), -sin(a1) * cos(a2) * cos(a3) - cos(a1) * sin(a3),  sin(a1) * sin(a2)},
        { cos(a1) * cos(a2) * sin(a3) + sin(a1) * cos(a3),  cos(a1) * cos(a2) * cos(a3) - sin(a1) * sin(a3), -cos(a1) * sin(a2)},
        {                               sin(a2) * sin(a3),                         sin(a2) * cos(a3),                   cos(a2)},

    };
    return Mat3_3;
}

KMat<double> TranPose::EulToRot_ZXZ_T(KMat<double>& _EulAngle)
{
    KMat<double> Mat4_4(4, 4);

    // 角度转弧度
    double a1 = _EulAngle(0, 3) / 180.0f * PI;
    double a2 = _EulAngle(0, 4) / 180.0f * PI;
    double a3 = _EulAngle(0, 5) / 180.0f * PI;
    Mat4_4 = {
        {-sin(a1) * cos(a2) * sin(a3) + cos(a1) * cos(a3), -sin(a1) * cos(a2) * cos(a3) - cos(a1) * sin(a3),  sin(a1) * sin(a2),_EulAngle(0, 0)},
        { cos(a1) * cos(a2) * sin(a3) + sin(a1) * cos(a3),  cos(a1) * cos(a2) * cos(a3) - sin(a1) * sin(a3), -cos(a1) * sin(a2),_EulAngle(0, 1)},
        {                               sin(a2) * sin(a3),                         sin(a2) * cos(a3),                   cos(a2),_EulAngle(0, 2)},
        {0,0,0,1}

    };
    return Mat4_4;
}


KMat<double> TranPose::EulToRot_ZYX(KMat<double>& _EulAngle)
{

    double a1 = _EulAngle(0, 3) / 180.0f * PI;
    double a2 = _EulAngle(0, 4) / 180.0f * PI;
    double a3 = _EulAngle(0, 5) / 180.0f * PI;


    KMat<double> RotateMat(3, 3);
    RotateMat = EulToRot_X(a1) * EulToRot_Y(a2) * EulToRot_Z(a3);

    return RotateMat;
}

KMat<double> TranPose::EulToRot_ZYX_T(KMat<double>& _EulAngle)
{
    double a1 = _EulAngle(0, 3) / 180.0f * PI;
    double a2 = _EulAngle(0, 4) / 180.0f * PI;
    double a3 = _EulAngle(0, 5) / 180.0f * PI;

    KMat<double> RotateMat(3, 3);
    RotateMat = EulToRot_X(a1) * EulToRot_Y(a2) * EulToRot_Z(a3);

    KMat<double> RotateMat_T(4, 4);

    return RotateMat;
}
