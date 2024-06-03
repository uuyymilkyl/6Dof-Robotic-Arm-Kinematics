#include "f_TransPose.hpp"

KMat<float> TrPose::EulToRot_XYZ(KMat<float>& _EulAngle)
{
    KMat<float> Mat3_3(3, 3);

    // 角度转弧度
    float a1 = _EulAngle(0, 0) / 180.0f * PI;
    float a2 = _EulAngle(0, 1) / 180.0f * PI;
    float a3 = _EulAngle(0, 2) / 180.0f * PI;
    Mat3_3 = {
        {cos(a2) * cos(a3),                               -cos(a2) * sin(a3)                              ,            sin(a2)},
        {sin(a1) * sin(a2) * cos(a3) + cos(a1) * sin(a3), -sin(a1) * sin(a2) * sin(a3) + cos(a1) * cos(a3), -sin(a1) * cos(a2)},
        {-cos(a1) * sin(a2) * cos(a3) + sin(a1) * sin(a3),  cos(a1) * sin(a2) * sin(a3) + sin(a1) * cos(a3),  cos(a1) * cos(a2)}

    };
    return Mat3_3;
}

KMat<float> TrPose::EulToRot_XYZ_T(KMat<float>& _EulAngle)
{
    KMat<float> Mat4_4(4, 4);

    // 角度转弧度
    float a1 = _EulAngle(0, 0) / 180.0f * PI ;
    float a2 = _EulAngle(0, 1) / 180.0f * PI;
    float a3 = _EulAngle(0, 2) / 180.0f * PI;
    Mat4_4 = {
        {cos(a2) * cos(a3),                               -cos(a2) * sin(a3)                              ,            sin(a2),0},
        {sin(a1) * sin(a2) * cos(a3) + cos(a1) * sin(a3), -sin(a1) * sin(a2) * sin(a3) + cos(a1) * cos(a3), -sin(a1) * cos(a2),0},
        {-cos(a1)* sin(a2) * cos(a3) + sin(a1) * sin(a3),  cos(a1) * sin(a2) * sin(a3) + sin(a1) * cos(a3),  cos(a1) * cos(a2),0},
        {0,0,0,1}

    };
    return Mat4_4;
}

KMat<float> TrPose::RotToEul_XYZ(KMat<float>& _Rot)
{
    KMat<float> EulAngle(1, 3);

    EulAngle = { 
      { (180 / PI) * atan2(-_Rot(1, 2),  _Rot(2, 2)), 
        (180 / PI) * atan2(_Rot(0, 2), sqrt(1-( _Rot(0,2)*_Rot(0,2) ))),
        (180 / PI) * atan2(- _Rot(0, 1), _Rot(0,0))} 
    };

    return EulAngle;
}

KMat<float> TrPose::EulToQ4_ZXZ(KMat<float>& _EulAngle)
{

    KMat<float> Q4(1, 4);

    // 角度转弧度
    float a1 = _EulAngle(0, 0) / 180.0f * PI;
    float a2 = _EulAngle(0, 1) / 180.0f * PI;
    float a3 = _EulAngle(0, 2) / 180.0f * PI;

    float q1 = +cos(a2 / 2) * cos((a1+a3)/2);
    float q2 = -sin(a2 / 2) * cos((a1-a3)/2);
    float q3 = +sin(a2 / 2) * sin((a1-a3)/2);
    float q4 = +cos(a2 / 2) * sin((a1+a3)/2);

    Q4 = { {q1,q2,q3,q4} };

    return Q4;
}

KMat<float> TrPose::EulToQ4_ZYX(KMat<float>& _EulAngle)
{
    KMat<float> Q4(1, 4);

    // 角度转弧度
    float a1 = _EulAngle(0, 0) / 180.0f * PI;
    float a2 = _EulAngle(0, 1) / 180.0f * PI;
    float a3 = _EulAngle(0, 2) / 180.0f * PI;

    float q1 = +sin(a1 / 2) * sin(a2 / 2) * sin(a3 / 2) + cos(a1 / 2) * cos(a2 / 2) * cos(a3 / 2);
    float q2 = -sin(a1 / 2) * sin(a2 / 2) * cos(a3 / 2) + sin(a3 / 2) * cos(a1 / 2) * cos(a2 / 2);
    float q3 = +sin(a1 / 2) * sin(a3 / 2) * cos(a2 / 2) + sin(a2 / 2) * cos(a1 / 2) * cos(a3 / 2);
    float q4 = +sin(a1 / 2) * cos(a2 / 2) * cos(a3 / 2) - sin(a2 / 2) * sin(a3 / 2) * cos(a1 / 2);

    Q4 = { {q1,q2,q3,q4} };

    return Q4;
}

KMat<float> TrPose::EulToQ4_XYZ(KMat<float>& _EulAngle)
{
    KMat<float> Q4(1, 4);

    // 角度转弧度
    float a1 = _EulAngle(0, 0) / 180.0f * PI;
    float a2 = _EulAngle(0, 1) / 180.0f * PI;
    float a3 = _EulAngle(0, 2) / 180.0f * PI;

    float q1 = -sin(a1 / 2) * sin(a2 / 2) * sin(a3 / 2) + cos(a1 / 2) * cos(a2 / 2) * cos(a3 / 2);
    float q2 =  sin(a1 / 2) * cos(a2 / 2) * cos(a3 / 2) + sin(a2 / 2) * sin(a3 / 2) * cos(a1 / 2);
    float q3 = -sin(a1 / 2) * sin(a3 / 2) * cos(a2 / 2) + sin(a2 / 2) * cos(a1 / 2) * cos(a3 / 2);
    float q4 = sin(a1 / 2) * sin(a2 / 2) * cos(a3 / 2) + sin(a3 / 2) * cos(a1 / 2) * cos(a2 / 2);

    Q4 = { {q1,q2,q3,q4} };

    return Q4;
}

KMat<float> TrPose::VectorToRot(KMat<float>& _Vector1_3)
{
    KMat<float> Mat3_3(3, 3);

    return Mat3_3;
}

KMat<float> TrPose::VectorXYZToRotTcp(KMat<float>& _VectorX, KMat<float>& _VectorY, KMat<float>& _VectorZ)
{
    KMat<float> Mat3_3(3, 3);
 
    return Mat3_3;
}

KMat<float> TrPose::EulToRot_ZXZ(KMat<float>& _EulAngle)
{
    KMat<float> Mat3_3(3, 3);

    // 角度转弧度
    float a1 = _EulAngle(0, 0) / 180.0f * PI;
    float a2 = _EulAngle(0, 1) / 180.0f * PI;
    float a3 = _EulAngle(0, 2) / 180.0f * PI;
    Mat3_3 = {
        {-sin(a1) * cos(a2) * sin(a3) + cos(a1) * cos(a3), -sin(a1) * cos(a2) * cos(a3) - cos(a1) * sin(a3),  sin(a1) * sin(a2)},
        { cos(a1) * cos(a2) * sin(a3) + sin(a1) * cos(a3),  cos(a1) * cos(a2) * cos(a3) - sin(a1) * sin(a3), -cos(a1) * sin(a2)},
        {                               sin(a2) * sin(a3),                         sin(a2) * cos(a3),                   cos(a2)},

    };
    return Mat3_3;
}

KMat<float> TrPose::EulToRot_ZXZ_T(KMat<float>& _EulAngle)
{
    KMat<float> Mat4_4(4, 4);

    // 角度转弧度
    float a1 = _EulAngle(0, 0) / 180.0f * PI;
    float a2 = _EulAngle(0, 1) / 180.0f * PI;
    float a3 = _EulAngle(0, 2) / 180.0f * PI;
    Mat4_4 = {
        {-sin(a1) * cos(a2) * sin(a3) + cos(a1) * cos(a3), -sin(a1) * cos(a2) * cos(a3) - cos(a1) * sin(a3),  sin(a1) * sin(a2)},
        { cos(a1) * cos(a2) * sin(a3) + sin(a1) * cos(a3),  cos(a1) * cos(a2) * cos(a3) - sin(a1) * sin(a3), -cos(a1) * sin(a2)},
        {                               sin(a2) * sin(a3),                         sin(a2) * cos(a3),                   cos(a2)},
        {0,0,0,1}

    };
    return Mat4_4;
}

KMat<float> TrPose::RotToEul_ZXZ(KMat<float>& _RotateMat3_3or4_4)
{
    return KMat<float>();
}

KMat<float> TrPose::EulToRot_ZYX(KMat<float>& _EulAngle)
{
    return KMat<float>();
}

KMat<float> TrPose::EulToRot_ZYX_T(KMat<float>& _EulAngle)
{
    return KMat<float>();
}
