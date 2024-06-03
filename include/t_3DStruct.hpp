#ifndef _M3DSTUCT_HPP_
#define _M3DSTUCT_HPP_

#include <iostream>

struct PlaneVector3D {
    float a, b, c;

    PlaneVector3D normalize()
    {
        float magnitude = std::sqrt(a * a + b * b + c * c);
        PlaneVector3D Normal;

        if (magnitude > 0.0)
        {
            Normal.a = a / magnitude;
            Normal.b = b / magnitude;
            Normal.c = c / magnitude;
            return Normal;
        }
        else {
            // 处理零向量的情况，避免除以零
            std::cerr << "Error: Attempted to normalize a zero vector." << std::endl;
            Normal.a = 0;
            Normal.b = 0;
            Normal.c = 0;
            return Normal;
        }
    }

    void negative() {
        a = -a;
        b = -b;
        c = -c;
    }

    float norm() {
        return sqrt(a * a + b * b + c * c);
    }
};

struct Point2D
{
    float x;
    float y;

    Point2D(float x_val, float y_val) : x(x_val), y(y_val) {}


};

struct Plane3D
{
    float a;
    float b;
    float c;
    float d;
    // ax + by +cz +d  =0
    Plane3D(float _a, float _b, float _c, float _d) :a(_a), b(_b), c(_c), d(_d)
    {
    }

    void negative() {
        a = -a;
        b = -b;
        c = -c;
        d = -d;
    }

    float getMagnitude() const
    {
        return std::sqrt(a * a + b * b + c * c);
    }

    PlaneVector3D normalize()
    {
        float magnitude = std::sqrt(a * a + b * b + c * c);
        PlaneVector3D Normal;

        if (magnitude > 0.0)
        {
            Normal.a = a / magnitude;
            Normal.b = b / magnitude;
            Normal.c = c / magnitude;
            return Normal;
        }
        else {
            // 处理零向量的情况，避免除以零
            std::cerr << "Error: Attempted to normalize a zero vector." << std::endl;
            Normal.a = 0;
            Normal.b = 0;
            Normal.c = 0;
            return Normal;
        }
    }

    PlaneVector3D GetVector()
    {
        PlaneVector3D vector;
        vector.a = a;
        vector.b = b;
        vector.c = c;
        return  vector;
    }
    void showCoeff(std::string _name)
    {
        std::cout << "  " << _name << " Plane a : " << a << " b : " << b << " c : " << c << " d : " << d << " " << std::endl;
    }
};

template <typename T>
class Point3D
{
public:
    T x, y, z;
    Point3D(T _x, T _y, T _z);
    ~Point3D();
    // 重载加法
    Point3D operator+(const Point3D& p) const
    {
        return Point3D(x + p.x, y + p.y, z + p.z);
    }
    // 重载减法
    Point3D operator-(const Point3D& p) const
    {
        return Point3D(x - p.x, y - p.y, z - p.z);
    }
    // 重载乘法
    template <typename U>
    Point3D operator*(U scalar) const
    {
        return Point3D(x * scalar, y * scalar, z * scalar);
    }
    // 重载除法
    template <typename U>
    Point3D operator/(U scalar) const
    {
        return Point3D(x / scalar, y / scalar, z / scalar);
    }



public:
    T Distance(Point3D<T>& _Point);

private:

    T m_X;
    T m_Y;
    T m_Z;
};


template<typename T>
inline Point3D<T>::Point3D(T _x, T _y, T _z)
{
    m_X = x = _x;
    m_Y = y = _y;
    m_Z = z = _z;
}

template<typename T>
inline Point3D<T>::~Point3D()
{
}

template<typename T>
inline T Point3D<T>::Distance(Point3D<T>& _Point)
{
    T CurX = _Point.x;
    T CurY = _Point.y;
    T CurZ = _Point.z;
    T Distance = sqrt(pow(CurX - m_X, 2) + pow(CurY - m_Y, 2) + pow(CurZ - m_Z, 2));

    return Distance;
}



#endif