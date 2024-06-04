/**
*****************************************************************************
*	@file	t_Matrix.hpp
*	@brief	矩阵模板类  template of Matrix calculation
*
*	@author		HuangMinYu
*	@date		2023.12.19
*	@version	V2.0
*
*****************************************************************************
*/

#ifndef _6DOFROBOTICS_KMATRIX_HPP_
#define _6DOFROBOTICS_KMATRIX_HPP_


#include <vector>
#include<iostream>

const float EPSILON = 1e-12; // 精度阈值
const float PI = 3.14159265358;

template<typename T>
class KMat
{
public:
	KMat();
	KMat(int _nRows, int _nCols);
	~KMat();

	void operator=(const KMat<T>& _M);                                        ///< 等于 - 矩阵 - 赋值此矩阵
	void operator=(std::initializer_list<std::initializer_list<T>> _values);  ///< 等于 - 矩阵 - 赋值此二维数组
	void operator=(std::vector<std::vector<T>> _values);  ///< 等于 - 矩阵 - 赋值此二维数组
	T& operator()(int _nRow, int _nCol);


	KMat<T> operator+(const KMat<T>& _M);    ///< 加法
	KMat<T> operator-(const KMat<T>& _M);    ///< 剪法
	KMat<T> operator*(const KMat<T>& _M);    ///< 乘法
	KMat<T> operator/(const KMat<T>& _M);    ///< 除法 - 矩阵 (除一个矩阵等于左乘）
	KMat<T> operator/(T _Value);             ///< 除法 - 倍数 (缩放_Value倍）


	void _assign(KMat<T>& _M, int startRow, int endRow, int startCol, int endCol);  //部分赋值
	void _append_row(KMat<T>& _M);   //追加为行
	void _append_col(KMat<T>& _M);   //追加为列
	//打印
	void _Print();

	T _Det(); //求矩阵的行列式

	KMat<T> _Inv3(); /// <3*3矩阵求逆
	KMat<T> _Inv4(); /// <4*4矩阵求逆

	KMat<T> _Normalize(); //归一化

	KMat<T> _Orthogonal(KMat<T> &_inputMat); //正交化

	KMat<T> _Trans(); ///< 矩阵转置

	T _Trace(); /// < 矩阵求迹

	//取旋转矩阵
	KMat<T> _GetR();

	//取位移矩阵
	KMat<T> _GetT();


	//求向量模长
	static T _GetModuleLength(KMat<T> &_mat);

	//求向量叉乘(只允许向量使用）
	static KMat<T> _Cross(KMat<T>& _matA,KMat<T> &_matB);

	//求向量点乘
	static KMat<T> _Dot(KMat<T>& _matA, KMat<T>& _matB);


public:

	int m_nRows;
	int m_nCols;
	std::vector<std::vector<T>> m_vMat;

};



template<typename T>
inline KMat<T>::KMat()
{

}

template<typename T>
inline KMat<T>::KMat(int _nRows, int _nCols)
{
	m_nRows = _nRows;
	m_nCols = _nCols;
	m_vMat.resize(_nRows, std::vector<T>(_nCols, 0)); //初始化VMatrix的每个vector内的数值都为0
}

template<typename T>
inline KMat<T>::~KMat()
{
}

template<typename T>
inline void KMat<T>::operator=(const KMat<T>& _M)
{
	m_nRows = _M.m_nRows;
	m_nCols = _M.m_nCols;
	m_vMat = _M.m_vMat;
}

template<typename T>
inline void KMat<T>::operator=(std::initializer_list<std::initializer_list<T>> _values)
{
	m_nRows = _values.size();
	m_nCols = (m_nRows > 0) ? _values.begin()->size() : 0;
	m_vMat.resize(m_nRows);

	int rowIdx = 0;
	for (const auto& row : _values)
	{
		m_vMat[rowIdx].resize(m_nCols);
		int colIdx = 0;
		for (const auto& element : row)
		{
			m_vMat[rowIdx][colIdx] = element;
			colIdx++;
		}
		rowIdx++;
	}
}

template<typename T>
inline void KMat<T>::operator=(std::vector<std::vector<T>> _values)
{
	m_nRows = _values.size();
	m_nCols = (m_nRows > 0) ? _values.begin()->size() : 0;
	m_vMat.resize(m_nRows);

	int rowIdx = 0;
	for (const auto& row : _values)
	{
		m_vMat[rowIdx].resize(m_nCols);
		int colIdx = 0;
		for (const auto& element : row)
		{
			m_vMat[rowIdx][colIdx] = element;
			colIdx++;

		}
		rowIdx++;
	}
}

template<typename T>
inline T& KMat<T>::operator()(int _nRow, int _nCol)
{
	return m_vMat[_nRow][_nCol];
}

template<typename T>
inline KMat<T> KMat<T>::operator+(const KMat& _M)
{
	int newnRows = m_nRows;
	int newnCols = m_nCols;

	int disRows = this->m_nRows - _M.m_nRows;
	int disCols = this->m_nCols - _M.m_nCols;

	if (disRows != (0) || (disCols) != (0))
	{
		throw std::invalid_argument("加法矩阵未对齐");
	}

	KMat<T> result(newnRows, newnRows);

	for (int i = 0; i < m_nRows; i++)
	{
		for (int j = 0; j < m_nCols; j++)
		{
			result.m_vMat[i][j] = m_vMat[i][j] + _M.m_vMat[i][j];
		}
	}

	return result;
}

template<typename T>
inline KMat<T> KMat<T>::operator-(const KMat<T>& _M)
{
	int disRows = this->m_nRows - _M.m_nRows;
	int disCols = this->m_nCols - _M.m_nCols;
	if (disRows != (0) || (disCols) != (0))
	{
		throw std::invalid_argument("减法矩阵未对齐");
	}

	KMat<T> result(this->m_nRows,this->m_nCols);
	for (int i = 0; i < m_nRows; i++)
	{
		for (int j = 0; j < m_nCols; j++)
		{
			result.m_vMat[i][j] = m_vMat[i][j] - _M.m_vMat[i][j];
		}
	}

	return result;
}

template<typename T>
inline KMat<T> KMat<T>::operator*(const KMat<T>& _M)
{
	int m = this->m_nRows;
	int n = this->m_nCols;
	int p = _M.m_nCols;

	KMat<T> result(m, p);
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < p; j++)
		{
			for (int k = 0; k < n; k++)
			{
				result.m_vMat[i][j] += m_vMat[i][k] * _M.m_vMat[k][j];
			}
		}
	}
	return result;
}

template<typename T>
inline KMat<T> KMat<T>::operator/(const KMat<T>& _M)
{
	int m = _M.m_nRows;
	int n = _M.m_nCols;
	int p = this->m_nCols;

	KMat<T> result(m, p);
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < p; j++)
		{
			for (int k = 0; k < n; k++)
			{
				result.m_vMat[i][j] +=  _M.m_vMat[k][j] * m_vMat[i][k] ;
			}
		}
	}
	return result;
}

template<typename T>
inline KMat<T> KMat<T>::operator/(T _Value)
{
	int m = m_nRows;
	int n = m_nCols;

	KMat<T> result(m, n);
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			result(i, j) = this->m_vMat[i][j] / _Value;
		}
	}
	return result;
}

template<typename T>
inline void KMat<T>::_assign(KMat<T>& _M, int startRow, int endRow, int startCol, int endCol)
{
	if ((endRow - startRow + 1) != (_M.m_nRows) || (endCol - startCol + 1) != (_M.m_nCols))
	{
		throw std::invalid_argument("矩阵行列式或输入索引超限");
	}

	for (int i = 0; i < _M.m_nRows; ++i)
	{
		for (int j = 0; j < _M.m_nCols; ++j)
		{
			this->m_vMat[startRow - 1 + i][startCol - 1 + j] = _M.m_vMat[i][j];
		}
	}
}

template<typename T>
inline void KMat<T>::_append_row(KMat<T>& _M)
{
	int OriRowsize = this->m_nRows;
	int NewRowsize = (_M.m_nRows + OriRowsize);

	KMat<T> result(NewRowsize, this->m_nCols);

	result._assign(m_vMat, 1, m_nRows, 1, m_nCols);
	result._assign(_M, m_nRows+1, NewRowsize, 1, m_nCols);
}

template<typename T>
inline void KMat<T>::_append_col(KMat<T>& _M)
{
	int OriColsize = this->m_nCols;
	int NewColsize = (_M.m_nCols + OriColsize);
	KMat<T> result(this->m_nRows, NewColsize);

	result._assign(m_vMat, 1, m_nRows, 1, m_nCols);
	result._assign(_M, m_nCols+1, NewColsize, 1, m_nRows);
}



template<typename T>
inline KMat<T> KMat<T>::_Inv3()
{
	KMat<T> matT;
	if (m_nCols != 3 && m_nRows != 3)
	{
		throw std::invalid_argument("矩阵不为3x3");
	}



	return matT;
}

template<typename T>
inline KMat<T> KMat<T>::_Inv4()
{
	return KMat<T>();
}

template<typename T>
inline KMat<T> KMat<T>::_Normalize()
{

}

template<typename T>
inline KMat<T> KMat<T>::_Orthogonal(KMat<T>& _inputMat)
{
	KMat<T> rotateMat(3, 3);

	KMat<T> orthogonalMat(3, 3);

	if(_inputMat.m_nCols == 4 && _inputMat.m_nRows==4)
	{
		rotateMat = _inputMat._GetR();
	}
	else if (_inputMat.m_nCols == 3 && _inputMat.m_nRows == 3)
	{
		rotateMat = _inputMat;
	}

	// 步骤1：将第一个向量单位化
	for (int i = 0; i < 3; ++i)
	{
		T colLength = 0;
		for (int j = 0; j < 3; ++j)
		{
			colLength += rotateMat(i,j) * rotateMat(i,j);
		}
		colLength = std::sqrt(colLength);

		for (int j = 0; j < 3; ++j)
		{
			orthogonalMat(i,j) = rotateMat(i,j) / colLength;
		}

	}

	// 步骤2：对剩余向量进行正交化
	for (int k = 1; k < 3; ++k)
	{
		for (int i = 0; i < 3; ++i)
		{
			T dotProduct = 0;
			for (int j = 0; j < 3; ++j)
			{
				dotProduct += rotateMat(i,j) * orthogonalMat(k,j);
			}
			for (int j = 0; j < 3; ++j)
			{
				orthogonalMat(k,j) -= dotProduct * orthogonalMat(i,j);
			}
		}
	}

	if (_inputMat.m_nCols == 4 && _inputMat.m_nRows == 4)
	{
		KMat<T> fillupMat(4, 4);
		fillupMat._assign(orthogonalMat, 1, 3, 1, 3);
		fillupMat(0, 3) = _inputMat(0, 3);
		fillupMat(1, 3) = _inputMat(1, 3);
		fillupMat(2, 3) = _inputMat(2, 3);
		fillupMat(3, 3) = _inputMat(3, 3);
		return fillupMat;
	}
	return orthogonalMat;
}

template<typename T>
inline KMat<T> KMat<T>::_Trans()
{
	KMat<T> ResultMat(this->m_nCols,this->m_nRows);

	return KMat<T>();
}



template<typename T>
inline T KMat<T>::_Trace()
{
	if (m_nRows != m_nCols) {
		throw std::invalid_argument("矩阵非方针，不可求迹.");
	}
	T result;
	for (int i = 0; i < m_nCols; i++)
	{
		result += m_vMat[i][i];
	}

	return result;
}

template<typename T>
inline KMat<T> KMat<T>::_GetR()
{
	KMat<T> R(3, 3);
	R = {
		{m_vMat[0][0],m_vMat[0][1],m_vMat[0][2]},
		{m_vMat[1][0],m_vMat[1][1],m_vMat[1][2]},
		{m_vMat[2][0],m_vMat[2][1],m_vMat[2][2]}
		};
	return R;
}

template<typename T>
inline KMat<T> KMat<T>::_GetT()
{
	KMat<T> T(3, 1);
	T= {
		{m_vMat[0][3]},
		{m_vMat[1][3]},
		{m_vMat[2][3]}
		};
	return T;
}

template<typename T>
inline T KMat<T>::_GetModuleLength(KMat<T>& _mat)
{
	T tValue(0);
	for (int i = 0; i < _mat.m_nRows; i++)
	{
		for (int j = 0; j < _mat.m_nCols; j++)
		{
			tValue += _mat(i, j) * _mat(i, j);
		}
	}

	return std::sqrt(tValue);
}

template<typename T>
inline KMat<T> KMat<T>::_Cross(KMat<T>& _matA, KMat<T>& _matB)
{
	KMat<T> tResult(_matA.m_nRows, _matA.m_nCols);
	if (_matA.m_nRows == 3)
	{ //竖向向量

		tResult(0, 0) = _matA(1, 0) * _matB(2, 0) - _matA(2, 0) * _matB(1, 0);
		tResult(1, 0) = _matA(2, 0) * _matB(0, 0) - _matA(0, 0) * _matB(2, 0);
		tResult(2, 0) = _matA(0, 0) * _matB(1, 0) - _matA(1, 0) * _matB(0, 0);
	}
	if (_matA.m_nCols == 3)
	{
		//横向向量
		tResult(0, 1) = _matA(0,1) * _matB(0, 2) - _matA(0,2) * _matB( 0,1);
		tResult(0, 2) = _matA(0,2) * _matB(0, 0) - _matA(0,0) * _matB( 0,2);
		tResult(0, 3) = _matA(0,0) * _matB(0, 1) - _matA(0,1) * _matB( 0,0);
	}
	return tResult;
}



template<typename T>
inline KMat<T> KMat<T>::_Dot(KMat<T>& _matA, KMat<T>& _matB)
{
	return T();
}

template<typename T>
inline void KMat<T>::_Print()
{
	for (int i = 0; i < m_nRows; ++i) {
		for (int j = 0; j < m_nCols; ++j) {
			std::cout << m_vMat[i][j] << " ";
		}
		std::cout << std::endl;
	}
}



template<typename T>
inline T KMat<T>::_Det()
{

	if (m_nRows != m_nCols) {
		throw std::invalid_argument("矩阵非方针，不可求行列式.");
	}

	if (m_nRows == 1)
	{
		return m_vMat[0][0];
	}
	else if (m_nRows == 2)
	{
		return m_vMat[0][0] * m_vMat[1][1] - m_vMat[0][1] * m_vMat[1][0];
	}
	else {
		T det = 0;
		for (int i = 0; i < m_nRows; i++)
		{
			KMat<T> subMat(m_nRows - 1, m_nCols - 1);
			for (int j = 1; j < m_nRows; j++) {
				for (int k = 0; k < m_nCols; k++) {
					if (k < i) {
						subMat.m_vMat[j - 1][k] = m_vMat[j][k];
					}
					else if (k > i) {
						subMat.m_vMat[j - 1][k - 1] = m_vMat[j][k];
					}
				}
			}
			T sign = (i % 2 == 0) ? 1 : -1;
			det += sign * m_vMat[0][i] * subMat._Det();
		}
		return det;
	}
}


#endif