#ifndef _6DOFROBOTICS_KMATRIX_HPP_
#define _6DOFROBOTICS_KMATRIX_HPP_


#include <vector>
#include<iostream>


template<typename T>
class KMat
{
public:
	KMat();
	KMat(int _nRows, int _nCols);
	~KMat();

	void operator=(const KMat<T>& _M);                                        ///< 等于 - 矩阵 - 赋值此矩阵
	void operator=(std::initializer_list<std::initializer_list<T>> _values);  ///< 等于 - 矩阵 - 赋值此二维数组

	KMat<T> operator+(const KMat<T>& _M);    ///< 加法     
	KMat<T> operator-(const KMat<T>& _M);    ///< 剪法
	KMat<T> operator*(const KMat<T>& _M);    ///< 乘法
	KMat<T> operator/(const KMat<T>& _M);    ///< 除法 - 矩阵 (除一个矩阵等于左乘）
	KMat<T> operator/(T _Value);             ///< 除法 - 倍数 (缩放_Value倍）


	void _assign(KMat<T>& _M, int startRow, int endRow, int startCol, int endCol);  //部分赋值 
	void _append_row(KMat<T>& _M);   //追加为行
	void _append_col(KMat<T>& _M);   //追加为列
	 
private:

	int m_nRows;
	int m_nCols;
	std::vector<std::vector<T>> m_vMat;

};



template<typename T>
inline KMat<T>::KMat()
{
	m_nRows = _nRows;
	m_nCols = _nCols;
	m_vMat.resize(_nRows, std::vector<T>(_nCols, 0));
}

template<typename T>
inline KMat<T>::KMat(int _nRows, int _nCols)
{
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
			result.m_vMat[i][j] = m_vMat[i][j] + _mat.m_vMat[i][j];
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
			result.m_vMat[i][j] = m_vMat[i][j] - _mat.m_vMat[i][j];
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
				result.m_vMat[i][j] += m_vMat[i][k] * _mat.m_vMat[k][j];
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

	VMatrix<T> result(m, n);
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			result(i, j) = this->m_vMat[i][j] / _value;
		}
	}
	return result;
}

template<typename T>
inline void KMat<T>::_assign(KMat<T>& _M, int startRow, int endRow, int startCol, int endCol)
{
	if ((endRow - startRow + 1) != (_M.nRows) || (endCol - startCol + 1) != (_M.nCols)) 
	{
		throw std::invalid_argument("矩阵行列式或输入索引超限");
	}

	for (int i = 0; i < _M.nRows; ++i) 
	{
		for (int j = 0; j < _M.nCols; ++j) 
		{
			this->m_vMat[startRow - 1 + i][startCol - 1 + j] = _M.data[i][j];
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


#endif