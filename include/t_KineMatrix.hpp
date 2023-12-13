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

	void operator=(const KMat<T>& _M);                                        ///< ���ص��� - ���� - ��ֵ�˾���
	void operator=(std::initializer_list<std::initializer_list<T>> _values);  ///< ���ص��� - ���� - ��ֵ�˶�ά����

	KMat<T> operator+(const KMat<T>& _M);    ///< ���ؼӷ�     (��ʵ�ֲ�ͬ�������ľ������)
	KMat<T> operator-(const KMat<T>& _M);    ///< ���ؼ���
	KMat<T> operator*(const KMat<T>& _M);    ///< ���س˷�
	KMat<T> operator/(const KMat<T>& _M);    ///< ���س��� - ���� (��һ�����������ˣ�
	KMat<T> operator/(T _Value);             ///< ���س��� - ���� (����_Value����


	void Evaluate

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
	int newnRows;
	int newnCols;
	if (this->m_nRows >= _M.m_nRows)
	{
		newnRows = this->m_nRows;
	}
	else
	{
		newnRows = _M.m_nRows;
	}

	if (this->m_nCols >= _M.m_nCols)
	{
		newnCols = this->m_nCols;
	}
	else
	{
		newnCols = _M.m_nCols;
	}

	KMat<T> newMat(newnRows, newnCols);


}


#endif