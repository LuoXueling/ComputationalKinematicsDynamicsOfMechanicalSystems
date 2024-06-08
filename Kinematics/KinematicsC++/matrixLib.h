#pragma once
#ifndef MATRIXLIB_H_INCLUDED
#define MATRIXLIB_H_INCLUDED
// ��˹��ȥ�����ⷽ�̡�����ʽ�ο�https://github.com/akalicki/matrix

#include <iostream>
using namespace std;

// �½�double** mat���൱��zeros�����ھ��������
double** newMat(int row, int column);
// ����double** mat�����ھ�������
double** copyMat(double** obj);
// ����ά����ת��Ϊ��̬���飬���ڲ���
double** array2mat(int row, int col, double* a);
// ���ڸ�˹��ȥ�󷵻�L��U��������
struct multiMatrix;

class matrix {
	friend ostream& operator<< (ostream& os, matrix& obj);
	
	// ��������أ����ڼ򻯼���ʽ
	friend matrix operator+ (const matrix& A, const matrix& obj);
	friend matrix operator- (const matrix& A, const matrix& obj);
	friend matrix operator- (const matrix& A);
	friend matrix operator* (const matrix& A, const matrix& obj);
	friend matrix operator* (const matrix& A, double obj);
	friend matrix operator* (const matrix& A, int obj);
	friend matrix operator* (double obj, const matrix& A);
	friend matrix operator* (int obj, const matrix& A);
	friend matrix operator/ (const matrix& A, double obj);
	friend matrix operator/ (const matrix& A, int obj);
	
public:
	double** mat;
	int row, column;
	matrix() {
		row = 1;
		column = 1;
		mat = newMat(1, 1);
	};
	// ���ƹ��캯��
	matrix(const matrix& B);
	// �൱��zeros
	matrix(int r, int col);
	// ����mat�������ɾ���
	matrix(double **m);
	// �����������ɾ���������ʽΪa[0]
	matrix(int r, int col, double* a);
	~matrix();

	void operator= (const matrix& obj);

	// ����任
	matrix T();
	matrix inv();

	// ��Bƴ�����·���
	void append(const matrix& B);
	// ��Bƴ�����ҷ���
	void combine(const matrix& B);
	// ��B���룬���Ϊa��b�У��Ǳ��0��ʼ��
	void insert(const matrix& B,int a,int b);
	// ɾ��ɾ�У����������������ʽ����������û�õ�����Ϊ������ʽ�������ʹ��LU�ֽⷨ
	void deleterow(int a);
	void deletecol(int a);
	// �����У����ڸ�˹��ȥ��
	void swapRows(int r1, int r2);
	// ��˹��ȥ����������������ϣ����ܹ����س��ȱ任����L�����ΪU���������
	multiMatrix gaussianEliminate();
	// �ú������ڸ�˹��ȥ��Ľ������Ϊ���������ȥ����໹����I����ֻ��������
	matrix rowReduceFromGaussian();
};
// ����ʽ������LU�ֽ�
double det(matrix A);
// ��λ����
matrix I(int n);
// ת�ã���A.T()�ȼ�
matrix Transp(matrix A);
// ֱ�ӷ���⣬result=inv(A)*b�����ܳ��ֲ�̬���
matrix solve(matrix A, matrix b);
//�������
matrix augment(matrix A, matrix b);
// 2-����
double norm(matrix A);

#endif // MATRIXLIB_H_INCLUDED