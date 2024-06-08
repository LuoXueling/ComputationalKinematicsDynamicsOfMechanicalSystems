#pragma once
#ifndef MATRIXLIB_H_INCLUDED
#define MATRIXLIB_H_INCLUDED
// 高斯消去法、解方程、行列式参考https://github.com/akalicki/matrix

#include <iostream>
using namespace std;

#define PI 3.1415926536

// 新建double** mat，相当于zeros，用于矩阵的生成
double** newMat(int row, int column);
// 复制double** mat，用于矩阵生成
double** copyMat(double** obj);
// 将二维数组转换为动态数组，用于测试
double** array2mat(int row, int col, double* a);

struct multiMatrix;

class matrix {
	friend ostream& operator<< (ostream& os, matrix& obj);
	
	// 运算符重载，用于简化计算式
	friend matrix operator+ (const matrix& A, const matrix& obj);
	friend matrix operator+ (const matrix& A, const double obj);
	friend matrix operator- (const matrix& A, const matrix& obj);
	friend matrix operator- (const matrix& A, const double obj);
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
	// 复制构造函数
	matrix(const matrix& B);
	// 相当于zeros
	matrix(int r, int col);
	// 利用mat数据生成矩阵
	matrix(double **m);
	// 利用数组生成矩阵，输入形式为a[0]
	matrix(int r, int col, double* a);
	~matrix();

	void operator= (const matrix& obj);

	// 矩阵变换
	matrix T();
	matrix inv();

	//切片,左闭右开
	matrix slice(int rowmin, int rowmax, int colmin, int colmax);
	// 将B拼接在下方行
	void append(const matrix& B);
	// 将B拼接在右方列
	void combine(const matrix& B);
	// 将B插入，起点为a行b列（角标从0开始）
	void insert(const matrix& B,int a,int b);
	// insert是在原矩阵上增加，part会覆盖原矩阵的部分
	void part(const matrix& B, int a, int b);
	// 删行删列
	void deleterow(int a);
	void deletecol(int a);
	// 交换行，用于高斯消去法
	void swapRows(int r1, int r2);
	// 高斯消去法（用在增广矩阵上），能够返回初等变换矩阵L和左侧为U的增广矩阵
	multiMatrix gaussianEliminate();
	// 该函数用于高斯消去后的结果，因为增广矩阵消去后左侧还不是I，而只是上三角
	matrix rowReduceFromGaussian();
};

// 用于高斯消去后返回L、U两个矩阵
struct multiMatrix {
	matrix mat1;
	matrix mat2;
};
// 广义逆
matrix gen_inv(matrix A);
// 行列式，基于LU分解
double det(matrix A);
// 单位矩阵
matrix I(int n);
// 转置，与A.T()等价
matrix Transp(matrix A);
// 直接法求解，result=inv(A)*b，可能出现病态情况
matrix solve(matrix A, matrix b);
//增广矩阵
matrix augment(matrix A, matrix b);
// 2-范数
double norm(matrix A);
// 最大值位置
matrix max_index(matrix A);

#endif // MATRIXLIB_H_INCLUDED