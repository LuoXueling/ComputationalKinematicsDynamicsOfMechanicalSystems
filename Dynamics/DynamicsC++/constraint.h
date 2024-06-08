#pragma once
#ifndef CONSTRAINT_H_INCLUDED
#define CONSTRAINT_H_INCLUDED

// 约束库，新增约束时，在头文件、cpp文件、getMat函数、findCons函数中都要修改

#include <string>
#include <iostream>
#include "matrixLib.h"
#include "rigidBody.h"
using namespace std;

class constraint {
	friend ostream& operator<<(ostream& os, constraint& obj);
public:
	string type;
	int b1, b2;
	matrix s1, s2, v1, v2, C;
	// 后续如果使用自定义函数，要考虑匿名函数方法 https://blog.csdn.net/chenlycly/article/details/82627585
	// 当约束为函数时（多项式，包括常数多项式），则INP.txt中c位置的第一项为多项式项数
	double* coef=nullptr;
	constraint() : b1(-1), b2(-1) {};
	constraint(string type0, int inp_b1, matrix inp_s1, matrix inp_v1, int inp_b2, matrix inp_s2, matrix inp_v2,double* c_inp);
	void setCons(string type0, int inp_b1, matrix inp_s1, matrix inp_v1);
	void setCons(string type0, int inp_b1, matrix inp_s1, matrix inp_v1, double* c_inp);
	void setCons(string type0, int inp_b1, matrix inp_s1, matrix inp_v1, double* c_inp, matrix C_inp);
	void setCons(string type0, int inp_b1, matrix inp_s1, matrix inp_v1, int inp_b2, matrix inp_s2, matrix inp_v2);
	void setCons(string type0, int inp_b1, matrix inp_s1, matrix inp_v1, int inp_b2, matrix inp_s2, matrix inp_v2, double* c_inp);
	void setCons(string type0, int inp_b1, matrix inp_s1, matrix inp_v1, int inp_b2, matrix inp_s2, matrix inp_v2, double* c_inp, matrix C_inp);
	/*~constraint() { 
		if (coef != nullptr) { 
			delete[] coef; 
		} 
	}*/
};

// 判断字符串数组中是否有字符串
bool findCons(const string source[], string target);
// 多项式函数
matrix c(double t, double* coef);
// 多项式导数
matrix cdt(double t, double* coef);
// 多项式二阶导
matrix cddt(double t, double* coef);
// 坐标变换矩阵
matrix A(double phi);

const string AX = "ax";
matrix ax(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string AY = "ay";
matrix ay(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string APhi = "aphi";
matrix aphi(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string AD = "ad";
matrix ad(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string RPhi = "rphi";
matrix rphi(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string RD = "rd";
matrix rd(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string T = "t";
matrix tcons(rigidBody* bodys[], constraint cons, int consNo, double t, int param); //注意此处不是t而是tcons
const string R = "r";
matrix r(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string RT = "rt";
matrix rt(rigidBody* bodys[], constraint cons, int consNo, double t, int param);

const string AXD = "axd";
matrix axd(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string AYD = "ayd";
matrix ayd(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string APhiD = "aphid";
matrix aphid(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string ADD = "add";
matrix add(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string RPhiD = "rphid";
matrix rphid(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string RRD = "rrd";
matrix rrd(rigidBody* bodys[], constraint cons, int consNo, double t, int param); //rrd在输入c时要包括theta1,theta2
const string RDD = "rdd";
matrix rdd(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string TDD = "tdd";
matrix tdd(rigidBody* bodys[], constraint cons, int consNo, double t, int param);

// 单刚体约束
const string singBodyCons[] = { AX,AY,APhi,AD,AXD,AYD,APhiD,ADD };
// 双刚体约束
const string multiBodyCons[] = { RPhi,RD,T,R,RT,RPhiD,RRD,RDD,TDD };
// 存在c，且c为常数的约束
const string constCons[] = { AY,AX,APhi,AD,RPhi,RD,RT};
// 存在c，且c为函数的约束
const string funcCons[] = { AXD,AYD,APhiD,ADD,RPhiD,RRD,RDD,TDD };

#endif // CONSTRAINT_H_INCLUDED