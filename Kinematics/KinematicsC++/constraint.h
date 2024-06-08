#pragma once
#ifndef CONSTRAINT_H_INCLUDED
#define CONSTRAINT_H_INCLUDED

// Լ���⣬����Լ��ʱ����ͷ�ļ���cpp�ļ���getMat������findCons�����ж�Ҫ�޸�

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
	// �������ʹ���Զ��庯����Ҫ���������������� https://blog.csdn.net/chenlycly/article/details/82627585
	// ��Լ��Ϊ����ʱ������ʽ��������������ʽ������INP.txt��cλ�õĵ�һ��Ϊ����ʽ����
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

// �ж��ַ����������Ƿ����ַ���
bool findCons(const string source[], string target);
// ����ʽ����
matrix c(double t, double* coef);
// ����ʽ����
matrix cdt(double t, double* coef);
// ����ʽ���׵�
matrix cddt(double t, double* coef);
// ����任����
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
matrix tcons(rigidBody* bodys[], constraint cons, int consNo, double t, int param); //ע��˴�����t����tcons
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
matrix rrd(rigidBody* bodys[], constraint cons, int consNo, double t, int param); //rrd������cʱҪ����theta1,theta2
const string RDD = "rdd";
matrix rdd(rigidBody* bodys[], constraint cons, int consNo, double t, int param);
const string TDD = "tdd";
matrix tdd(rigidBody* bodys[], constraint cons, int consNo, double t, int param);

// ������Լ��
const string singBodyCons[] = { AX,AY,APhi,AD,AXD,AYD,APhiD,ADD };
// ˫����Լ��
const string multiBodyCons[] = { RPhi,RD,T,R,RT,RPhiD,RRD,RDD,TDD };
// ����c����cΪ������Լ��
const string constCons[] = { AY,AX,APhi,AD,RPhi,RD,RT};
// ����c����cΪ������Լ��
const string funcCons[] = { AXD,AYD,APhiD,ADD,RPhiD,RRD,RDD,TDD };

#endif // CONSTRAINT_H_INCLUDED