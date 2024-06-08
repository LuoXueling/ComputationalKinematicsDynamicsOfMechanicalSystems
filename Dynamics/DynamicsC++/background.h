#pragma once
#ifndef BACKGROUND_H_INCLUDED
#define BACKGROUND_H_INCLUDED

#include "matrixLib.h"
#include "rigidBody.h"
#include "constraint.h"

// ��������
void getFilename();
// ��ȡINP�ļ���ȫ�ֱ�����
void readINP();
// ����������
void formM();
// ���ɹ���������
void formQa();
// ����ʼλ�Ρ��ٶ�
matrix solveInit();
// ��ȡy����
matrix gety();
// ���Mixed differential-algebraic equations�����õ�qddt��lambda
multiMatrix solveMixedDiffAlgeEq(double t, matrix y);
// ���Constraint stablization
matrix consStab(double t, matrix y);
// ����dynamics����
int process();
// ����equilibrium����
matrix getR();
matrix getPSIq();
int equilibrium();
// ���csv�ļ�
void outputCSV();
// ��һ�дӶ�ȡһ������
template<typename T> T getNum(string line);
// ��ȡ���־���1-Լ�����̡�2-�Ÿ��Ⱦ���3-�ٶȷ������4-���ٶȷ�������
matrix getMat(double t, int param, bool isAdded = false);
// ��ȡ��ǰ�������ꡢ�ٶ�
matrix getq();
matrix getqdt();
// ��ʾ����
void update_slider(double a, double b);

#endif
