#pragma once
#ifndef BACKGROUND_H_INCLUDED
#define BACKGROUND_H_INCLUDED

#include "matrixLib.h"
#include "rigidBody.h"
#include "constraint.h"


// ��������
// ��ȡINP�ļ���ȫ�ֱ�����
void readINP(double* t0, double* te, double* dt, double* e1, double* e2, string filepath);
// ����
int process(double t0, double te, double dt, double e1, double e2);
// ���csv�ļ�
void outputCSV();
// ��һ�дӶ�ȡһ������
template<typename T> T getNum(string line);
// ��ȡ���־���1-Լ�����̡�2-�Ÿ��Ⱦ���3-�ٶȷ������4-���ٶȷ�������
matrix getMat(double t, int param);
// ��ȡ��ǰ��������
matrix getq();
void update_slider(double a, double b);

#endif
