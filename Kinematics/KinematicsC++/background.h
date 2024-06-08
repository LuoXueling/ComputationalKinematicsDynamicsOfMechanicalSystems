#pragma once
#ifndef BACKGROUND_H_INCLUDED
#define BACKGROUND_H_INCLUDED

#include "matrixLib.h"
#include "rigidBody.h"
#include "constraint.h"


// 函数声明
// 读取INP文件到全局变量中
void readINP(double* t0, double* te, double* dt, double* e1, double* e2, string filepath);
// 计算
int process(double t0, double te, double dt, double e1, double e2);
// 输出csv文件
void outputCSV();
// 从一行从读取一个数字
template<typename T> T getNum(string line);
// 获取四种矩阵：1-约束方程、2-雅各比矩阵、3-速度方程右项、4-加速度方程右项
matrix getMat(double t, int param);
// 获取当前刚体坐标
matrix getq();
void update_slider(double a, double b);

#endif
