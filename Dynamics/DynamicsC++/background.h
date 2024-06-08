#pragma once
#ifndef BACKGROUND_H_INCLUDED
#define BACKGROUND_H_INCLUDED

#include "matrixLib.h"
#include "rigidBody.h"
#include "constraint.h"

// 函数声明
void getFilename();
// 读取INP文件到全局变量中
void readINP();
// 生成质量阵
void formM();
// 生成广义力矩阵
void formQa();
// 求解初始位形、速度
matrix solveInit();
// 获取y矩阵
matrix gety();
// 求解Mixed differential-algebraic equations，即得到qddt和lambda
multiMatrix solveMixedDiffAlgeEq(double t, matrix y);
// 求解Constraint stablization
matrix consStab(double t, matrix y);
// 计算dynamics问题
int process();
// 计算equilibrium问题
matrix getR();
matrix getPSIq();
int equilibrium();
// 输出csv文件
void outputCSV();
// 从一行从读取一个数字
template<typename T> T getNum(string line);
// 获取四种矩阵：1-约束方程、2-雅各比矩阵、3-速度方程右项、4-加速度方程右项
matrix getMat(double t, int param, bool isAdded = false);
// 获取当前刚体坐标、速度
matrix getq();
matrix getqdt();
// 显示进度
void update_slider(double a, double b);

#endif
