#pragma once
#ifndef RIGIDBODY_H_INCLUDED
#define RIGIDBODY_H_INCLUDED
#include "matrixLib.h"

class rigidBody {
	friend ostream& operator<<(ostream& os, rigidBody& obj);
public:
	double x, y, phi;
	double xdt, ydt, phidt;
	double xddt, yddt, phiddt;
	
	matrix pos, vel, acc;

	rigidBody() :x(0), y(0), phi(0), xdt(0), ydt(0), phidt(0), xddt(0), yddt(0), phiddt(0) {};
	rigidBody(double x0, double y0, double phi0);
	void setRigidBody(double x0, double y0, double phi0);
	
	// 用于绘图
	matrix pointList;
	void setPointList(double** pList);

	// 记录刚体姿态、右向与对应时间
	void addq(double t,double x0,double y0,double phi0);
	void addqdt(double t, double x0dt, double y0dt, double phi0dt);
	void addqddt(double t, double x0dt, double y0dt, double phi0dt);
};

#endif // RIGIDBODY_H_INCLUDED