#include "rigidBody.h"
#include "matrixLib.h"

rigidBody::rigidBody(double x0, double y0, double phi0) {
	setq(x0, y0, phi0);

}

void rigidBody::setq(double x0, double y0, double phi0) {
	x = x0;
	y = y0;
	phi = phi0;
}

void rigidBody::setqdt(double x0dt, double y0dt, double phi0dt) {
	xdt = x0dt;
	ydt = y0dt;
	phidt = phi0dt;
}

void rigidBody::setInertiaParams(double tmpm, double tmpj) {
	m = tmpm;
	J = tmpj;
}

void rigidBody::setPointList(double** pList) {
	matrix tmp(pList);
	pointList = tmp;
}

ostream& operator<<(ostream& os, rigidBody& obj) {
	os << "x = " << obj.x << " y = " << obj.y << " phi = " << obj.phi << " m = " << obj.m << " Jc = " << obj.J << endl;
	return os;
}

void rigidBody::addq(double t, double x0, double y0, double phi0) {
	x = x0;
	y = y0;
	phi = phi0;
	double tmp[1][4] = { t,x0,y0,phi0 };
	matrix tmp2(1, 4, tmp[0]);
	if (pos.column == 1) {
		pos = tmp2;
	}
	else {
		pos.append(tmp2);
	}
}

void rigidBody::addqdt(double t, double x0dt, double y0dt, double phi0dt) {
	xdt = x0dt;
	ydt = y0dt;
	phidt = phi0dt;
	double tmp[1][4] = { t,x0dt,y0dt,phi0dt };
	matrix tmp2(1, 4, tmp[0]);
	if (vel.column == 1) {
		vel = tmp2;
	}
	else {
		vel.append(tmp2);
	}
}

void rigidBody::addqddt(double t, double x0ddt, double y0ddt, double phi0ddt) {
	xddt = x0ddt;
	yddt = y0ddt;
	phiddt = phi0ddt;
	double tmp[1][4] = { t,x0ddt,y0ddt,phi0ddt };
	matrix tmp2(1, 4, tmp[0]);
	if (acc.column == 1) {
		acc = tmp2;
	}
	else {
		acc.append(tmp2);
	}
}