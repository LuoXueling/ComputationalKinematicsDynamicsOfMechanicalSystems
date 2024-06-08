#include <string>
#include "constraint.h"
#include "matrixLib.h"
#include <cmath>
using namespace std;

double x_base[2][1] = { 1,0 }, y_base[2][1] = { 0,1 }, R_base[2][2] = { 0,-1,1,0 };
matrix x_b(2, 1, x_base[0]), y_b(2, 1, y_base[0]), R_M(2, 2, R_base[0]);

extern matrix PHIq;

constraint::constraint(string type0, int inp_b1, matrix inp_s1, matrix inp_v1, int inp_b2, matrix inp_s2, matrix inp_v2,double* c_inp) {
	setCons(type0, inp_b1, inp_s1, inp_v1, inp_b2, inp_s2, inp_v2,c_inp);
}

void constraint::setCons(string type0, int inp_b1, matrix inp_s1, matrix inp_v1) {
	type = type0;
	b1 = inp_b1;
	s1 = inp_s1;
	v1 = inp_v1;
}

void constraint::setCons(string type0, int inp_b1, matrix inp_s1, matrix inp_v1, double* c_inp) {
	type = type0;
	b1 = inp_b1;
	s1 = inp_s1;
	v1 = inp_v1;
	coef = c_inp;
}

void constraint::setCons(string type0, int inp_b1, matrix inp_s1, matrix inp_v1, double* c_inp,matrix C_inp) {
	type = type0;
	b1 = inp_b1;
	s1 = inp_s1;
	v1 = inp_v1;
	coef = c_inp;
	C = C_inp;
}

void constraint::setCons(string type0, int inp_b1, matrix inp_s1, matrix inp_v1, int inp_b2, matrix inp_s2, matrix inp_v2) {
	type = type0;
	b1 = inp_b1;
	b2 = inp_b2;
	s1 = inp_s1;
	s2 = inp_s2;
	v1 = inp_v1;
	v2 = inp_v2;
}

void constraint::setCons(string type0, int inp_b1, matrix inp_s1, matrix inp_v1, int inp_b2, matrix inp_s2, matrix inp_v2,double* c_inp) {
	type = type0;
	b1 = inp_b1;
	b2 = inp_b2;
	s1 = inp_s1;
	s2 = inp_s2;
	v1 = inp_v1;
	v2 = inp_v2;
	coef = c_inp;
}

void constraint::setCons(string type0, int inp_b1, matrix inp_s1, matrix inp_v1, int inp_b2, matrix inp_s2, matrix inp_v2, double* c_inp, matrix C_inp) {
	type = type0;
	b1 = inp_b1;
	b2 = inp_b2;
	s1 = inp_s1;
	s2 = inp_s2;
	v1 = inp_v1;
	v2 = inp_v2;
	coef = c_inp;
	C = C_inp;
}

ostream& operator<<(ostream& os, constraint& obj) {
	os << "Type : " << obj.type;
	matrix ts1 = obj.s1.T(), tv1 = obj.v1.T();
	os << " Body 1 : " << obj.b1 << " s1.' = " << ts1 << " v1.' = " << tv1;
	if (findCons(multiBodyCons, obj.type)) {
		matrix ts2 = obj.s2.T(), tv2 = obj.v2.T();
		os << " Body 2 : " << obj.b2 << " s2.' = " << ts2 << " v2.' = " << tv2 ;
	}
	if (findCons(constCons,obj.type)||findCons(funcCons,obj.type)) {
		os << " c(t) = ";
		int times = _msize(obj.coef) / sizeof(*obj.coef);
		for (int i = 0; i < times; i++) {
			os << obj.coef[i];
			if (i == 1) {
				os << "t ";
			}
			else if (i > 1) {
				os << "t^" << i;
			}
			if (i < times - 1) {
				os << "+";
			}
		}
	}
	if (obj.type == AD || obj.type == ADD) {
		matrix tC = obj.C.T();
		os << " C.' = " << tC;
	}
	os << endl;
	return os;
}

bool findCons(const string source[], string target) {
	int len = 0;
	if (source[0] == AX) len = 8;
	else if (source[0] == RPhi) len = 9;
	else if (source[0] == AY) len = 7;
	else if (source[0] == AXD) len = 8;
	for (int i = 0; i < len; i++) {
		if (target == source[i]) {
			return true;
		}
	}
	return false;
}

matrix c(double t, double* coef) {
	int times = _msize(coef) / sizeof(*coef);
	double res = 0;
	for (int j = 0; j < times; j++) {
		res += coef[j] * pow(t, j);
	}
	double tmp[1][1] = { res };
	matrix tmp2(1, 1, tmp[0]);
	return tmp2;
}

matrix cdt(double t, double* coef) {
	int times = _msize(coef) / sizeof(*coef);
	double res = 0;
	for (int j = 1; j < times; j++) {
		res += j * coef[j] * pow(t, j - 1);
	}
	double tmp[1][1] = { res };
	matrix tmp2(1, 1, tmp[0]);
	return tmp2;
}

matrix cddt(double t, double* coef) {
	int times = _msize(coef) / sizeof(*coef);
	double res = 0;
	for (int j = 2; j < times; j++) {
		res += j * (j - 1) * coef[j] * pow(t, j - 2);
	}
	double tmp[1][1] = { res };
	matrix tmp2(1, 1, tmp[0]);
	return tmp2;
}

matrix A(double phi) {
	double a[2][2] = { cos(phi),-sin(phi),sin(phi),cos(phi) };
	matrix tmp(2, 2, a[0]);
	return tmp;
}

matrix ax(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1;
	matrix s1 = cons.s1;
	double* coef = cons.coef;
	double x = bodys[b1]->x;
	double y = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double pdt = bodys[b1]->phidt;
	double r1_base[2][1] = {x,y};
	matrix r1(2, 1, r1_base[0]);
	matrix res;
	switch (param) {
	case 1: {
		res = x_b.T() * (r1 + A(p1) * s1) - c(t, coef);
		break; 
	}
	case 2: {
		res = x_b.T();
		res.combine(x_b.T() * R_M * A(p1) * s1);
		PHIq.insert(res, consNo, b1 * 3);
		break;
	}
	case 3: {
		break;
	}
	case 4: {
		res = x_b.T() * A(p1) * s1 * pow(pdt, 2);
		break;
	}
	}
	return res;
}

matrix ay(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1;
	matrix s1 = cons.s1;
	double* coef = cons.coef;
	double x = bodys[b1]->x;
	double y = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double pdt = bodys[b1]->phidt;
	double r1_base[2][1] = { x,y };
	matrix r1(2, 1, r1_base[0]);
	matrix res;
	switch (param) {
	case 1: {
		res = y_b.T() * (r1 + A(p1) * s1) - c(t, coef);
		break;
	}
	case 2: {
		res = y_b.T();
		res.combine(y_b.T() * R_M * A(p1) * s1);
		PHIq.insert(res, consNo, b1 * 3);
		break;
	}
	case 3: {
		break; 
	}
	case 4: {
		res = y_b.T() * A(p1) * s1 * pow(pdt, 2);
		break;
	}
	}
	return res;
}

matrix aphi(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1;
	matrix s1 = cons.s1;
	double* coef = cons.coef;

	double phi1_base[1][1] = { bodys[b1]->phi };
	matrix phi1(1, 1, phi1_base[0]);
	matrix res;
	switch (param) {
	case 1: {
		res = phi1 - c(t, coef);
		break; 
	}
	case 2: {
		double tmp[1][3] = { 0,0,1 };
		matrix tmp2(1, 3, tmp[0]);
		res = tmp2;
		PHIq.insert(res, consNo, b1 * 3);
		break;
	}
	case 3: {
		break;
	}
	case 4: {
		break;
	}
	}
	return res;
}

matrix ad(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1;
	matrix s1 = cons.s1;
	double* coef = cons.coef;
	matrix C = cons.C;
	double x = bodys[b1]->x;
	double y = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double xdt = bodys[b1]->xdt;
	double ydt = bodys[b1]->ydt;
	double pdt = bodys[b1]->phidt;
	double r1_base[2][1] = { x,y };
	matrix r1(2, 1, r1_base[0]);

	matrix h = (r1 + A(p1) * s1 - C);
	matrix res;
	switch (param) {
	case 1: {
		res = h.T() * h - c(t, coef) * c(t, coef);
		break;
	}
	case 2: {
		matrix tmp = I(2);
		tmp.combine(R_M * A(p1) * s1);
		res = 2 * h.T() * tmp;
		PHIq.insert(res, consNo, b1 * 3);
		break;
	}
	case 3: {
		break;
	}
	case 4: {
		double r1dt_base[2][1] = { xdt,ydt };
		matrix r1dt(2, 1, r1dt_base[0]);
		matrix hdt = r1dt + R_M * A(p1) * s1 * pdt;
		res = 2 * h.T() * A(p1) * s1 * pow(pdt, 2) - 2 * hdt.T()* hdt;
		break;
	}
	}
	return res;
}

matrix rphi(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1, b2 = cons.b2;
	matrix s1 = cons.s1, s2 = cons.s2, v1 = cons.v1, v2 = cons.v2;
	double* coef = cons.coef;
	matrix C = cons.C;
	double x1 = bodys[b1]->x;
	double y1 = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double phi1_base[1][1] = { p1 };
	double x2 = bodys[b2]->x;
	double y2 = bodys[b2]->y;
	double p2 = bodys[b2]->phi;
	double phi2_base[1][1] = { p2 };
	double r1_base[2][1] = { x1,y1 }, r2_base[2][1] = { x2,y2 };
	matrix r1(2, 1, r1_base[0]), r2(2, 1, r2_base[0]);
	matrix phi1(1, 1, phi1_base[0]), phi2(1, 1, phi2_base[0]);
	matrix res;
	switch (param) {
	case 1: {
		res = phi2 - phi1 - c(t, coef);
		break;
	}
	case 2: {
		double PHIq1_base[1][3] = { 0,0,-1 };
		double PHIq2_base[1][3] = { 0,0,1 };
		matrix PHIq1(1, 3, PHIq1_base[0]), PHIq2(1, 3, PHIq2_base[0]);
		PHIq.insert(PHIq1, consNo, b1 * 3);
		PHIq.insert(PHIq2, consNo, b2 * 3);
		break;
	}
	case 3: {
		break;
	}
	case 4: {
		break;
	}
	}
	return res;
}

matrix rd(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1, b2 = cons.b2;
	matrix s1 = cons.s1, s2 = cons.s2, v1 = cons.v1, v2 = cons.v2;
	double* coef = cons.coef;
	matrix C = cons.C;
	double x1 = bodys[b1]->x;
	double y1 = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double x1dt = bodys[b1]->xdt;
	double y1dt = bodys[b1]->ydt;
	double p1dt = bodys[b1]->phidt;
	double phi1_base[1][1] = { p1 };
	double x2 = bodys[b2]->x;
	double y2 = bodys[b2]->y;
	double p2 = bodys[b2]->phi;
	double x2dt = bodys[b2]->xdt;
	double y2dt = bodys[b2]->ydt;
	double p2dt = bodys[b2]->phidt;
	double phi2_base[1][1] = { p2 };
	double r1_base[2][1] = { x1,y1 }, r2_base[2][1] = { x2,y2 };
	matrix r1(2, 1, r1_base[0]), r2(2, 1, r2_base[0]);
	matrix phi1(1, 1, phi1_base[0]), phi2(1, 1, phi2_base[0]);

	matrix h = r2 + A(p2) * s2 - r1 - A(p1) * s1;
	matrix res;
	switch (param) {
	case 1: {
		res = h.T() * h - c(t, coef) * c(t, coef);
		break;
	}
	case 2: {
		matrix PHIq1 = -2 * h.T();
		PHIq1.combine(-2 * h.T() * R_M * A(p1) * s1);
		matrix PHIq2 = 2 * h.T();
		PHIq2.combine( 2 * h.T() * R_M * A(p2) * s2);
		PHIq.insert(PHIq1, consNo, b1 * 3);
		PHIq.insert(PHIq2, consNo, b2 * 3);
		break;
	}
	case 3: {
		break;
	}
	case 4: {
		double r1dt_base[2][1] = { x1dt,y1dt };
		double r2dt_base[2][1] = { x2dt,y2dt };
		matrix r1dt(2, 1, r1dt_base[0]), r2dt(2, 1, r2dt_base[0]);
		matrix hdt = r2dt + R_M * A(p2) * s2 * p2dt - r1dt - R_M * A(p1) * s1 * p1dt;
		res = -2 * hdt.T() * hdt + 2 * h.T() * (A(p2) * s2 * pow(p2dt, 2) - A(p1) * s1 * pow(p1dt, 2));
		break;
	}
	}
	return res;
}

matrix tcons(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1, b2 = cons.b2;
	matrix s1 = cons.s1, s2 = cons.s2, v1 = cons.v1, v2 = cons.v2;
	matrix C = cons.C;
	double x1 = bodys[b1]->x;
	double y1 = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double x1dt = bodys[b1]->xdt;
	double y1dt = bodys[b1]->ydt;
	double p1dt = bodys[b1]->phidt;
	double phi1_base[1][1] = { p1 };
	double x2 = bodys[b2]->x;
	double y2 = bodys[b2]->y;
	double p2 = bodys[b2]->phi;
	double x2dt = bodys[b2]->xdt;
	double y2dt = bodys[b2]->ydt;
	double p2dt = bodys[b2]->phidt;
	double phi2_base[1][1] = { p2 };
	double r1_base[2][1] = { x1,y1 }, r2_base[2][1] = { x2,y2 };
	matrix r1(2, 1, r1_base[0]), r2(2, 1, r2_base[0]);
	matrix phi1(1, 1, phi1_base[0]), phi2(1, 1, phi2_base[0]);

	matrix h = r2 + A(p2) * s2 - r1 - A(p1) * s1;
	matrix tmp1 = (R_M * A(p1) * v1).T() * h;
	matrix tmp2 = (R_M * A(p1) * v1).T() * A(p2) * v2;
	tmp1.append(tmp2);

	matrix res;
	switch (param) {
	case 1: {
		res = tmp1;
		break;
	}
	case 2: {
		matrix PHIq1 = -v1.T()*(R_M*A(p1)).T();
		PHIq1.combine(-v1.T() * A(p1).T() * (r2 - r1) - v1.T() * A(p1).T() * A(p2) * s2);
		matrix tmp1(1, 2);
		tmp1.combine(-v1.T() * A(p1).T() * A(p2) * v2);
		PHIq1.append(tmp1);

		matrix PHIq2 = v1.T()*(R_M*A(p1)).T();
		PHIq2.combine(v1.T() * A(p1).T() * A(p2) * s2);
		matrix tmp2(1, 2);
		tmp2.combine(v1.T() * A(p1).T() * A(p2) * v2);
		PHIq2.append(tmp2);

		PHIq.insert(PHIq1, consNo, b1 * 3);
		PHIq.insert(PHIq2, consNo, b2 * 3);
		break;
	}
	case 3: {
		matrix temp(2, 1);
		res = temp;
		break;
	}
	case 4: {
		double r1dt_base[2][1] = { x1dt,y1dt };
		double r2dt_base[2][1] = { x2dt,y2dt };
		matrix r1dt(2, 1, r1dt_base[0]), r2dt(2, 1, r2dt_base[0]);
		res = v1.T() * (R_M * A(p1)).T() * (r2 - r1) * pow(p1dt, 2) + 2 * p1dt * v1.T() * A(p1).T() * (r2dt - r1dt) ;
		matrix tmp;
		res.append(tmp);
		break;
	}
	}
	return res;
}

matrix r(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1, b2 = cons.b2;
	matrix s1 = cons.s1, s2 = cons.s2, v1 = cons.v1, v2 = cons.v2;
	matrix C = cons.C;
	double x1 = bodys[b1]->x;
	double y1 = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double p1dt = bodys[b1]->phidt;
	double phi1_base[1][1] = { p1 };
	double x2 = bodys[b2]->x;
	double y2 = bodys[b2]->y;
	double p2 = bodys[b2]->phi;
	double p2dt = bodys[b2]->phidt;
	double phi2_base[1][1] = { p2 };
	double r1_base[2][1] = { x1,y1 }, r2_base[2][1] = { x2,y2 };
	matrix r1(2, 1, r1_base[0]), r2(2, 1, r2_base[0]);
	matrix phi1(1, 1, phi1_base[0]), phi2(1, 1, phi2_base[0]);

	matrix res;
	switch (param) {
	case 1: {
		res = r2 + A(p2) * s2 - r1 - A(p1) * s1;
		break;
	}
	case 2: {
		matrix PHIq1 = -I(2);
		PHIq1.combine(-R_M * A(p1) * s1);
		matrix PHIq2 = I(2);
		PHIq2.combine(R_M * A(p2) * s2);
		//cout << R_M << endl << endl;
		//matrix tmp = A(p1);
		//cout << tmp << endl << endl;
		//cout << s1 << endl << endl;
		//tmp = -R_M * A(p1) * s1;
		//cout << tmp << endl << endl;
		//cout << PHIq1 << endl << endl;
		//cout << PHIq2 << endl << endl;
		PHIq.insert(PHIq1, consNo, b1 * 3);
		PHIq.insert(PHIq2, consNo, b2 * 3);
		break;
	}
	case 3: {
		matrix temp(2,1);
		res = temp;
		break;
	}
	case 4: {
		res = A(p2) * s2 * pow(p2dt, 2) - A(p1) * s1 * pow(p1dt, 2);
		break;
	}
	}
	return res;
}

matrix rt(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1, b2 = cons.b2;
	matrix s1 = cons.s1, s2 = cons.s2, v1 = cons.v1, v2 = cons.v2;
	double* coef = cons.coef;
	matrix C = cons.C;
	double x1 = bodys[b1]->x;
	double y1 = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double x1dt = bodys[b1]->xdt;
	double y1dt = bodys[b1]->ydt;
	double p1dt = bodys[b1]->phidt;
	double phi1_base[1][1] = { p1 };
	double x2 = bodys[b2]->x;
	double y2 = bodys[b2]->y;
	double p2 = bodys[b2]->phi;
	double x2dt = bodys[b2]->xdt;
	double y2dt = bodys[b2]->ydt;
	double p2dt = bodys[b2]->phidt;
	double phi2_base[1][1] = { p2 };
	double r1_base[2][1] = { x1,y1 }, r2_base[2][1] = { x2,y2 };
	matrix r1(2, 1, r1_base[0]), r2(2, 1, r2_base[0]);
	matrix phi1(1, 1, phi1_base[0]), phi2(1, 1, phi2_base[0]);

	matrix h = r2 + A(p2) * s2 - r1 - A(p1) * s1;

	matrix res;
	switch (param) {
	case 1: {
		res = (R_M * A(p1) * v1).T() * h / norm(v1) - c(t, coef);
		break;
	}
	case 2: {
		matrix PHIq1 = -v1.T() * (R_M * A(p1)).T() / norm(v1);
		PHIq1.combine((-v1.T() * A(p1).T() * (r2 - r1) - v1.T() * A(p1).T() * A(p2) * s2) / norm(v1));

		matrix PHIq2 = v1.T() * (R_M * A(p1)).T() / norm(v1);
		PHIq2.combine(v1.T() * A(p1).T() * A(p2) * s2 / norm(v1));
		PHIq.insert(PHIq1, consNo, b1 * 3);
		PHIq.insert(PHIq2, consNo, b2 * 3);
		break;
	}
	case 3: {
		break;
	}
	case 4: {
		double r1dt_base[2][1] = { x1dt,y1dt };
		double r2dt_base[2][1] = { x2dt,y2dt };
		matrix r1dt(2, 1, r1dt_base[0]), r2dt(2, 1, r2dt_base[0]);
		res = (1 / norm(v1)) * v1.T() * (2 * A(p1).T() * (r2dt - r1dt) * p1dt + (R_M * A(p1)).T() * (r2 - r1) * pow(p1dt, 2) - R_M * A(p1).T() * A(p2) * s2 * pow(p2dt - p1dt, 2));
		break;
	}
	}
	return res;
}

matrix axd(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1;
	matrix s1 = cons.s1;
	double* coef = cons.coef;
	double x = bodys[b1]->x;
	double y = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double pdt = bodys[b1]->phidt;
	double r1_base[2][1] = { x,y };
	matrix r1(2, 1, r1_base[0]);
	matrix res;
	switch (param) {
	case 1: {
		res = x_b.T() * (r1 + A(p1) * s1) - c(t, coef);
		break;
	}
	case 2: {
		res = x_b.T();
		res.combine(x_b.T() * R_M * A(p1) * s1);
		PHIq.insert(res, consNo, b1 * 3);
		break;
	}
	case 3: {
		res = cdt(t, coef);
		break;
	}
	case 4: {
		res = x_b.T() * A(p1) * s1 * pow(pdt, 2) + cddt(t, coef);
		break;
	}
	}
	return res;
}

matrix ayd(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1;
	matrix s1 = cons.s1;
	double* coef = cons.coef;
	double x = bodys[b1]->x;
	double y = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double pdt = bodys[b1]->phidt;
	double r1_base[2][1] = { x,y };
	matrix r1(2, 1, r1_base[0]);
	matrix res;
	switch (param) {
	case 1: {
		res = y_b.T() * (r1 + A(p1) * s1) - c(t, coef);
		break;
	}
	case 2: {
		res = y_b.T();
		res.combine(y_b.T() * R_M * A(p1) * s1);
		PHIq.insert(res, consNo, b1 * 3);
		break;
	}
	case 3: {
		res = cdt(t, coef);
		break; 
	}
	case 4: {
		res = y_b.T() * A(p1) * s1 * pow(pdt, 2) + cddt(t, coef);
		break;
	}
	}
	return res;
}

matrix aphid(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1;
	matrix s1 = cons.s1;
	double* coef = cons.coef;

	double phi1_base[1][1] = { bodys[b1]->phi };
	matrix phi1(1, 1, phi1_base[0]);
	matrix res;
	switch (param) {
	case 1: {
		res = phi1 - c(t, coef);
		break;
	}
	case 2: {
		double tmp[1][3] = { 0,0,1 };
		matrix tmp2(1, 3, tmp[0]);
		res = tmp2;
		PHIq.insert(res, consNo, b1 * 3);
		break;
	}
	case 3: {
		res = cdt(t, coef);
		break;
	}
	case 4: {
		res = cddt(t, coef);
		break;
	}
	}
	return res;
}

matrix add(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1;
	matrix s1 = cons.s1;
	double* coef = cons.coef;
	matrix C = cons.C;
	double x = bodys[b1]->x;
	double y = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double xdt = bodys[b1]->xdt;
	double ydt = bodys[b1]->ydt;
	double pdt = bodys[b1]->phidt;
	double r1_base[2][1] = { x,y };
	matrix r1(2, 1, r1_base[0]);

	matrix h = (r1 + A(p1) * s1 - C);
	matrix res;
	switch (param) {
	case 1: {
		res = h.T() * h - c(t, coef) * c(t, coef);
		break;
	}
	case 2: {
		matrix tmp = I(2);
		tmp.combine(R_M * A(p1) * s1);
		res = 2 * h.T() * tmp;
		PHIq.insert(res, consNo, b1 * 3);
		break;
	}
	case 3: {
		res = 2 * c(t, coef) * cdt(t, coef);
		break;
	}
	case 4: {
		double r1dt_base[2][1] = { xdt,ydt };
		matrix r1dt(2, 1, r1dt_base[0]);
		matrix hdt = r1dt + R_M * A(p1) * s1 * pdt;
		res = 2 * h.T() * A(p1) * s1 * pow(pdt, 2) - 2 * hdt.T() * hdt + 2 * c(t, coef) * cddt(t, coef) + 2 * cdt(t, coef) * cdt(t, coef);
		break;
	}
	}
	return res;
}

matrix rphid(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1, b2 = cons.b2;
	matrix s1 = cons.s1, s2 = cons.s2, v1 = cons.v1, v2 = cons.v2;
	double* coef = cons.coef;
	matrix C = cons.C;
	double x1 = bodys[b1]->x;
	double y1 = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double phi1_base[1][1] = { p1 };
	double x2 = bodys[b2]->x;
	double y2 = bodys[b2]->y;
	double p2 = bodys[b2]->phi;
	double phi2_base[1][1] = { p2 };
	double r1_base[2][1] = { x1,y1 }, r2_base[2][1] = { x2,y2 };
	matrix r1(2, 1, r1_base[0]), r2(2, 1, r2_base[0]);
	matrix phi1(1, 1, phi1_base[0]), phi2(1, 1, phi2_base[0]);
	matrix res;
	switch (param) {
	case 1: {
		res = phi2 - phi1 - c(t, coef);
		break;
	}
	case 2: {
		double PHIq1_base[1][3] = { 0,0,-1 };
		double PHIq2_base[1][3] = { 0,0,1 };
		matrix PHIq1(1, 3, PHIq1_base[0]), PHIq2(1, 3, PHIq2_base[0]);
		PHIq.insert(PHIq1, consNo, b1 * 3);
		PHIq.insert(PHIq2, consNo, b2 * 3);
		break;
	}
	case 3: {
		res = cdt(t, coef);
		break;
	}
	case 4: {
		res = cddt(t, coef);
		break;
	}
	}
	return res;
}

matrix rrd(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1, b2 = cons.b2;
	matrix s1 = cons.s1, s2 = cons.s2, v1 = cons.v1, v2 = cons.v2;
	double* coef = cons.coef;
	matrix C = cons.C;
	double x1 = bodys[b1]->x;
	double y1 = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double phi1_base[1][1] = { p1 };
	double x2 = bodys[b2]->x;
	double y2 = bodys[b2]->y;
	double p2 = bodys[b2]->phi;
	double phi2_base[1][1] = { p2 };
	double r1_base[2][1] = { x1,y1 }, r2_base[2][1] = { x2,y2 };
	matrix r1(2, 1, r1_base[0]), r2(2, 1, r2_base[0]);
	matrix phi1(1, 1, phi1_base[0]), phi2(1, 1, phi2_base[0]);
	matrix res;
	switch (param) {
	case 1: {
		res = phi2 - phi1 - c(t, coef);
		break;
	}
	case 2: {
		double PHIq1_base[1][3] = { 0,0,-1 };
		double PHIq2_base[1][3] = { 0,0,1 };
		matrix PHIq1(1, 3, PHIq1_base[0]), PHIq2(1, 3, PHIq2_base[0]);
		PHIq.insert(PHIq1, consNo, b1 * 3);
		PHIq.insert(PHIq2, consNo, b2 * 3);
		break;
	}
	case 3: {
		res = cdt(t, coef);
		break;
	}
	case 4: {
		res = cddt(t, coef);
		break;
	}
	}
	return res;
}

matrix rdd(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1, b2 = cons.b2;
	matrix s1 = cons.s1, s2 = cons.s2, v1 = cons.v1, v2 = cons.v2;
	double* coef = cons.coef;
	matrix C = cons.C;
	double x1 = bodys[b1]->x;
	double y1 = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double x1dt = bodys[b1]->xdt;
	double y1dt = bodys[b1]->ydt;
	double p1dt = bodys[b1]->phidt;
	double phi1_base[1][1] = { p1 };
	double x2 = bodys[b2]->x;
	double y2 = bodys[b2]->y;
	double p2 = bodys[b2]->phi;
	double x2dt = bodys[b2]->xdt;
	double y2dt = bodys[b2]->ydt;
	double p2dt = bodys[b2]->phidt;
	double phi2_base[1][1] = { p2 };
	double r1_base[2][1] = { x1,y1 }, r2_base[2][1] = { x2,y2 };
	matrix r1(2, 1, r1_base[0]), r2(2, 1, r2_base[0]);
	matrix phi1(1, 1, phi1_base[0]), phi2(1, 1, phi2_base[0]);

	matrix h = r2 + A(p2) * s2 - r1 - A(p1) * s1;
	matrix res;
	switch (param) {
	case 1: {
		res = h.T() * h - c(t, coef) * c(t, coef);
		break;
	}
	case 2: {
		matrix PHIq1 = -2 * h.T();
		PHIq1.combine(-2 * h.T() * R_M * A(p1) * s1);
		matrix PHIq2 = 2 * h.T();
		PHIq2.combine(2 * h.T() * R_M * A(p2) * s2);
		PHIq.insert(PHIq1, consNo, b1 * 3);
		PHIq.insert(PHIq2, consNo, b2 * 3);
		break;
	}
	case 3: {
		res = 2 * c(t, coef) * cdt(t, coef);
		break;
	}
	case 4: {
		double r1dt_base[2][1] = { x1dt,y1dt };
		double r2dt_base[2][1] = { x2dt,y2dt };
		matrix r1dt(2, 1, r1dt_base[0]), r2dt(2, 1, r2dt_base[0]);
		matrix hdt = r2dt + R_M * A(p2) * s2 * p2dt - r1dt - R_M * A(p1) * s1 * p1dt;
		res = -2 * hdt.T() * hdt + 2 * h.T() * (A(p2) * s2 * pow(p2dt, 2) - A(p1) * s1 * pow(p1dt, 2)) + 2 * c(t, coef) * cddt(t, coef) + 2 * cdt(t, coef) * cdt(t, coef);
		break;
	}
	}
	return res;
}

matrix tdd(rigidBody* bodys[], constraint cons, int consNo, double t, int param) {
	int b1 = cons.b1, b2 = cons.b2;
	matrix s1 = cons.s1, s2 = cons.s2, v1 = cons.v1, v2 = cons.v2;
	double* coef = cons.coef;
	matrix C = cons.C;
	double x1 = bodys[b1]->x;
	double y1 = bodys[b1]->y;
	double p1 = bodys[b1]->phi;
	double x1dt = bodys[b1]->xdt;
	double y1dt = bodys[b1]->ydt;
	double p1dt = bodys[b1]->phidt;
	double phi1_base[1][1] = { p1 };
	double x2 = bodys[b2]->x;
	double y2 = bodys[b2]->y;
	double p2 = bodys[b2]->phi;
	double x2dt = bodys[b2]->xdt;
	double y2dt = bodys[b2]->ydt;
	double p2dt = bodys[b2]->phidt;
	double phi2_base[1][1] = { p2 };
	double r1_base[2][1] = { x1,y1 }, r2_base[2][1] = { x2,y2 };
	matrix r1(2, 1, r1_base[0]), r2(2, 1, r2_base[0]);
	matrix phi1(1, 1, phi1_base[0]), phi2(1, 1, phi2_base[0]);

	matrix h = r2 + A(p2) * s2 - r1 - A(p1) * s1;

	matrix res;
	switch (param) {
	case 1: {
		res = (A(p1) * v1).T() / norm(v1) * h - c(t, coef);
		break;
	}
	case 2: {
		matrix PHIq1 = v1.T() * A(p1).T();
		matrix tmp(2, 2);
		PHIq1.combine(tmp);
		PHIq.insert(PHIq1, consNo, b1 * 3);
		break;
	}
	case 3: {
		res = norm(v1) * cdt(t, coef);
		break;
	}
	case 4: {
		double r1dt_base[2][1] = { x1dt,y1dt };
		double r2dt_base[2][1] = { x2dt,y2dt };
		matrix r1dt(2, 1, r1dt_base[0]), r2dt(2, 1, r2dt_base[0]);
		res = v1.T() * (A(p1).T() * (r2 - r1) * pow(p1dt, 2) - (R_M * A(p1)).T() * (r2dt - r1dt) * 2 * p1dt) + norm(v1) * cddt(t, coef);
		break;
	}
	}
	return res;
}