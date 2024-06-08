#include "matrixLib.h"
#include "rigidBody.h"
#include "constraint.h"
#include "background.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <stdlib.h>
#include <ctime>
#include <windows.h>
//主程序

using namespace std;

// user define
string root, filename;
string inpFilePath;
string oupFileName = "OUT";

// constant variables
const int MAX_BODYS = 100;
const int MAX_CONS = 100;
const int MAX_ITER = 1000;

// global variables
rigidBody* bodys[MAX_BODYS];
constraint* cons[MAX_CONS];
int bodyNum, consNum;
matrix q, PHI, PHIq, v, Gamma; //均定义为全局变量，事实上PHIq为全局变量即可
double t;

int main(int argc, char* argv[]) {
	cout << "CPP program start" << endl;
	char* st1 = argv[1];
	char* st2 = argv[2];
	clock_t tstart, tend;
	root = st1, filename = st2;
	//cin >> root >> filename;
	inpFilePath = root + "/" + filename + ".txt";
	tstart = clock();
	double t0,te,dt,e1,e2;
	readINP(&t0, &te, &dt, &e1, &e2, inpFilePath);
	process(t0, te, dt, e1, e2);
	outputCSV();
	tend = clock();
	cout << "Total time used : " << tend - tstart << " ms " << endl;
	Sleep(1000);
	return 0;
}

void readINP(double* t0,double* te,double* dt,double* e1,double* e2, string filepath) {
	cout << "Reading file " << filepath << "." << endl;
	ifstream file(inpFilePath, ios::in);
	string tmp;

	cout << "Reading bodys." << endl;
	getline(file, tmp);
	bodyNum = getNum<int>(tmp);

	for (int i = 0; i < bodyNum; i++) {
		bodys[i] = new rigidBody;
		double x0, y0, phi0;
		file >> x0 >> y0 >> phi0;
		bodys[i]->setRigidBody(x0, y0, phi0);
		cout << *bodys[i];
	}

	getline(file, tmp);
	getline(file, tmp);
	getline(file, tmp);

	cout << "Reading constraints." << endl;
	consNum = getNum<int>(tmp);

	for (int i = 0; i < consNum; i++) {
		cons[i] = new constraint;
		string type;
		int b1, b2;
		double s1[2][1];
		double s2[2][1];
		double v1[2][1];
		double v2[2][1];
		double* coef;
		double C_inp[2][1];

		file >> type;
		if (findCons(singBodyCons, type)) {
			file >> b1 >> s1[0][0] >> s1[1][0] >> v1[0][0] >> v1[1][0];
			matrix s1(2, 1, s1[0]), v1(2, 1, v1[0]);
			if (findCons(constCons, type) || findCons(funcCons, type)) {
				int tmp;
				file >> tmp;
				coef = new double[tmp];
				for (int j = 0; j < tmp; j++) {
					file >> coef[j];
				}
				if (type == AD || type == ADD) {
					file >> C_inp[0][0] >> C_inp[1][0];
					matrix C(2, 1, C_inp[0]);
					cons[i]->setCons(type, b1, s1, v1, coef, C);
				}
				else cons[i]->setCons(type, b1, s1, v1, coef);
				
			}
			else {
				cons[i]->setCons(type, b1, s1, v1);
			}
		}
		else if (findCons(multiBodyCons, type)) {
			file >> b1 >> s1[0][0] >> s1[1][0] >> v1[0][0] >> v1[1][0];
			file >> b2 >> s2[0][0] >> s2[1][0] >> v2[0][0] >> v2[1][0];
			matrix s1(2, 1, s1[0]), s2(2, 1, s2[0]), v1(2, 1, v1[0]), v2(2, 1, v2[0]);
			if (findCons(constCons, type) || findCons(funcCons, type)) {
				int tmp;
				file >> tmp;
				coef = new double[tmp];
				for (int j = 0; j < tmp; j++) {
					file >> coef[j];
				}
				if (type == AD || type == ADD) {
					file >> C_inp[0][0] >> C_inp[1][0];
					matrix C(2, 1, C_inp[0]);
					cons[i]->setCons(type, b1, s1, v1, b2, s2, v2, coef, C);
				}
				else cons[i]->setCons(type, b1, s1, v1, b2, s2, v2, coef);
			}
			else {
				cons[i]->setCons(type, b1, s1, v1, b2, s2, v2);
			}
		}

		cout << *cons[i];
	}

	getline(file, tmp);
	getline(file, tmp);
	getline(file, tmp);
	cout << "Reading parametres." << endl;
	*t0 = getNum<double>(tmp);
	getline(file, tmp);
	*te = getNum<double>(tmp);
	getline(file, tmp);
	*dt = getNum<double>(tmp);
	getline(file, tmp);
	*e1 = getNum<double>(tmp);
	getline(file, tmp);
	*e2 = getNum<double>(tmp);
	file.close();
	cout << "Reading completed." << endl;
}

int process(double t0, double te, double dt, double e1, double e2) {
	t = t0;
	for (t; t <= te; t += dt) {
		PHI = getMat(t, 1);
		update_slider(t, te);
		//cout << "Solving " << t << " / " << te << "." << endl;

		int iter_cnt = 0;
		double detPHIq = 1e+10;
		double tmpdet = 1e+10;
		while (norm(PHI) > e1) {
			//cout << "norm(PHI) = " << norm(PHI) << endl;
			
			getMat(t, 2);
			//cout << PHI << endl << endl;
			//cout << PHIq << endl << endl;
			if (abs(det(PHIq)) > e2) {
				//cout << PHIq << endl << endl;
				//cout << PHI << endl << endl;
				matrix dq = solve(PHIq, -PHI);
				//cout << dq << endl << endl;
				matrix q = getq() + dq;
				for (int i = 0; i < bodyNum; i++) {
					bodys[i]->setRigidBody(q.mat[3 * i][0], q.mat[3 * i + 1][0], q.mat[3 * i + 2][0]);
				}
			}
			else {
				cout << "Improper initial value." << endl;
				return -1;
			}
			if (iter_cnt > MAX_ITER) {
				cout << "Improper initial value." << endl;
				return -1;
			}
			PHI = getMat(t, 1);
			//cout << PHI << endl << endl;
			//cout << "End : " << iter_cnt << " norm(PHI) = " << norm(PHI) << endl;
			iter_cnt++;
		}

		getMat(t, 2);
		
		//cout << PHI << endl << endl;
		//cout << v << endl << endl;
		//cout << PHIq << endl << endl;
		//cout << abs(det(PHIq)) << endl << endl;
		if (abs(det(PHIq)) > e2) {
			v = getMat(t, 3);
			matrix qdt = solve(PHIq, v);
			for (int i = 0; i < bodyNum; i++) {
				bodys[i]->addqdt(t, qdt.mat[3 * i][0], qdt.mat[3 * i + 1][0], qdt.mat[3 * i + 2][0]);
			}
			Gamma = getMat(t, 4);
			matrix qddt = solve(PHIq, Gamma);
			for (int i = 0; i < bodyNum; i++) {
				bodys[i]->addqddt(t, qddt.mat[3 * i][0], qddt.mat[3 * i + 1][0], qddt.mat[3 * i + 2][0]);
			}
			matrix q0 = getq();
			matrix q = q0 + qdt * dt + qddt * dt * dt / 2;
			for (int i = 0; i < bodyNum; i++) {
				if (t == t0) {
					bodys[i]->addq(t, q0.mat[3 * i][0], q0.mat[3 * i + 1][0], q0.mat[3 * i + 2][0]);
				}
				bodys[i]->addq(t+dt, q.mat[3 * i][0], q.mat[3 * i + 1][0], q.mat[3 * i + 2][0]);
			}
		}
		else {
			cout << "Singularity" << endl;
			return -1;
		}
	}
	cout << "[>>>>>>>>>>>>>>>>>>>>]" << endl;
	return 0;
}

void outputCSV() {
	for (int i = 0; i < bodyNum; i++) {
		ofstream file;
		file.open(root + "/" + filename + "_" + oupFileName + "_body" + to_string(i) + "_pos.csv", ios::trunc);
		file << bodys[i]->pos;
		file.close();
		file.open(root + "/" + filename + "_" + oupFileName + "_body" + to_string(i) + "_vel.csv", ios::trunc);
		file << bodys[i]->vel;
		file.close();
		file.open(root + "/" + filename + "_" + oupFileName + "_body" + to_string(i) + "_acc.csv", ios::trunc);
		file << bodys[i]->acc;
		file.close();
	}
}

template<typename T> T getNum(string line) {
	T nums = 0;
	int cnt = 0;
	bool flagPoint = false;
	for (int i = 0; i < line.length(); i++) {
		if (line[i] == '=') {
			nums = 0;
		}
		if ((line[i] >= '0') && (line[i] <= '9')) {
			nums = nums * 10 + line[i] - '0';
			if (flagPoint == true) {
				cnt++;
			}
		}
		else if (line[i] == '.') {
			flagPoint = true;
		}
	}
	if (flagPoint == true) {
		return nums / pow(10, cnt);
	}
	else {
		return nums;
	}
}

matrix getMat(double t, int param) {
	matrix res;
	if (param == 2) {
		matrix tmp(3 * bodyNum, 3 * bodyNum);
		PHIq = tmp;
	}
	int consNo=0;
	for (int i = 0; i < consNum; i++) {
		string type = cons[i]->type;
		matrix tmp;
		if (type == AX) tmp = ax(bodys, *cons[i], consNo, t, param);
		else if (type == AY)	tmp = ay(bodys, *cons[i], consNo, t, param);
		else if (type == APhi)	tmp = aphi(bodys, *cons[i], consNo, t, param);
		else if (type == AD)	tmp = ad(bodys, *cons[i], consNo, t, param);
		else if (type == RPhi)	tmp = rphi(bodys, *cons[i], consNo, t, param);
		else if (type == RD)	tmp = rd(bodys, *cons[i], consNo, t, param);
		else if (type == T)		tmp = tcons(bodys, *cons[i], consNo, t, param);
		else if (type == R)		tmp = r(bodys, *cons[i], consNo, t, param);
		else if (type == RT) 	tmp = rt(bodys, *cons[i], consNo, t, param);

		else if (type == AXD)	tmp = axd(bodys, *cons[i], consNo, t, param);
		else if (type == AYD)	tmp = ayd(bodys, *cons[i], consNo, t, param);
		else if (type == APhiD) tmp = aphid(bodys, *cons[i], consNo, t, param);
		else if (type == ADD)	tmp = add(bodys, *cons[i], consNo, t, param);
		else if (type == RPhiD)	tmp = rphid(bodys, *cons[i], consNo, t, param);
		else if (type == RDD) 	tmp = rdd(bodys, *cons[i], consNo, t, param);
		else if (type == TDD) 	tmp = tdd(bodys, *cons[i], consNo, t, param);
		else if (type == RRD) 	tmp = rrd(bodys, *cons[i], consNo, t, param);
		else { cout << "Type error for constraint " << i << "." << endl; }

		if (type == R || type == T) {
			consNo += 2;
		}
		else consNo += 1;

		//cout << PHIq << endl << endl;;
		if (i == 0 && param != 2) {
			res = tmp;
		}
		else if (param != 2) {
			res.append(tmp);
			//cout << res << endl << endl;
		}
	}
	return res;
}

matrix getq() {
	matrix res;
	for (int i = 0; i < bodyNum; i++) {
		double x = bodys[i]->x;
		double y = bodys[i]->y;
		double phi = bodys[i]->phi;
		double q0[3][1] = { x,y,phi };
		matrix tmp(3, 1, q0[0]);
		if (i == 0) {
			res = tmp;
		}
		else res.append(tmp);
	}
	return res;
}

void update_slider(double a, double b) {
	cout << "[";
	for (int i = 0; i < ceil(a * 20 / b); i++)
		cout << ">";
	for (int i = 0; i < 20-ceil(a * 20 / b); i++)
		cout << "-";
	cout << "]";
	cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
}