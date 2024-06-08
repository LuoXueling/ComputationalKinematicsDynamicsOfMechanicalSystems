#include "matrixLib.h"
#include "rigidBody.h"
#include "constraint.h"
#include "background.h"
#include "dynamicsLib.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <ctime>
#include <windows.h>
#include <Commdlg.h>
#include <stdio.h>
//主程序

using namespace std;

// user define
string root, filename;
string inpFilePath;
string oupFileName = "OUT";

// constant variables
const int MAX_BODYS = 100;
const int MAX_CONS = 100;
const int MAX_FCES = 100;
const int MAX_ACTS = 100;
const int MAX_ITER = 1000;

// global variables
rigidBody* bodys[MAX_BODYS];
constraint* cons[MAX_CONS];
constraint* addCons[MAX_CONS];
force* forces[MAX_FCES];
actuator* actuators[MAX_ACTS];

int bodyNum, consNum, addConsNum, forceNum, actuatorNum;
string mode;
matrix M, Qa, addPHI;
matrix q, PHI, PHIq, v, Gamma; //均定义为全局变量，事实上PHIq为全局变量即可
matrix indepCoords;
matrix lambda;
double g = 9.8;
double t;
double t0, te, dt, e1, e2;

int main(int argc, char* argv[]) {
	cout << "CPP program start" << endl;
	if (argc == 3) {
		char* st1 = argv[1];
		char* st2 = argv[2];
		root = st1, filename = st2;
		inpFilePath = root + ".txt";
	}
	else {
		getFilename();
	}
	CreateDirectory(root.c_str(), NULL);
	clock_t tstart, tend;

	tstart = clock();

	readINP();
	if (mode == "dynamics") {
		process();
		outputCSV();
	}
	else if (mode == "equilibrium") {
		equilibrium();
	}
	tend = clock();
	cout << "Total time used : " << tend - tstart << " ms " << endl;
	Sleep(1000);
	return 0;
}

void getFilename() {
	OPENFILENAME ofn;
	char path[100];
	char name[100];
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.lpstrFile = (LPSTR)path;
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = sizeof(path);
	ofn.lpstrFilter = (LPCSTR)"Text\0*.txt\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = name;
	ofn.nMaxFileTitle = sizeof(name);
	ofn.lpstrInitialDir = NULL;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
	if (GetOpenFileName(&ofn))
	{
		inpFilePath = path;
		cout << inpFilePath << endl;
		char* buf;
		filename = strtok_s(name, ".", &buf);
		root = strtok_s(path, ".", &buf);
	}
	else
	{
		cout << "cancelled" << endl;
	}
}

void readINP() {
	cout << "Reading file " << inpFilePath << "." << endl;
	ifstream file(inpFilePath, ios::in);
	string tmp;

	file >> mode;
	cout << "Mode : " << mode << endl;

	cout << "Reading bodys." << endl;
	getline(file, tmp);
	getline(file, tmp);
	getline(file, tmp);
	bodyNum = getNum<int>(tmp);

	for (int i = 0; i < bodyNum; i++) {
		bodys[i] = new rigidBody;
		double x0, y0, phi0, m, j;
		file >> x0 >> y0 >> phi0;
		file >> m >> j;
		bodys[i]->setq(x0, y0, phi0);
		bodys[i]->setInertiaParams(m, j);
		cout << *bodys[i];
	}
	
	getline(file, tmp);
	getline(file, tmp);
	getline(file, tmp);

	cout << "Reading constraints." << endl;
	consNum = getNum<int>(tmp);
	int consNo = 0;
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
		if (type == R || type == T) {
			consNo += 2;
		}
		else consNo += 1;

		cout << *cons[i];
	}
	addConsNum = 3 * bodyNum - consNo;

	getline(file, tmp);
	getline(file, tmp);
	getline(file, tmp);
	cout << "Reading Independent coordinates." << endl;
	for (int i = 0; i < addConsNum; i++) {
		double tmp[1][4] = { 0,0,0 };
		file >> tmp[0][0] >> tmp[0][1] >> tmp[0][2] >> tmp[0][3];
		matrix tmpmat(1,4,tmp[0]);
		if (i == 0) {
			indepCoords = tmpmat;
		}
		else {
			indepCoords.append(tmpmat);
		}
		cout << "Indep. coord: body " << indepCoords.mat[i][0] << " coord No." << indepCoords.mat[i][1];
		cout << " initial position= " << indepCoords.mat[i][2] << " initial velocity = " << indepCoords.mat[i][3] << endl;
	}

	getline(file, tmp);
	getline(file, tmp);
	getline(file, tmp);
	cout << "Reading forces." << endl;
	forceNum = getNum<int>(tmp);
	for (int i = 0; i < forceNum; i++) {
		forces[i] = new force;
		file >> forces[i]->type;
		if (forces[i]->type == "af" || forces[i]->type == "rf") {
			file >> forces[i]->b;
			matrix tmp(2, 1);
			file >> tmp.mat[0][0] >> tmp.mat[1][0];
			forces[i]->s = tmp;
			file >> tmp.mat[0][0] >> tmp.mat[1][0];
			forces[i]->vector = tmp;
		}
		else if (forces[i]->type == "m") {
			file >> forces[i]->b >> forces[i]->value;
		}
		else if (forces[i]->type == "g") {
			matrix tmp(2, 1);
			file >> tmp.mat[0][0] >> tmp.mat[1][0];
			forces[i]->vector = tmp;
		}
		cout << *forces[i];
	}
	getline(file, tmp);
	getline(file, tmp);
	getline(file, tmp);
	cout << "Reading actuators." << endl;
	actuatorNum = getNum<int>(tmp);
	for (int i = 0; i < actuatorNum; i++) {
		actuators[i] = new actuator;
		file >> actuators[i]->type;
		file >> actuators[i]->F;
		file >> actuators[i]->k;
		file >> actuators[i]->c;
		file >> actuators[i]->l0;
		if (actuators[i]->type == "tra") {
			file >> actuators[i]->b1;
			matrix tmp(2, 1);
			file >> tmp.mat[0][0] >> tmp.mat[1][0];
			actuators[i]->s1 = tmp;
			file >> actuators[i]->b2;
			file >> tmp.mat[0][0] >> tmp.mat[1][0];
			actuators[i]->s2 = tmp;
		}
		else if (actuators[i]->type == "tor") {
			file >> actuators[i]->b1;
			file >> actuators[i]->b2;
		}
		cout << *actuators[i];
	}

	getline(file, tmp);
	getline(file, tmp);
	if (actuatorNum != 0) {
		getline(file, tmp);
	}
	cout << "Reading parametres." << endl;
	t0 = getNum<double>(tmp);
	getline(file, tmp);
	te = getNum<double>(tmp);
	getline(file, tmp);
	dt = getNum<double>(tmp);
	getline(file, tmp);
	e1 = getNum<double>(tmp);
	getline(file, tmp);
	e2 = getNum<double>(tmp);
	cout << t0 << " " << te << " " << dt << " " << e1 << " " << e2 << endl;
	file.close();
	cout << "Reading completed." << endl;
}

int process() {
	formM();
	matrix y, f, v, dy;
	multiMatrix res;
	// 生成初始时的坐标
	y = solveInit(); 
	res = solveMixedDiffAlgeEq(0, y);
	f = res.mat1;
	lambda = res.mat2.T();

	for (int i = 0; i < bodyNum; i++) {
		bodys[i]->addq(0, y.mat[3 * i][0], y.mat[3 * i + 1][0], y.mat[3 * i + 2][0]);
		bodys[i]->addqdt(0, y.mat[3 * bodyNum + 3 * i][0], y.mat[3 * bodyNum + 3 * i + 1][0], y.mat[3 * bodyNum + 3 * i + 2][0]);
		bodys[i]->addqddt(0, f.mat[3 * bodyNum + 3 * i][0], f.mat[3 * bodyNum + 3 * i + 1][0], f.mat[3 * bodyNum + 3 * i + 2][0]);
	}

	// 龙格库塔法时间推进
	for (t = t0; t <= te;) {
		update_slider(t, te);//更新进度条
		y = gety();
		matrix f1, f2, f3, f4;
		f1 = solveMixedDiffAlgeEq(t, y).mat1;
		f2 = solveMixedDiffAlgeEq(t + 0.5 * dt, y + 0.5 * dt * f1).mat1;
		f3 = solveMixedDiffAlgeEq(t + 0.5 * dt, y + 0.5 * dt * f2).mat1;
		f4 = solveMixedDiffAlgeEq(t + dt, y + dt * f3).mat1;

		dy = (f1 + 2 * f2 + 2 * f3 + f4) * dt / 6;
		y = y + dy;
		for (int i = 0; i < bodyNum; i++) {
			bodys[i]->setq(y.mat[3 * i][0], y.mat[3 * i + 1][0], y.mat[3 * i + 2][0]);
			bodys[i]->setqdt(y.mat[3 * bodyNum + 3 * i][0], y.mat[3 * bodyNum + 3 * i + 1][0], y.mat[3 * bodyNum + 3 * i + 2][0]);
		}

		t += dt;

		y = consStab(t, y);

		res = solveMixedDiffAlgeEq(t, y);
		f = res.mat1;

		// 记录该时间点最终的坐标值与乘子
		lambda.append(res.mat2.T());
		for (int i = 0; i < bodyNum; i++) {
			bodys[i]->addq(t, y.mat[3 * i][0], y.mat[3 * i + 1][0], y.mat[3 * i + 2][0]);
			bodys[i]->addqdt(t, y.mat[3 * bodyNum + 3 * i][0], y.mat[3 * bodyNum + 3 * i + 1][0], y.mat[3 * bodyNum + 3 * i + 2][0]);
			bodys[i]->addqddt(t, f.mat[3 * bodyNum + 3 * i][0], f.mat[3 * bodyNum + 3 * i + 1][0], f.mat[3 * bodyNum + 3 * i + 2][0]);
		}
	}
	cout << "[>>>>>>>>>>>>>>>>>>>>]" << endl;
	return 0;
}

matrix getR() {
	matrix y, qu, qv, PHIu, PHIv, Qu, Qv;
	// 下面求PSI
	// 将独立坐标与非独立坐标的q、PHIq、Qa分离
	matrix q = getq();
	formQa();
	getMat(t0, 2);
	PHI = getMat(t0, 1);
	qu = q;
	PHIu = PHIq;
	Qu = Qa;
	int cnt = 0;
	for (int i = 0; i < addConsNum; i++) {
		int r;
		r = 3 * (int)indepCoords.mat[i][0] + (int)indepCoords.mat[i][1] - 1;
		if (i == 0) {
			qv = q.slice(r, r + 1, -1, -1);
			PHIv = PHIq.slice(-1, -1, r, r + 1);
			Qv = Qa.slice(r, r + 1, -1, -1);
		}
		else {
			qv.append(q.slice(r, r + 1, -1, -1));
			PHIv.combine(PHIq.slice(-1, -1, r, r + 1));
			Qv.append(Qa.slice(r, r + 1, -1, -1));
		}
		qu.deleterow(r - cnt);
		PHIu.deletecol(r - cnt);
		Qu.deleterow(r - cnt);
		cnt++;
	}

	matrix H;
	H = PHIu.inv() * PHIv;
	matrix R = H.T() * Qu - Qv;
	return R;
}

matrix getPSIq() {
	matrix R, refR, dR;
	matrix PSI, PSIq, dPSI;
	double dx = 0.001;

	for (int i = 0; i < bodyNum; i++) {
		for (int j = 0; j < 3; j++) {

			// 该坐标增加一个微元
			switch (j) {
			case 0:
				bodys[i]->x += dx;
				break;
			case 1:
				bodys[i]->y += dx;
				break;
			case 2:
				bodys[i]->phi += dx;
				break;
			}

			R = getR();

			switch (j) {
			case 0:
				bodys[i]->x -= dx;
				break;
			case 1:
				bodys[i]->y -= dx;
				break;
			case 2:
				bodys[i]->phi -= dx;
				break;
			}
			
			refR = getR();
			dR = (R - refR) / dx;

			if (i == 0 && j == 0) {
				PSIq = dR;
			}
			else {
				PSIq.combine(dR);
			}
		}
	}
	getMat(t0, 2);
	PSIq.append(PHIq);
	return PSIq;
}

matrix getPSI() {
	matrix R = getR();
	PHI = getMat(t0, 1);
	R = R.T();
	R.combine(PHI.T());
	matrix PSI = R.T();
	return PSI;
}

int equilibrium() {
	cout << "Solving equilibrium problem..." << endl;
	formM();
	matrix dq, q;
	//solveInit();
	matrix PSI = getPSI();
	q = getq();
	matrix PSIq = getPSIq();

	double del = norm(PSI);

	while (del > e1){
		if (abs(det(PSIq)) < e2) {
			cout << "Improper initial value" << endl;
			break;
		}
		dq = solve(PSIq, -PSI);
		q = getq() + 0.1 * dq;
		for (int i = 0; i < bodyNum; i++) {
			bodys[i]->setq(q.mat[3 * i][0], q.mat[3 * i + 1][0], q.mat[3 * i + 2][0]);
		}
		PSI = getPSI();
		PSIq = getPSIq();
		del = norm(PSI);
		// cout << del << " " << bodys[1]->phi << endl;
	}
	q = getq();

	cout << endl << "One possible equilibrium result. Try other initial params.";
	cout << endl << q << endl << endl;

	for (int i = 0; i < bodyNum; i++) {
		bodys[i]->addq(0, q.mat[3 * i][0], q.mat[3 * i + 1][0], q.mat[3 * i + 2][0]);
	}
	ofstream file;
	for (int i = 0; i < bodyNum; i++) {
		file.open(root + "\\" + filename + "_" + oupFileName + "_body" + to_string(i) + "_pos.csv", ios::trunc);
		file << bodys[i]->pos;
		file.close();
	}
	return 0;
}

void formM() {
	double** matM;
	matM = newMat(3 * bodyNum, 3 * bodyNum);
	matrix tmpM(matM);
	M = tmpM;
	for (int i = 0; i < bodyNum; i++) {
		M.mat[3 * i][3 * i] = bodys[i]->m;
		M.mat[3 * i + 1][3 * i + 1] = bodys[i]->m;
		M.mat[3 * i + 2][3 * i + 2] = bodys[i]->J;
	}
}

void formQa() {
	matrix tmp(3 * bodyNum, 1);
	Qa = tmp;
	matrix vec;
	double R_base[2][2] = { 0,-1,1,0 };
	matrix R_M(2, 2, R_base[0]);

	// 针对力载荷，生成广义力
	for (int i = 0; i < forceNum; i++) {
		vec = forces[i]->vector;
		if (forces[i]->type == "af") {
			int b = forces[i]->b;
			matrix s = forces[i]->s;
			Qa.mat[3 * b][0] += vec.mat[0][0];
			Qa.mat[3 * b + 1][0] += vec.mat[1][0];
			Qa.mat[3 * b + 2][0] += (s.T() * (R_M * A(bodys[b]->phi)).T() * vec).mat[0][0];
		}
		if (forces[i]->type == "rf") {
			int b = forces[i]->b;
			matrix s = forces[i]->s;
			Qa.mat[3 * b][0] += vec.mat[0][0];
			Qa.mat[3 * b + 1][0] += vec.mat[1][0];
			Qa.mat[3 * b + 2][0] += (s.T() * (R_M * A(bodys[b]->phi)).T() * A(bodys[b]->phi) * vec).mat[0][0];
		}
		else if (forces[i]->type == "m") {
			int b = forces[i]->b;
			double value = forces[i]->value;
			Qa.mat[3 * b + 2][0] += value;
		}
		else if (forces[i]->type == "g") {
			for (int j = 0; j < bodyNum; j++) {
				Qa.mat[3 * j][0] += vec.mat[0][0] * g * bodys[i]->m;
				Qa.mat[3 * j + 1][0] += vec.mat[1][0] * g * bodys[i]->m;
			}
		}
	}
	// 对于作业中特定的力，生成特定的矩阵
	if (filename == "INP4") {
		double x3 = bodys[2]->x;
		double x3dt = bodys[2]->xdt;
		if (x3dt < 0) {
			Qa.mat[6][0] += 0;
		}
		else if (x3dt > 0 && x3 < 1.5) {
			Qa.mat[6][0] += 0;
		}
		else if (x3dt > 0 && x3 >= 1.5 && x3 <= 5) {
			Qa.mat[6][0] += -282857 / (6 - x3) + 62857;
		}
		else if (x3dt > 0 && x3 > 5 && x3 <= 5.5) {
			Qa.mat[6][0] += -110000 * (1 - sin(2 * 3.14159265 * (x3 - 5.25)));
		}
	}

	// 针对力元，生成广义力
	for (int i = 0; i < actuatorNum; i++) {
		if (actuators[i]->type == "tra") {
			int b1 = actuators[i]->b1, b2 = actuators[i]->b2;
			double F = actuators[i]->F, k = actuators[i]->k, c = actuators[i]->c, l0 = actuators[i]->l0;
			matrix s1 = actuators[i]->s1, s2 = actuators[i]->s2;
			double x1 = bodys[b1]->x;
			double y1 = bodys[b1]->y;
			double p1 = bodys[b1]->phi;
			double x1dt = bodys[b1]->xdt;
			double y1dt = bodys[b1]->ydt;
			double p1dt = bodys[b1]->phidt;
			double x2 = bodys[b2]->x;
			double y2 = bodys[b2]->y;
			double p2 = bodys[b2]->phi;
			double x2dt = bodys[b2]->xdt;
			double y2dt = bodys[b2]->ydt;
			double p2dt = bodys[b2]->phidt;
			double r1_base[2][1] = { x1,y1 }, r2_base[2][1] = { x2,y2 };
			double r1dt_base[2][1] = { x1dt,y1dt }, r2dt_base[2][1] = { x2dt,y2dt };
			matrix r1(2, 1, r1_base[0]), r2(2, 1, r2_base[0]);
			matrix r1dt(2, 1, r1dt_base[0]), r2dt(2, 1, r2dt_base[0]);

			matrix d = r2 + A(p2) * s2 - r1 - A(p1) * s1;
			double l = norm(d);
			matrix ddt = d.T() / l * (r2dt + R_M * A(p2) * s2 * p2dt - r1dt - R_M * A(p1) * s1 * p1dt);
			double dl = ddt.mat[0][0];

			double f = k * (l - l0) + c * dl + F;
			//cout << "l = " <<l<< endl;
			//q = getq();
			//cout << q << endl<<endl;
			Qa.mat[3 * b1][0] += f / l * d.mat[0][0];
			Qa.mat[3 * b1 + 1][0] += f / l * d.mat[1][0];
			Qa.mat[3 * b1 + 2][0] += (f / l * (s1.T() * (R_M * A(p1)).T() * d)).mat[0][0];

			Qa.mat[3 * b2][0] += -f / l * d.mat[0][0];
			Qa.mat[3 * b2 + 1][0] += -f / l * d.mat[1][0];
			Qa.mat[3 * b2 + 2][0] += (-f / l * (s2.T() * (R_M * A(p2)).T() * d)).mat[0][0];
		}
		if (actuators[i]->type == "tor") {
			int b1 = actuators[i]->b1, b2 = actuators[i]->b2;
			double F = actuators[i]->F, k = actuators[i]->k, c = actuators[i]->c, l0 = actuators[i]->l0;
			double p1 = bodys[b1]->phi;
			double p2 = bodys[b2]->phi;
			double p1dt = bodys[b1]->phidt;
			double p2dt = bodys[b2]->phidt;
			double d = p2 - p1;
			double ddt = p2dt-p1dt;

			double n = k * (d - l0) + c * ddt + F;

			Qa.mat[3 * b1 + 2][0] += n;

			Qa.mat[3 * b2 + 2][0] += -n;
		}
	}
}

void outputCSV() {
	ofstream file;
	for (int i = 0; i < bodyNum; i++) {
		file.open(root+"\\"+filename + "_" + oupFileName + "_body" + to_string(i) + "_pos.csv", ios::trunc);
		file << bodys[i]->pos;
		file.close();
		file.open(root + "\\" + filename + "_" + oupFileName + "_body" + to_string(i) + "_vel.csv", ios::trunc);
		file << bodys[i]->vel;
		file.close();
		file.open(root + "\\" + filename + "_" + oupFileName + "_body" + to_string(i) + "_acc.csv", ios::trunc);
		file << bodys[i]->acc;
		file.close();
	}
	file.open(root + "\\" + filename + "_" + oupFileName + "_lambda.csv", ios::trunc);
	file << lambda;
	file.close();
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

matrix getMat(double t, int param, bool isAdded) {
	matrix res;
	if (param == 2 && !isAdded) {
		matrix tmp(3 * bodyNum - addConsNum, 3 * bodyNum);
		PHIq = tmp;
	}
	else if (param == 2 && isAdded) {
		matrix tmp(3 * bodyNum, 3 * bodyNum);
		PHIq = tmp;
	}
	int consNo=0;
	int limit = consNum;
	if (isAdded) {
		limit = consNum + addConsNum;
	}
	for (int i = 0; i < limit; i++) {
		constraint* tmpcons;
		if (i >= consNum) {
			tmpcons = addCons[i-consNum];
		}
		else {
			tmpcons = cons[i];
		}
		string type = tmpcons->type;
		matrix tmp;
		if (type == AX) tmp = ax(bodys, *tmpcons, consNo, t, param);
		else if (type == AY)	tmp = ay(bodys, *tmpcons, consNo, t, param);
		else if (type == APhi)	tmp = aphi(bodys, *tmpcons, consNo, t, param);
		else if (type == AD)	tmp = ad(bodys, *tmpcons, consNo, t, param);
		else if (type == RPhi)	tmp = rphi(bodys, *tmpcons, consNo, t, param);
		else if (type == RD)	tmp = rd(bodys, *tmpcons, consNo, t, param);
		else if (type == T)		tmp = tcons(bodys, *tmpcons, consNo, t, param);
		else if (type == R)		tmp = r(bodys, *tmpcons, consNo, t, param);
		else if (type == RT) 	tmp = rt(bodys, *tmpcons, consNo, t, param);

		else if (type == AXD)	tmp = axd(bodys, *tmpcons, consNo, t, param);
		else if (type == AYD)	tmp = ayd(bodys, *tmpcons, consNo, t, param);
		else if (type == APhiD) tmp = aphid(bodys, *tmpcons, consNo, t, param);
		else if (type == ADD)	tmp = add(bodys, *tmpcons, consNo, t, param);
		else if (type == RPhiD)	tmp = rphid(bodys, *tmpcons, consNo, t, param);
		else if (type == RDD) 	tmp = rdd(bodys, *tmpcons, consNo, t, param);
		else if (type == TDD) 	tmp = tdd(bodys, *tmpcons, consNo, t, param);
		else if (type == RRD) 	tmp = rrd(bodys, *tmpcons, consNo, t, param);
		else { cout << "Type error for constraint " << i << "." << endl; }

		if (type == R || type == T) {
			consNo += 2;
		}
		else consNo += 1;

		if (i == 0 && param != 2) {
			res = tmp;
		}
		else if (param != 2) {
			res.append(tmp);
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

matrix getqdt() {
	matrix res;
	for (int i = 0; i < bodyNum; i++) {
		double xdt = bodys[i]->xdt;
		double ydt = bodys[i]->ydt;
		double phidt = bodys[i]->phidt;
		double q0dt[3][1] = { xdt,ydt,phidt };
		matrix tmp(3, 1, q0dt[0]);
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

matrix solveInit() {
	matrix res;
	//cout << "Add Constraints : " << endl;
	for (int i = 0; i < addConsNum; i++) {
		addCons[i] = new constraint;
		double s1[2][1] = { 0,0 };
		double v1[2][1] = { 0,0 };
		double* coef = new double[2];
		coef[0] = indepCoords.mat[i][2];
		coef[1] = indepCoords.mat[i][3];
		matrix mats1(2, 1, s1[0]), matv1(2, 1, v1[0]);
		// 根据独立坐标确定约束类型
		if (indepCoords.mat[i][1] == 1) {
			addCons[i]->setCons("axd", indepCoords.mat[i][0], mats1, matv1, coef);
		}
		else if (indepCoords.mat[i][1] == 2) {
			addCons[i]->setCons("ayd", indepCoords.mat[i][0], mats1, matv1, coef);
		}
		else if (indepCoords.mat[i][1] == 3) {
			addCons[i]->setCons("aphid", indepCoords.mat[i][0], mats1, matv1, coef);
		}
		//cout << *addCons[i];
	}

	// 以下用运动学方法求解初值
	PHI = getMat(0, 1, true);
	int iter_cnt = 0;
	double detPHIq = 1e+10;
	double tmpdet = 1e+10;
	while (norm(PHI) > e1) {
		getMat(0, 2, true);
		if (abs(det(PHIq)) > e2) {
			matrix dq = solve(PHIq, -PHI);
			matrix q = getq() + dq;
			for (int i = 0; i < bodyNum; i++) {
				bodys[i]->setq(q.mat[3 * i][0], q.mat[3 * i + 1][0], q.mat[3 * i + 2][0]);
			}
		}
		else {
			cout << "Improper initial value." << endl;
			break;
		}
		if (iter_cnt > MAX_ITER) {
			cout << "Improper initial value." << endl;
			break;
		}
		PHI = getMat(0, 1, true);
		iter_cnt++;
	}

	getMat(0, 2, true);

	if (abs(det(PHIq)) > e2) {
		v = getMat(0, 3, true);
		matrix qdt = solve(PHIq, v);
		matrix q = getq();
		res = q.T();
		res.combine(qdt.T());
		res = res.T();
	}
	else {
		cout << "Singularity" << endl;
	}
	return res;
}

matrix gety() {
	matrix res;
	matrix qdt = getqdt();
	matrix q = getq();
	res = q.T();
	res.combine(qdt.T());
	res = res.T();
	return res;
}

multiMatrix solveMixedDiffAlgeEq(double t, matrix y) {
	// 根据y设定刚体坐标，用于计算约束和速度方程
	for (int i = 0; i < bodyNum; i++) {
		bodys[i]->setq(y.mat[3 * i][0], y.mat[3 * i + 1][0], y.mat[3 * i + 2][0]);
		bodys[i]->setqdt(y.mat[3 * bodyNum + 3 * i][0], y.mat[3 * bodyNum + 3 * i + 1][0], y.mat[3 * bodyNum + 3 * i + 2][0]);
	}
	formQa();
	getMat(t, 2);
	matrix gamma= getMat(t, 4);

	// 形成系数矩阵A
	matrix A = M;
	A.combine(PHIq.T());
	matrix tmp=PHIq;
	matrix O(3 * bodyNum - addConsNum, 3 * bodyNum - addConsNum);
	tmp.combine(O);
	A.append(tmp);

	matrix B = Qa;
	B.append(gamma);

	matrix X;
	X = solve(A, B);
	matrix f=getqdt();
	f.append(X.slice(0, 3 * bodyNum, -1, -1));

	multiMatrix res;
	res.mat1 = f;
	res.mat2 = X.slice(3 * bodyNum, 2 * 3 * bodyNum - addConsNum, -1, -1);
	tmp = I(1);
	tmp.mat[0][0] = t;
	tmp.append(res.mat2);
	res.mat2 = tmp;
	return res;
}

matrix consStab(double t, matrix y) {
	for (int i = 0; i < bodyNum; i++) {
		bodys[i]->setq(y.mat[3 * i][0], y.mat[3 * i + 1][0], y.mat[3 * i + 2][0]);
		bodys[i]->setqdt(y.mat[3 * bodyNum + 3 * i][0], y.mat[3 * bodyNum + 3 * i + 1][0], y.mat[3 * bodyNum + 3 * i + 2][0]);
	}
	PHI = getMat(t, 1);
	getMat(t, 2);
	v = getMat(t, 3);
	matrix res = y;
	matrix qdt=getqdt();
	while (norm(PHI) > e1 || norm(PHIq * qdt - v) > e2) {
		// 计算修正的q
		q = getq();
		qdt = getqdt();
		matrix dq;
		dq = -gen_inv(PHIq) * PHI;
		q = q + dq;
		for (int i = 0; i < bodyNum; i++) {
			bodys[i]->setq(q.mat[3 * i][0], q.mat[3 * i + 1][0], q.mat[3 * i + 2][0]);
		}
		formQa();
		PHI = getMat(t, 1);
		getMat(t, 2);
		v = getMat(t, 3);

		// 计算修正q后的的qdt
		matrix dqdt;
		dqdt = -gen_inv(PHIq) * (PHIq * qdt - v);
		qdt = qdt + dqdt;
		for (int i = 0; i < bodyNum; i++) {
			bodys[i]->setqdt(qdt.mat[3 * i][0], qdt.mat[3 * i + 1][0], qdt.mat[3 * i + 2][0]);
		}
		formQa();
		PHI = getMat(t, 1);
		getMat(t, 2);
		v = getMat(t, 3);
	}
	res = gety();
	return res;
}