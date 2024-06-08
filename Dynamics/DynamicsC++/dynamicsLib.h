#pragma once
#ifndef DYNAMICSLIB_H_INCLUDED
#define DYNAMICSLIB_H_INCLUDED
#include "matrixLib.h"
#include <string>

class force {
	friend ostream& operator<< (ostream& os, force& obj);
public:
	string type;
	matrix vector;
	matrix s;
	double value;
	int b;
	force() :type("UnKnown"),b(-1),value(0) {};
};

class actuator {
	friend ostream& operator<< (ostream& os, actuator& obj);
public:
	string type;
	double F,k,c,l0;
	int b1, b2;
	matrix s1, s2;
	actuator() :type("UnKnown"), F(0), k(0), c(0), l0(0) {};
};

#endif