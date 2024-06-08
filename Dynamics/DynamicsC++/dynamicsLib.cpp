#include "dynamicsLib.h"

ostream& operator<< (ostream& os, force& obj) {
	os << "Force type : " << obj.type ;
	if (obj.type == "af" || obj.type == "rf") {
		os << " on body " << obj.b;
		matrix tmp;
		tmp = obj.s.T();
		os << ", s = " << tmp;
		tmp = obj.vector.T();
		os << ", vector = " << tmp;
	}
	else if (obj.type == "m") {
		os << " on body " << obj.b << " , value = " <<obj.value;
	}
	else if (obj.type == "g") {
		matrix tmp;
		tmp = obj.vector.T();
		os << " vector = " << tmp;
	}
	os << endl;
	return os;
}

ostream& operator<< (ostream& os, actuator& obj) {
	os << "Actuator type : " << obj.type << " F = " << obj.F << " k = " << obj.k  <<" c = " << obj.c << " l0 = " << obj.l0;
	os << " body 1 : " << obj.b1 << " body 2 : " << obj.b2;
	if (obj.type == "tra") {
		matrix tmp;
		tmp = obj.s1.T();
		os << " s1 = " << tmp;
		tmp = obj.s2.T();
		os << " s2 = " << tmp;
	}
	os << endl;
	return os;
}