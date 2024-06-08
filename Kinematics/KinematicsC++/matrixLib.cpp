#include "matrixLib.h"
#include <algorithm>
#include<cmath>
#include <math.h>

struct multiMatrix {
	matrix mat1;
	matrix mat2;
};

double** newMat(int row, int column) {
	double** tmp;
	tmp = new double* [row];
	for (int i = 0; i < row; i++) {
		tmp[i] = new double[column];
		memset(tmp[i], 0, column*sizeof(tmp[i][0]));
	}
	return tmp;
}

double** copyMat(double** obj) {
	int r = _msize(obj) / sizeof(*obj);
	int col = _msize(obj[0]) / sizeof(*obj[0]);
	double** tmp = newMat(r, col);
	for (int i = 0; i < r; i++) {
		for (int j = 0; j < col; j++) {
			if (abs(obj[i][j]) < 1e-50) {
				tmp[i][j] = 0;
			}
			else {
				tmp[i][j] = obj[i][j];
			}
		}
	}
	return tmp;
}

double** array2mat(int row, int col, double* a) {
	double** m = new double* [row];
	for (int i = 0; i < row; i++) {
		m[i] = new double[col];
	}
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			double tmp = *(a + i * col + j);
			m[i][j] = tmp;
		}
	}
	return m;
}

matrix::matrix(const matrix& B) {
	row = B.row;
	column = B.column;
	mat = copyMat(B.mat);
}

matrix::matrix(int r, int col) {
	row = r;
	column = col;
	mat = newMat(r, col);
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			mat[i][j] = 0;
		}
	}
}

matrix::matrix(double** m) {
	row = _msize(m) / sizeof(*m);
	column = _msize(m[0]) / sizeof(*m[0]);
	mat = copyMat(m);
}

matrix::matrix(int r, int col, double* a) {
	row = r;
	column = col;
	mat = array2mat(r, col, a);
}

ostream& operator<< (ostream& os, matrix& obj)
{
	//os << "row = " << obj.row << endl;
	//os << "colum = " << obj.column << endl;
	for (int i = 0; i < obj.row; i++) {
		//os << "[\t";
		for (int j = 0; j < obj.column; j++) {
			os << obj.mat[i][j];
			if (j < obj.column - 1) {
				os << ",";
			}
		}
		//os << "]";
		if (i < obj.row - 1) {
			os << endl;
		}
	}
	return os;
}

matrix::~matrix() {
	for (int i = 0; i < row; i++) {
		delete [] mat[i];
	}
	delete[] mat;
}

void matrix::operator= (const matrix& obj) {
	for (int i = 0; i < row; i++) {
		delete[] mat[i];
	}
	delete[] mat;
	row = obj.row;
	column = obj.column;
	mat = copyMat(obj.mat);
}

matrix operator+ (const matrix& A, const matrix& obj) {
	matrix tmp(A.row,A.column);
	for (int i = 0; i < A.row; i++) {
		for (int j = 0; j < A.column; j++) {
			tmp.mat[i][j] = A.mat[i][j] + obj.mat[i][j];
		}
	}
	return tmp;
}

matrix operator- (const matrix& A, const matrix& obj) {
	matrix tmp(A.row, A.column);
	for (int i = 0; i < A.row; i++) {
		for (int j = 0; j < A.column; j++) {
			tmp.mat[i][j] = A.mat[i][j] - obj.mat[i][j];
			if (abs(tmp.mat[i][j]) < 1e-50) {
				tmp.mat[i][j] = 0;
			}
		}
	}
	return tmp;
}

matrix operator- (const matrix& A) {
	matrix tmp(A.row, A.column);
	for (int i = 0; i < A.row; i++) {
		for (int j = 0; j < A.column; j++) {
			tmp.mat[i][j] = -A.mat[i][j];
			if (abs(tmp.mat[i][j]) < 1e-50) {
				tmp.mat[i][j] = 0;
			}
		}
	}
	return tmp;
}

matrix operator* (const matrix& A, const matrix& obj) {
	matrix tmp(A.row, obj.column);
	for (int i = 0; i < tmp.row; i++) {
		for (int j = 0; j < tmp.column; j++) {
			double res = 0;
			for (int r = 0; r < A.column; r++) {
				res += A.mat[i][r] * obj.mat[r][j];
			}
			tmp.mat[i][j] = res;
			if (abs(tmp.mat[i][j]) < 1e-50) {
				tmp.mat[i][j] = 0;
			}
		}
	}
	return tmp;
}

matrix operator* (const matrix& A, double obj) {
	matrix tmp(A.row,A.column);
	for (int i = 0; i < tmp.row; i++) {
		for (int j = 0; j < tmp.column; j++) {
			tmp.mat[i][j] = A.mat[i][j] * obj;
			if (abs(tmp.mat[i][j]) < 1e-50) {
				tmp.mat[i][j] = 0;
			}
		}
	}
	return tmp;
}

matrix operator* (const matrix& A, int obj) {
	matrix tmp(A.row, A.column);
	for (int i = 0; i < tmp.row; i++) {
		for (int j = 0; j < tmp.column; j++) {
			tmp.mat[i][j] = A.mat[i][j] * obj;
			if (abs(tmp.mat[i][j]) < 1e-50) {
				tmp.mat[i][j] = 0;
			}
		}
	}
	return tmp;
}

matrix operator* (double obj, const matrix& A) {
	matrix tmp(A.row, A.column);
	for (int i = 0; i < tmp.row; i++) {
		for (int j = 0; j < tmp.column; j++) {
			tmp.mat[i][j] = A.mat[i][j] * obj;
			if (abs(tmp.mat[i][j]) < 1e-50) {
				tmp.mat[i][j] = 0;
			}
		}
	}
	return tmp;
}

matrix operator* (int obj, const matrix& A) {
	matrix tmp(A.row, A.column);
	for (int i = 0; i < tmp.row; i++) {
		for (int j = 0; j < tmp.column; j++) {
			tmp.mat[i][j] = A.mat[i][j] * obj;
			if (abs(tmp.mat[i][j]) < 1e-50) {
				tmp.mat[i][j] = 0;
			}
		}
	}
	return tmp;
}

matrix operator/ (const matrix& A, double obj) {
	matrix tmp(A.row, A.column);
	for (int i = 0; i < tmp.row; i++) {
		for (int j = 0; j < tmp.column; j++) {
			tmp.mat[i][j] = A.mat[i][j] / obj;
			if (abs(tmp.mat[i][j]) < 1e-50) {
				tmp.mat[i][j] = 0;
			}
		}
	}
	return tmp;
}

matrix operator/ (const matrix& A, int obj) {
	matrix tmp(A.row, A.column);
	for (int i = 0; i < tmp.row; i++) {
		for (int j = 0; j < tmp.column; j++) {
			tmp.mat[i][j] = A.mat[i][j] / obj;
			if (abs(tmp.mat[i][j]) < 1e-50) {
				tmp.mat[i][j] = 0;
			}
		}
	}
	return tmp;
}

matrix matrix::T() {
	matrix tmp(column, row);
	for (int i = 0; i < tmp.row; i++) {
		for (int j = 0; j < tmp.column; j++) {
			tmp.mat[i][j] = mat[j][i];
		}
	}
	return tmp;
}

matrix Transp(matrix A) {
	matrix tmp(A.column, A.row);
	for (int i = 0; i < tmp.row; i++) {
		for (int j = 0; j < tmp.column; j++) {
			tmp.mat[i][j] = A.mat[j][i];
		}
	}
	return tmp;
}

void matrix::append(const matrix& B) {
	double** tmp = newMat(row + B.row, column);
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			tmp[i][j] = mat[i][j];
		}
	}
	for (int i = row; i < row + B.row; i++) {
		for (int j = 0; j < column; j++) {
			tmp[i][j] = B.mat[i - row][j];
		}
	}
	for (int i = 0; i < row; i++) {
		delete[] mat[i];
	}
	delete[] mat;

	row += B.row;
	mat = tmp;
}

void matrix::combine(const matrix& B) {
	double** tmp = newMat(row, column + B.column);
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column + B.column; j++) {
			if (j < column) {
				tmp[i][j] = mat[i][j];
			}
			else {
				tmp[i][j] = B.mat[i][j - column];
			}
		}
	}
	for (int i = 0; i < row; i++) {
		delete[] mat[i];
	}
	delete[] mat;
	column += B.column;
	mat = tmp;
}

void matrix::insert(const matrix& B, int a, int b) {
	matrix temp = B;
	//cout << temp << endl << endl;
	double** tmp = copyMat(mat);
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			if (i >= a && i < (a + B.row) && j >= b && j < (b + B.column)) {
				tmp[i][j] += B.mat[i-a][j-b];
			}
		}
	}
	for (int i = 0; i < row; i++) {
		delete[] mat[i];
	}
	delete[] mat;
	mat = tmp;
	//cout << *this << endl << endl;
}

multiMatrix matrix::gaussianEliminate() {
	//https://zhuanlan.zhihu.com/p/84210687
	// L是消去时的初等变换矩阵
	matrix Ab(*this);
	matrix L = I(row);
	int rows = Ab.row;
	int cols = Ab.column;
	int Acols = cols - 1;

	int i = 0; // row tracker
	int j = 0; // column tracker

	// iterate through the rows
	while (i < rows)
	{
		// find a pivot for the row
		bool pivot_found = false;
		while (j < Acols && !pivot_found)
		{
			if (Ab.mat[i][j] != 0) { // pivot not equal to 0
				pivot_found = true;
			}
			else { // check for a possible swap
				int max_row = i;
				double max_val = 0;
				for (int k = i + 1; k < rows; ++k)
				{
					double cur_abs = Ab.mat[k][j] >= 0 ? Ab.mat[k][j] : -1 * Ab.mat[k][j];
					if (cur_abs > max_val)
					{
						max_row = k;
						max_val = cur_abs;
					}
				}
				if (max_row != i) {
					Ab.swapRows(max_row, i);
					pivot_found = true;
				}
				else {
					j++;
				}
			}
		}

		// perform elimination as normal if pivot was found
		if (pivot_found)
		{
			for (int t = i + 1; t < rows; ++t) {
				for (int s = j + 1; s < cols; ++s) {
					// 行与行相减
					L.mat[t][i] = (Ab.mat[t][j] / Ab.mat[i][j]);
					Ab.mat[t][s] = Ab.mat[t][s] - Ab.mat[i][s] * (Ab.mat[t][j] / Ab.mat[i][j]);
					if (abs(Ab.mat[t][s]) < 1e-30)
						Ab.mat[t][s] = 0;
				}
				Ab.mat[t][j] = 0;
			}
		}

		i++;
		j++;
	}
	multiMatrix res;
	res.mat1 = L;
	res.mat2 = Ab;
	return res;
}

matrix matrix::rowReduceFromGaussian() {
	matrix R(*this);
	int rows = R.row;
	int cols = R.column;

	int i = rows - 1; // row tracker
	int j = cols - 2; // column tracker

	// iterate through every row
	while (i >= 0)
	{
		// find the pivot column
		int k = j - 1;
		while (k >= 0) {
			if (R.mat[i][k] != 0)
				j = k;
			k--;
		}

		// zero out elements above pivots if pivot not 0
		if (R.mat[i][j] != 0) {

			for (int t = i - 1; t >= 0; --t) {
				for (int s = 0; s < cols; ++s) {
					if (s != j) {
						R.mat[t][s] = R.mat[t][s] - R.mat[i][s] * (R.mat[t][j] / R.mat[i][j]);
						if (abs(R.mat[t][s]) < 1e-30)
							R.mat[t][s] = 0;
					}
				}
				R.mat[t][j] = 0;
			}

			// divide row by pivot
			for (int k = j + 1; k < cols; ++k) {
				R.mat[i][k] = R.mat[i][k] / R.mat[i][j];
				if (abs(R.mat[i][k]) < 1e-30)
					R.mat[i][k] = 0;
			}
			R.mat[i][j] = 1;

		}

		i--;
		j--;
	}

	return R;
}

double det(matrix A) {
	//cout << A << endl << endl;
	matrix AI = augment(A, I(A.row));
	multiMatrix LAb = AI.gaussianEliminate();
	matrix L = LAb.mat1;
	//cout << L << endl << endl;
	matrix Ub = LAb.mat2;
	//cout << Ub << endl << endl;
	//matrix temp = L * Ub;
	//cout << temp << endl << endl;
	double sum=1;
	for (int i = 0; i < L.row; i++) {
		sum *= L.mat[i][i];
		sum *= Ub.mat[i][i];
	}
	return sum;
}

matrix I(int n) {
	matrix tmp(n, n);
	for (int i = 0; i < n; i++) {
		tmp.mat[i][i] = 1;
	}
	return tmp;
}

void matrix::deleterow(int a) {
	double** tmp = newMat(row - 1, column);
	for (int i = 0; i < row; i++) {
		if (i == a) {
			continue;
		}
		for (int j = 0; j < column; j++) {
			if (i < a) {
				tmp[i][j] = mat[i][j];
			}
			else if (i > a) {
				tmp[i - 1][j] = mat[i][j];
			}
		}
	}
	for (int i = 0; i < row; i++) {
		delete[] mat[i];
	}
	delete[] mat;
	row -= 1;
	mat = tmp;
}

void matrix::deletecol(int a) {
	double** tmp = newMat(row, column - 1);
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			if (j == a) {
				continue;
			}
			else if (j < a) {
				tmp[i][j] = mat[i][j];
			}
			else if (j > a) {
				tmp[i][j-1] = mat[i][j];
			}
		}
	}
	for (int i = 0; i < row; i++) {
		delete[] mat[i];
	}
	delete[] mat;
	column -= 1;
	mat = tmp;
}

void matrix::swapRows(int r1, int r2) {
	double* temp = mat[r1];
	mat[r1] = mat[r2];
	mat[r2] = temp;
}

matrix matrix::inv() {
//https://zhuanlan.zhihu.com/p/84210687
	matrix AI = augment(*this, I(row));
	//cout << AI << endl << endl;
	matrix Ub = AI.gaussianEliminate().mat2;
	//cout << Ub << endl << endl;
	matrix IAInverse = Ub.rowReduceFromGaussian();
	//cout << IAInverse << endl << endl;
	matrix AInverse(row, column);
	for (int i = 0; i < AInverse.row; ++i) {
		for (int j = 0; j < AInverse.column; ++j) {
			AInverse.mat[i][j] = IAInverse.mat[i][j + column];
		}
	}
	return AInverse;
}

double norm(matrix A) {
	double num = 0;
	for (int i = 0; i < A.row; i++) {
		for (int j = 0; j < A.column; j++) {
			num += pow(A.mat[i][j], 2);
		}
	}
	num = sqrt(num);
	return num;
}

matrix solve(matrix A, matrix b) {
	
	matrix temp = A.inv() * b;
	return temp;
	
	/*
	// Gaussian elimination
	for (int i = 0; i < A.row; ++i) {
		if (A.mat[i][i] == 0) {
			// pivot 0 - throw error
			throw domain_error("Error: the coefficient matrix has 0 as a pivot. Please fix the input and try again.");
		}
		for (int j = i + 1; j < A.row; ++j) {
			for (int k = i + 1; k < A.column; ++k) {
				A.mat[j][k] -= A.mat[i][k] * (A.mat[j][i] / A.mat[i][i]);
				if (A.mat[j][k] < 1e-30 && A.mat[j][k] > -1 * 1e-30)
					A.mat[j][k] = 0;
			}
			b.mat[j][0] -= b.mat[i][0] * (A.mat[j][i] / A.mat[i][i]);
			if (A.mat[j][0] < 1e-30 && A.mat[j][0] > -1 * 1e-30)
				A.mat[j][0] = 0;
			A.mat[j][i] = 0;
		}
	}

	// Back substitution
	matrix x(b.row, 1);
	x.mat[x.row - 1][0] = b.mat[x.row - 1][0] / A.mat[x.row - 1][x.row - 1];
	if (x.mat[x.row - 1][0] < 1e-30 && x.mat[x.row - 1][0] > -1 * 1e-30)
		x.mat[x.row - 1][0] = 0;
	for (int i = x.row - 2; i >= 0; --i) {
		double sum = 0;
		for (int j = i + 1; j < x.row; ++j) {
			sum += A.mat[i][j] * x.mat[j][0];
		}
		x.mat[i][0] = (b.mat[i][0] - sum) / A.mat[i][i];
		if (x.mat[i][0] < 1e-30 && x.mat[i][0] > -1 * 1e-30)
			x.mat[i][0] = 0;
	}
	return x;
	*/
}

matrix augment(matrix A, matrix b) {
	//增广矩阵
	matrix AB=A;
	AB.combine(b);
	return AB;
}