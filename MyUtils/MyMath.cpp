#include "MyMath.h"
#include <assert.h>
#include <cmath>

#define PI 3.1415926
/************************************************

MyMath

*/

void MyMath::Hello() {
	std::cout << "hello world" << std::endl;
}

void MyMathTest() {
	std::cout << "my math test" << std::endl;
}

double MyMath::NormL1(const VectorBase &vec) {
	double result = 0;
	for (int i = 0; i < vec.GetElemNum(); ++i) {
		result += abs(vec.elem[i]);
	}
	return result;
}

double MyMath::NormL2(const VectorBase &vec) {
	double result = 0;
	for (int i = 0; i < vec.GetElemNum(); ++i) {
		result += vec.elem[i] * vec.elem[i];
	}
	return sqrt(result);
}

double MyMath::Dot(const VectorBase &vec1, const VectorBase &vec2) {
	int len = vec1.GetElemNum();
	assert(len == vec2.GetElemNum());
	double result = 0;
	for (int i = 0; i < len; ++i) {
		result += vec1.elem[i] * vec2.elem[i];
	}
	return result;
}

Vector3 MyMath::Cross(const Vector3 &vec1, const Vector3 &vec2) {
	Vector3 result;
	result.elem[0] = vec1.elem[1] * vec2.elem[2] - vec1.elem[2] * vec2.elem[1];
	result.elem[1] = vec1.elem[2] * vec2.elem[0] - vec1.elem[0] * vec2.elem[2];
	result.elem[2] = vec1.elem[0] * vec2.elem[1] - vec1.elem[1] * vec2.elem[0];
	return result;
}

double MyMath::Determinant(const Matrix2& mat) {
	double result = mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1];
	return result;
}

double MyMath::Determinant(const Matrix3& mat) {
	double result = mat.matrix[0][0] * mat.matrix[1].elem[1] * mat.matrix[2].elem[2] +
		mat.matrix[0].elem[1] * mat.matrix[1].elem[2] * mat.matrix[2].elem[0] +
		mat.matrix[0].elem[2] * mat.matrix[1].elem[0] * mat.matrix[2].elem[1];
	result -= mat.matrix[2].elem[0] * mat.matrix[1].elem[1] * mat.matrix[0].elem[2] +
		mat.matrix[2].elem[1] * mat.matrix[1].elem[2] * mat.matrix[0].elem[0] +
		mat.matrix[2].elem[2] * mat.matrix[1].elem[0] * mat.matrix[0].elem[1];
	return result;
}

double MyMath::Determinant(const Matrix4& mat) {
	double result = 0;
	for (int i = 0; i < 4; ++i) {
		result += mat[0][i] * mat.GetCofactor(0, i).Det();
	}
	return result;
}


double MyMath::Deg2Rad(const double deg) {
	return PI * deg / 180;
}

double MyMath::Rad2Deg(const double rad) {
	return rad / PI * 180;
}

Matrix3 MyMath::GetRotationMatrix3AroundX(double deg) {
	double rad = MyMath::Deg2Rad(deg);
	double sinX = std::sin(rad), cosX = std::cos(rad);
	Vector3 vec1(1, 0, 0), vec2(0, cosX, -sinX), vec3(0, sinX, cosX);
	return Matrix3(vec1, vec2, vec3);
}

Matrix3 MyMath::GetRotationMatrix3AroundY(double deg) {
	double rad = MyMath::Deg2Rad(deg);
	double sinX = std::sin(rad), cosX = std::cos(rad);
	Vector3 vec1(cosX, 0, sinX), vec2(0, 1, 0), vec3(-sinX, 0, cosX);
	return Matrix3(vec1, vec2, vec3);
}

Matrix3 MyMath::GetRotationMatrix3AroundZ(double deg) {
	double rad = MyMath::Deg2Rad(deg);
	double sinX = std::sin(rad), cosX = std::cos(rad);
	Vector3 vec1(cosX, -sinX, 0), vec2(sinX, cosX, 0), vec3(0, 0, 1);
	return Matrix3(vec1, vec2, vec3);
}

MatrixT MyMath::GetRotationMatrixAroundX(double deg) {
	Matrix3 mat3 = MyMath::GetRotationMatrix3AroundX(deg);
	Vector3 vec;
	return MatrixT(mat3, vec);
}

MatrixT MyMath::GetRotationMatrixAroundY(double deg) {
	Matrix3 mat3 = MyMath::GetRotationMatrix3AroundY(deg);
	Vector3 vec;
	return MatrixT(mat3, vec);
}

MatrixT MyMath::GetRotationMatrixAroundZ(double deg) {
	Matrix3 mat3 = MyMath::GetRotationMatrix3AroundZ(deg);
	Vector3 vec;
	return MatrixT(mat3, vec);
}

Matrix3 MyMath::GetRotationMatrix3Around(const Vector3& vec, double deg) {
	Vector3 tmp = vec;
	tmp = tmp.Normalized();
	double x = tmp[0], y = tmp[1], z = tmp[2], rad = MyMath::Deg2Rad(deg);
	double s = std::sin(rad), c = std::cos(rad);
	Vector3 vec1(c + (1 - c)*x*x, (1 - c)*x*y - s*z, (1 - c)*x*z + s*y),
		vec2((1 - c)*x*y + s*z, c + (1 - c)*y*y, (1 - c)*y*z - s*x),
		vec3((1 - c)*x*z - s*y, (1 - c)*y*z + s*x, c + (1 - c)*z*z);
	return Matrix3(vec1, vec2, vec3);
}

MatrixT MyMath::GetRotationMatrixAround(const Vector3& vec, double deg) {
	Matrix3 mat3 = MyMath::GetRotationMatrix3Around(vec, deg);
	Vector3 vec3;
	return MatrixT(mat3, vec3);
}

MatrixT MyMath::GetMoveMatrix(const Vector3& offset) {
	Matrix3 mat3 = Matrix3::GetIdentity();
	return MatrixT(mat3, offset);
}

MatrixT MyMath::CoordToWorldMatrix(const CoordinateSystem& coord) {
	Matrix3 tmp(coord.GetX(), coord.GetY(), coord.GetZ());
	return MatrixT(tmp.Transpose(), coord.GetO());
}

MatrixT MyMath::WorldToCoordMatrix(const CoordinateSystem& coord) {
	Matrix3 tmpM(coord.GetX(), coord.GetY(), coord.GetZ());
	Vector3 tmpV = tmpM * coord.GetO();
	tmpV = tmpV * -1;
	return MatrixT(tmpM, tmpV);
}

Quaternion MyMath::QuaternionMul(const Quaternion& q1, const Quaternion& q2) {
	Vector3 v, v1(q1[1], q1[2], q1[3]), v2(q2[1], q2[2], q2[3]);
	double s, s1 = q1[0], s2 = q2[0];
	s = s1 * s2 - Dot(v1, v2);
	v = v2 * s1 + v1 * s2 + Cross(v1, v2);
	return Quaternion(v, s);
}

Quaternion MyMath::GetQuaternionAround(const Vector3& axis, double deg) {
	deg /= 2;
	double rad = Deg2Rad(deg);
	double c = std::cos(rad), s = std::sin(rad);
	Vector3 tmp;
	tmp = axis;
	tmp = tmp * s;
	return Quaternion(tmp, c);
}

Vector3 MyMath::Rotate(const Vector3& vec, const Vector3& axis, double deg) {
	Quaternion q1 = GetQuaternionAround(axis, deg), q(vec, 0), q2;
	//q = vec;
	q2 = q1.Inverse();
	std::cout << "q1: " << q1 << std::endl << "q2: " << q2 << std::endl;

	Quaternion result = QuaternionMul(q1, q);
	std::cout << "tmp: " << result << std::endl;
	result = QuaternionMul(result, q2);
	Vector3 ans(result[1], result[2], result[3]);
	return ans;

}


/*************************************************

Vector

*/

inline double VectorBase::GetElemNum() const {
	return elemNum;
}

std::ostream & operator<<(std::ostream & os, const VectorBase & vec)
{
	os << "(";
	for (int i = 0; i < vec.elemNum - 1; ++i) {
		os << vec.elem[i] << ", ";
	}
	os << vec.elem[vec.elemNum - 1] << ")";
	return os;
}

VectorBase VectorBase::operator+(const VectorBase& vec) {
	VectorBase result = vec;
	if (this->elemNum != vec.elemNum) {
		std::cerr << "type err: " << this << " and " << vec << std::endl;
	}
	assert(this->elemNum == vec.elemNum);
	for (int i = 0; i < vec.elemNum; ++i) {
		result.elem[i] = elem[i] + vec.elem[i];
	}
	return result;
}

VectorBase VectorBase::operator-(const VectorBase& vec) {
	VectorBase result = vec;
	if (this->elemNum != vec.elemNum) {
		std::cerr << "type err: " << this << " and " << vec << std::endl;
	}
	assert(this->elemNum == vec.elemNum);
	for (int i = 0; i < vec.elemNum; ++i) {
		result.elem[i] = elem[i] - vec.elem[i];
	}
	return result;
}

VectorBase VectorBase::operator*(const VectorBase& vec) {
	VectorBase result = vec;
	if (this->elemNum != vec.elemNum) {
		std::cerr << "type err: " << this << " and " << vec << std::endl;
	}
	assert(this->elemNum == vec.elemNum);
	for (int i = 0; i < vec.elemNum; ++i) {
		result.elem[i] = elem[i] * vec.elem[i];
	}
	return result;
}

VectorBase VectorBase::operator/(const VectorBase& vec) {
	VectorBase result = vec;
	if (this->elemNum != vec.elemNum) {
		std::cerr << "type err: " << this << " and " << vec << std::endl;
	}
	assert(this->elemNum == vec.elemNum);
	for (int i = 0; i < vec.elemNum; ++i) {
		result.elem[i] = elem[i] / vec.elem[i];
	}
	return result;
}

VectorBase VectorBase::operator+(double num) {
	VectorBase result = *this;

	for (int i = 0; i < this->elemNum; ++i) {
		result.elem[i] = elem[i] + num;
	}
	return result;
}

VectorBase VectorBase::operator/(double num) {
	VectorBase result = *this;

	for (int i = 0; i < this->elemNum; ++i) {
		result.elem[i] = elem[i] / num;
	}
	return result;
}

VectorBase VectorBase::operator-(double num) {
	VectorBase result = *this;

	for (int i = 0; i < this->elemNum; ++i) {
		result.elem[i] = elem[i] - num;
	}
	return result;
}

VectorBase VectorBase::operator*(double num) {
	//std::cout << "operator *" << std::endl;
	//VectorBase *result = new VectorBase();

	VectorBase result = *this;
	for (int i = 0; i < this->elemNum; ++i) {
		result.elem[i] = elem[i] * num;
	}
	return result;
}

double VectorBase::operator[](int n) const {
	assert(n < elemNum);
	return elem[n];
}

VectorBase& VectorBase::operator=(const VectorBase& vec) {
	for (int i = 0; i < 4; ++i) {
		elem[i] = vec.elem[i];
	}
	return *this;
}

bool VectorBase::operator==(const VectorBase& vec) {
	int num = INT_MAX;
	if (elemNum == vec.GetElemNum()) {
		num = elemNum;
	}
	else {
		std::cout << "type err!" << std::endl;
		std::cout << *this << " and " << vec << std::endl;
		num = (elemNum < vec.GetElemNum() ? elemNum : vec.GetElemNum());
	}
	for (int i = 0; i < num; ++i) {
		if (this->elem[i] != vec.elem[i])
			return false;
	}
	return true;
}

VectorBase::VectorBase() {
	//std::cout << "VectorBase construct start" << std::endl;
	elem[0] = elem[1] = elem[2] = elem[3] = 0;
	elemNum = 4;
	//std::cout << "VectorBase construct start" << std::endl;
	//std::cout << "VectorBase construct over" << std::endl;
}

VectorBase::~VectorBase() {
	//std::cout << "VectorBase deconstruct start" << std::endl;
	//std::cout << "delete: " << *this << std::endl;
	//delete[] elem;
	//std::cout << "VectorBase deconstruct over" << std::endl;
}

VectorBase VectorBase::Normalized() const {
	VectorBase result = *this;
	double l2 = MyMath::NormL2(result);
	for (int i = 0; i < elemNum; ++i) {
		result.elem[i] = elem[i] / l2;
	}
	return result;
}




/*************************************************

Vector2

*/

Vector2::Vector2() {
	elem[0] = elem[1] = 0;
	elemNum = 2;
}

Vector2::Vector2(double x, double y) {
	elem[0] = x;
	elem[1] = y;
	elemNum = 2;
}

inline double Vector2::GetX() const { return elem[0]; }
inline double Vector2::GetY() const { return elem[1]; }

Vector2& Vector2::operator=(const VectorBase& vec) {
	for (int i = 0; i < elemNum; ++i) {
		elem[i] = vec.elem[i];
	}
	return *this;
}

Vector2& Vector2::operator=(const Vector2& vec) {
	for (int i = 0; i < elemNum; ++i) {
		elem[i] = vec.elem[i];
	}
	return *this;
}

Vector2::~Vector2() {
	std::cout << "Vector2 destruct" << std::endl;
}



/**********************************************************

Vector3

*/
Vector3::Vector3() {
	elem[0] = elem[1] = elem[2] = 0;
	elemNum = 3;
}

Vector3::Vector3(double x, double y, double z) {
	elem[0] = x;
	elem[1] = y;
	elem[2] = z;
	elemNum = 3;
}

inline double Vector3::GetX() const { return elem[0]; }
inline double Vector3::GetY() const { return elem[1]; }
inline double Vector3::GetZ() const { return elem[2]; }

Vector3& Vector3::operator=(const VectorBase& vec) {
	for (int i = 0; i < elemNum; ++i) {
		elem[i] = vec.elem[i];
	}
	return *this;
}



/***************************************************************

Vector4

*/
Vector4::Vector4() {
	elem[0] = elem[1] = elem[2] = elem[3] = 0;
	elemNum = 4;
}

Vector4::Vector4(double x, double y, double z, double w) {
	elem[0] = x;
	elem[1] = y;
	elem[2] = z;
	elem[3] = w;
	elemNum = 4;
}

inline double Vector4::GetX() const { return elem[0]; }
inline double Vector4::GetY() const { return elem[1]; }
inline double Vector4::GetZ() const { return elem[2]; }
inline double Vector4::GetW() const { return elem[3]; }

Vector4& Vector4::operator=(const VectorBase& vec) {
	for (int i = 0; i < elemNum; ++i) {
		elem[i] = vec.elem[i];
	}
	return *this;
}

Vector3 Vector4::GetSubVector(unsigned index) {
	double tmp[3];
	for (int i = 0, j = 0; i < 4; ++i) {
		if (i == index)
			continue;
		tmp[j++] = elem[i];
	}
	Vector3 result(tmp[0], tmp[1], tmp[2]);
	return result;
}


/***************************************************************

Matrix2

*/
std::ostream& operator<<(std::ostream& os, const Matrix2 mat) {
	os << "[" << mat.matrix[0] << std::endl
		<< mat.matrix[1] << "]" << std::endl;
	return os;
}

Vector2 Matrix2::operator[](unsigned index) const {
	assert(index < 2);
	Vector2 result = matrix[index];
	return result;
}

Matrix2 Matrix2::operator+(double num) {
	Matrix2 result = *this;
	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < 2; ++j) {
			result.matrix[i].elem[j] += num;
		}
	}
	return result;
}

Matrix2 Matrix2::operator-(double num) {
	Matrix2 result = *this;
	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < 2; ++j) {
			result.matrix[i].elem[j] -= num;
		}
	}
	return result;
}

Matrix2 Matrix2::operator*(double num) {
	Matrix2 result = *this;
	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < 2; ++j) {
			result.matrix[i].elem[j] *= num;
		}
	}
	return result;
}

Matrix2 Matrix2::operator/(double num) {
	Matrix2 result = *this;
	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < 2; ++j) {
			result.matrix[i].elem[j] /= num;
		}
	}
	return result;
}

Matrix2 Matrix2::operator*(const Matrix2& mat) {
	Matrix2 result;
	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < 2; ++j) {
			double tmp = MyMath::Dot(this->GetRow(i), mat.GetCol(j));
			result.matrix[i].elem[j] = tmp;
		}
	}
	return result;
}

bool Matrix2::operator==(const Matrix2& mat) {
	for (int i = 0; i < 2; ++i) {
		if (this->matrix[i] == mat[i])
			continue;
		else
			return false;
	}
	return true;
}

Matrix2::Matrix2() {
	matrix = new Vector2[2];
}

Matrix2::Matrix2(const Vector2& v1, const Vector2& v2) {
	matrix = new Vector2[2];
	matrix[0] = v1;
	matrix[1] = v2;
}

Matrix2::~Matrix2() {
	std::cout << "Matrix2 deconstruct start" << std::endl;
	delete[] matrix;
	std::cout << "Matrix2 deconstruct over" << std::endl;
}

double Matrix2::Determinant() {
	return MyMath::Determinant(*this);
}

double Matrix2::Det() {
	return MyMath::Determinant(*this);
}

Matrix2 Matrix2::Inverse() {
	double det = this->Det();
	Matrix2 result;
	result.matrix[0].elem[0] = this->matrix[1][1] / det;
	result.matrix[0].elem[1] = -this->matrix[1][0] / det;
	result.matrix[1].elem[0] = -this->matrix[0][1] / det;
	result.matrix[1].elem[1] = this->matrix[0][0] / det;
	return result;
}

Vector2 Matrix2::GetRow(unsigned index) const {
	assert(index < 2);
	Vector2 result = matrix[index];
	return result;
}

Vector2 Matrix2::GetCol(unsigned index) const {
	assert(index < 2);
	Vector2 result;
	for (int i = 0; i < 2; ++i) {
		result.elem[i] = matrix[i][index];
	}
	return result;
}

Matrix2 Matrix2::GetIdentity() {
	Vector2 vec[2];
	for (int i = 0; i < 2; ++i) {
		vec[i].elem[i] = 1;
	}
	return Matrix2(vec[0], vec[1]);
}

Matrix2 Matrix2::Transpose() {
	Matrix2 result = *this;
	std::swap(result.matrix[0].elem[1], result.matrix[1].elem[0]);
	return result;
}

/***************************************************************

Matrix3

*/
std::ostream& operator<<(std::ostream& os, const Matrix3 mat) {
	os << "[" << mat.matrix[0] << std::endl 
		<< mat.matrix[1] << std::endl
		<< mat.matrix[2] << "]" << std::endl;
	return os;
}

Vector3 Matrix3::operator[](unsigned index) const {
	assert(index < 3);
	Vector3 result = matrix[index];
	return result;
}

Matrix3 Matrix3::operator+(double num) {
	Matrix3 result = *this;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			result.matrix[i].elem[j] += num;
		}
	}
	return result;
}

Matrix3 Matrix3::operator-(double num) {
	Matrix3 result = *this;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			result.matrix[i].elem[j] -= num;
		}
	}
	return result;
}

Matrix3 Matrix3::operator*(double num) {
	Matrix3 result = *this;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			result.matrix[i].elem[j] *= num;
		}
	}
	return result;
}

Matrix3 Matrix3::operator/(double num) {
	Matrix3 result = *this;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			result.matrix[i].elem[j] /= num;
		}
	}
	return result;
}

Matrix3 Matrix3::operator*(const Matrix3& mat) {
	Matrix3 result;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			double tmp = MyMath::Dot(this->GetRow(i), mat.GetCol(j));
			result.matrix[i].elem[j] = tmp;
		}
	}
	return result;
}

Vector3 Matrix3::operator*(const Vector3& vec) {
	Vector3 result;
	for (int i = 0; i < 3; ++i) {
		result.elem[i] = MyMath::Dot(this->GetRow(i), vec);
	}
	return result;
}

bool Matrix3::operator==(const Matrix3& mat) {
	for (int i = 0; i < 3; ++i) {
		if (this->matrix[i] == mat[i])
			continue;
		else
			return false;
	}
	return true;
}

Matrix3::Matrix3() {}

Matrix3::Matrix3(const Vector3& v1, const Vector3& v2, const Vector3& v3) {
	matrix[0] = v1;
	matrix[1] = v2;
	matrix[2] = v3;
}

double Matrix3::Determinant() {
	return MyMath::Determinant(*this);
}

double Matrix3::Det() {
	return MyMath::Determinant(*this);
}

Vector3 Matrix3::GetRow(unsigned index) const {
	assert(index < 3);
	Vector3 result = matrix[index];
	return result;
}

Vector3 Matrix3::GetCol(unsigned index) const {
	assert(index < 3);
	Vector3 result;
	for (int i = 0; i < 3; ++i) {
		result.elem[i] = matrix[i][index];
	}
	return result;
}

Matrix2 Matrix3::GetMinor(unsigned rowIndex, unsigned colIndex) {
	assert(rowIndex < 3 && colIndex < 3);
	Matrix2 result;
	for (unsigned i = 0, row = 0; i < 3; ++i) {
		if (i == rowIndex) {
			continue;
		}
		for (unsigned j = 0, col = 0; j < 3; ++j) {
			if (j == colIndex) {
				continue;
			}
			result.matrix[row].elem[col++] = this->matrix[i][j];
		}
		++row;
	}
	return result;
}

Matrix2 Matrix3::GetCofactor(unsigned rowIndex, unsigned colIndex) {
	assert(rowIndex < 3 && colIndex < 3);
	Matrix2 result = this->GetMinor(rowIndex, colIndex);
	if ((rowIndex + colIndex) % 2 == 0) {
		return result;
	}
	else {
		return result * -1;
	}
}

Matrix3 Matrix3::GetIdentity() {
	Vector3 vec[3];
	for (int i = 0; i < 4; ++i) {
		vec[i].elem[i] = 1;
	}
	return Matrix3(vec[0], vec[1], vec[2]);
}

Matrix3 Matrix3::Transpose() {
	Matrix3 result = *this;
	std::swap(result.matrix[0].elem[1], result.matrix[1].elem[0]);
	std::swap(result.matrix[0].elem[2], result.matrix[2].elem[0]);
	std::swap(result.matrix[1].elem[2], result.matrix[2].elem[1]);
	return result;
}

Matrix3 Matrix3::Inverse() {
	Matrix3 result;
	double det = this->Det();
	if (det == 0) {
		std::cerr << "det = 0! NO INVERSE!" << std::endl;
		return result;
	}
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			result.matrix[i].elem[j] = this->GetCofactor(i, j).Det() / det;
		}
	}
	return result;
}


/***************************************************************

Matrix4

*/

std::ostream& operator<<(std::ostream& os, const Matrix4 mat) {
	os << "[" << mat.matrix[0] << std::endl
		<< mat.matrix[1] << std::endl
		<< mat.matrix[2] << std::endl
		<< mat.matrix[3] << "]" << std::endl;
	return os;
}

Vector4 Matrix4::operator[](unsigned index) const {
	assert(index < 4);
	Vector4 result = matrix[index];
	return result;
}

Matrix4 Matrix4::operator+(double num) {
	Matrix4 result = *this;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result.matrix[i].elem[j] += num;
		}
	}
	return result;
}

Matrix4 Matrix4::operator-(double num) {
	Matrix4 result = *this;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result.matrix[i].elem[j] -= num;
		}
	}
	return result;
}

Matrix4 Matrix4::operator*(double num) {
	Matrix4 result = *this;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result.matrix[i].elem[j] *= num;
		}
	}
	return result;
}

Matrix4 Matrix4::operator/(double num) {
	Matrix4 result = *this;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result.matrix[i].elem[j] /= num;
		}
	}
	return result;
}

Matrix4 Matrix4::operator*(const Matrix4& mat) {
	Matrix4 result;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			double tmp = MyMath::Dot(this->GetRow(i), mat.GetCol(j));
			result.matrix[i].elem[j] = tmp;
		}
	}
	return result;
}

Vector4 Matrix4::operator*(const Vector4& vec) {
	Vector4 result;
	for (int i = 0; i < 4; ++i) {
		result.elem[i] = MyMath::Dot(this->GetRow(i), vec);
	}
	return result;
}

bool Matrix4::operator==(const Matrix4& mat) {
	for (int i = 0; i < 4; ++i) {
		if (this->matrix[i] == mat[i])
			continue;
		else
			return false;
	}
	return true;
}

Matrix4::Matrix4() {}

Matrix4::Matrix4(const Vector4& v1, const Vector4& v2, const Vector4& v3, const Vector4& v4) {
	matrix[0] = v1;
	matrix[1] = v2;
	matrix[2] = v3;
	matrix[3] = v4;
}

double Matrix4::Determinant() {
	return MyMath::Determinant(*this);
}

double Matrix4::Det() {
	return MyMath::Determinant(*this);
}

Vector4 Matrix4::GetRow(unsigned index) const {
	assert(index < 4);
	Vector4 result = matrix[index];
	return result;
}

Vector4 Matrix4::GetCol(unsigned index) const {
	assert(index < 4);
	Vector4 result;
	for (int i = 0; i < 4; ++i) {
		result.elem[i] = matrix[i][index];
	}
	return result;
}

Matrix3 Matrix4::GetMinor(unsigned rowIndex, unsigned colIndex) const {
	assert(rowIndex < 4 && colIndex < 4);
	Matrix3 result;
	for (unsigned i = 0, row = 0; i < 4; ++i) {
		if (i == rowIndex) {
			continue;
		}
		for (unsigned j = 0, col = 0; j < 4; ++j) {
			if (j == colIndex) {
				continue;
			}
			result.matrix[row].elem[col++] = this->matrix[i][j];
		}
		++row;
	}
	return result;
}

Matrix3 Matrix4::GetCofactor(unsigned rowIndex, unsigned colIndex) const {
	assert(rowIndex < 4 && colIndex < 4);
	Matrix3 result = this->GetMinor(rowIndex, colIndex);
	if ((rowIndex + colIndex) % 2 == 0) {
		return result;
	}
	else {
		return result * -1;
	}
}

Matrix4 Matrix4::GetIdentity() {
	Vector4 vec[4];
	for (int i = 0; i < 4; ++i) {
		vec[i].elem[i] = 1;
	}
	return Matrix4(vec[0], vec[1], vec[2], vec[3]);
}

Matrix4 Matrix4::Transpose() {
	Matrix4 result = *this;
	Vector4 cols[4];
	for (int i = 0; i < 4; ++i) {
		cols[i] = result.GetCol(i);
	}
	for (int i = 0; i < 4; ++i) {
		result.matrix[i] = cols[i];
	}
	return result;
}

Matrix4 Matrix4::Inverse() {
	Matrix4 result;
	double det = this->Det();
	if (det == 0) {
		std::cerr << "det = 0! NO INVERSE!" << std::endl;
		return result;
	}
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result.matrix[i].elem[j] = this->GetCofactor(i, j).Det() / det;
		}
	}
	return result;
}


/***************************************************************

MatrixT

*/
MatrixT::MatrixT(const Matrix3& mat, const Vector3& vec) {
	//Vector4 tmp[4];
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			matrix[i].elem[j] = mat[i][j];
		}
		matrix[i].elem[3] = vec[i];
	}
	matrix[3].elem[0] = matrix[3].elem[1] = matrix[3].elem[2] = 0;
	matrix[3].elem[3] = 1;

}
MatrixT::MatrixT() {}

MatrixT& MatrixT::operator=(const Matrix4& mat) {
	for (int i = 0; i < 4; ++i) {
		matrix[i] = mat[i];
	}
	return *this;
}

Matrix3 MatrixT::GetMatrix3() {
	Vector3 tmp[3];
	for (int i = 0; i < 3; ++i) {
		tmp[i] = this->matrix[i].GetSubVector(3);
	}
	return Matrix3(tmp[0], tmp[1], tmp[2]);
}

MatrixT MatrixT::Inverse() {
	Matrix3 mat3 = this->GetMatrix3();
	Vector3 vec3(this->matrix[0][3], this->matrix[1][3], this->matrix[2][3]);
	mat3 = mat3.Inverse();
	Vector3 tmp = mat3 * vec3;
	Vector3 vec(-tmp[0], -tmp[1], -tmp[2]);
	return MatrixT(mat3, vec);
}


/***************************************************************

Point

*/



std::ostream& operator<<(std::ostream& os, const Point p) {

	Vector3 tmp = p.GetXYZ();
	os << "(" << tmp[0] << ", "
		<< tmp[1] << ", "
		<< tmp[2] << ", "
		<< p.GetW() << ")";
	return os;
}

Point::Point() {}

Point::Point(const Vector3& XYZ) {
	xyz = XYZ;
	w = 1;
}

Point::Point(const Vector3& XYZ, double W) {
	xyz = XYZ;
	w = W;
}

Point::Point(double X, double Y, double Z, double W) : xyz(X, Y, Z) , w(W){

}

Vector3 Point::GetXYZ() const {
	return xyz;
}

double Point::GetW() const {
	return w;
}


/***************************************************************

CoordinateSystem

*/

std::ostream& operator<<(std::ostream& os, const CoordinateSystem& coord) {
	os << "X: " << coord.GetX() << std::endl
		<< "Y: " << coord.GetY() << std::endl
		<< "Z: " << coord.GetZ() << std::endl
		<< "O: " << coord.GetO() << std::endl;
	return os;
}

CoordinateSystem::CoordinateSystem() {

}

CoordinateSystem::CoordinateSystem(const Vector3& X, const Vector3& Y, const Vector3& Z, const Vector3& O) {
	x = X.Normalized();
	y = Y.Normalized();
	z = Z.Normalized();
	o = O;
}

Vector3 CoordinateSystem::GetO() const { return o;}

Vector3 CoordinateSystem::GetX() const {return x;}

Vector3 CoordinateSystem::GetY() const {return y;}

Vector3 CoordinateSystem::GetZ() const {return z;}




/***************************************************************

Quaternion

*/

/*
std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
	std::cout << q;
}
	*/

Quaternion& Quaternion::operator=(const Vector4& vec) {
	for (int i = 0; i < 4; ++i) {
		elem[i] = vec[i];
	}
	return *this;
}

Quaternion& Quaternion::operator=(const VectorBase& vec) {
	for (int i = 0; i < 4; ++i) {
		elem[i] = vec[i];
	}
	return *this;
}

Quaternion::Quaternion() {}

Quaternion::Quaternion(const Vector3& V, double S) {
	elem[0] = S;
	elem[1] = V[0]; 
	elem[2] = V[1];
	elem[3] = V[2];
}

Quaternion Quaternion::GetConjugate() {
	Quaternion result = *this;
	result.elem[1] *= -1;
	result.elem[2] *= -1;
	result.elem[3] *= -1;
	return result;
}

Quaternion Quaternion::Inverse() {
	Quaternion result, conj = this->GetConjugate();
	double n2 = MyMath::Dot(*this, *this);
	result = conj / n2;
	return result;
}

