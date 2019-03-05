#pragma once
#include <iostream>
#include <cmath>
#include <string>

/*
*
*
Vector
*
*
*/

class VectorBase {
public:
	friend std::ostream &operator<<(std::ostream &os, const VectorBase &vec);
	VectorBase operator+(const VectorBase& vec);
	VectorBase operator-(const VectorBase& vec);
	VectorBase operator*(const VectorBase& vec);
	VectorBase operator/(const VectorBase& vec);
	VectorBase operator+(double num);
	VectorBase operator-(double num);
	VectorBase operator*(double num);
	VectorBase operator/(double num);
	bool operator==(const VectorBase& vec);

	double operator[](int n) const;
	VectorBase &operator=(const VectorBase &vec);

	VectorBase();
	VectorBase Normalized() const;

	double elem[4];
	//double *elem;
	inline double GetElemNum() const;

	virtual ~VectorBase();
	
protected:
	int elemNum;
};

std::ostream &operator<<(std::ostream &os, const VectorBase &vec);



/*
*
*
Vector2
*
*
*/
class Vector2 : public VectorBase {
public:
	Vector2();
	Vector2(double x, double y);
	inline double GetX() const;
	inline double GetY() const;
	Vector2 &operator=(const VectorBase &vec);
	Vector2 &operator=(const Vector2 &vec);

	virtual ~Vector2();

};

/*
*
*
Vector3
*
*
*/
class Vector3 : public VectorBase {
public:
	Vector3();
	Vector3(double x, double y, double z);
	inline double GetX() const;
	inline double GetY() const;
	inline double GetZ() const;
	Vector3 &operator=(const VectorBase &vec);

};

/*
*
*
Vector4
*
*
*/
class Vector4 : public VectorBase {
public:
	Vector4();
	Vector4(double x, double y, double z, double w);
	inline double GetX() const;
	inline double GetY() const;
	inline double GetZ() const;
	inline double GetW() const;
	Vector4 &operator=(const VectorBase &vec);
	Vector3 GetSubVector(unsigned index);
};



/*
*
*
Matrix2
*
*
*/
class Matrix2 {
public:
	friend std::ostream& operator<<(std::ostream& os, const Matrix2 mat);
	Vector2 operator[](unsigned index) const;
	Matrix2 operator+(double num);
	Matrix2 operator-(double num);
	Matrix2 operator*(double num);
	Matrix2 operator/(double num);
	Matrix2 operator*(const Matrix2& mat);
	bool operator==(const Matrix2& mat);
	Matrix2();
	Matrix2(const Vector2& v1, const Vector2& v2);
	virtual ~Matrix2();

	double Determinant();
	double Det();
	Matrix2 Transpose();
	Matrix2 Inverse();
	Vector2 GetRow(unsigned index) const;
	Vector2 GetCol(unsigned index) const;
	static Matrix2 GetIdentity();

	Vector2 *matrix;
private:


};
std::ostream& operator<<(std::ostream& os, const Matrix2 mat);


/*
*
*
Matrix3
*
*
*/
class Matrix3 {
public:
	friend std::ostream& operator<<(std::ostream& os, const Matrix3 mat);
	Vector3 operator[](unsigned index) const;
	Matrix3 operator+(double num);
	Matrix3 operator-(double num);
	Matrix3 operator*(double num);
	Matrix3 operator/(double num);
	Matrix3 operator*(const Matrix3& mat);
	Vector3 operator*(const Vector3& vec);
	bool operator==(const Matrix3& mat);
	Matrix3();
	Matrix3(const Vector3& v1, const Vector3& v2, const Vector3& v3);

	double Determinant();
	double Det();
	Matrix2 GetMinor(unsigned rowIndex, unsigned colIndex);
	Matrix2 GetCofactor(unsigned rowIndex, unsigned colIndex);
	Matrix3 Transpose();
	Matrix3 Inverse();
	Vector3 GetRow(unsigned index) const;
	Vector3 GetCol(unsigned index) const;

	static Matrix3 GetIdentity();
	


	Vector3 matrix[3];
private:
	

};
std::ostream& operator<<(std::ostream& os, const Matrix3 mat);

/*
*
*
Matrix4
*
*
*/
class Matrix4 {
public:
	friend std::ostream& operator<<(std::ostream& os, const Matrix4 mat);
	Vector4 operator[](unsigned index) const;
	Matrix4 operator+(double num);
	Matrix4 operator-(double num);
	Matrix4 operator*(double num);
	Matrix4 operator/(double num);
	Matrix4 operator*(const Matrix4& mat);
	Vector4 operator*(const Vector4& vec);
	bool operator==(const Matrix4& mat);
	Matrix4();
	Matrix4(const Vector4& v1, const Vector4& v2, const Vector4& v3, const Vector4& v4);
	

	double Determinant();
	double Det();

	Matrix3 GetMinor(unsigned rowIndex, unsigned colIndex) const;
	Matrix3 GetCofactor(unsigned rowIndex, unsigned colIndex) const;
	Matrix4 Transpose();
	Matrix4 Inverse();
	Vector4 GetRow(unsigned index) const;
	Vector4 GetCol(unsigned index) const;
	static Matrix4 GetIdentity();

	Vector4 matrix[4];
private:


};
std::ostream& operator<<(std::ostream& os, const Matrix4 mat);

class MatrixT : public Matrix4{
public:
	MatrixT& operator=(const Matrix4& mat);
	Matrix3 GetMatrix3();
	MatrixT(const Matrix3& mat, const Vector3& vec);
	MatrixT();
	MatrixT Inverse();
};

/*
*
*
Point
*
*
*/
class Point {
	
public:
	friend std::ostream& operator<<(std::ostream& os, const Point p);

	Point();
	Point(const Vector3& XYZ);
	Point(const Vector3& XYZ, double W);
	Point(double X, double Y, double Z, double W);

	Vector3 GetXYZ() const;
	double GetW() const;
private:
	Vector3 xyz;
	double w;
};

std::ostream& operator<<(std::ostream& os, const Point p);


/*
*
*
CoordinateSystem
*
*
*/
class CoordinateSystem {
public:
	friend std::ostream& operator<<(std::ostream& os, const CoordinateSystem& coord);

	CoordinateSystem();
	CoordinateSystem(const Vector3& X, const Vector3& Y, const Vector3& Z, const Vector3& O);
	
	Vector3 GetX() const;
	Vector3 GetY() const;
	Vector3 GetZ() const;
	Vector3 GetO() const;

	std::string name;
private:
	Vector3 x, y, z, o;

};
std::ostream& operator<<(std::ostream& os, const CoordinateSystem& coord);


/*
*
*
Quaternion
*
*
*/
class Quaternion :public Vector4 {
public:
	//friend std::ostream& operator<<(std::ostream& os, const Quaternion& q);
	Quaternion& operator=(const Vector4& vec);
	Quaternion& operator=(const VectorBase& vec);

	Quaternion();
	Quaternion(const Vector3& V, double S);
	Quaternion GetConjugate();
	Quaternion Inverse();

private:

};
//std::ostream& operator<<(std::ostream& os, const Quaternion& q);



/*
*
*
MyMath
*
*
*/
class MyMath {
public:
	static void Hello();
	static double NormL1(const VectorBase& vec);
	static double NormL2(const VectorBase& vec);
	static double Dot(const VectorBase& vec1, const VectorBase& vec2);
	static Vector3 Cross(const Vector3& vec1, const Vector3& vec2);
	static Quaternion QuaternionMul(const Quaternion& q1, const Quaternion& q2);

	static double Determinant(const Matrix2& mat);
	static double Determinant(const Matrix3& mat);
	static double Determinant(const Matrix4& mat);

	static double Deg2Rad(const double deg);
	static double Rad2Deg(const double rad);

	static Matrix3 GetRotationMatrix3AroundX(double deg);
	static Matrix3 GetRotationMatrix3AroundY(double deg);
	static Matrix3 GetRotationMatrix3AroundZ(double deg);
	static MatrixT GetRotationMatrixAroundX(double deg);
	static MatrixT GetRotationMatrixAroundY(double deg);
	static MatrixT GetRotationMatrixAroundZ(double deg);

	static Matrix3 GetRotationMatrix3Around(const Vector3& axis, double deg);
	static MatrixT GetRotationMatrixAround(const Vector3& axis, double deg);
	static Quaternion GetQuaternionAround(const Vector3& axis, double deg);
	static Vector3 Rotate(const Vector3& vec, const Vector3& axis, double deg);

	static MatrixT GetMoveMatrix(const Vector3& offset);

	static MatrixT CoordToWorldMatrix(const CoordinateSystem& coord);
	static MatrixT WorldToCoordMatrix(const CoordinateSystem& coord);


};
