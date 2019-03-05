#include "MyMath.h"
#include <crtdbg.h>
#include <iostream>
#include <iomanip>

using namespace std;
using ::MyMath;

int main() {
	
	MyMath::Hello();	
	
	
	Vector2 v2(1, 2), v2Test(2, 3);
	Matrix2 mat2();
	Vector3 v3{ 1, 2, 3 }, v3Test;
	Vector3 axisX(1, 0, 0), axisY(0, 1, 0), axisZ(0, 0, 1), O(0, 0, 0);
	
	Vector4 v4(1, 0, 0, 1), v44(0, 0, 2, 0), v444(3, 5, 5, 6), v4444(4, 3, 11, 1);
	Matrix3 mat3(v3, v3, axisX);
	Matrix4 mat4(v4, v44, v444, v4444);
	cout << fixed << setprecision(2);
	v2Test = v2 * 5;
	Point p(v3, 1);
	MatrixT matT = MyMath::GetRotationMatrixAround(v3, 90);
	CoordinateSystem coord(axisX, axisY, axisZ, O);
	Quaternion q(v3, 1);
	VectorBase v;
	v3Test = MyMath::Rotate(axisY, axisX, 90);
	q = MyMath::GetQuaternionAround(axisX, 90);
	//v = MyMath::GetRotationMatrixAroundX(90) * v44;
	cout << v3Test << endl;
	//cout << q.Normalized() << endl;
	system("pause");
	return 0;
}