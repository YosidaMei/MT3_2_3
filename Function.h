#pragma once
#include "Vector3.h"
#include <Novice.h>
#include "Matrix.h"


class Function
{
};

struct Sphere {
	Vector3 center;
	float radius;
};
struct Line
{
	Vector3 origin;
	Vector3 diff;
};
struct Ray
{
	Vector3 origin;
	Vector3 diff;
};
struct Segment
{
	Vector3 origin;
	Vector3 diff;
};
struct Plane
{
	Vector3 normal;
	float distance;
};


void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix);
void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color);
Vector3 Project(const Vector3& v1, const Vector3& v2);
Vector3 ClosestPoint(const Vector3& point, const Segment& segment);
float Lengh(const Vector3& v);
bool IsCollisionS2S(const Sphere& s1, const Sphere& s2);
float InnerProduct(const Vector3& v1, const Vector3& v2);
bool IsCollisionS2P(const Sphere& sphere, Plane& plane);
void DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color);
Vector3 Perpendicular(const Vector3& vector);
Vector3 Cross(const Vector3& v1, const Vector3& v2);
