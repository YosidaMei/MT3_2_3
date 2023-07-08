#include "Function.h"

void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	const float kGridHalfWidth = 2.0f;//グリッドの半分の幅
	const uint32_t kSubivision = 10;//分割数
	const float kGridEvery = (kGridHalfWidth * 2.0f) / float(kSubivision);
	//左から右に線を引く
	for (uint32_t zIndex = 0; zIndex <= kSubivision; ++zIndex) {
		Vector3 startPos = { kGridHalfWidth,0, kGridEvery * zIndex - kGridHalfWidth };
		Vector3 endPos = { -kGridHalfWidth,0, kGridEvery * zIndex - kGridHalfWidth };
		//スクリーン座標に変換
		Vector3 startNdcVertex = Transform(startPos, viewProjectionMatrix);
		startPos = Transform(startNdcVertex, viewportMatrix);
		Vector3 endNdcVertex = Transform(endPos, viewProjectionMatrix);
		endPos = Transform(endNdcVertex, viewportMatrix);
		//描画
		if (kGridEvery * zIndex - kGridHalfWidth != 0) {
			Novice::DrawLine(
				int(startPos.x), int(startPos.y),
				int(endPos.x), int(endPos.y), 0xAAAAAAFF);
		}
		else {
			Novice::DrawLine(
				int(startPos.x), int(startPos.y),
				int(endPos.x), int(endPos.y), 0x000000FF);
		}

	}
	//奥から手前に線を引く
	for (uint32_t xIndex = 0; xIndex <= kSubivision; ++xIndex) {
		Vector3 startPos = { kGridEvery * xIndex - kGridHalfWidth ,0,kGridHalfWidth };
		Vector3 endPos = { kGridEvery * xIndex - kGridHalfWidth,0,-kGridHalfWidth };
		//スクリーン座標に変換
		Vector3 startNdcVertex = Transform(startPos, viewProjectionMatrix);
		startPos = Transform(startNdcVertex, viewportMatrix);
		Vector3 endNdcVertex = Transform(endPos, viewProjectionMatrix);
		endPos = Transform(endNdcVertex, viewportMatrix);
		//描画
		if (kGridEvery * xIndex - kGridHalfWidth != 0) {
			Novice::DrawLine(
				int(startPos.x), int(startPos.y),
				int(endPos.x), int(endPos.y), 0xAAAAAAFF);
		}
		else {
			Novice::DrawLine(
				int(startPos.x), int(startPos.y),
				int(endPos.x), int(endPos.y), 0x000000FF);
		}
	}
}

void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	const uint32_t kSubdivision = 20;
	const float kLonEvery = 2 * (float)M_PI / (float)kSubdivision;
	const float kLatEvery = (float)M_PI / (float)kSubdivision;
	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex) {
		float  lat = -1 * (float)M_PI / 2 + kLatEvery * latIndex;
		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex) {
			float lon = lonIndex * kLonEvery;
			Vector3 a, b, c;
			a = { sphere.radius * std::cos(lat) * std::cos(lon), sphere.radius * std::sin(lat),  sphere.radius * std::cos(lat) * std::sin(lon) };
			a = Add(a, sphere.center);
			b = { sphere.radius * std::cos(lat + kLatEvery) * std::cos(lon),sphere.radius * std::sin(lat + kLatEvery),sphere.radius * std::cos(lat + kLatEvery) * std::sin(lon) };
			b = Add(b, sphere.center);
			c = { sphere.radius * std::cos(lat) * std::cos(lon + kLonEvery),sphere.radius * std::sin(lat),sphere.radius * std::cos(lat) * std::sin(lon + kLonEvery) };
			c = Add(c, sphere.center);
			//スクリーン座標変換
			a = Transform(a, viewProjectionMatrix);
			a = Transform(a, viewportMatrix);
			b = Transform(b, viewProjectionMatrix);
			b = Transform(b, viewportMatrix);
			c = Transform(c, viewProjectionMatrix);
			c = Transform(c, viewportMatrix);
			//描画
			Novice::DrawLine((int)a.x, (int)a.y, (int)b.x, (int)b.y, color);
			Novice::DrawLine((int)a.x, (int)a.y, (int)c.x, (int)c.y, color);
		}
	}
}

Vector3 Project(const Vector3& v1, const Vector3& v2) {
	Vector3 v3, v4;
	float product;
	v4 = Normalize(v2);
	product = (v1.x * v4.x) + (v1.y * v4.y) + (v1.z * v4.z);
	v3 = Scaler(product, v4);
	return v3;
}

Vector3 ClosestPoint(const Vector3& point, const Segment& segment) {
	Vector3 cp;
	cp = Add(segment.origin, point);
	return cp;
}

float Lengh(const Vector3& v) {
	float length;
	length = (v.x * v.x) + (v.y * v.y) + (v.z * v.z);
	return length;
}

//球の衝突判定
bool IsCollisionS2S(const Sphere& s1, const Sphere& s2) {
	//二つの球の中心間の距離を求める
	float distance = Lengh(Subtract(s1.center, s2.center));
	//半径の合計よりも短ければ衝突
	if (distance <= s1.radius + s2.radius) {
		return true;
	}
	else
	{
		return false;
	}

}
//内積
float InnerProduct(const Vector3& v1, const Vector3& v2) {
	float product;
	product = (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
	return product;
}

//クロス積
Vector3 Cross(const Vector3& v1, const Vector3& v2) {
	Vector3 cross;
	cross.x = (v1.y * v2.z - v1.z * v2.y);
	cross.y = (v1.z * v2.x - v1.x * v2.z);
	cross.z = (v1.x * v2.y - v1.y * v2.x);
	return cross;
}

//球と平面の衝突判定
bool IsCollisionS2P(const Sphere& sphere, Plane& plane) {
	float length, d;
	Vector3 n;
	n = Normalize(plane.normal);
	//球の距離を求める
	d = InnerProduct(n, sphere.center);
	length = d - plane.distance;
	//当たったら
	if (abs(length) <= sphere.radius) {
		return true;
	}
	else
	{
		return false;
	}
}

Vector3 Perpendicular(const Vector3& vector) {
	if (vector.x != 0.0f || vector.y != 0.0f) {
		return { -vector.y,vector.x,0.0f };
	}
	return { 0.0f,-vector.z,vector.y };
}

//平面の描画
void DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Vector3 center = Scaler(plane.distance, plane.normal);
	Vector3 perpendiculars[4];
	perpendiculars[0] = Normalize(Perpendicular(plane.normal));
	perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y,-perpendiculars[0].z };
	perpendiculars[2] = Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };
	Vector3 points[4];
	for (int32_t index = 0; index < 4; ++index) {
		Vector3 extend = Scaler(2.0f, perpendiculars[index]);
		Vector3 point = Add(center, extend);
		points[index] = Transform(Transform(point, viewProjectionMatrix), viewportMatrix);
	}
	Novice::DrawLine(points[0].x, points[0].y, points[2].x, points[2].y, WHITE);
	Novice::DrawLine(points[1].x, points[1].y, points[2].x, points[2].y, WHITE);
	Novice::DrawLine(points[1].x, points[1].y, points[3].x, points[3].y, WHITE);
	Novice::DrawLine(points[3].x, points[3].y, points[0].x, points[0].y, WHITE);

}