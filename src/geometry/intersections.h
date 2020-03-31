#ifndef INTERSECTIONS_H
#define INTERSECTIONS_H

#include <Eigen/Dense>

namespace Geometry {

typedef Eigen::Vector3f Vec3;
typedef Eigen::Vector4f Vec4;
typedef Eigen::Matrix4f Mat4;
typedef Eigen::Matrix3f Mat3;


float rayTriangleIntersect(
    const Vec3 &orig, const Vec3 &dir,
    const Vec3 &v0, const Vec3 &v1, const Vec3 &v2);

}



#endif // INTERSECTIONS_H
