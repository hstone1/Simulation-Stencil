#include "intersections.h"

namespace Geometry {


/*
 * Modification of the triangle ray intersection method presented here. To provide the intersection location and work
 * with Eigen.
 *
 * https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
 */
float rayTriangleIntersect(
    const Vec3 &orig, const Vec3 &dir,
    const Vec3 &v0, const Vec3 &v1, const Vec3 &v2)
{
    // compute plane's normal
    Vec3 v0v1 = v1 - v0;
    Vec3 v0v2 = v2 - v0;
    Vec3 N = v0v1.cross(v0v2); // N

    float t =  N.dot(v0 - orig) / N.dot(dir);
    // check if the triangle is in behind the ray
    if (t < 0) return -3; // the triangle is behind

    // compute the intersection point using equation 1
    Vec3 P = orig + t * dir;

    // Step 2: inside-outside test
    Vec3 C; // vector perpendicular to triangle's plane

    // edge 0
    Vec3 edge0 = v1 - v0;
    Vec3 vp0 = P - v0;
    C = edge0.cross(vp0);
    if (N.dot(C) < 0) return -4;

    // edge 1
    Vec3 edge1 = v2 - v1;
    Vec3 vp1 = P - v1;
    C = edge1.cross(vp1);
    if (N.dot(C) < 0)  return -5;

    // edge 2
    Vec3 edge2 = v0 - v2;
    Vec3 vp2 = P - v2;
    C = edge2.cross(vp2);
    if (N.dot(C) < 0) return -6;

    return t; // this ray hits the triangle
}
}
