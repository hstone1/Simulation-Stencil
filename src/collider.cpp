#include "collider.h"

SphereCollider::SphereCollider(Eigen::Vector3f center, float radius) : m_radius(radius), m_center(center) {

}

Eigen::Vector3f SphereCollider::dirToOutside(Eigen::Vector3f pos) {
    float dist = (pos - m_center).norm();
    if (dist >= m_radius) {
        return {0.0, 0.0, 0.0};
    } else {
        return (pos - m_center) * ((m_radius - dist) / dist);
    }
}
