#ifndef COLLIDER_H
#define COLLIDER_H

#include <Eigen/Core>

class Collider {
public:
    virtual ~Collider() = default;
    virtual Eigen::Vector3f dirToOutside(Eigen::Vector3f pos) = 0;
};

class SphereCollider : public Collider {
public:
    SphereCollider(Eigen::Vector3f center, float radius);
    Eigen::Vector3f dirToOutside(Eigen::Vector3f pos);

private:
    float m_radius;
    Eigen::Vector3f m_center;
};



#endif // COLLIDER_H
