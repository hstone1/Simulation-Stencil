#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <particlesystem.h>

class Integrator
{
public:
    Integrator(ParticleSystem *ps, std::vector<Eigen::Vector3f> positions,
               std::vector<Eigen::Vector3i> &tris);
    const std::vector<Eigen::Vector3f> &positions() const;
    void step(float time, float power, Eigen::Vector3f rayO, Eigen::Vector3f rayD);

private:
    std::vector<Eigen::Vector3f> pos;
    std::vector<Eigen::Vector3f> vel;
    std::vector<Eigen::Vector3i> m_tris;

    ParticleSystem *system_;
};


#endif // INTEGRATOR_H
