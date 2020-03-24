#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <particlesystem.h>

class Integrator
{
public:
    Integrator(ParticleSystem *ps, std::vector<Eigen::Vector3f> positions);
    const std::vector<Eigen::Vector3f> &positions() const;
    void step(float time);

private:
    std::vector<Eigen::Vector3f> pos;
    std::vector<Eigen::Vector3f> vel;
    ParticleSystem *system_;
};


#endif // INTEGRATOR_H
