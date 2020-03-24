#ifndef SIMULATION_H
#define SIMULATION_H

#include "graphics/shape.h"

#include "integrator.h"
#include "particlesystem.h"

class Shader;

class Simulation
{
public:
    Simulation();
    ~Simulation();

    void init();

    void update(float seconds);

    void draw(Shader *shader);

    void toggleWire();
private:
    Shape m_shape;
    Shape m_ground;
    void initGround();

private:
    ParticleSystem *system;
    Integrator *solver;

private:
    static std::vector<Eigen::Vector3i> computeSurfaceFaces(const std::vector<Eigen::Vector4i> &tets, const std::vector<Eigen::Vector3f> &locs);
};

#endif // SIMULATION_H
