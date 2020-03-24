#include "integrator.h"

using namespace std;
using namespace Eigen;

Integrator::Integrator(ParticleSystem *ps, std::vector<Eigen::Vector3f> positions) : pos(positions.begin(), positions.end()), system_(ps) {
    vel.reserve(pos.size());
    for (ulong i = 0;i < pos.size(); i++) {
        vel.push_back({0, 0, 0});
    }
}

const std::vector<Eigen::Vector3f> &Integrator::positions() const {
    return pos;
}

void Integrator::step(float dT) {
    vector<Vector3f> acc(pos.size());
    system_->computeAcceleration(pos, vel, acc);
    for (ulong i = 0;i < pos.size(); i++) {
        pos[i] += vel[i];
        vel[i] += acc[i];
    }
}
