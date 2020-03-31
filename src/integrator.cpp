#include "integrator.h"
#include "geometry/intersections.h"
#include <iostream>

using namespace std;
using namespace Eigen;

Integrator::Integrator(ParticleSystem *ps,
                       vector<Vector3f> positions,
                       vector<Vector3i> &tris) : pos(positions.begin(), positions.end()), system_(ps), m_tris(tris.begin(), tris.end()) {
    vel.reserve(pos.size());
    for (ulong i = 0;i < pos.size(); i++) {
        vel.push_back({0, 0, 0});
    }
}

const std::vector<Eigen::Vector3f> &Integrator::positions() const {
    return pos;
}

void Integrator::step(float dT, bool down, Vector3f rayO, Vector3f rayD) {
    if (down) {
        Vector3i *bestTri = nullptr;
        float bestTime = 10000000;
        for (Vector3i &tri : m_tris) {

            float time = Geometry::rayTriangleIntersect(rayO, rayD, pos[tri[0]], pos[tri[1]], pos[tri[2]]);
            if (time > 0 && time < bestTime) {
                bestTri = &tri;
                bestTime = time;
            }
        }

        if (bestTime < 1000) {
            vel[bestTri->x()] -= rayD.normalized() * dT * 2000;
            vel[bestTri->y()] -= rayD.normalized() * dT * 2000;
            vel[bestTri->z()] -= rayD.normalized() * dT * 2000;

            cout << "hit" << endl;
        }
    }

    vector<Vector3f> acc1(pos.size());
    vector<Vector3f> acc2(pos.size());
    vector<Vector3f> acc3(pos.size());
    vector<Vector3f> acc4(pos.size());

    vector<Vector3f> pos1(pos.begin(), pos.end());
    vector<Vector3f> vel1(vel.begin(), vel.end());
    vector<Vector3f> avel(vel.begin(), vel.end());

    // RK4
    system_->computeAcceleration(pos, vel, acc1);
    for (ulong i = 0;i < pos.size(); i++) {
        pos[i] = pos1[i] + 0.5 * vel[i] * dT;
        vel[i] = vel1[i] + 0.5 * acc1[i] * dT;
        avel[i] += 2 * vel[i];
    }

    system_->computeAcceleration(pos, vel, acc2);
    for (ulong i = 0;i < pos.size(); i++) {
        pos[i] = pos1[i] + 0.5 * vel[i] * dT;
        vel[i] = vel1[i] + 0.5 * acc2[i] * dT;
        avel[i] += 2 * vel[i];
    }

    system_->computeAcceleration(pos, vel, acc3);
    for (ulong i = 0;i < pos.size(); i++) {
        pos[i] = pos1[i] + vel[i] * dT;
        vel[i] = vel1[i] + acc3[i] * dT;
        avel[i] += vel[i];
    }

     system_->computeAcceleration(pos, vel, acc4);
     for (ulong i = 0;i < pos.size(); i++) {
         pos[i] = pos1[i] + avel[i] / 6.f * dT;
         vel[i] = vel1[i] + (acc1[i] + 2 * acc2[i] + 2 * acc3[i] + acc4[i]) / 6.0f * dT;
     }



}
