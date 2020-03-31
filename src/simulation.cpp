#include "simulation.h"

#include <iostream>
#include <fstream>
#include <unordered_set>

#include "graphics/sphere.h"
#include "graphics/MeshLoader.h"

#include "clsettings.h"

using namespace Eigen;
using namespace std;

Simulation::Simulation() {
}

Simulation::~Simulation() {
    delete solver;
    delete system;
}

void Simulation::init()
{
    // STUDENTS: This code loads up the tetrahedral mesh in 'example-meshes/single-tet.mesh'
    //    (note: your working directory must be set to the root directory of the starter code
    //    repo for this file to load correctly). You'll probably want to instead have this code
    //    load up a tet mesh based on e.g. a file path specified with a command line argument.
    std::vector<Vector3f> vertices;
    std::vector<Vector4i> tets;
    if(MeshLoader::loadTetMesh(ConfigStore::getMeshFilename(), vertices, tets)) {
        // STUDENTS: This code computes the surface mesh of the loaded tet mesh, i.e. the faces
        //    of tetrahedra which are on the exterior surface of the object. Right now, this is
        //    hard-coded for the single-tet mesh. You'll need to implement surface mesh extraction
        //    for arbitrary tet meshes. Think about how you can identify which tetrahedron faces
        //    are surface faces...
        std::vector<Vector3i> faces = computeSurfaceFaces(tets, vertices);
        m_shape.init(vertices, faces, tets);

        system = new ParticleSystem(tets, vertices, 100.f, 100.f, 2.f, 2.f, 1.0f, 9.8f);
        system->addCollider(new SphereCollider({0, -1000, 0}, 998.f));
        system->addCollider(new SphereCollider({1.f, -1.4f, 0}, 0.6f));

        solver = new Integrator(system, vertices, faces);
    }
    m_shape.setModelMatrix(Affine3f(Eigen::Translation3f(0, 2, 0)));

    std::vector<Vector3f> sphereVerts;
    std::vector<Vector3i> sphereFaces;
    for (int i = 0; i < 200; i++) {
        sphereVerts.emplace_back(2 * Vector3f{SPHERE_POS[i * 9 + 0], SPHERE_POS[i * 9 + 1], SPHERE_POS[i * 9 + 2]});
        sphereVerts.emplace_back(2 * Vector3f{SPHERE_POS[i * 9 + 3], SPHERE_POS[i * 9 + 4], SPHERE_POS[i * 9 + 5]});
        sphereVerts.emplace_back(2 * Vector3f{SPHERE_POS[i * 9 + 6], SPHERE_POS[i * 9 + 7], SPHERE_POS[i * 9 + 8]});
        sphereFaces.emplace_back(Vector3i{i * 3, i * 3 + 1, i * 3 + 2});
    }
    m_sphere.init(sphereVerts, sphereFaces);
    m_sphere.setModelMatrix(Affine3f(Translation3f(1.f, -1.4f + 2, 0) * Scaling(0.6f)));


    initGround();
}

void Simulation::update(float seconds, float power, Vector3f rayO, Vector3f rayD) {
    // STUDENTS: This method should contain all the time-stepping logic for your simulation.
    //   Specifically, the code you write here should compute new, updated vertex positions for your
    //   simulation mesh, and it should then call m_shape.setVertices to update the display with those
    //   newly-updated vertices.

    // STUDENTS: As currently written, the program will just continually compute simulation timesteps as long
    //    as the program is running (see View::tick in view.cpp) . You might want to e.g. add a hotkey for pausing
    //    the simulation, and perhaps start the simulation out in a paused state.

    solver->step(seconds, power, rayO, rayD);
    m_shape.setVertices(solver->positions());
}

void Simulation::draw(Shader *shader) {
    m_shape.draw(shader);
    m_ground.draw(shader);
    m_sphere.draw(shader);
}

void Simulation::write(QString filename) {
    std::ofstream out;
    out.open(filename.toStdString());
    for (Vector3f pos : solver->positions()) {
        out << "v " << pos[0] << " " << pos[2] << " " << pos[1] << endl;
    }
    for (Vector3i face : m_shape.faces()) {
        out << "f " << (1 + face[0]) << " " << (1 + face[1]) << " " << (1 + face[2]) << endl;
    }
    out.flush();
    out.close();
}

void Simulation::toggleWire() {
    m_shape.toggleWireframe();
}

void Simulation::initGround() {
    std::vector<Vector3f> groundVerts;
    std::vector<Vector3i> groundFaces;
    groundVerts.emplace_back(-5, 0, -5);
    groundVerts.emplace_back(-5, 0, 5);
    groundVerts.emplace_back(5, 0, 5);
    groundVerts.emplace_back(5, 0, -5);
    groundFaces.emplace_back(0, 1, 2);
    groundFaces.emplace_back(0, 2, 3);
    m_ground.init(groundVerts, groundFaces);
}

Vector3i sortedTuple(Vector3i tuple) {
    int mn = min(min(tuple[0], tuple[1]), tuple[2]);
    int mx = max(max(tuple[0], tuple[1]), tuple[2]);
    int md = tuple[0] + tuple[1] + tuple[2] - mn - mx;
    return {mn, md, mx};
}

Vector3i orientOutwards(Vector3i inds, int extra, const std::vector<Vector3f> &locs) {
    Vector3f v1 = locs[inds[1]] - locs[inds[0]];
    Vector3f v2 = locs[inds[2]] - locs[inds[0]];
    Vector3f e = locs[extra] - locs[inds[0]];

    if (v1.cross(v2).dot(e) > 0) {
        return {inds[0], inds[2], inds[1]};
    } else {
        return inds;
    }
}

struct vector_set_hash : std::unary_function<Vector3i, size_t> {
  std::size_t operator()(Vector3i const& vec) const {
    Vector3i sorted = sortedTuple(vec);
    size_t seed = 0;
    for (int i = 0; i < 3; ++i) {
      int elem = sorted[i];
      seed ^= std::hash<int>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

struct vector_set_eq {
public:
    bool operator()(const Vector3i & v1, const Vector3i & v2) const {
        return sortedTuple(v1) == sortedTuple(v2);
    }
};

std::vector<Vector3i> Simulation::computeSurfaceFaces(const std::vector<Vector4i> &tets, const std::vector<Vector3f> &locs) {
    unordered_set<Vector3i, vector_set_hash, vector_set_eq> faces;

    for (Vector4i tet : tets) {
        Vector3i f1 = orientOutwards({tet[0], tet[1], tet[2]}, tet[3], locs);
        if (faces.count(f1) > 0) { faces.erase(f1); } else { faces.insert(f1); }
        Vector3i f2 = orientOutwards({tet[0], tet[1], tet[3]}, tet[2], locs);
        if (faces.count(f2) > 0) { faces.erase(f2); } else { faces.insert(f2); }
        Vector3i f3 = orientOutwards({tet[0], tet[2], tet[3]}, tet[1], locs);
        if (faces.count(f3) > 0) { faces.erase(f3); } else { faces.insert(f3); }
        Vector3i f4 = orientOutwards({tet[1], tet[2], tet[3]}, tet[0], locs);
        if (faces.count(f4) > 0) { faces.erase(f4); } else { faces.insert(f4); }
    }

    std::vector<Vector3i> surfaceFaces{faces.begin(), faces.end()};
    return surfaceFaces;
}
