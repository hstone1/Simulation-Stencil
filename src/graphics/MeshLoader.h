#ifndef MESHLOADER_H
#define MESHLOADER_H

#include "HEigenLib.h"
#include <vector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4i)

class MeshLoader
{
public:
    static bool loadTetMesh(const std::string &filepath, std::vector<Eigen::Vector3f> &vertices, std::vector<Eigen::Vector4i> &tets);
private:
    MeshLoader();
};

#endif // MESHLOADER_H
