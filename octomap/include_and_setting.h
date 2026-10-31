#ifndef INCLUDE_AND_SETTING_H
#define INCLUDE_AND_SETTING_H

#ifndef OCTOMAP_DEPRECATED
#define insertScan insertPointCloud
#endif
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

namespace octomap_python {

template <class Tree>
void expandedPointCounts(Tree* tree, size_t& occupied, size_t& free) {
  occupied = free = 0;
  for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
    const size_t dimension = size_t(1) << (tree->getTreeDepth() - it.getDepth());
    (tree->isNodeOccupied(*it) ? occupied : free) += dimension * dimension * dimension;
  }
}

template <class Tree>
void extractExpandedPoints(Tree* tree, double* occupied, double* free) {
  // Match extractPointCloud's historical float32 resolution arithmetic.
  const float resolution = tree->getResolution();
  for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
    const size_t dimension = size_t(1) << (tree->getTreeDepth() - it.getDepth());
    const auto center = it.getCoordinate();
    const double offset = (dimension / 2.0 - 0.5) * resolution;
    const double ox = center.x() - offset;
    const double oy = center.y() - offset;
    const double oz = center.z() - offset;
    double*& target = tree->isNodeOccupied(*it) ? occupied : free;
    for (size_t x = 0; x < dimension; ++x) {
      for (size_t y = 0; y < dimension; ++y) {
        for (size_t z = 0; z < dimension; ++z) {
          *target++ = ox + x * double(resolution);
          *target++ = oy + y * double(resolution);
          *target++ = oz + z * double(resolution);
        }
      }
    }
  }
}

}  // namespace octomap_python

#endif
