#include <iostream>
#include <vector>

#include <taser/trajectories/uniform_r3_spline_trajectory.h>

using namespace taser::trajectories;

int main() {
  std::cout << "Begin" << std::endl;

  auto traj = std::make_shared<UniformR3SplineTrajectory>();

  std::cout << "dt=" << traj->dt() << ", t0=" << traj->t0();
  std::cout << " nknots=" << traj->NumKnots() << std::endl;
  std::cout << "End OK" << std::endl;
  return 0;
}