#pragma once

#include <array>
#include <vector>
#include <Eigen/Geometry>

#include "abb_irb4600_ikfast/abb_irb4600_ikfast.h"

namespace block1_stanislavskyi
{

struct TrajectoryPoint
{
  double t;
  ikfast_abb::JointValues q;
  ikfast_abb::JointValues dq;
  ikfast_abb::JointValues ddq;
};

class Manipulator
{
public:
  Manipulator();

  std::vector<TrajectoryPoint> generatePTP(
    const ikfast_abb::JointValues & q_start,
    const ikfast_abb::JointValues & q_goal,
    double duration,
    double dt) const;

  std::vector<TrajectoryPoint> generatePTPVia(
    const ikfast_abb::JointValues & q_start,
    const ikfast_abb::JointValues & q_via,
    const ikfast_abb::JointValues & q_goal,
    double duration1,
    double duration2,
    double dt) const;

  std::vector<TrajectoryPoint> generateLIN(
    const ikfast_abb::JointValues & q_start,
    const Eigen::Affine3d & target_pose,
    double duration,
    double dt) const;

  Eigen::Affine3d offsetAlongToolX(
    const ikfast_abb::JointValues & q,
    double offset) const;
    
  bool chooseBestSolution(
    const ikfast_abb::Solutions & solutions,
    const ikfast_abb::JointValues & q_prev,
    ikfast_abb::JointValues & q_best) const;

private:
  struct QuinticCoeffs
  {
    double a0, a1, a2, a3, a4, a5;
  };

  QuinticCoeffs computeQuintic(
    double q0, double qT,
    double v0, double vT,
    double acc0, double accT,
    double T) const;

  void sampleQuintic(
    const QuinticCoeffs & c,
    double t,
    double & q,
    double & dq,
    double & ddq) const;

};

}  // namespace block1_stanislavskyi