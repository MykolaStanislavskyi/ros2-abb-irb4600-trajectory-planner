#include "block1_stanislavskyi/manipulator.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace block1_stanislavskyi
{

Manipulator::Manipulator() {}

Manipulator::QuinticCoeffs Manipulator::computeQuintic(
  double q0, double qT,
  double v0, double vT,
  double acc0, double accT,
  double T) const
{
  QuinticCoeffs c;

  c.a0 = q0;
  c.a1 = v0;
  c.a2 = 0.5 * acc0;

  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;

  c.a3 =
    (20.0 * (qT - q0)
    - (8.0 * vT + 12.0 * v0) * T
    - (3.0 * acc0 - accT) * T2) / (2.0 * T3);

  c.a4 =
    (30.0 * (q0 - qT)
    + (14.0 * vT + 16.0 * v0) * T
    + (3.0 * acc0 - 2.0 * accT) * T2) / (2.0 * T4);

  c.a5 =
    (12.0 * (qT - q0)
    - (6.0 * vT + 6.0 * v0) * T
    - (acc0 - accT) * T2) / (2.0 * T5);

  return c;
}

void Manipulator::sampleQuintic(
  const QuinticCoeffs & c,
  double t,
  double & q,
  double & dq,
  double & ddq) const
{
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;

  q =
    c.a0 +
    c.a1 * t +
    c.a2 * t2 +
    c.a3 * t3 +
    c.a4 * t4 +
    c.a5 * t5;

  dq =
    c.a1 +
    2.0 * c.a2 * t +
    3.0 * c.a3 * t2 +
    4.0 * c.a4 * t3 +
    5.0 * c.a5 * t4;

  ddq =
    2.0 * c.a2 +
    6.0 * c.a3 * t +
    12.0 * c.a4 * t2 +
    20.0 * c.a5 * t3;
}

std::vector<TrajectoryPoint> Manipulator::generatePTP(
  const ikfast_abb::JointValues & q_start,
  const ikfast_abb::JointValues & q_goal,
  double duration,
  double dt) const
{
  std::array<QuinticCoeffs, 6> coeffs;

  for (size_t i = 0; i < 6; ++i) {
    coeffs[i] = computeQuintic(
      q_start[i],
      q_goal[i],
      0.0,
      0.0,
      0.0,
      0.0,
      duration);
  }

  std::vector<TrajectoryPoint> traj;

  for (double t = 0.0; t <= duration + 1e-9; t += dt) {
    TrajectoryPoint p;
    p.t = t;

    for (size_t i = 0; i < 6; ++i) {
      sampleQuintic(coeffs[i], t, p.q[i], p.dq[i], p.ddq[i]);
    }

    traj.push_back(p);
  }

  return traj;
}

std::vector<TrajectoryPoint> Manipulator::generatePTPVia(
  const ikfast_abb::JointValues & q_start,
  const ikfast_abb::JointValues & q_via,
  const ikfast_abb::JointValues & q_goal,
  double duration1,
  double duration2,
  double dt) const
{
  std::array<QuinticCoeffs, 6> coeffs1;
  std::array<QuinticCoeffs, 6> coeffs2;

  ikfast_abb::JointValues v_via;

  for (size_t i = 0; i < 6; ++i) {
    const double v_in = (q_via[i] - q_start[i]) / duration1;
    const double v_out = (q_goal[i] - q_via[i]) / duration2;

    // Ненульова швидкість у VIA-точці:
    // робот проходить через Tvia, а не зупиняється в ній.
    v_via[i] = 0.5 * (v_in + v_out);

    coeffs1[i] = computeQuintic(
      q_start[i],
      q_via[i],
      0.0,
      v_via[i],
      0.0,
      0.0,
      duration1);

    coeffs2[i] = computeQuintic(
      q_via[i],
      q_goal[i],
      v_via[i],
      0.0,
      0.0,
      0.0,
      duration2);
  }

  std::vector<TrajectoryPoint> traj;

  // Перша частина: q_start -> q_via
  for (double t = 0.0; t <= duration1 + 1e-9; t += dt) {
    TrajectoryPoint p;
    p.t = t;

    for (size_t i = 0; i < 6; ++i) {
      sampleQuintic(coeffs1[i], t, p.q[i], p.dq[i], p.ddq[i]);
    }

    traj.push_back(p);
  }

  // Друга частина: q_via -> q_goal
  // Починаємо з dt, щоб не дублювати VIA-точку.
  for (double t = dt; t <= duration2 + 1e-9; t += dt) {
    TrajectoryPoint p;
    p.t = duration1 + t;

    for (size_t i = 0; i < 6; ++i) {
      sampleQuintic(coeffs2[i], t, p.q[i], p.dq[i], p.ddq[i]);
    }

    traj.push_back(p);
  }

  return traj;
}

bool Manipulator::chooseBestSolution(
  const ikfast_abb::Solutions & solutions,
  const ikfast_abb::JointValues & q_prev,
  ikfast_abb::JointValues & q_best) const
{
  if (solutions.empty()) {
    return false;
  }

  double best_dist = std::numeric_limits<double>::max();

  for (const auto & sol : solutions) {
    double dist = 0.0;

    for (size_t i = 0; i < 6; ++i) {
      const double d = sol[i] - q_prev[i];
      dist += d * d;
    }

    if (dist < best_dist) {
      best_dist = dist;
      q_best = sol;
    }
  }

  return true;
}

std::vector<TrajectoryPoint> Manipulator::generateLIN(
  const ikfast_abb::JointValues & q_start,
  const Eigen::Affine3d & target_pose,
  double duration,
  double dt) const
{
  std::vector<TrajectoryPoint> traj;

  const Eigen::Affine3d start_pose = ikfast_abb::computeFk(q_start);
  const Eigen::Vector3d p0 = start_pose.translation();
  const Eigen::Vector3d p1 = target_pose.translation();

  // Орієнтація інструмента стала під час LIN.
  const Eigen::Matrix3d R = start_pose.rotation();

  ikfast_abb::JointValues q_prev = q_start;

  for (double t = 0.0; t <= duration + 1e-9; t += dt) {
    const double tau = t / duration;

    // Quintic time scaling для плавного LIN:
    // s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0
    const double s =
      10.0 * std::pow(tau, 3) -
      15.0 * std::pow(tau, 4) +
      6.0 * std::pow(tau, 5);

    const Eigen::Vector3d p = p0 + s * (p1 - p0);

    Eigen::Affine3d desired = Eigen::Affine3d::Identity();
    desired.linear() = R;
    desired.translation() = p;

    const auto solutions = ikfast_abb::computeIK(desired);

    ikfast_abb::JointValues q_best;
    if (!chooseBestSolution(solutions, q_prev, q_best)) {
      throw std::runtime_error("No IK solution during LIN");
    }

    TrajectoryPoint point;
    point.t = t;
    point.q = q_best;

    if (traj.empty()) {
      point.dq.fill(0.0);
      point.ddq.fill(0.0);
    } else {
      for (size_t i = 0; i < 6; ++i) {
        point.dq[i] = (q_best[i] - q_prev[i]) / dt;
        point.ddq[i] = 0.0;
      }
    }

    traj.push_back(point);
    q_prev = q_best;
  }

  // Чисельне прискорення з різниці швидкостей.
  for (size_t k = 1; k < traj.size(); ++k) {
    for (size_t i = 0; i < 6; ++i) {
      traj[k].ddq[i] = (traj[k].dq[i] - traj[k - 1].dq[i]) / dt;
    }
  }

  if (!traj.empty()) {
    traj.front().dq.fill(0.0);
    traj.front().ddq.fill(0.0);
    traj.back().dq.fill(0.0);
    traj.back().ddq.fill(0.0);
  }

  return traj;
}

Eigen::Affine3d Manipulator::offsetAlongToolX(
  const ikfast_abb::JointValues & q,
  double offset) const
{
  Eigen::Affine3d pose = ikfast_abb::computeFk(q);
  Eigen::Vector3d local_x = pose.linear().col(0);
  pose.translation() += offset * local_x;
  return pose;
}

}  // namespace block1_stanislavskyi