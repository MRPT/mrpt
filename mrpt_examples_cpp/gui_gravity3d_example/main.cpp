/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/round.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/random.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CSphere.h>

#include <iostream>
#include <thread>
#include <vector>

namespace
{
constexpr size_t N_MASSES = 1500;
constexpr double BOX = 1000;
constexpr double V0 = 300;
const double MASS_MIN = std::log(300.0);
const double MASS_MAX = std::log(5000.0);
constexpr double M2R = 0.5;
constexpr double LARGEST_STEP = 0.0005;
constexpr double G = 300;
constexpr double COLLIS_LOSS = 0.99;

struct TMass
{
  mrpt::math::TVector3D pos = {0, 0, 0};
  mrpt::math::TVector3D vel = {0, 0, 0};
  double mass{1};
  double radius;
  mrpt::viz::CSphere::Ptr obj3d;
};

using TForce = mrpt::math::TVector3D;

void simulateGravity(std::vector<TMass>& objs, double At)
{
  const size_t N = objs.size();
  std::vector<TForce> forces(N);

  // Index in the array must be larger than its content!!
  std::vector<std::pair<size_t, double>> lstMass_i_joins_j(N, {std::string::npos, 10000.0});

  for (size_t i = 0; i < (N - 1); i++)
  {
    const double Ri = objs[i].radius;

    for (size_t j = i + 1; j < N; j++)
    {
      const double D = (objs[j].pos - objs[i].pos).norm();

      auto delta = objs[j].pos - objs[i].pos;

      if (D == 0)
      {
        continue;
      }

      const double Rj = objs[j].radius;

      if (D < (Ri + Rj))  // Collision!!
      {
        if (D < lstMass_i_joins_j[j].second)
        {
          lstMass_i_joins_j[j].first = i;
          lstMass_i_joins_j[j].second = D;
        }
      }
      else
      {
        const double K = G * objs[i].mass * objs[j].mass / mrpt::square(std::max(D, 1.0));
        const double D_1 = 1.0 / D;
        delta *= D_1;

        forces[i] += delta * K;
        forces[j] -= delta * K;
      }
    }
  }

  for (size_t i = 0; i < N; i++)
  {
    const double M_1 = 1.0 / objs[i].mass;

    forces[i] *= M_1;

    objs[i].vel += forces[i] * At;
    objs[i].pos += objs[i].vel * At;
  }

  for (int i = static_cast<int>(N) - 1; i >= 0; i--)
  {
    const size_t newObj = lstMass_i_joins_j[i].first;
    if (newObj == std::string::npos)
    {
      continue;
    }

    const double Mi = objs[i].mass;
    const double Mj = objs[newObj].mass;
    const double newMass = Mi + Mj;
    const double newMass_1 = 1.0 / newMass;

    objs[newObj].vel = COLLIS_LOSS * newMass_1 * (Mj * objs[newObj].vel + Mi * objs[i].vel);
    objs[newObj].pos = newMass_1 * (Mj * objs[newObj].pos + Mi * objs[i].pos);

    objs[newObj].mass = newMass;
    objs[newObj].radius = M2R * std::pow(newMass, 1.0 / 3.0);
    objs[newObj].obj3d->setRadius(mrpt::d2f(objs[newObj].radius));

    objs[i].obj3d->setVisibility(false);
    objs.erase(objs.begin() + i);
  }
}

void GravityDemo()
{
  mrpt::gui::CDisplayWindow3D win("MRPT example: 3D gravity simulator- JLBC 2008", 1000, 700);

  mrpt::random::getRandomGenerator().randomize();

  win.setCameraElevationDeg(50.0f);
  win.setCameraZoom(1000);
  win.getDefaultViewport()->setViewportClipDistances(1, 10000);

  auto& theScene = win.get3DSceneAndLock();

  {
    auto obj = mrpt::viz::CGridPlaneXY::Create(-2000, 2000, -2000, 2000, 0, 100);
    obj->setColor(0.3f, 0.3f, 0.3f);
    theScene->insert(obj);
  }

  std::vector<TMass> masses(N_MASSES);

  for (size_t i = 0; i < N_MASSES; i++)
  {
    masses[i].pos.x = mrpt::random::getRandomGenerator().drawUniform(-BOX, BOX);
    masses[i].pos.y = mrpt::random::getRandomGenerator().drawUniform(-BOX, BOX);
    masses[i].pos.z = mrpt::random::getRandomGenerator().drawUniform(-BOX, BOX) / 10;

    const double a = std::atan2(masses[i].pos.y, masses[i].pos.x);

    masses[i].vel.x =
        -V0 * std::sin(a) + mrpt::random::getRandomGenerator().drawUniform(-V0 * 0.05, V0 * 0.05);
    masses[i].vel.y =
        V0 * std::cos(a) + mrpt::random::getRandomGenerator().drawUniform(-V0 * 0.05, V0 * 0.05);
    masses[i].vel.z = 0;

    masses[i].mass = std::exp(mrpt::random::getRandomGenerator().drawUniform(MASS_MIN, MASS_MAX));
    auto& obj = masses[i].obj3d = mrpt::viz::CSphere::Create();

    obj->setColor(
        mrpt::random::getRandomGenerator().drawUniform<float>(0.1, 0.9),
        mrpt::random::getRandomGenerator().drawUniform<float>(0.1, 0.9),
        mrpt::random::getRandomGenerator().drawUniform<float>(0.1, 0.9));

    masses[i].radius = M2R * std::pow(masses[i].mass, 1.0 / 3.0);
    obj->setRadius(mrpt::d2f(masses[i].radius));
    obj->setLocation(masses[i].pos);
    theScene->insert(obj);
  }

  win.unlockAccess3DScene();

  mrpt::system::CTicTac tictac;
  tictac.Tic();

  double t0 = tictac.Tac();

  while (!mrpt::system::os::kbhit() && win.isOpen())
  {
    double t1 = tictac.Tac();
    double At = t1 - t0;
    t0 = t1;

    win.get3DSceneAndLock();

    size_t n_steps = mrpt::round(std::ceil(At / LARGEST_STEP) + 1);
    double At_steps = At / static_cast<double>(n_steps);
    n_steps = std::min(n_steps, static_cast<size_t>(3));
    for (size_t j = 0; j < n_steps; j++)
    {
      simulateGravity(masses, At_steps);
    }

    for (auto& m : masses)
    {
      m.obj3d->setLocation(m.pos);
    }
    win.unlockAccess3DScene();

    win.forceRepaint();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  };
}
}  // namespace

int main()
{
  try
  {
    GravityDemo();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}