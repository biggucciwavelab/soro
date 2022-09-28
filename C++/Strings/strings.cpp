//=============================================================================
// Strings.cpp
// Authors: Qiyuan Zhou, Declan Mulroy, Esteban Lopez
// WaveLab
// Illinois Institute of Technology
// Dec. 19 2019
//=============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Simulation of the strings robot
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/ChTimerParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

#include "chrono/physics/ChLinkTSDA.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;
using std::cout;
using std::endl;

ChTimer<double> timer; //Timer object

// Tilt angle (about global Y axis) of the floor.
double tilt_angle = 0 * CH_C_PI / 180;

//**********************//
//********Note**********//
//** gravity in z axis**//
//* for opengl renderer*//
//**********************//
//**********************//

// -----------------------------------------------------------------------------
// TODO: define some global parameters
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// TODO: Create a container for robot to live in
// -----------------------------------------------------------------------------
void AddFloor(ChSystemParallelNSC* sys) {
    // IDs for the two bodies
    int FloorId = -200;

    // Create a common material
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);

    // Create the containing bin (4 x 4 x 1)
    auto bin = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
    bin->SetMaterialSurface(mat);
    bin->SetIdentifier(FloorId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(Q_from_AngY(tilt_angle));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    ChVector<> hdim(2, 2, 0.5);  //half sizes
    double hthick = 0.1;

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hdim.y(), hthick), ChVector<>(0, 0, -hthick));
    utils::AddBoxGeometry(bin.get(), ChVector<>(hthick, hdim.y(), hdim.z()),
        ChVector<>(-hdim.x() - hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), ChVector<>(hthick, hdim.y(), hdim.z()),
        ChVector<>(hdim.x() + hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hthick, hdim.z()),
        ChVector<>(0, -hdim.y() - hthick, hdim.z()));
    utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hthick, hdim.z()),
        ChVector<>(0, hdim.y() + hthick, hdim.z()));
    bin->GetCollisionModel()->BuildModel();

    sys->AddBody(bin);
}

// -----------------------------------------------------------------------------
// TODO: Create interior particles
// -----------------------------------------------------------------------------
void AddInterior(ChSystemParallel* sys) {
    // Common material
    auto intMat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    intMat->SetFriction(0.4f);

    // Create the falling balls
    int intId = 0;
    double mass = 1;
    double radius = 0.15;
    double height = 0.15;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

    for (int ix = -count_X; ix <= count_X; ix++) {
        for (int iy = -count_Y; iy <= count_Y; iy++) {
            ChVector<> pos(0.4 * ix, 0.4 * iy, 1);

            auto int_part = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
            int_part->SetMaterialSurface(intMat);

            int_part->SetIdentifier(intId++);
            int_part->SetMass(mass);
            int_part->SetInertiaXX(inertia);
            int_part->SetPos(pos);
            int_part->SetRot(ChQuaternion<>(1, 0, 0, 0));
            int_part->SetBodyFixed(false);
            int_part->SetCollide(true);

            int_part->GetCollisionModel()->ClearModel();
            utils::AddCylinderGeometry(int_part.get(), radius, height);
            int_part->GetCollisionModel()->BuildModel();

            sys->AddBody(int_part);
        }
    }
}

// -----------------------------------------------------------------------------
// TODO: Create boundary particles and add springs
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// TODO: Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    int threads = 8;

    // Simulation parameters
    // ---------------------

    double gravity = 9.81;
    double time_step = 1e-3;
    double time_end = 2;

    double out_fps = 30;

    uint max_iteration = 300;
    real tolerance = 1e-3;

    // Create system
    // -------------

    ChSystemParallelNSC msystem;

    // Set number of threads.
    int threads = 16;
    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    CHOMPfunctions::SetNumThreads(threads);

    // Set gravitational acceleration
    msystem.Set_G_acc(ChVector<>(0, 0, -gravity));

    // Set solver parameters
    msystem.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    msystem.GetSettings()->solver.max_iteration_normal = max_iteration / 3;
    msystem.GetSettings()->solver.max_iteration_sliding = max_iteration / 3;
    msystem.GetSettings()->solver.max_iteration_spinning = 0;
    msystem.GetSettings()->solver.max_iteration_bilateral = max_iteration / 3;
    msystem.GetSettings()->solver.tolerance = tolerance;
    msystem.GetSettings()->solver.alpha = 0;
    msystem.GetSettings()->solver.contact_recovery_speed = 10000;
    msystem.ChangeSolverType(SolverType::SPGQP);
    msystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    msystem.GetSettings()->collision.collision_envelope = 0.01;
    msystem.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // Create the fixed and moving bodies
    // ----------------------------------

    AddFloor(&msystem);
    AddInterior(&msystem);

    // Perform the simulation
    // ----------------------

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1200, 800, "Strings", &msystem);
    gl_window.SetCamera(ChVector<>(0, -6, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::SOLID);

    // Uncomment the following two lines for the OpenGL manager to automatically
    // run the simulation in an infinite loop.
    // gl_window.StartDrawLoop(time_step);
    // return 0;

    while (true) {
        if (gl_window.Active()) {
            gl_window.DoStepDynamics(time_step);
            gl_window.Render();
            ////if (gl_window.Running()) {
            ////  msystem.CalculateContactForces();
            ////  real3 frc = msystem.GetBodyContactForce(0);
            ////  std::cout << frc.x << "  " << frc.y << "  " << frc.z << std::endl;
            ////}

            //TODO: Program logic for spring forces
        }
        else {
            break;
        }
    }
#else
    // Run simulation for specified time
    int num_steps = (int)std::ceil(time_end / time_step);
    double time = 0;

    for (int i = 0; i < num_steps; i++) {
        msystem.DoStepDynamics(time_step);
        time += time_step;
    }
#endif

    return 0;
}
