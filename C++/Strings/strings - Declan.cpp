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

#include <cmath>
#include <cstdio>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChLoadsBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/physics/ChLoadContainer.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::fea;
using namespace irr;
using std::cout;
using std::endl;



int main(int argc, char* argv[]) {

	freopen("Strings.txt", "w", stdout);  // Toggle logging to .txt file
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	SetChronoDataPath(CHRONO_DATA_DIR);
	double pi = 3.1415926535897;
	// set variables
	double diameter = .07;
	double height = .12;
	double nb = 8;
	double mass = .12;
	float volume = pi * pow(diameter / 2, 2);

	float rho = mass / volume;

	float R1 = (diameter * nb) / (pi * 2) + .1;

	ChSystemSMC system;
	system.Set_G_acc(ChVector<>(0, -9.81, 0));


	// create a generic material
	auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
	mysurfmaterial->SetYoungModulus(6e4);
	mysurfmaterial->SetFriction(0.3f);
	mysurfmaterial->SetRestitution(0.2f);
	mysurfmaterial->SetAdhesion(0);

	// 2. Create the mesh that will contain the finite elements, and add it to the system
	auto mesh = chrono_types::make_shared<ChMesh>();
	system.Add(mesh);

	// creat floor
	auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(10, -.1, 10, 2700, true);
	mfloor->SetBodyFixed(true);
	mfloor->SetMaterialSurface(mysurfmaterial);
	system.Add(mfloor);

	// Create robots
	for (int i = 0; i < nb; ++i) {
		float theta = i * 2 * pi / nb;  // define angle

		float x = R1 * cos(theta);  // define x position

		float z = R1 * sin(theta);  // define z postion

		auto cylinder = chrono_types::make_shared<ChBodyEasyCylinder>(diameter, height, rho, true, true);


		cylinder->SetMaterialSurface(mysurfmaterial);

		cylinder->SetBodyFixed(false);
		// move cylinder to end of beam
		cylinder->SetPos(ChVector<>(x, height, z));
		cylinder->SetMaterialSurface(mysurfmaterial);


		// add it to the system
		system.Add(cylinder);

		auto plane_plane = chrono_types::make_shared<ChLinkLockPlanePlane>();
		plane_plane->Initialize(mfloor, cylinder, ChCoordsys<>(ChVector<>(1, 0, 1), ChQuaternion<>(1, 0, 0, 0)));
		system.AddLink(plane_plane);

	}
	// Change solver
	auto solver = chrono_types::make_shared<ChSolverMINRES>();
	solver->SetMaxIterations(200);
	solver->SetTolerance(1e-10);
	solver->EnableWarmStart(true);
	system.SetSolver(solver);

	// Change integrator:
	//system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // default: fast, 1st order
	system.SetTimestepperType(ChTimestepper::Type::HHT);  // precise, slower, might iterate each step

   // 9. Prepare visualization with Irrlicht
   //    Note that Irrlicht uses left-handed frames with Y up.

   // Create the Irrlicht application and set-up the camera.
	ChIrrApp application(&system,                            // pointer to the mechanical system
		L"FEA cable collide demo",          // title of the Irrlicht window
		core::dimension2d<u32>(1024, 768),  // window dimension (width x height)
		false,                              // use full screen?
		true,                               // enable stencil shadows?
		true);                              // enable antialiasing?

	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0.1f, 0.2f, -2.0f),  // camera location
		core::vector3df(0.0f, 0.0f, 0.0f));  // "look at" location

// Let the Irrlicht application convert the visualization assets.
	application.AssetBindAll();
	application.AssetUpdateAll();

	// 10. Perform the simulation.

	// Specify the step-size.
	application.SetTimestep(0.001);
	application.SetTryRealtime(true);

	while (application.GetDevice()->run()) {
		// Initialize the graphical scene.
		application.BeginScene();

		// Render all visualization objects.
		application.DrawAll();

		// Draw an XZ grid at the global origin to add in visualization.
		ChIrrTools::drawGrid(application.GetVideoDriver(), 0.1, 0.1, 20, 20,
			ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
			video::SColor(255, 80, 100, 100), true);

		// Advance simulation by one step.
		application.DoStep();

		// Finalize the graphical scene.
		application.EndScene();
	}



}