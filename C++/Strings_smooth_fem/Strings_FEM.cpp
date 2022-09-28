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
#include <cstdio>
#include <cmath>
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChLoadsBeam.h"




using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::fea;
using namespace irr;
using std::cout;
using std::endl;



int main(int argc, char* argv[]) {

	//freopen("Strings.txt", "w", stdout);  // Toggle logging to .txt file
	//GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	SetChronoDataPath(CHRONO_DATA_DIR);
	double pi = 3.1415926535897;

	//----Set Variables----//
	double diameter = .07;
	double height = .12;
	double nb = 100;
	double mass = .12;
	//---------Material properties-----------//
	double E = 6e4;
	double mu = 0.3;
	double er = 0;
	double ea = 0;
	float volume = pi * pow(diameter / 2, 2);	// Volume
	float rho = mass / volume;		// density
	float R1 = ((diameter * nb) / (pi * 2)) + .1;		// Create parimeter of robots


	//-----------------Beam properties--------------------//

	double bheight = .12;
	double bwidth = .012;

	// Create System//
	ChSystemSMC system;

	// gravity
	system.Set_G_acc(ChVector<>(0, -9.81, 0));	// Create Gravity in y direction

	double tstep = .001;

	auto col_g = std::make_shared<ChColorAsset>();
	col_g->SetColor(ChColor(0, 1, 0));


	auto col_r = std::make_shared<ChColorAsset>();
	col_r->SetColor(ChColor(1, 0, 0));


	// create a generic material
	auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
	mysurfmaterial->SetYoungModulus(E);	// youngs modulus
	mysurfmaterial->SetFriction(mu);
	mysurfmaterial->SetRestitution(er);
	mysurfmaterial->SetAdhesion(ea);

	// create mesh
	auto mesh = chrono_types::make_shared<ChMesh>();
	system.Add(mesh);

	// beam material
	auto beam_material = std::make_shared<ChBeamSectionAdvanced>();
	beam_material->SetAsRectangularSection(bheight, bwidth);
	beam_material->SetYoungModulus(0.01e9);
	beam_material->SetGshearModulus(0.01e9 * 0.3);
	beam_material->SetBeamRaleyghDamping(0.01);


	// Create floor
	auto floor = std::make_shared<ChBodyEasyBox>(10, 0.2, 10, 1000, true,true);
	system.Add(floor);
	floor->SetBodyFixed(true);
	floor->SetPos(ChVector<>(0, -0.1, 0));
	floor->SetMaterialSurface(mysurfmaterial);



	// create nodes
	std::vector<std::shared_ptr<ChNodeFEAxyzrot> > beam_nodes;
	int N_nodes = nb;
	for (int in = 0; in < N_nodes; ++in) {
		float theta = in * 2 * pi / nb;  // define angle
		float x = R1 * cos(theta);  // define x position
		float z = R1 * sin(theta);  // define z postion
		ChVector<> position(x, height/2, z);
		auto node = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(position));
		// add it to mesh
		mesh->AddNode(node);
		//node->SetFixed(true);
		beam_nodes.push_back(node);
	}
	// create elements
	for (int ie = 0; ie < N_nodes - 1; ++ie) {
		// create the element
		auto element = std::make_shared<ChElementBeamEuler>();

		// set the connected nodes (pick two consecutive nodes in our beam_nodes container)
		element->SetNodes(beam_nodes[ie], beam_nodes[ie + 1]);
	
		// set the material
		element->SetSection(beam_material);

		// add it to mesh
		mesh->AddElement(element);
	}
	
	// last element
	double ie = N_nodes - 1;
	// create the element
	auto element = std::make_shared<ChElementBeamEuler>();

	// set the connected nodes (pick two consecutive nodes in our beam_nodes container)
	element->SetNodes(beam_nodes[ie], beam_nodes[0]);

	// set the material
	element->SetSection(beam_material);

	// add it to mesh
	mesh->AddElement(element);

	ChQuaternion<> z2y;
	ChQuaternion<> z2x;
	z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
	z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));

	// CREATE ROBOTS
	for (int i = 0; i < nb; ++i) {
		float theta = (i * 2 * pi) / nb;  // define angle
		float x = R1 * cos(theta);  // define x position
		float z = R1 * sin(theta);  // define z postion

		// create elements

		// Create the cylinders
		auto bot = chrono_types::make_shared<ChBodyEasyCylinder>(diameter/2, height, rho, true, true);
		bot->SetMaterialSurface(mysurfmaterial);
		bot->SetBodyFixed(false);
		bot->SetId(i);

		bot->SetPos(ChVector<>(x, height / 2, z));
		bot->SetMaterialSurface(mysurfmaterial);
		bot->AddAsset(col_g);
		// lock an end of the wire to the cylinder
		system.Add(bot);

		// fix to robot
		auto constraint_pos = std::make_shared<ChLinkMateSpherical>();
		constraint_pos->Initialize(
			beam_nodes[i],  // node to constraint
			bot,          // body to constraint
			false,          // false: next 2 pos are in absolute coords, true: in relative coords
			beam_nodes[i]->GetPos(), // sphere ball pos 
			beam_nodes[i]->GetPos()  // sphere cavity pos
		);
		system.Add(constraint_pos);


		// fix robots to floor
		auto plane_plane = chrono_types::make_shared<ChLinkLockPlanePlane>();
		plane_plane->Initialize(bot, floor, ChCoordsys<>(ChVector<>(1,0,1), z2y));
		system.AddLink(plane_plane);

		// add it to the system
	

	}

	// CREATE INTERIOR
	std::vector<int> ni{ {90,80,70,60,50,40,30,20,10} };

	// length 
	double len = ni.size();
	//cout << len << endl;
	for (int i = 0; i < len; ++i)
	{
		//cout << i << endl;
		for (int j = 0; j < ni[i]; ++j)
		{
			//cout << j << endl;
			double R2 = diameter * ni[i]/ (pi * 2)+.1;// define radius

			float theta = (j * 2 * pi) / ni[i];  // define angle
			float x = R2 * cos(theta);  // define x position
			float z = R2 * sin(theta);  // define z postion



			auto interior = chrono_types::make_shared<ChBodyEasyCylinder>(diameter/2, height, rho, true, true);
			interior->SetMaterialSurface(mysurfmaterial);
			interior->SetBodyFixed(false);
			

			interior->SetPos(ChVector<>(x, height/2 , z));
			interior->SetMaterialSurface(mysurfmaterial);
			interior->AddAsset(col_r);
			// lock an end of the wire to the cylinder
			system.Add(interior);

			// fix robots to floor
			auto plane_plane = chrono_types::make_shared<ChLinkLockPlanePlane>();
			plane_plane->Initialize(interior, floor, ChCoordsys<>(ChVector<>(1,0,1),z2y));
			system.AddLink(plane_plane);

		}
	
	
	}




	// Use our SMC surface material properties 
	floor->SetMaterialSurface(mysurfmaterial);
	// Create the contact surface and add to the mesh
	auto mcontactcloud = std::make_shared<ChContactSurfaceNodeCloud>();
	mesh->AddContactSurface(mcontactcloud);

	// Must use this to 'populate' the contact surface use larger point size to match beam section radius
	mcontactcloud->AddAllNodes(0.01);

	// Use our SMC surface material properties 
	mcontactcloud->SetMaterialSurface(mysurfmaterial);



	// Visuals
	auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(mesh.get()));
	mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ANCF_BEAM_AX);
	mvisualizebeamA->SetColorscaleMinMax(-0.005, 0.005);
	mvisualizebeamA->SetSmoothFaces(true);
	mvisualizebeamA->SetWireframe(false);
	mesh->AddAsset(mvisualizebeamA);

	auto mvisualizebeamC = std::make_shared<ChVisualizationFEAmesh>(*(mesh.get()));
	mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
	mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
	mvisualizebeamC->SetSymbolsThickness(0.006);
	mvisualizebeamC->SetSymbolsScale(0.005);
	mvisualizebeamC->SetZbufferHide(false);
	mesh->AddAsset(mvisualizebeamC);



	// Change solver
	auto solver = chrono_types::make_shared<ChSolverMINRES>();
	solver->SetMaxIterations(200);
	solver->SetTolerance(1e-10);
	solver->EnableWarmStart(true);
	system.SetSolver(solver);

	// Change integrator:
	//system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // default: fast, 1st order
	//system.SetTimestepperType(ChTimestepper::Type::HHT);  // precise, slower, might iterate each step

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
	application.SetTimestep(tstep);
	application.SetTryRealtime(false);

	while (application.GetDevice()->run()) {
		// Initialize the graphical scene.
		application.BeginScene();
		double t = system.GetChTime();
		cout << t << endl;
		// Render all visualization objects.
		application.DrawAll();



		// Advance simulation by one step.
		application.DoStep();

		// Finalize the graphical scene.
		application.EndScene();
	}



}