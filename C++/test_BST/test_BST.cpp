// =============================================================================
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
// Authors: Alessandro Tasora
// =============================================================================
//
// FEA for thin shells of Kirchhoff-Love type, with BST triangle finite elements
//
// =============================================================================

#include <vector>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/timestepper/ChTimestepper.h"

#include "chrono/fea/ChElementShellBST.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/core/ChTimer.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono_postprocess/ChGnuPlot.h"
#include "chrono_thirdparty/filesystem/path.h"

#include <numeric>
#include <direct.h>

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::postprocess;
using namespace irr;
using namespace std;

int main(int argc, char* argv[]) {

	// Check that all input arguments were passed


	//-----------------------------------------
	// **Note: Units are g and cm everywhere**    
	//-----------------------------------------

	cout << "Order of command line args:" << "\n";
	cout << "pack_frac, int_radius, r_i, r_o, data_path, mesh_file, out_dir" << "\n";

	// Entry for packing fraction
	float pack_frac;
	if (argc < 2) {
		cout << "Ratio of volume taken by sphere/volume in tube\n";
		cout << "Enter the packing fraction (cm/s): \n";
		std::cin >> pack_frac;
		cout << "Packing fraction is: " << 100 * pack_frac << "%\n";
	}
	else {
		pack_frac = atof(argv[1]);
		cout << "Packing fraction is: " << 100 * pack_frac << "%\n";
	}

	// Entry for interior particle size
	float int_radius;
	if (argc < 3) {
		cout << "Enter the interior particle size (cm): \n";
		std::cin >> int_radius;
		cout << "Interior particle size is: " << int_radius << "cm\n";
	}
	else {
		int_radius = atof(argv[2]);
		cout << "Interior particle size is: " << int_radius << "cm\n";
	}

	// Entry for torus inner radius
	float r_i;
	if (argc < 4) {
		cout << "Enter the torus inner radius (cm): \n";
		std::cin >> r_i;
		cout << "Torus inner radius is: " << r_i << "cm\n";
	}
	else {
		r_i = atof(argv[3]);
		cout << "Torus inner radius is: " << r_i << "cm\n";
	}

	// Entry for torus outer radius
	float r_o;
	if (argc < 5) {
		cout << "Enter the torus outer radius (cm): \n";
		std::cin >> r_o;
		cout << "Torus outer radius is: " << r_o << "cm\n";
	}
	else {
		r_o = atof(argv[4]);
		cout << "Torus outer radius is: " << r_o << "cm\n";
	}

	// Entry for Chrono data path
	if (argc < 6) {
		cout << "Please enter a data path argument";
		return 2;
	}
	const string data_path = argv[5];
	cout << "Data path is: " << data_path << "\n";

	// Entry for mesh .obj import
	if (argc < 7) {
		cout << "Please enter a mesh file name";
		return 2;
	}
	const char* mesh_file = argv[6];

	// Entry for output directory
	if (argc < 8) {
		cout << "Please enter an output directory";
		return 2;
	}
	const string out_dir = argv[7];

    GetLog() << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
	chrono::SetChronoDataPath(data_path);
	collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(.01);  // Effective radius of curvature for all SCM contacts.
	collision::ChCollisionModel::SetDefaultSuggestedMargin(0.1);
	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.1);

	// default material
	auto default_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>(); 
	default_mat->SetYoungModulus(2e7);
	default_mat->SetFriction(0.3f);
	default_mat->SetAdhesion(0);

	ChTimer<double> timer;
	ChTimer<double> total_timer;

    // Create output directory
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;
	my_system.Set_G_acc(ChVector<>(0, -981, 0));

//--------------------------------------
//        Mesh    
//--------------------------------------

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_system.Add(my_mesh);

    // my_system.Set_G_acc(VNULL); // to remove gravity effect, or:
    // my_mesh->SetAutomaticGravity(false);  
	
    ChVector<> load_force;

    // Load shell from a .obj file containing a triangle mesh surface
	float c = 29;					// Large torus radius
	float a = 4;					// Torus ring radius
	float init_height = 0;

	// Make the mesh
	if (false) {

		double density = 1.5;
		double E = 5e5;
		double nu = 0.4;
		double thickness = 0.5;

		auto melasticity = chrono_types::make_shared<ChElasticityKirchhoffIsothropic>(E, nu);
		auto material = chrono_types::make_shared<ChMaterialShellKirchhoff>(melasticity);
		material->SetDensity(density);

		ChMeshFileLoader::BSTShellFromObjFile(my_mesh, mesh_file, material, thickness);

		// FEA node contacts
		auto mcontactcloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>();
		my_mesh->AddContactSurface(mcontactcloud);
		mcontactcloud->AddAllNodes(0.5);  // use larger point size to match beam section radius
		mcontactcloud->SetMaterialSurface(default_mat);
		my_system.Add(my_mesh);

		/*
		// FEA mesh contact surface
		auto mcontactsurf = chrono_types::make_shared<ChContactSurfaceMesh>();
		my_mesh->AddContactSurface(mcontactsurf);
		mcontactsurf->AddFacesFromBoundary(0.001);				// do this after my_mesh->AddContactSurface
		mcontactsurf->SetMaterialSurface(default_mat);          // use the SMC penalty contacts
		mcontactsurf->SurfaceAddCollisionModelsToSystem(&my_system);
		// Remember to add the mesh to the system!
		my_system.Add(my_mesh);
		*/

		unsigned int elements = my_mesh->GetNelements();
		std::cout << "Number of elements: " << elements << "\n";
	}

//--------------------------------------
//        Make the big bots    
//--------------------------------------
	// Common material and ID
	auto bot_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
	bot_mat->SetFriction(0.4f);
	bot_mat->SetRestitution(0.9);
	bot_mat->SetYoungModulus(2e7);
	int bot_id = 0;

	int bot_number = 8;
	//ChVector<> bot_inertia = 0.5 * ChVector<>(1, 1, 1);  //Diagonal moment of inertia of bots
	float bot_mass = 350;
	float bot_radius = 2.8;
	float bot_height = 8;
	std::vector<std::shared_ptr<ChForce >> forces;

	// Visualization mesh
	//auto bot_vis = chrono_types::make_shared<ChObjShapeFile>();
	//bot_vis->SetFilename("bot_vis.obj");

	// Create the bots
	if (true) {
		for (float u = 0; u < CH_C_2PI - CH_C_2PI / bot_number; u = u + CH_C_2PI / bot_number) {

			float x = c * cos(u);
			float y = init_height;
			float z = c * sin(u);

			auto bot = chrono_types::make_shared<ChBodyEasyCylinder>(bot_radius, bot_height,1, true, true);
			bot->SetMaterialSurface(bot_mat);
			bot->SetMass(bot_mass);
			//bot->SetInertiaXX(bot_inertia);
			bot->SetPos(ChVector<>(x, y, z));
			
			bot->SetId(bot_id);
			// Attach a force to the bot
			/*
			auto bot_force = chrono_types::make_shared<ChForce>();
			bot_force->SetMode(ChForce().FORCE);
			bot->AddForce(bot_force);
			bot_force->SetDir(VECT_X);
			bot_force->SetMforce(5e6);
			forces.push_back(bot_force);
			*/

			// Attach visualization mesh to bot
			//bot->AddAsset(mobjmesh);

			//Add to counter and add to system
			bot_id++;
			my_system.Add(bot);
		}
		std::cout << "Number of big bots: " << bot_id << "\n";
	}

//--------------------------------------
//        Make the interior    
//--------------------------------------
	const double bot_angle = 2 * tan(bot_radius / c); //[rad] angle taken up by each robot
	const double bot_seperation = (CH_C_2PI / bot_number) - bot_angle; //[rad] angle of free space between each robot
	const double int_mass = 0.95;                          //[g] mass of interior particles
	//ChVector<> int_inertia = 6027e-6 * ChVector<>(1, 1, 1);    //Diagonal moment of inertia of interior particles
	const double int_c = (0.95 * CH_C_PI * c / int_radius);     //not quite sure what this parameter is exactly
	int int_id = 0;
	float int_angle = 2*tan(int_radius / c);

	// TODO: Spawn interior particles
	if (true) {
		// Make sure we avoid hitting robots
		for (float torus = 1.25 * bot_angle / 2; torus < CH_C_2PI - 1.25*bot_angle; torus += bot_seperation + bot_angle) {
			// Angles
			for (float u = int_angle/4 + torus; u < bot_seperation + torus - int_angle/2; u += 2*tan(int_radius/c)) {
				// TODO: At each height
				for (float height = -0.95*a; height < 0.95 * a; height += 3 * int_radius) {
					// TODO: At each ring
					for (float radius = c-a; radius < r_o; radius += 3 * int_radius) {
						// Point on circle
						float x = radius * cos(u);
						float y = height;
						float z = radius * sin(u);

						auto int_part = chrono_types::make_shared<ChBodyEasySphere>(int_radius, 1, true, true);
						//auto int_part = chrono_types::make_shared<ChBodyEasyBox>(int_radius, true);
						int_part->SetMaterialSurface(default_mat);
						int_part->SetMass(int_mass);
						//int_part->SetInertiaXX(int_inertia);
						int_part->SetPos(ChVector<>(x, y, z));
						int_part->SetBodyFixed(false);

						//Add to counter and add to system
						int_id++;
						my_system.AddBody(int_part);
					}
				}
			}
		}
		
		// TODO: Spawn interior in cylinder shape near the holes
		// Angles
		for (float u = 0; u < CH_C_2PI; u += 2 * tan(int_radius / c)) {
			// TODO: At each height
			for (float height = -0.95 * a; height < 0.95 * a; height += 3 * int_radius) {
				// TODO: At each ring
				for (float radius = r_i; radius < c - a; radius += 3 * int_radius) {
					// Point on circle
					float x = radius * cos(u);
					float y = height;
					float z = radius * sin(u);

					auto int_part = chrono_types::make_shared<ChBodyEasySphere>(int_radius, 1, true);
					//auto int_part = chrono_types::make_shared<ChBodyEasyBox>(int_radius, true);
					int_part->SetMaterialSurface(default_mat);
					int_part->SetMass(int_mass);
					//int_part->SetInertiaXX(int_inertia);
					int_part->SetPos(ChVector<>(x, y, z));
					int_part->SetBodyFixed(false);

					//Add to counter and add to system
					int_id++;
					my_system.AddBody(int_part);
				}
			}
		}

		std::cout << "Number of interior particles: " << int_id << "\n";
	}

//--------------------------------------
//        Generating environment        
//--------------------------------------
	
	// Create a floor as a simple collision primitive:
	auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(1000, 1, 1000, .1, true);
	mfloor->SetPos(ChVector<>(0, -10, 0));
	mfloor->SetBodyFixed(true);
	mfloor->SetMaterialSurface(default_mat);

	// Use texture for floor
	if (true)
	{
		auto masset_texture = chrono_types::make_shared<ChTexture>();
		masset_texture->SetTextureFilename("concrete.jpg");
		mfloor->AddAsset(masset_texture);
	}

	// Use color for floor
	if (false) {
		auto floor_color = chrono_types::make_shared<ChColorAsset>();
		floor_color->SetColor(ChColor(.5, .5, .5));
		mfloor->AddAsset(floor_color);
	}
		
	my_system.Add(mfloor);

	// Create a wall to stop the robot's motion
	if (true) {
		auto wall = chrono_types::make_shared<ChBodyEasyBox>(2, 15, 4*c, .7, true);
		wall->SetPos(ChVector<>(-c-3*a, init_height, 0));
		wall->SetBodyFixed(true);
		wall->SetMaterialSurface(default_mat);

		if (true)
		{
			auto masset_texture = chrono_types::make_shared<ChTexture>();
			masset_texture->SetTextureFilename("concrete.jpg");
			wall->AddAsset(masset_texture);
		}

		if (false) {
			auto floor_color = chrono_types::make_shared<ChColorAsset>();
			floor_color->SetColor(ChColor(.5, .5, .5));
			wall->AddAsset(floor_color);
		}

		my_system.Add(wall);
	}

	// Object to push into the robot
	auto pusher = chrono_types::make_shared <ChBodyEasyCylinder>(bot_radius, bot_radius*2.5, 1, true, true);
	pusher->SetPos(ChVector<>(c + a + 1.5 * bot_radius, init_height, 0));
	pusher->SetMaterialSurface(default_mat);
	pusher->SetId(123);

	// Attach a force to the pusher
	auto push_force = chrono_types::make_shared<ChForce>();
	push_force->SetMode(ChForce().FORCE);
	pusher->AddForce(push_force);
	push_force->SetDir(-VECT_X);

	// Constrain motion of plunger to x axis
	auto my_link_CA = chrono_types::make_shared<ChLinkLockPrismatic>();
	my_link_CA->SetName("Pusher_x");
	my_link_CA->Initialize(pusher, mfloor, ChCoordsys<>(ChVector<>(1, 0, 0), Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, ChVector<>(0,1,0))));
	my_system.AddLink(my_link_CA);

	my_system.Add(pusher);

//--------------------------------------
//        Visualization       
//--------------------------------------
	// Mesh visuals
	{auto mvisualizeshellA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	//mvisualizeshellA->SetSmoothFaces(true);
	//mvisualizeshellA->SetWireframe(true);
	mvisualizeshellA->E_GLYPH_ELEM_TENS_STRAIN;
	mvisualizeshellA->SetShellResolution(2);
	//mvisualizeshellA->SetBackfaceCull(true);
	my_mesh->AddAsset(mvisualizeshellA);

	auto mvisualizeshellB = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizeshellB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_STRESS_VONMISES);
	mvisualizeshellB->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
	mvisualizeshellB->SetSymbolsThickness(0.5);
	my_mesh->AddAsset(mvisualizeshellB);
	}

	// Create the Irrlicht visualization (open the Irrlicht device,
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&my_system, L"test_BST",                    // Window title
		core::dimension2d<u32>(1200, 800),  // Window dimensions
		false,                              // Fullscreen?
		true,                              // Shadows?
		true,                              // Anti-Aliasing?
		video::EDT_OPENGL);                 // Graphics Driver

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddLightWithShadow(irr::core::vector3df(200, 200, 200), irr::core::vector3df(0, 0, 0), 600, 0.2, 600, 50);
	application.AddLight(irr::core::vector3df(-200, -200, 0), 600, irr::video::SColorf(0.6, 1, 1, 1));
	application.AddTypicalLights();
	application.SetPaused(false);
	application.SetShowInfos(false);
	application.SetPlotCOGFrames(true);
	application.SetVideoframeSave(true);
	application.SetVideoframeSaveInterval(10);
	application.AddTypicalCamera(core::vector3df(30.f, 30.f, -60.f), core::vector3df(0.f, 0.f, 0.f));
	application.SetContactsDrawMode(ChIrrTools::CONTACT_NORMALS);
	application.SetSymbolscale(2.f);
	application.AssetBindAll();
	application.AssetUpdateAll();
	application.AddShadowAll();
	
//--------------------------------------
//        THE SOFT-REAL-TIME CYCLE       
//--------------------------------------

    // Change solver to MKL
	if (true) {
		auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
		mkl_solver->UseSparsityPatternLearner(true);
		mkl_solver->LockSparsityPattern(true);
		my_system.SetSolver(mkl_solver);
	}

	// Set the time stepper
	auto stepper = chrono_types::make_shared<ChTimestepperEulerImplicit>(&my_system);
	my_system.SetTimestepper(stepper);

	// HHT Time Stepper
	if (false) {
		std::cout << "Using HHT timestepper" << "\n";
		auto stepper = chrono_types::make_shared<ChTimestepperHHT>(&my_system);
		my_system.SetTimestepper(stepper);

		stepper->SetAlpha(-0.2);
		stepper->SetMaxiters(10);
		stepper->SetAbsTolerances(1e-5, 1e-3);

		// Do not use POSITION mode if there are 3D rigid bodies in the system
		// (POSITION mode technically incorrect when quaternions present).
		stepper->SetMode(ChTimestepperHHT::ACCELERATION);
		stepper->SetScaling(false);
		stepper->SetStepControl(true);
		stepper->SetMinStepSize(1e-8);
		stepper->SetVerbose(false);
	}
	
	// GMRES solver
	if (false) {
		auto gmres_solver = chrono_types::make_shared<ChSolverGMRES>();
		gmres_solver->SetMaxIterations(50);
		my_system.SetSolver(gmres_solver);
	}

    double timestep = 5e-4;
    application.SetTimestep(timestep);
    my_system.Setup();
    my_system.Update();

	// Create output filestream and data collection objects
	std::ofstream dataFile(out_dir+"/pusher.csv");
	std::ofstream statFile(out_dir+"/simStats.txt");
	std::vector<float> solvertimes;
	string video = out_dir.c_str() + string("_frames");

    double mtime = 0;
	int n = 0;

	total_timer.start();
	while (application.GetDevice()->run()) {
		float t = my_system.GetChTime();
		//double push_pos = my_system.SearchBodyID(123)->GetPos().x();
		//double push_pos = 1;
		//std::cout << "\n" << "Simulation Time= " << t << "s" << "\n";
		timer.start();
		application.BeginScene();
		application.DrawAll();
		application.DoStep();
		application.EndScene();
		timer.stop();
		//std::cout << "Time elapsed= " << timer.GetTimeMillisecondsIntermediate() << "ms" << "\n";
		n++;
		solvertimes.push_back(timer.GetTimeMillisecondsIntermediate());

		if (t > 0.1) {
			// Push the plunger
			push_force->SetMforce(5e5);		// [N]
			// Write plunger data to file
			dataFile << t << "," << -pusher->GetPos().x() + c + a + bot_radius << "\n";
		}

		if (t > 0.2) {
			total_timer.stop();
			float average_time = std::accumulate(solvertimes.begin(), solvertimes.end(), 0.0) / solvertimes.size();
			int ret = rename("video_capture", video.c_str());
			rmdir("video_capture");
			statFile << "Average solve time: " << average_time << "ms" << "\n";
			statFile << "Total solve time: " << total_timer.GetTimeSeconds() << "s \n";
			return 0;
		}
		/*
		if (t > 0.1) {
			for (int i = 0; i < bot_number; i++) {
				forces[i]->SetMforce(2e5);
				forces[i]->SetDir(ChVector<float>(1, 0, 0));
			}
		}
		*/
	}
    return 0;
}
