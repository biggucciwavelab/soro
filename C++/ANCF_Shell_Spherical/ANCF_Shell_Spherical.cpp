//=============================================================================
// ANCF_Shell_Spherical.cpp
// Authors: Qiyuan Zhou
// WaveLab
// Illinois Institute of Technology
// Nov. 14 2019
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
// Code for generating a toroidal ANCF Shell Element
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono_mkl/ChSolverMKL.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/core/ChTimer.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace irr;
using std::cout;
using std::endl;

ChTimer<double> timer;

// If true, let each element compute gravitational forces.
// Otherwise, use generic mesh-level gravity load.
// For ANCF elements, prefer 'true'.
bool element_gravity = true;


double time_step = 1e-4; //[s] Integration step size.
double t_end = 0.5; //[s] length of simulation

ChSolver::Type solver_type = ChSolver::Type::PARDISO; // Solver (PARDISO or GMRES).

// Timestepper (HHT or EULER_IMPLICIT or EULER_IMPLICIT_LINEARIZED). 
// Prefer HHT (or maybe EULER_IMPLICIT).
ChTimestepper::Type timestepper_type = ChTimestepper::Type::HHT; //Timestepper

bool mesh_generate = true;      //Toggle mesh generation
bool mesh_visual = true;        //Toggle visualization of mesh

bool interior_generate = false;  //Toggle whether or not to generate interior
bool interior_visual = true;    //Toggle visualization of interior

bool bot_generate = false;       //Toggle whether or not to generate bots
bool bot_visualize = true;      //Toggle visualization of bots

int main(int argc, char* argv[]) {
    freopen("outputShell.txt", "w", stdout); //Toggle logging to .txt file
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    chrono::SetChronoDataPath("C:/Chrono/Builds/chrono-develop/bin/data/"); //Qiyuan x399
    ChSystemSMC my_system;
    my_system.Set_G_acc(ChVector<>(0,-9.81,0));
    
    //--------------------------------------//
    //      CREATE THE PHYSICAL SYSTEM      //
    //--------------------------------------//
    collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);  // Effective radius of curvature for all SCM contacts.
    //collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);  // max outside detection envelope
    //collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0001);    // max inside penetration
    //my_system.SetMaxPenetrationRecoverySpeed(1);
    //my_system.SetMinBounceSpeed(0.1);
    double sphere_swept_thickness = 0.001;                            // outward additional layer around meshes

    //--------------------------------------//
    //       Create materials               //
    //--------------------------------------//

    // Contact materials
    auto int_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>(); //interior
    int_mat->SetYoungModulus(2e4);
    int_mat->SetFriction(0.2f);
    int_mat->SetRestitution(0);
    int_mat->SetAdhesion(0);

    auto bot_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>(); //big bots
    bot_mat->SetYoungModulus(1e4);
    bot_mat->SetFriction(0.4f);
    bot_mat->SetRestitution(0);
    bot_mat->SetAdhesion(0);

    auto mem_surf = chrono_types::make_shared<ChMaterialSurfaceSMC>(); //membrane
    mem_surf->SetYoungModulus(2e4);
    mem_surf->SetFriction(0.4f);
    mem_surf->SetRestitution(0);
    mem_surf->SetAdhesion(0);

    auto default_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>(); //default
    default_mat->SetYoungModulus(1e4);
    default_mat->SetFriction(0.4f);
    default_mat->SetRestitution(0);
    default_mat->SetAdhesion(0);

    // Membrane
    // Create an orthotropic material. All layers for all elements share the same material.
    double rho_mem = 3000;
    ChVector<> E_mem(2.1e7, 2.1e7, 2.1e7);
    ChVector<> nu_mem(0.3, 0.3, 0.3);
    ChVector<> G_mem(8.0769231e6, 8.0769231e6, 8.0769231e6);
    auto mem_mat = chrono_types::make_shared<ChMaterialShellANCF>(rho_mem, E_mem, nu_mem, G_mem);

    //--------------------------------------//
    //        Settings for membrane         //
    //--------------------------------------//

    double dz = 0.001;  // [m] thickness of shell
    double a_damp = 0.00; //alpha damping term for shell

    // Size of torus
    double c = 0.3;  // [m]: radius from center of hole to center of tube
    double a = 0.05;  // [m]: radius of tube

    // Discretization of torus **Must be divisible by 4**
    int n_c = 16;              // Half the discretizations in large structure (X-Z plane)
    int n_a = 8;              // Half the discretizations in torus tube (along cirumference of small tube)
    int tot = 4 * n_c * n_a;  // Total discretizations in torus

    // Initial height of objects
    double init_height =  0.5*a;

    // Logic check
    if (a >= c) {
        cout << "a cannot be larger than c"
            << "\n";
        return 0;
    }

    //--------------------------------------//
	//        Settings for interior         //
	//--------------------------------------//

	// Size of discrete interior particles
	double int_radius = 0.01; // [m] radius
	double mass_int = (0.95 / 1000); //[kg]
    auto int_diaginertia = ChVector<double>(1e-8 * (60.27, 60.27, 60.27)); //Rotational mass inertia

	//--------------------------------------//
	//        Settings for big bots         //
	//--------------------------------------//

    // Size and mass properties
    double bot_radius = 0.95*a; // [m] radius
    double bot_mass = (0.95 / 1000); //[kg]
    auto bot_diaginertia = ChVector<double>(60.27 * (1e-8), 60.27 * (1e-8), 60.27 * (1e-8)); //Rotational mass inertia

    
    int bot_number = 8; // Number of big bots, evenly spaced around the circle
    double bot_angle = 2 * tan(bot_radius / c); //[rad] angle taken up by each robot
    double bot_seperation = (CH_C_2PI/bot_number)-bot_angle; //[rad] angle of free space between each robot
       
    //--------------------------------------//
    //        Create FEA nodes              //
    //--------------------------------------//

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto mem_mesh = chrono_types::make_shared<ChMesh>();
    int num_nodes = 0;
    for (double u = 0; u < CH_C_2PI; u = u + CH_C_PI / n_c) {
        for (double v = 0; v < CH_C_2PI; v = v + CH_C_PI / n_a) {
            // Point on torus
            double x = cos(u) * (c + a * cos(v));
            double y = a * sin(v) + init_height;
            double z = sin(u) * (c + a * cos(v));
            ChVector<double> location(x, y, z);

            
            // Tangent with respect to parameter u
            double t1_x = sin(u) * (-(a * cos(v) + c));
            double t1_y = 0;
            double t1_z = cos(u) * (a * cos(v) + c);

            // Tangent with respect to parameter v
            double t2_x = -a * cos(u) * sin(v);
            double t2_y = a * cos(v);
            double t2_z = -a * sin(u) * sin(v);

            // Normal vector components (cross product of tangents)
            double uvi = t1_y * t2_z - t2_y * t1_z;
            double uvj = t2_x * t1_z - t1_x * t2_z;
            double uvk = t1_x * t2_y - t2_x * t1_y;

            ChVector<> N(uvi, uvj, uvk);
            N.Normalize();
            
            
            //double dx = -cos(v) * cos(u);
           // double dy = -sin(v);
           // double dz = -cos(v) * sin(u);
            //ChVector<> N(dx, dy, dz);
            

            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(x, y, z), N);
            node->SetMass(0);
            mem_mesh->AddNode(node);
            num_nodes++;
        }
    } 

    cout << "num_nodes= " << num_nodes << "\n";

    // Get a handle to the tip node.
    auto first_node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(0));

    //--------------------------------------//
    //         Adding ANCF elements         //
    //--------------------------------------//
    double dx = CH_C_2PI * (c + a) / (2 * n_c);
    double dy = CH_C_PI * a / n_a;

    int reg_elements = 0;
    int R_elements = 0;
    int r_elements = 0;
    // Regular elements:
    for (int i = 0; i < (2 * n_c) - 1; i++) {
        for (int j = 0; j < (2 * n_a) - 1; j++) {
            // Adjacent nodes
            int node0 = (i * n_c) + j;
            int node1 = (i * n_c) + j + (2 * n_a);
            int node2 = (i * n_c) + j + (2 * n_a) + 1;
            int node3 = (i * n_c) + j + 1;
            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node0)),
                std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node1)),
                std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node2)),
                std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node3)));

            element->SetDimensions(dx, dy);             // Set element dimensions
            element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mem_mat);  // Single layer with 0deg. fiber angle
            element->SetAlphaDamp(a_damp);                           // Structural damping for this element
            element->SetGravityOn(element_gravity);  // turn internal gravitational force calculation on/off
            mem_mesh->AddElement(element);
            reg_elements++;
        }
    }
    cout << "reg_elements= " << reg_elements << "\n";

    // End (large R) elements:
    for (int l = 0; l < 2 * n_a - 1; l++) {
        // Adjacent nodes
        int node0 = (4 * n_c * n_a) - (2 * n_a) + l;
        int node1 = l;
        int node2 = l + 1;
        int node3 = (4 * n_c * n_a) - (2 * n_a) + 1 + l;
        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementShellANCF>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node0)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node1)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node2)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node3)));

        element->SetDimensions(dx, dy);             // Set element dimensions
        element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mem_mat);  // Single layer with 0deg. fiber angle
        element->SetAlphaDamp(a_damp);                           // Structural damping for this element
        element->SetGravityOn(element_gravity);               // turn internal gravitational force calculation on/off
        mem_mesh->AddElement(element);
        R_elements++;
    }
    cout << "R_elements= " << R_elements << "\n";
    
    // Ends (small r) elements:
    for (int m = 1; m < (2 * n_c); m++) {
        // Adjacent nodes
        int node0 = (m * n_c) - 1;
        int node1 = (m * n_c) + (2 * n_a) - 1;
        int node2 = ((m - 1) * n_c) + (2 * n_a);
        int node3 = ((m - 1) * n_c);
        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementShellANCF>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node0)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node1)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node2)),
            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(node3)));

        element->SetDimensions(dx, dy);             // Set element dimensions
        element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mem_mat);  // Single layer with 0deg. fiber angle
        element->SetAlphaDamp(a_damp);                           // Structural damping for this element
        element->SetGravityOn(element_gravity);               // turn internal gravitational force calculation on/off
        mem_mesh->AddElement(element);
        r_elements++;
    }
    cout << "r_elements= " << r_elements << "\n";
    cout << "tot_elements= " << r_elements+R_elements+reg_elements+1 << "\n";
    
    // End element(R and r)
    auto element = chrono_types::make_shared<ChElementShellANCF>();
    element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(tot - 1)),
        std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode((n_a * 2) - 1)),
        std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode(0)),
        std::dynamic_pointer_cast<ChNodeFEAxyzD>(mem_mesh->GetNode((2 * n_c) * n_c - 2 * n_a)));

    element->SetDimensions(dx, dy);             // Set element dimensions
    element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mem_mat);  // Single layer with 0deg. fiber angle
    element->SetAlphaDamp(a_damp);                           // Structural damping for this element
    element->SetGravityOn(element_gravity);               // turn internal gravitational force calculation off
    mem_mesh->AddElement(element);

    // Enable/disable mesh-level automatic gravity load
    mem_mesh->SetAutomaticGravity(!element_gravity);
    
 /*
    // FEA mesh contact surface
    auto mcontactsurf = chrono_types::make_shared<ChContactSurfaceMesh>();
    mem_mesh->AddContactSurface(mcontactsurf);
    mcontactsurf->AddFacesFromBoundary(sphere_swept_thickness);  // do this after my_mesh->AddContactSurface
    mcontactsurf->SetMaterialSurface(mem_surf);            // use the SMC penalty contacts
  */
    
    // FEA node contacts
    auto mcontactcloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>();
    mem_mesh->AddContactSurface(mcontactcloud);
    mcontactcloud->AddAllNodes(dz);  // use larger point size to match beam section radius
    mcontactcloud->SetMaterialSurface(mem_surf);
    
    //--------------------------------------//
    //           ANCF Visualization         //
    //--------------------------------------//

    if(mesh_visual==true){
        
        auto mvisualizemesh = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mem_mesh.get()));
        mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
        mvisualizemesh->SetColorscaleMinMax(0.0, 0.2);
        mvisualizemesh->SetShrinkElements(true, 0.85);
        mvisualizemesh->SetSmoothFaces(true);
        mem_mesh->AddAsset(mvisualizemesh);

        auto mvisualizemeshcoll = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mem_mesh.get()));
        mvisualizemeshcoll->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_CONTACTSURFACES);
        mvisualizemeshcoll->SetWireframe(true);
        mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
        mem_mesh->AddAsset(mvisualizemeshcoll);

        auto mvisualizemeshC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mem_mesh.get()));
        mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
        mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
        mvisualizemeshC->SetSymbolsThickness(0.004);
        mem_mesh->AddAsset(mvisualizemeshC);

        auto mvisualizemeshD = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mem_mesh.get()));
        //mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
        mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
        //mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
        mvisualizemeshD->SetSymbolsScale(1);
        mvisualizemeshD->SetColorscaleMinMax(-0.5, 0.5);
        mvisualizemeshD->SetZbufferHide(false);
        mem_mesh->AddAsset(mvisualizemeshD);
    }

    if(mesh_generate==true){
        my_system.Add(mem_mesh);  // Add the mesh to the system
    }
    
    //--------------------------------------//
    //            Generating bots           //
    //--------------------------------------//

    //properties for interior generation
    int bot_counter = 0;

    for (double u = 0; u < CH_C_2PI; u = u + CH_C_2PI / bot_number) {

        double x = c * cos(u);
        double y = init_height;
        double z = c * sin(u);
        
        auto bot = chrono_types::make_shared<ChBodyEasySphere>(bot_radius, 1000, true, bot_visualize);
        bot->SetMaterialSurface(bot_mat);
        bot->SetPos(ChVector<>(x, y, z));
        bot->SetId(bot_counter);
        bot_counter++;

        if (bot_generate == true) {
            my_system.Add(bot);
        }
    }

    //--------------------------------------//
    //          Generating interior         //
    //--------------------------------------//

    auto ball = chrono_types::make_shared<ChParticlesClones>();
    ball->SetMass(mass_int);
    ball->SetInertiaXX(int_diaginertia);
    ball->SetMaterialSurface(int_mat);

    //Collision shape (shared by all particle clones) Must be defined BEFORE adding particles
    ball->GetCollisionModel()->ClearModel();
    ball->GetCollisionModel()->AddSphere(int_radius);
    ball->GetCollisionModel()->BuildModel();
    ball->SetCollide(true);

    
    //properties for interior generation
    double int_c = (0.95*CH_C_PI * c / int_radius);

    int numInterior = 1;


    for(double torus = bot_angle/2; torus <= CH_C_2PI - bot_angle; torus = torus + bot_seperation + bot_angle){

    for (double n = 0; n <= 0.95 * a - int_radius; n = n + 2 * int_radius) { //for each ring
        double int_a=(.95 * CH_C_PI * (n / int_radius));
        if (int_a == 0) {
            int_a = 1;
        }
        for (double u = torus; u < torus+bot_seperation; u = u + CH_C_2PI / int_c) {        //along tube direction
            for (double v = 0; v < CH_C_2PI; v = v + CH_C_2PI / int_a) {    //along large structure
                // Point on torus
                double x = cos(u) * (c + n * cos(v));
                double y = n * sin(v) + init_height;
                double z = sin(u) * (c + n * cos(v));
                ball->AddParticle(ChCoordsys<>(ChVector<>(x, y, z)));
                numInterior++;
            }
        }
    }
    }
    if(interior_visual==true){
        //Visualization shape (shared by all particle clones)
        auto ball_shape = chrono_types::make_shared<ChSphereShape>();
        ball_shape->GetSphereGeometry().rad = int_radius;
        ball->AddAsset(ball_shape);
        ball->GetAssets().push_back(ball_shape);
    }
    

    if (interior_generate == true) {
        my_system.Add(ball); //Add to system
        cout << "Number of interior particles= " << numInterior << "\n";
    }

    //--------------------------------------//
    //        Generating environment        //
    //--------------------------------------//

    // Create a floor as a simple collision primitive:
    auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(5, 0.1, 5, 700, true);
    mfloor->SetPos(ChVector<>(0, -0.1, 0));
    mfloor->SetBodyFixed(true);
    mfloor->SetMaterialSurface(default_mat);
    my_system.Add(mfloor);

    auto masset_texture = chrono_types::make_shared<ChTexture>();
    masset_texture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    mfloor->AddAsset(masset_texture);

    // two falling objects:
    auto mcube = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 5000, true);
    mcube->SetPos(ChVector<>(0.6, init_height, 0.6));
    mcube->SetMaterialSurface(default_mat);
    my_system.Add(mcube);

    auto msphere = chrono_types::make_shared<ChBodyEasySphere>(0.1, 5000, true);
    msphere->SetPos(ChVector<>(0.8, init_height, 0.6));
    msphere->SetMaterialSurface(default_mat);
    my_system.Add(msphere);

    //--------------------------------------//
    //   Create the Irrlicht visualization  //
    //--------------------------------------//

    ChIrrApp application(&my_system,                         // Target system
        L"FEA contacts",                    // Window title
        core::dimension2d<u32>(1200, 800),  // Window dimensions
        false,                              // Fullscreen?
        true,                              // Shadows?
        true,                              // Anti-Aliasing?
        video::EDT_OPENGL);                 // Graphics Driver
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, (f32)0.6, -1));
    application.AddLightWithShadow(core::vector3df(1.5, 5.5, -2.5), core::vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512,
        video::SColorf(1, 1, 1));
    //application.SetContactsDrawMode(ChIrrTools::CONTACT_DISTANCES);
    application.SetContactsDrawMode(ChIrrTools::CONTACT_FORCES);
    application.SetSymbolscale(0.01f);
    application.AssetBindAll();
    application.AssetUpdateAll();
    application.AddShadowAll();

    //--------------------------------------//
    //         Running the simulation       //
    //--------------------------------------//

    switch (solver_type) {
    case ChSolver::Type::PARDISO: {
        cout << "Using PARDISO solver" << endl;
        auto solver = chrono_types::make_shared<ChSolverMKL>();
        my_system.SetSolver(solver);
        solver->UseSparsityPatternLearner(true);
        solver->LockSparsityPattern(true);
        solver->SetVerbose(false);
        break;
    }
    case ChSolver::Type::GMRES: {
        cout << "Using GMRES solver" << endl;
        auto solver = chrono_types::make_shared<ChSolverGMRES>();
        my_system.SetSolver(solver);
        solver->SetMaxIterations(150);
        solver->SetTolerance(1e-10);
        solver->EnableDiagonalPreconditioner(true);
        if (timestepper_type == ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED)
            solver->EnableWarmStart(true);
        solver->SetVerbose(false);
        break;
    }
    default:
        cout << "\nUnknown solver (use PARDISO or GMRES)\n" << endl;
        return 1;
    }

    switch (timestepper_type) {
    case ChTimestepper::Type::HHT: {
        cout << "Using HHT timestepper" << endl;
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
        break;
    }
    case ChTimestepper::Type::EULER_IMPLICIT: {
        cout << "Using EULER_IMPLICIT timestepper" << endl;
        my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
        break;
    }
    case ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED: {
        cout << "Using EULER_IMPLICIT_LINEARIZED timestepper" << endl;
        my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
        break;
    }
    default:
        cout << "\nUnknown timestepper (use HHT or EULER_IMPLICIT or EULER_IMPLICIT_LINEARIZED)\n" << endl;
        return 1;
    }

    application.SetTimestep(time_step);
    application.SetTryRealtime(false);

    while (application.GetDevice()->run()) {
        double t = my_system.GetChTime();
        cout << "\n" << "Simulation Time= " << t <<"s"<<"\n";
        timer.start();
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
        timer.stop();
        cout << "Time elapsed= " << timer.GetTimeMillisecondsIntermediate() << "ms" << "\n";

        if (t > t_end) {
            return 0;
        }
    }
    return 0;
}