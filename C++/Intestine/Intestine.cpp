//=============================================================================
// Intestine.cpp
// Author: Qiyuan Zhou
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
// Simulation of the intestine robot via particles and springs
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/ChTimerParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkSpring.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/physics/ChForce.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;
using std::cout;
using std::cin;
using std::endl;

ChTimer<double> timer; //Timer object

// Define some global parameters

// Tilt angle (about global Y axis) of the floor.
double tilt_angle = 180 * CH_C_PI / 180;

// Toggle generation and visualization
bool mem_generate = true;
bool int_generate = true;
bool bot_generate = true;

// Membrane Settings
const float mem_radius = 5e-3;      // [m]
const float mem_rho = 2e2;          // [kg/m^3]
const float mem_cohesion = 1e9;     // [N] Cohesion between membrane particles
ChVector<> mem_inertia = 1e-5 * ChVector<>(1, 1, 1); //Roational mass inertia matrix

const float c = 0.2;    //[m]: radius from center of hole to center of tube
const float a = 0.05;   //[m]: radius of the tube
float aa = a;           //[m]: non-constant version of a
float init_height = 2.2 * a; //Initial height

//Factors of 360: {1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 15, 18, 20, 24, 30, 36, 40, 45, 60, 72, 90, 120, 180}
const int n_c = 60;         //Half the discretizations in large structure (X-Z plane)
const int n_a = 18;         //Half the discretizations in torus tube (along cirumference of small tube)

//Spring Settings
const float rl= mem_radius;     //[m]: resting length of springs
float k1 = 7500;               //[N/m]: Straight springs
float b1 = k1/10;                   //[N*s/m]: Straight springs
float k2 = 5000;                //[N/m]: Diagonal springs
float b2 = k2/10;                   //[N*s/m]: Diagonal springs

//Pressure settings
float pressure = 20e3;             //[Pa]: Desired jamming pressure

// Material settings
const float mem_friction = 0.4;         //[1]
const float mem_compliance = 0.0001;   //[mm/N]

// Big bot settings
const int bot_number = 8;               // Number of big bots, evenly spaced around the circle
const double bot_mass = 0.2;            //[kg] mass of bots
const double bot_radius = 0.8*a;       //[m] radius of bots
ChVector<> bot_inertia = 0.0005 * ChVector<>(1, 1, 1);  //Diagonal moment of inertia of bots

//Interior particle settings
const double int_mass = 0.95/1000;                          //[kg] mass of interior particles
const double int_radius = 0.01;                             //[m] radius of interior particles
ChVector<> int_inertia = 6027e-10 * ChVector<>(1, 1, 1);    //Diagonal moment of inertia of interior particles
const double int_c = (0.95 * CH_C_PI * c / int_radius);     //not quite sure what this parameter is exactly

//Calculated parameters
const double bot_angle = 2 * tan(bot_radius / c); //[rad] angle taken up by each robot
const double bot_seperation = (CH_C_2PI / bot_number) - bot_angle; //[rad] angle of free space between each robot

//Empty vectors
std::vector<std::shared_ptr<ChForce>> pressure_force;    // vector to store pressure force terms
std::vector<std::shared_ptr<ChForce>> bot_forces;    // vector to store bot force terms

//==============================================================================================

//Find the surface normals
ChVector<> findNorm(ChVector<> vectorA, ChVector<> vectorB) {
    ChVector<> norm = vectorA.Cross(vectorB);
    return norm.GetNormalized();
}

//Find the magnitude of the pressure force term
float findMag(ChVector<> vectorA, ChVector<> vectorB) {
    //approximate area
    ChVector<> cross = vectorA.Cross(vectorB);
    float area = cross.Length();

    //multiply area by desired pressure, P=F/A
    float force = area * pressure;
    return force;
}

void addPressure(ChSystemParallelNSC* sys, std::vector<std::shared_ptr<ChForce>> pressure) {
    int mem_counter = 0;

    // Loop over all membrane particles
    for (float u = 0; u < CH_C_2PI; u += CH_C_PI / n_c) {
        int k = 0;
        for (float v = 0; v < CH_C_2PI - CH_C_PI / n_a; v += CH_C_PI / n_a) {

            // -----------------------------------------------------------------------------
            // Add forces
            // -----------------------------------------------------------------------------

            // Cases:
            // Regular indexing (in order)
            // Indexing at the ends of small loops
            // Indexing at the end of large loop
            // Indexing at the end of large and small loop

            // Regular indexing (in order)
            if (mem_counter >= 1+2*n_a && mem_counter % (2 * n_a) != 0)
            {
                ChVector<> normal=findNorm(sys->SearchBodyID(mem_counter)->GetPos() - sys->SearchBodyID(mem_counter-1)->GetPos(),
                                           sys->SearchBodyID(mem_counter)->GetPos() - sys->SearchBodyID(mem_counter - (2 * n_a))->GetPos());
                float pforce = findMag(sys->SearchBodyID(mem_counter)->GetPos() - sys->SearchBodyID(mem_counter - 1)->GetPos(),
                                       sys->SearchBodyID(mem_counter)->GetPos() - sys->SearchBodyID(mem_counter - (2 * n_a))->GetPos());
                pressure[mem_counter]->SetDir(normal);
                pressure[mem_counter]->SetMforce(pforce);
            }

            // Indexing at the ends of small loops
            if (mem_counter >= 1 + 2 * n_a && ((mem_counter + 1) % (2 * n_a)) == 0)
            {

                ChVector<> normal = findNorm(sys->SearchBodyID(mem_counter)->GetPos() - sys->SearchBodyID(mem_counter - (2 * n_a) + 1)->GetPos(),
                                             sys->SearchBodyID(mem_counter)->GetPos() - sys->SearchBodyID(mem_counter - (2 * n_a))->GetPos());
                float pforce = findMag(sys->SearchBodyID(mem_counter)->GetPos() - sys->SearchBodyID(mem_counter - (2 * n_a) + 1)->GetPos(),
                                       sys->SearchBodyID(mem_counter)->GetPos() - sys->SearchBodyID(mem_counter - (2 * n_a))->GetPos());
                pressure[mem_counter]->SetDir(normal);
                pressure[mem_counter]->SetMforce(pforce);
            }
            // Increment counters
            k++;
            mem_counter++;
        }
    }
}

void locomote(ChSystemParallelNSC* sys, std::vector<std::shared_ptr<ChForce>> forces) {
    int bot = 0;

    // Loop over all bots
    for (int u = 0; u < forces.size(); u++) {

        // Add forces
        forces[u]->SetMforce(100.);
        forces[u]->SetDir(ChVector<>(1, 0, 0));
    }
}

// Create a floor for robot to live on
void AddFloor(ChSystemParallelNSC* sys) {
    // IDs for the two bodies
    int FloorId = -200;

    // Create a common material
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);
    mat->SetCompliance(mem_compliance);
    mat->SetRestitution(0.8);

    // Create the floor (8 x 8 x 4)
    auto bin = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
    bin->SetMaterialSurface(mat);
    bin->SetIdentifier(FloorId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, -0.1));
    bin->SetRot(Q_from_AngY(tilt_angle));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);
    ChVector<> hdim(3, 3, 1);  //half sizes
    double hthick = 0.05;

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hdim.y(), hthick), ChVector<>(0, 0, -hthick));
    bin->GetCollisionModel()->BuildModel();

    sys->AddBody(bin);
}

// Create boundary particles
void AddMembrane(ChSystemParallelNSC* sys) {
    //Create material and ID
    auto mem_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>(); //membrane
    mem_mat->SetFriction(mem_friction);
    mem_mat->SetCompliance(mem_compliance);
    mem_mat->SetRestitution(0.8);
    mem_mat->SetCohesion(0);
    int mem_counter = 0;

    //Place and create the particles in membrane
    for (float u = 0; u < CH_C_2PI; u += CH_C_PI / n_c) {
        int k = 0;
        for (float v = 0; v < CH_C_2PI-CH_C_PI/n_a; v += CH_C_PI / n_a) {
           
            float x = cos(u) * (c + a * cos(v));
            float y = sin(u) * (c + a * cos(v));
            float z = a * sin(v) + init_height;
            
            auto mem_part = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
            mem_part->SetMaterialSurface(mem_mat);
            mem_part->SetDensity(mem_rho);
            mem_part->SetInertiaXX(mem_inertia);
            mem_part->SetPos(ChVector<>(x, y, z));
            mem_part->SetBodyFixed(false);
            mem_part->SetCollide(true);
            mem_part->SetMaxSpeed(10);
            mem_part->SetIdentifier(mem_counter);
            //mem_part->SetId(mem_counter);

            //Set up collisions
            mem_part->GetCollisionModel()->ClearModel();
            utils::AddSphereGeometry(mem_part.get(), mem_radius);
            mem_part->GetCollisionModel()->BuildModel();

            // Attach a force to the membrane
            auto mem_force = chrono_types::make_shared<ChForce>();
            mem_force->SetMode(ChForce().FORCE);
            mem_part->AddForce(mem_force);
            mem_force->SetMforce(0.f);
            mem_force->SetDir(ChVector<>(x, y, z));
            mem_force->SetIdentifier(mem_counter+5*n_c*n_a);
            pressure_force.push_back(mem_force);
            // -----------------------------------------------------------------------------
            // Add springs
            // -----------------------------------------------------------------------------
                       
            // Small circle direction (k1)
            if (mem_counter >= 1 && mem_counter % (2 * n_a) != 0)
            {
                
                auto spring1 = chrono_types::make_shared<ChLinkTSDA>();
                spring1->Initialize(mem_part, sys->SearchBodyID(mem_counter - 1), true, ChVector<>(0,0,0),ChVector<>(0,0,0),false);
                spring1->SetSpringCoefficient(k1);
                spring1->SetDampingCoefficient(b1);
                spring1->SetRestLength(rl);
                
                /*
                auto spring1 = chrono_types::make_shared<ChLinkSpring>();
                spring1->Initialize(mem_part, sys->SearchBodyID(mem_counter - 1), true, ChVector<>(0, 0, 0), ChVector<>(0, 0, 0), false);
                spring1->Set_SpringK(k1);
                spring1->Set_SpringR(b1);
                spring1->Set_SpringRestLength(rl);
                */
                sys->AddLink(spring1);
            }

            // Small circle direction (k1) ends
            if (mem_counter >= 1 && ((mem_counter + 1) % (2 * n_a)) == 0)
            {
                
                auto spring2 = chrono_types::make_shared<ChLinkTSDA>();
                spring2->Initialize(mem_part, sys->SearchBodyID(mem_counter - (2*n_a)+1), true, ChVector<>(0, 0, 0), ChVector<>(0, 0, 0),false);
                spring2->SetSpringCoefficient(k1);
                spring2->SetDampingCoefficient(b1);
                spring2->SetRestLength(rl);
                //spring2->SetActuatorForce(force);
                
                /*
                auto spring2 = chrono_types::make_shared<ChLinkSpring>();
                spring2->Initialize(mem_part, sys->SearchBodyID(mem_counter - (2 * n_a) + 1), true, ChVector<>(0, 0, 0), ChVector<>(0, 0, 0), false);
                spring2->Set_SpringK(k1);
                spring2->Set_SpringR(b1);
                spring2->Set_SpringRestLength(rl);
                */
                sys->AddLink(spring2);
            }
            
            // Large circle direction (k1)
            if (mem_counter > 2*n_a)
            {
                
                auto spring3 = chrono_types::make_shared<ChLinkTSDA>();
                spring3->Initialize(mem_part, sys->SearchBodyID(mem_counter - (2 * n_a)), true, ChVector<>(0, 0, 0), ChVector<>(0, 0, 0),false);
                spring3->SetSpringCoefficient(k1);
                spring3->SetDampingCoefficient(b1);
                spring3->SetRestLength(rl);
                //spring3->SetActuatorForce(force);
                
                /*
                auto spring3 = chrono_types::make_shared<ChLinkSpring>();
                spring3->Initialize(mem_part, sys->SearchBodyID(mem_counter - (2 * n_a)), true, ChVector<>(0, 0, 0), ChVector<>(0, 0, 0), false);
                spring3->Set_SpringK(k1);
                spring3->Set_SpringR(b1);
                spring3->Set_SpringRestLength(rl);
                */
                sys->AddLink(spring3);
                
            }
            
            // Large circle direction (k1) ends
            int index1 = (4 * n_c * n_a - 2 * n_a);
            if (mem_counter > index1)
            {
                auto spring4 = chrono_types::make_shared<ChLinkTSDA>();
                spring4->Initialize(mem_part, sys->SearchBodyID(mem_counter - index1), true, ChVector<>(0, 0, 0), ChVector<>(0, 0, 0),false);
                spring4->SetSpringCoefficient(k1);
                spring4->SetDampingCoefficient(b1);
                spring4->SetRestLength(rl);

                sys->AddLink(spring4);
            }
            //Add to system
            sys->AddBody(mem_part);
            // Increment counters
            k++;
            mem_counter++;
        }
    }
    cout << "Number of membrane particles: " << mem_counter << "\n";
}

// Create interior particles
void AddInterior(ChSystemParallelNSC* sys) {
// Common material and ID
    auto int_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    int_mat->SetFriction(0.4f);
    int_mat->SetCompliance(mem_compliance);
    int_mat->SetRestitution(0.8);
    int int_id = 0;

    // Create the bots
    for (double torus = bot_angle / 2; torus <= CH_C_2PI - bot_angle; torus = torus + bot_seperation + bot_angle) {

        for (double n = 0; n <= 0.95 * (a - int_radius); n = n + 2 * int_radius) { //for each ring
            double int_a = (.95 * CH_C_PI * (n / int_radius));
            if (int_a == 0) {
                int_a = 1;
            }
            for (double u = torus; u < torus + bot_seperation; u = u + CH_C_2PI / int_c) {        //along tube direction
                for (double v = 0; v < CH_C_2PI; v = v + CH_C_2PI / int_a) {    //along large structure
                    // Point on torus
                    double x = cos(u) * (c + n * cos(v));
                    double y = sin(u) * (c + n * cos(v));
                    double z = n * sin(v) + init_height;
                    

                    auto int_part = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
                    int_part->SetMaterialSurface(int_mat);
                    int_part->SetMass(int_mass);
                    int_part->SetInertiaXX(int_inertia);
                    int_part->SetPos(ChVector<>(x, y, z));
                    int_part->SetBodyFixed(false);
                    int_part->SetId(int_id);
                    int_part->SetCollide(true);

                    //Set up collisions
                    int_part->GetCollisionModel()->ClearModel();
                    utils::AddSphereGeometry(int_part.get(), int_radius);
                    int_part->GetCollisionModel()->BuildModel();

                    //Add to counter and add to system
                    int_id++;
                    sys->AddBody(int_part);
                }
            }
        }
    }
    cout << "Number of interior particles: " << int_id << "\n";
}

// Create big bots
void AddBots(ChSystemParallelNSC* sys, std::vector<std::shared_ptr<ChForce>>& forces) {
    // Common material and ID
    auto bot_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    bot_mat->SetFriction(0.4f);
    bot_mat->SetCohesion(0);
    bot_mat->SetCompliance(mem_compliance);
    bot_mat->SetRestitution(0.8);
    int bot_id = 0;

    // Create the bots
    for (double u = 0; u < CH_C_2PI; u = u + CH_C_2PI / bot_number) {

        double x = c * cos(u);
        double y = c * sin(u);
        double z = init_height;
        
        auto bot = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
        bot->SetMaterialSurface(bot_mat);
        bot->SetMass(bot_mass);
        bot->SetInertiaXX(bot_inertia);
        bot->SetPos(ChVector<>(x, y, z));
        bot->SetBodyFixed(false);
        bot->SetId(bot_id);
        bot->SetCollide(true);

        //Set up collisions
        bot->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(bot.get(), bot_radius);
        bot->GetCollisionModel()->BuildModel();

        // Attach a force to the bot
        auto bot_force = chrono_types::make_shared<ChForce>();
        bot_force->SetMode(ChForce().FORCE);
        bot->AddForce(bot_force);
        bot_force->SetDir(VECT_X);
        forces.push_back(bot_force);

        //Add to counter and add to system
        bot_id++;
        sys->Add(bot);
    }
    cout << "Number of big bots: " << bot_id << "\n";
}


// Create the system, specify simulation parameters, and run simulation loop.
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Error checking:
    if (a > c) {
        char x;
        cout << "a cannot be larger than c!" << "\n"<<"press any key to exit program"<<"\n";
        cin >> x;
        return 1;
    }

    if (360 % n_a != 0)
    {
        char y;
        cout << "n_a must be a factor of 180!" << "\n" << "press any key to exit program" << "\n";
        cin >> y;
        return 2;
    }

    if (360 % n_c != 0)
    {
        char z;
        cout << "n_c must be a factor of 180!" << "\n" << "press any key to exit program" << "\n";
        cin >> z;
        return 3;
    }

    // Simulation parameters
    double gravity = 9.81;
    double time_step = 2e-4;
    double time_end = 2;
    double out_fps = 60;

    uint max_iteration = 300;
    real tolerance = 1e-5;

    // Create system
    ChSystemParallelNSC msystem;
    
    // Set number of threads.
    int threads = 24;
    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    CHOMPfunctions::SetNumThreads(threads);

    msystem.Set_G_acc(ChVector<>(0, 0, -gravity));  // Set gravitational acceleration

    // Set solver parameters
    msystem.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    msystem.GetSettings()->solver.max_iteration_normal = max_iteration/3;
    msystem.GetSettings()->solver.max_iteration_sliding = max_iteration / 3;
    msystem.GetSettings()->solver.max_iteration_spinning = 0;
    msystem.GetSettings()->solver.max_iteration_bilateral = max_iteration / 3;
    msystem.GetSettings()->solver.tolerance = tolerance;
    msystem.GetSettings()->solver.alpha = 0.1;
    msystem.GetSettings()->solver.contact_recovery_speed = 100000;
    msystem.ChangeSolverType(SolverType::APGD);
    msystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    msystem.GetSettings()->collision.collision_envelope = 0.1*mem_radius;
    msystem.GetSettings()->collision.bins_per_axis = vec3(16, 16, 16);

    // Create the fixed and moving bodies
    AddFloor(&msystem);

    if(mem_generate==true)
    {AddMembrane(&msystem);}

    if(bot_generate==true)
    {AddBots(&msystem, bot_forces); }

    if(int_generate==true)
    {AddInterior(&msystem); }

    locomote(&msystem, bot_forces);

    // Perform the simulation
    // ----------------------

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1200, 800, "Intestine", &msystem);
    //gl_window.SetCamera(ChVector<>(2*c, 0, init_height), ChVector<>(0, 0, init_height), ChVector<>(0, 0, 1),0.01f,0.01f);
    gl_window.SetCamera(ChVector<>(0.5, -0.25, 0.375), ChVector<>(0, 0, init_height), ChVector<>(0, 0, 1), 0.01f, 0.01f);
    gl_window.SetRenderMode(opengl::SOLID);

    // Uncomment the following two lines for the OpenGL manager to automatically
    // run the simulation in an infinite loop.
    // gl_window.StartDrawLoop(time_step);
    // return 0;
    float time = 0;
    while (true) {
        if (gl_window.Active()) {
            //time = msystem.GetChTime();
            
            locomote(&msystem,bot_forces);
            
            gl_window.DoStepDynamics(time_step);
            gl_window.Render();
            ////if (gl_window.Running()) {
            ////  msystem.CalculateContactForces();
            ////  real3 frc = msystem.GetBodyContactForce(0);
            ////  std::cout << frc.x << "  " << frc.y << "  " << frc.z << std::endl;
            ////}

            //TODO: Program logic for spring forces

            // Logic for pressure
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
