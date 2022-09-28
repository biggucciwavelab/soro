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
// A very simple example that can be used as template project for
// a Chrono::Engine simulator with 3D view.
// =============================================================================

#include <math.h> //Needed for sin and cos.

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkSpring.h"        //Added to include the spring.
#include "chrono/assets/ChPointPointDrawing.h"  //Visualize a spring as a coil.
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/solver/ChSolverPSSOR.h"
#include "chrono/core/ChTimer.h"

// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

// -----------------------------------------------------------------------------
// Global functions and variables
// -----------------------------------------------------------------------------

//Time Values
double tstep = 0.005;   //Time Step
double tset = 0.01;     //settling time
double tend = 9;        //Length of simulation
double tp = 7;          //Time to pull
double tj = 5;          //Jaming starts
double t = 0;           //Simulation time.

// Parameters for robot
double nb = 100;        // # of robots
double diameter = 0.07; // Diameter of interior cylinder
double R1 = ((diameter * nb) / (CH_C_PI * 2)) + .1;
std::vector<int> in_ring = { 104,97,91,85,78,72,66,60,53,47,41,34,28,22,16,9,3 }; //Ring pattern of interior.
int num_ring = in_ring.size();
float mu_f = 0.4f;
double mr = 0.18; //Mass of robots.
double mp = 0.03; //Mass of particles.
double mb = 0.3; //Mass of ball.
double height = 0.12; //Height of cylinder. For both bots and particles.
double hhalf = height / 2; //Half height of cylinder
double k = -0.5; //Spring constant (on the bots).

double kj = -30; //Jamming spring constant
double rl = 0.002; //Resting length
double rlmax = 0.02; //Max resting length
double rlj = 0; //Desired length jammed
double rljmax = 0.0001; //Max desired length jammed
double volume = CH_C_PI * (0.25) * height * (pow(diameter, 2));

double ratio = 0.25; //
double Rb = ratio * R1; //Radius of the ball
double offset = Rb + R1; //Offset of ball position
double rx = offset + 0.5; //x-position of ball
double ry = hhalf; //y-position of ball
double rz = 0; //z-position of ball

double volume_ball = CH_C_PI * 0.25 * height * pow(Rb * 2, 2); //Volume of ball
double rho_r = mr / volume; //Density of robot
double rho_p = mp / volume; //Density of particles
double rho_b = mb / volume_ball; //Density of ball

//External forces -I am a little confused on this part. -EL
double mag = 0; //[N] - magnitude of external force applied at each bot
double magf = (9.81) * mb * mu_f; //Force from frction an weight.
double magd = 200; //Desired Force
double mag2 = magd + magf; //Compensated force to account for friction.
double mag3 = 3;

//Some empty matrices for things to be collected in
std::vector<std::shared_ptr<ChBody>> obj;           // Bots and particles
std::vector<std::shared_ptr<ChBodyEasyBox>> bots;   // Bots
std::vector<std::shared_ptr<ChLinkSpring>> Springs; // Springs
std::vector<std::shared_ptr<ChForce>> forceb;       // Forces on the ball
std::vector<std::shared_ptr<ChForce>> force;        // External force objects
std::vector<double> templ;                          // Spring lengths
std::vector<double> Fm;                             // Force in the springs.

//Vectors with zeroes
std::vector<int> jamcall(nb, 0);
std::vector<int> botcall(nb, 0);

ChTimer<double> timer_step;     // Timer object for each time step
ChTimer<double> timer_total;    // Timer object for the total runtime

/*
//ID Number of robots to be active.
auto active = arange<int>(0, nb);
auto sactive1 = arange<int>(88, 99);
auto sactive2 = arange<int>(0, 12);
std::vector<int> sactive = sactive1;
sactive.insert(sactive.end(), sactive2.begin(), sactive2.end()); //Mimicing the hstack function in numpy.

//Active robots
for (int i = 0; i < sactive.size(); i++) {
    jamcall[i] = 1;
}

//For robots that are active fill botcall==1
for (int i = 0; i < active.size(); i++) {
    botcall[i] = 1;
}
*/

// -----------------------------------------------------------------------------
// Functions to be called on later
// -----------------------------------------------------------------------------

//Jam the springs
void Jamsprings(double& k, double& rl, double& rlmax, std::vector<std::shared_ptr<ChLinkSpring>> Springs, double t, double tj, std::vector<double> Fm, double kj, double rlj, double rljmax, double i, double jamcall) {
    if (t > tj) {
        if (jamcall == 1) {
            k = kj;
            rl = rlj;
            rlmax = rljmax;
        }
        else {
            k = k;
            rl = rl;
            rlmax = rlmax;
        }
    }
    //return k, rlmax, rl; Not needed, as the values of k, rlmax, and rl are referenced in this function.
}

// Set the spring rate dynamically
void setSpring(double k, double rl, double rlmax, std::vector<std::shared_ptr<ChLinkSpring>>& Springs, std::vector<double>& Fm, std::vector<double>& templ, int i) {
    double var1 = Springs[i]->Get_SpringLength();

    if (var1 < rl) {
        Springs[i]->Set_SpringF(0);
        double var2 = Springs[i]->Get_SpringF();
        Fm.push_back(var2);
    }

    if (var1 > rlmax) {
        Springs[i]->Set_SpringF(10 * k);
        double var2 = Springs[i]->Get_SpringF();
        Fm.push_back(var2);
    }

    else {
        Springs[i]->Set_SpringF(k);
        double var2 = Springs[i]->Get_SpringF();
        Fm.push_back(var2);
    }

    templ.push_back(var1);
}

// Todo: Centroid function
void centroid(std::vector<std::shared_ptr<ChBody>> obj) {

}

// Controller function
void Controller(ChSystemNSC& my_system, std::vector<std::shared_ptr<ChForce>>& force, double mag, std::vector<int> botcall, double tset, std::vector<double>& Fm,
    double nb, double k, std::vector<double>& templ, double t, double rl, double rlmax, 
    std::vector<std::shared_ptr<ChLinkSpring>>& Springs, std::vector<std::shared_ptr<ChForce>>& forceb,std::vector<int> jamcall,
    double rlj, double rljmax, double tj, double kj, double mag2, double magf, double tp,std::vector<std::shared_ptr<ChBody>> obj,
    double mag3,double Xpos,double Ypos,double Zpos,double tstep,double height,double xx,double zz) {
    
    for (int ii = 0; ii < (nb - 1); ii++) {
        Jamsprings(k, rl, rlmax, Springs, t, tj, Fm, kj, rlj, rljmax, ii, jamcall[ii]);
        setSpring(k, rl, rlmax, Springs, Fm, templ, ii);

        //If past settling time
        if (t > tset) { 
            for (int j = 0; j < force.size(); j++) {
                force[j]->SetMforce(mag);
                force[j]->SetDir(VECT_X);
            }
            for (int j = 0; j < forceb.size(); j++) {
                forceb[j]->SetMforce(-mag2);
                forceb[j]->SetDir(ChVector<>(1, 0, 0));
            }
        }
        
        //If past jamming time
        if (t > tj) {
            //skipping centroid and direction functions
            /* Note: Skipping this, as xd,yd,and zd cannot be called upon currently without Control XPos, YPos, and ZPos put in. -EL
            for (int l = 0; l < force.size(); l++) {
                force[l]->SetMforce(mag3);
                force[l]->SetDir(ChVector<>(xd, yd, zd));
                
            }*/
            for (int l = 0; l < forceb.size(); l++) {
                forceb[l]->SetMforce(-mag2);
                forceb[l]->SetDir(ChVector<>(1, 0, 0));
            }
        }

        //If past pulling time
        if (t > tp) {
            //skipping centroid and direction functions
            /* Note: Skipping this, as xd,yd,and zd cannot be called upon currently without Control XPos, YPos, and ZPos put in. -EL
            for (int l = 0; l < force.size(); l++) {
                force[l]->SetMforce(mag3);
                force[l]->SetDir(ChVector<>(xd, yd, zd));
            }
            */
            for (int l = 0; l < forceb.size(); l++) {
                forceb[l]->SetMforce(1.5 * magf);
                forceb[l]->SetDir(ChVector<>(1, 0, 0));
            }
        }

    }
}

// Todo: Data extraction
/*
void ExtractData(obj,nb,nt,Xpos,Zpos,Xforce,Yforce,Zforce,rott0,rott1,rott2,rott3,Xvel,Yvel,Zvel,ballp,Balls,Springs,Fm,templ) {

}
*/

template<typename T>  //Create the equivalent of numpy's arange function.
std::vector<T> arange(T start, T stop, T step = 1) {
    std::vector<T> values;
    for (T value = start; value < stop; value += step)
        values.push_back(value);
    return values;
}
//========================================================================

// Create a floor for robot to live on
void AddFloor(ChSystemNSC* my_system) {
    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    floor_mat->SetFriction(0.3f);
    floor_mat->SetDampingF(0.01f);
    floor_mat->SetRollingFriction(0.1f);
    floor_mat->SetSpinningFriction(0.1f);

    floor_mat->SetCompliance(0.0001f); //Do we need the compliance values if they are so small? -EL
    floor_mat->SetComplianceT(0.0001f);
    floor_mat->SetComplianceRolling(0.0001f);
    floor_mat->SetComplianceSpinning(0.0001f);

    auto floorBody = std::make_shared<ChBodyEasyBox>(8, .1, 8,  // x (length), y (height), z (width) dimensions
        3000,       // density
        true,       // contact geometry
        true        // enable visualization geometry
        );
    floorBody->SetPos(ChVector<>(0, -.1 / 2, 0));
    floorBody->SetMaterialSurface(floor_mat);
    floorBody->SetIdentifier(-1);
    floorBody->SetBodyFixed(true); //It is not moving!

      // Attach a RGB color asset to the floor.
    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(0.2f, 0.25f, 0.25f));
    floorBody->AddAsset(color);
    my_system->Add(floorBody);
}

// Create the bots with springs
void AddBots(ChSystemNSC* my_system) {

    // Make the material
    auto bot_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    bot_mat->SetFriction(mu_f);
    bot_mat->SetDampingF(0.01f);
    bot_mat->SetRollingFriction(0.1f);
    bot_mat->SetSpinningFriction(0.01f);

    bot_mat->SetCompliance(0.0001f); //Do we need the compliance values if they are so small? -EL
    bot_mat->SetComplianceT(0.0001f);
    bot_mat->SetComplianceRolling(0.0001f);
    bot_mat->SetComplianceSpinning(0.0001f);

    double p1, p2, p3, p4, h; //Points that the spring will be attached to.
    p1 = 0;
    p2 = diameter / 2;
    p3 = 0;
    p4 = -diameter / 2;
    h = 0;

    for (int i = 0; i < nb; i++) {
        double theta = i * ((2 * CH_C_PI) / nb);
        double x_pos = R1 * cos(theta);
        double y_pos = (height / 2); //As measured from COG of robot.
        double z_pos = R1 * sin(theta);

        auto bot = std::make_shared<ChBodyEasyBox>(diameter, height, diameter, rho_r, true, true); //Creating a bot using EasyBox. This already includes a collision model.
        bot->SetPos(ChVector<>(x_pos, y_pos, z_pos));   //Setting Bot position
        bot->SetMaterialSurface(bot_mat);               //Setting bot material

        auto rotation = ChQuaternion<>();
        rotation.Q_from_AngAxis(-theta, ChVector<>(0, 1, 0));
        bot->SetRot(rotation);      //Rotate the bot.

        bot->SetId(i);              //Setting the Id for the bot.
        bot->SetBodyFixed(false);   //Make sure the bots can move around.

        //Mating the bots to the floor so they don't tip.
        auto pt = chrono_types::make_shared<ChLinkMatePlane>();
        pt->Initialize(my_system->SearchBodyID(-1), bot, true, ChVector<>(0, 0, 0), ChVector<>(0, -height / 2, 0), ChVector<>(0, -1, 0), ChVector<>(0, 1, 0));
        my_system->AddLink(pt);     //Adding the link to the system.

        auto color_bot = std::make_shared<ChColorAsset>();
        color_bot->SetColor(ChColor(0.44f, 0.11f, 52));
        bot->AddAsset(color_bot);
        bots.push_back(bot);
        obj.push_back(bot);

        //Adding force objects to the bots
        if (botcall[i] == 1) {
            auto myforcex = chrono_types::make_shared<ChForce>();
            myforcex->SetMode(ChForce().FORCE);
            bot->AddForce(myforcex); //This MUST be done before setting force settings! If it isn't, the executable fails.
            myforcex->SetDir(VECT_X);
            myforcex->SetVrelpoint(ChVector<>(x_pos, 0.3 * y_pos, z_pos));
            myforcex->SetMforce(mag);
            force.push_back(myforcex);
        }

        //Creating Springs
        auto spring = chrono_types::make_shared<ChLinkSpring>();
        spring->SetName("spring");

        auto color_spring = std::make_shared<ChColorAsset>();
        color_spring->SetColor(ChColor(1, 0.75, 0));

        //Most of the springs are added in this loop.
        if (i >= 1) {
            spring->Initialize(bots[i - 1], bot, true, ChVector<>(p1, h, p2), ChVector<>(p3, h, p4), false);
            spring->Set_SpringF(k);
            spring->Set_SpringRestLength(rl);
            spring->AddAsset(color_spring);
            spring->AddAsset(std::make_shared<ChPointPointSpring>(0.01, 80, 15));
            my_system->AddLink(spring);
            Springs.push_back(spring);
        }

        //Make the first bot pink.
        if (i == 0) {
            auto color_first_bot = std::make_shared<ChColorAsset>();
            color_first_bot->SetColor(ChColor(0.9969, 0.078, 0.57422));
            bot->AddAsset(color_first_bot);
        }

        //Make the last bot green.
        if (i == nb-1) {
            auto color_last_bot = std::make_shared<ChColorAsset>();
            color_last_bot->SetColor(ChColor(0, 0.996, 0));
            bot->AddAsset(color_last_bot);
        }
    }

    //Connects the last bot to the first one.
    auto spring = chrono_types::make_shared<ChLinkSpring>();
    spring->SetName("spring");

    auto color_spring = std::make_shared<ChColorAsset>();
    color_spring->SetColor(ChColor(1, 0.75, 0));

    spring->Initialize(bots[nb - 1], bots[0], true, ChVector<>(p1, h, p2), ChVector<>(p3, h, p4), false);
    spring->Set_SpringF(k);
    spring->Set_SpringRestLength(rl);
    spring->AddAsset(color_spring);
    spring->AddAsset(std::make_shared<ChPointPointSpring>(0.01, 80, 15));
    my_system->AddLink(spring);
    Springs.push_back(spring);


    //Add all the robots to the system.
    for (int i = 0; i < nb; i++) {
        auto robot = bots[i];
        my_system->Add(robot);
    }
}

// Create the interior
void AddInterior(ChSystemNSC* my_system) {
    // Make the material
    auto bot_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    bot_mat->SetFriction(mu_f);
    bot_mat->SetDampingF(0.01f);
    bot_mat->SetRollingFriction(0.1f);
    bot_mat->SetSpinningFriction(0.01f);

    bot_mat->SetCompliance(0.0001f); //Do we need the compliance values if they are so small? -EL
    bot_mat->SetComplianceT(0.0001f);
    bot_mat->SetComplianceRolling(0.0001f);
    bot_mat->SetComplianceSpinning(0.0001f);

    int ni = 0;             //Number of interiors (filled with loop below).
    for (int i = 0; i < num_ring; i++)
    {
        ni += in_ring[i];
    }
    int nt = ni + nb; //Total number of bots and particles.
    double nr = nb / ni; //Ratio of nb over ni.

    for (int i = 0; i < num_ring; i++) {
        double R2 = (diameter * in_ring[i]) / (2 * CH_C_PI);

        for (int j = 0; j < in_ring[i]; j++) {
            double x_pos = R2 * cos(j * 2 * CH_C_PI / in_ring[i]);
            double y_pos = height / 2;
            double z_pos = R2 * sin(j * 2 * CH_C_PI / in_ring[i]);

            auto particle = std::make_shared<ChBodyEasyCylinder>(diameter / 2, height, rho_p, true, true);
            particle->SetPos(ChVector<>(x_pos, y_pos, z_pos));
            particle->SetMaterialSurface(bot_mat);

            //Skipped creating a collision model. Collision model assumed from ChEasyBody.

            auto color_particle = std::make_shared<ChColorAsset>();
            color_particle->SetColor(ChColor(0.996, 0.996, 0));
            particle->AddAsset(color_particle);

            auto pt = chrono_types::make_shared<ChLinkMatePlane>(); //Mating the bots to the floor so they don't tip.
            pt->Initialize(my_system->SearchBodyID(-1), particle, true, ChVector<>(0, 0, 0), ChVector<>(0, -height / 2, 0), ChVector<>(0, -1, 0), ChVector<>(0, 1, 0));
            my_system->AddLink(pt); //Adding the link to the system.

            obj.push_back(particle);
            my_system->Add(particle);
        }
    }
}

// -----------------------------------------------------------------------------
// Create the ball
// -----------------------------------------------------------------------------
void AddBall(ChSystemNSC* my_system) {
    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    floor_mat->SetFriction(0.3f);
    floor_mat->SetDampingF(0.01f);
    floor_mat->SetRollingFriction(0.1f);
    floor_mat->SetSpinningFriction(0.1f);

    floor_mat->SetCompliance(0.0001f); //Do we need the compliance values if they are so small? -EL
    floor_mat->SetComplianceT(0.0001f);
    floor_mat->SetComplianceRolling(0.0001f);
    floor_mat->SetComplianceSpinning(0.0001f);

    auto ball = std::make_shared<ChBodyEasyCylinder>(Rb, height, rho_b, true, true);
    ball->SetPos(ChVector<>(rx, ry, rz));
    ball->SetMaterialSurface(floor_mat);
    ball->SetBodyFixed(false);

    //Creating force object on ball.
    auto myforceb = std::make_shared<ChForce>();
    myforceb->SetMode(ChForce().FORCE);
    ball->AddForce(myforceb);
    myforceb->SetDir(VECT_X);
    myforceb->SetVrelpoint(ChVector<>(rx, 0.03 * ry, rz));
    forceb.push_back(myforceb);

    //Make the ball blue.
    auto color_ball = std::make_shared<ChColorAsset>();
    color_ball->SetColor(ChColor(0, 0, 1));
    ball->AddAsset(color_ball);

    //Mating the bots to the floor so they don't tip.
    auto pt = chrono_types::make_shared<ChLinkMatePlane>();
    pt->Initialize(my_system->SearchBodyID(-1), ball, true, ChVector<>(0, 0, 0), ChVector<>(0, -height / 2, 0), ChVector<>(0, -1, 0), ChVector<>(0, 1, 0));
    my_system->AddLink(pt); //Adding the link to the system.

    auto z2x = ChQuaternion<>();
    z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
    auto prismatic_ground_ball = std::make_shared<ChLinkLockPrismatic>();
    prismatic_ground_ball->Initialize(my_system->SearchBodyID(-1), ball, ChCoordsys<>(ChVector<>(5.5, 0, 0), z2x));
    my_system->AddLink(prismatic_ground_ball);

    my_system->Add(ball);
}

// -----------------------------------------------------------------------------
// Create walls
// -----------------------------------------------------------------------------
void AddWall(ChSystemNSC* my_system) {
    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    floor_mat->SetFriction(0.3f);
    floor_mat->SetDampingF(0.01f);
    floor_mat->SetRollingFriction(0.1f);
    floor_mat->SetSpinningFriction(0.1f);

    floor_mat->SetCompliance(0.0001f); //Do we need the compliance values if they are so small? -EL
    floor_mat->SetComplianceT(0.0001f);
    floor_mat->SetComplianceRolling(0.0001f);
    floor_mat->SetComplianceSpinning(0.0001f);

    auto ball = std::make_shared<ChBodyEasyCylinder>(Rb, height, rho_b, true, true);
    ball->SetPos(ChVector<>(rx, ry, rz));
    ball->SetMaterialSurface(floor_mat);
    ball->SetBodyFixed(false);

    double wall_rotate = CH_C_PI / 2;
    double wall_length = 5;
    double wall_height = 0.25;
    double wall_width = 0.1;
    double wall_mass = 05;
    auto wall_inertia = ChVector<>(1, 1, 1);

    //First wall.
    double z_wall = 0;
    double y_wall = wall_height / 2;
    double x_wall = -R1 - wall_width;

    auto wall = std::make_shared<ChBody>();
    wall->SetPos(ChVector<>(x_wall, y_wall, z_wall));
    wall->SetRot(Q_from_AngY(wall_rotate));
    wall->SetBodyFixed(true);

    wall->SetMass(wall_mass);
    wall->SetInertiaXX(wall_inertia);
    wall->SetMaterialSurface(floor_mat);

    wall->GetCollisionModel()->ClearModel();
    wall->GetCollisionModel()->AddBox(wall_length / 2, wall_height / 2, wall_width / 2);
    wall->GetCollisionModel()->BuildModel();
    wall->SetCollide(true);

    auto wall_shape = std::make_shared<ChBoxShape>();
    wall_shape->GetBoxGeometry().Size = ChVector<>(wall_length / 2, wall_height / 2, wall_width / 2);

    auto color_wall = std::make_shared<ChColorAsset>();
    color_wall->SetColor(ChColor(0.5, 0.5, 0.5));
    wall->AddAsset(color_wall);
    wall->AddAsset(wall_shape);

    my_system->Add(wall);

    //Angled Walls
    //First angled wall.
    double z_wall_1 = 0.55;
    double wall_rotate_1 = CH_C_PI / 6;
    double wall_length_1 = 2;
    double x_wall_1 = -R1 - wall_width - 0.3;

    auto wall_1 = std::make_shared<ChBody>();
    wall_1->SetPos(ChVector<>(x_wall_1, y_wall, -z_wall_1));
    wall_1->SetRot(Q_from_AngY(wall_rotate_1));
    wall_1->SetBodyFixed(true);

    wall_1->SetMass(wall_mass);
    wall_1->SetInertiaXX(wall_inertia);
    wall_1->SetMaterialSurface(floor_mat);

    wall_1->GetCollisionModel()->ClearModel();
    wall_1->GetCollisionModel()->AddBox(wall_length_1 / 2, wall_height / 2, wall_width / 2);
    wall_1->GetCollisionModel()->BuildModel();
    wall_1->SetCollide(true);

    auto wall_shape_1 = std::make_shared<ChBoxShape>();
    wall_shape_1->GetBoxGeometry().Size = ChVector<>(wall_length_1 / 2, wall_height / 2, wall_width / 2);

    wall_1->AddAsset(color_wall);
    wall_1->AddAsset(wall_shape_1);

    my_system->Add(wall_1);

    //Second angled wall
    auto wall_2 = std::make_shared<ChBody>();
    wall_2->SetPos(ChVector<>(x_wall_1, y_wall, z_wall_1));
    wall_2->SetRot(Q_from_AngY(-wall_rotate_1));
    wall_2->SetBodyFixed(true);

    wall_2->SetMass(wall_mass);
    wall_2->SetInertiaXX(wall_inertia);
    wall_2->SetMaterialSurface(floor_mat);

    wall_2->GetCollisionModel()->ClearModel();
    wall_2->GetCollisionModel()->AddBox(wall_length_1 / 2, wall_height / 2, wall_width / 2);
    wall_2->GetCollisionModel()->BuildModel();
    wall_2->SetCollide(true);

    wall_2->AddAsset(color_wall);
    wall_2->AddAsset(wall_shape_1);

    my_system->Add(wall_2);
}

// -----------------------------------------------------------------------------
// Main Code
// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    timer_total.start();                    // Start the timer
    SetChronoDataPath(CHRONO_DATA_DIR);     // Set path to Chrono data directory
    ChSystemNSC mphysicalSystem;            // Create a Chrono physical system

    // Set the solver and related settings
    mphysicalSystem.SetSolverType(chrono::ChSolver::Type::PSSOR);

    //======================================================================
    // Creating the physical system.
    //AddFloor(&mphysicalSystem);
    //AddBots(&mphysicalSystem);
    //AddInterior(&mphysicalSystem);
    //AddBall(&mphysicalSystem);
    //AddWall(&mphysicalSystem);   

    //======================================================================

    //----------------------//
    //      Time Values     //
    //----------------------//


    // Create the Irrlicht visualization
    ChIrrApp application(&mphysicalSystem,              // Target system
        L"FEA contacts",                                // Window title
        core::dimension2d<u32>(1200, 800),              // Window dimensions
        false,                                          // Fullscreen?
        true,                                           // Shadows?
        true,                                           // Anti-Aliasing?
        video::EDT_OPENGL);                             // Graphics Driver
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, (f32)0.6, -1),  // Position
                                 core::vector3df(0, (f32)0.6, -1)); // Aim point

    application.AddLightWithShadow(core::vector3df(1.5, 5.5, -2.5), // Position
                                   core::vector3df(0, 0, 0),        // Aim point
                                   3,                               // Radius
                                   2.2,                             // mnear
                                   7.2,                             // mfar
                                   40,                              // angle
                                   512,                             // Resolution
                                   video::SColorf(1, 1, 1),         // Color
                                   false,                           // Directional?
                                   false);                          // Clip border?
    application.SetContactsDrawMode(ChIrrTools::CONTACT_FORCES);
    application.SetSymbolscale(0.01f);
    application.AssetBindAll();
    application.AssetUpdateAll();
    application.AddShadowAll();

    // Run the simulation
    application.SetTimestep(tstep);
    application.SetTryRealtime(false);

    while (application.GetDevice()->run()) {
        float t = mphysicalSystem.GetChTime();
        std::cout << "\n" << "Simulation Time= " << t << "s" << "\n";
        timer_step.start();
        application.BeginScene();
        application.DrawAll();

        t += tstep;
        Controller(mphysicalSystem, force, mag, botcall, tset, Fm, nb, k, templ, t, rl, rlmax, Springs, forceb, jamcall, rlj, rljmax, tj, kj, mag2, magf, tp, obj, mag3,0,0,0,tstep,0,0,0);

        application.DoStep();
        application.EndScene();
        timer_step.stop();
        std::cout << "Time elapsed= " << timer_step.GetTimeMillisecondsIntermediate() << "ms" << "\n";

        if (t > tend) {
            application.GetDevice()->closeDevice();
            return 0;
        }
    }
    timer_total.stop();
    std::cout << "Total program run time= " << timer_step.GetTimeSeconds() << "s" << "\n";
    return 0;
}
