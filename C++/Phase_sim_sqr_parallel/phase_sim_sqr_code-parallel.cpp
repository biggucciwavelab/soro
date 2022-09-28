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
#include <cmath> //needed for CH_pi

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkTSDA.h"              //Added to include the spring.
#include "chrono/assets/ChPointPointDrawing.h"      //Visualize a spring as a coil.
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/solver/ChSolverPSSOR.h"
#include "chrono/core/ChTimer.h"
#include "chrono_opengl/ChOpenGLWindow.h" //Adding OpenGl rendering tool.

// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::collision;


// -----------------------------------------------------------------------------
// Global functions and variables
// -----------------------------------------------------------------------------

// Robot Parameters
double nb = 100;            // # of robots
int nt = 0;                 // Initializing collection of bots and particles.
double diameter = 0.07;     // Diameter of interior cylinder
double R1 = ((diameter * nb) / (CH_C_PI * 2)) + .12;
std::vector<int> in_ring = { 104,97,91,85,78,72,66,60,53,47,41,34,28,22,16,9,3 }; //Ring pattern of interior.
int num_ring = in_ring.size();
double mr = 0.18;           //Mass of robots.
double mp = 0.03;           //Mass of particles.
double mb = 0.3;            //Mass of ball.
double height = 0.12;       //Height of cylinder. For both bots and particles.
double hhalf = height / 2;  //Half height of cylinder


//Material 1 Properties, used for bots and particles
float mu_f = 0.4f;  //Friction
float mu_b = 0.01;  //Damping
float mu_r = 0.1;   //Rolling friction
float mu_s = 0.01;  //Spinning friction
float Ct = 0.00001; //Compliance of contact, in tangential direction
float C = 0.00001;  //Compliance of contact, in normal direction
float Cr = 0.0001;  //Rolling compliance of the contact, if using a nonzero rolling friction.
float Cs = 0.001;   //Spinning compliance of the contact, if using a nonzero rolling friction.

//Material 2 Properties, used for floor, ball, and walls
float mu_f2 = 0.3;  //Friction
float mu_b2 = 0.01; //Damping
float mu_r2 = 0.1;  //Rolling friction
float mu_s2 = 0.1;  //Spinning friction
float Ct2 = 0.00001;//Compliance of contact, in tangential direction
float C2 = 0.00001; //Compliance of contact, in normal direction
float Cr2 = 0.0001; //Rolling compliance of the contact, if using a nonzero rolling friction
float Cs2 = 0.0001; //Spinning compliancce of the contact, i using a nonzero rolling friction

// Spring Parameters
double k = 5;            //Spring constant (on the bots).
double kj = 30;            //Jamming spring constant
double rl = 0.0;          //Resting length
double rlmax = 0.02;        //Max resting length
double rlj = 0;             //Desired length jammed
double rljmax = 0.0001;     //Max desired length jammed
double volume = CH_C_PI * (0.25) * height * (pow(diameter, 2));

// Ball paramaters
double ratio = 0.25; 
double Rb = ratio * R1;     //Radius of the ball
double offset = Rb + R1;    //Offset of ball position
double rx = offset + 0.5;   //x-position of ball
double ry = hhalf;          //y-position of ball
double rz = 0;              //z-position of ball


// -----------------------------------------------------------------------------
// Create empty .dat files to be filled
// Note: You must MANUALLY change the out_dir!!!!!
// -----------------------------------------------------------------------------
//const std::string out_dir = "C:/Users/17088/Documents/Soft Robotics Research/el-soro_chrono/C++/Phase_sim_sqr_parallel/Data"; // Estaban
const std::string out_dir = "D:/WaveLab/Soft Robotics/Chrono_BitBucket/C++/Phase_sim_sqr_parallel/data/";   // Q
std::vector<double> springforce;
std::vector<double> springlength;

std::string templ = out_dir + "/templ.dat"; //Storing spring lengths
std::string Fm = out_dir + "/Fm.dat"; //Storing the force in the springs.

//Position of bodies
std::string Xpos = out_dir + "/Xpos.dat";
std::string Ypos = out_dir + "/Ypos.dat";
std::string Zpos = out_dir + "/Zpos.dat";
std::string ballp = out_dir + "/ballp.dat";

//Velocity of bodies
std::string Xvel = out_dir + "/Xvel.dat";
std::string Yvel = out_dir + "/Yvel.dat";
std::string Zvel = out_dir + "/Zvel.dat";

//Force on the bodies
std::string Xforce = out_dir + "/Xforce.dat";
std::string Yforce = out_dir + "/Yforce.dat";
std::string Zforce = out_dir + "/Zforce.dat";

//Rotation of bodies
std::string rott0 = out_dir + "/rott0.dat";
std::string rott1 = out_dir + "/rott1.dat";
std::string rott2 = out_dir + "/rott2.dat";
std::string rott3 = out_dir + "/rott3.dat";

//Preparing data output
ChStreamOutAsciiFile spring_length(templ.c_str());
ChStreamOutAsciiFile spring_force(Fm.c_str());

ChStreamOutAsciiFile X_pos(Xpos.c_str());
ChStreamOutAsciiFile Y_pos(Ypos.c_str());
ChStreamOutAsciiFile Z_pos(Zpos.c_str());
ChStreamOutAsciiFile ball_p(ballp.c_str());

ChStreamOutAsciiFile X_vel(Xvel.c_str());
ChStreamOutAsciiFile Y_vel(Yvel.c_str());
ChStreamOutAsciiFile Z_vel(Zvel.c_str());

ChStreamOutAsciiFile X_force(Xforce.c_str());
ChStreamOutAsciiFile Y_force(Yforce.c_str());
ChStreamOutAsciiFile Z_force(Zforce.c_str());

ChStreamOutAsciiFile rott_0(rott0.c_str());
ChStreamOutAsciiFile rott_1(rott1.c_str());
ChStreamOutAsciiFile rott_2(rott2.c_str());
ChStreamOutAsciiFile rott_3(rott3.c_str());

// -----------------------------------------------------------------------------
// External forces
// -----------------------------------------------------------------------------
double mag = 0;                     //[N] - magnitude of external force applied at each bot
double magf = (9.81) * mb * mu_f;   //Force from frction an weight.
double magd = 200;                  //Desired Force
double mag2 = magd + magf;          //Compensated force to account for friction.
double mag3 = 3;

template<typename T>  //Create the equivalent of numpy's arange function.
std::vector<T> arange(T start, T stop, T step = 1) {
    std::vector<T> values;
    for (T value = start; value < stop; value += step)
        values.push_back(value);
    return values;
}

//Some empty matrices for things to be collected in
std::vector<std::shared_ptr<ChBody>> obj;           // Bots and particles
std::vector<std::shared_ptr<ChBodyEasyBox>> bots;   // Bots
std::vector<std::shared_ptr<ChLinkTSDA>> Springs;   // Springs
std::vector<std::shared_ptr<ChForce>> forceb;       // Forces on the ball
std::vector<std::shared_ptr<ChForce>> force;        // External force objects
std::vector<double> springlengths;                  // Spring lengths
std::vector<double> springforces;                   // Force in the springs.
std::vector<std::shared_ptr<ChBody>> Balls;         // Balls

//Vectors with zeroes
std::vector<int> jamcall(nb, 0);
std::vector<int> botcall(nb, 0);

/*
ChTimer<double> timer_step;     // Timer object for each time step
ChTimer<double> timer_total;    // Timer object for the total runtime
*/

// -----------------------------------------------------------------------------
// Functions to be called on at simulation time
// -----------------------------------------------------------------------------

// Jam the springs
void Jamsprings(double& k, double& rl, double& rlmax, std::vector<std::shared_ptr<ChLinkTSDA>> Springs, double t, double tj, std::vector<double> Fm, double kj, double rlj, double rljmax, double i, double jamcall) {
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
}

// Set the spring rate dynamically
void setSpring(double k, double rl, double rlmax, std::vector<std::shared_ptr<ChLinkTSDA>>& Springs, std::vector<double>& Fm, std::vector<double>& templ, int i) {
    double var1 = Springs[i]->GetLength();

    if (var1 < rl) {
        Springs[i]->SetSpringCoefficient(0);
        double var2 = Springs[i]->GetForce();
        Fm.push_back(var2);
    }

    if (var1 > rlmax) {
        Springs[i]->SetSpringCoefficient(10 * k);
        double var2 = Springs[i]->GetForce();
        Fm.push_back(var2);
    }

    else {
        Springs[i]->SetSpringCoefficient(k);
        double var2 = Springs[i]->GetForce();
        Fm.push_back(var2);
    }

    templ.push_back(var1);
}

// Todo: Centroid function
void centroid(std::vector<std::shared_ptr<ChBody>> obj) {

}

// Controller function
void Controller(ChSystemParallelNSC& my_system, std::vector<std::shared_ptr<ChForce>>& force, double mag, std::vector<int> botcall, double tset, std::vector<double>& Fm,
    double nb, double k, std::vector<double>& templ, double t, double rl, double rlmax, 
    std::vector<std::shared_ptr<ChLinkTSDA>>& Springs, std::vector<std::shared_ptr<ChForce>>& forceb,std::vector<int> jamcall,
    double rlj, double rljmax, double tj, double kj, double mag2, double magf, double tp,std::vector<std::shared_ptr<ChBody>> obj,
    double mag3) {
    
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

// Data extraction
void ExtractData(std::vector<std::shared_ptr<ChBody>> obj, int nb, int nt,
    std::vector<double>& springforce, std::vector<double>& springlength,
    ChStreamOutAsciiFile& X_pos, ChStreamOutAsciiFile& Y_pos, ChStreamOutAsciiFile& Z_pos,
    ChStreamOutAsciiFile& X_force, ChStreamOutAsciiFile& Y_force, ChStreamOutAsciiFile& Z_force,
    ChStreamOutAsciiFile& rott_0, ChStreamOutAsciiFile& rott_1, ChStreamOutAsciiFile& rott_2, ChStreamOutAsciiFile& rott_3,
    ChStreamOutAsciiFile& X_vel, ChStreamOutAsciiFile& Y_vel, ChStreamOutAsciiFile& Z_vel,
    ChStreamOutAsciiFile& spring_length, ChStreamOutAsciiFile& spring_force,
    ChStreamOutAsciiFile& ball_p, std::vector<std::shared_ptr<ChBody>> Balls, double t, std::vector<std::shared_ptr<ChLinkTSDA>>& Springs) {

    //Writes down the time.
    X_pos << t << " ";
    Y_pos << t << " ";
    Z_pos << t << " ";

    X_force << t << " ";
    Y_force << t << " ";
    Z_force << t << " ";

    X_vel << t << " ";
    Y_vel << t << " ";
    Z_vel << t << " ";

    rott_0 << t << " ";
    rott_1 << t << " ";
    rott_2 << t << " ";
    rott_3 << t << " ";

    ball_p << t << " ";
    spring_length << t << " ";
    spring_force << t << " ";

    for (int i = 0; i < nt; i++) {
        auto tempx = obj[i]->Get_Xforce(); //Gets the force vector acting on the body
        auto tempxx = obj[i]->GetPos_dt(); //Gets the velocity vector of the body

        //Positions of bodies
        X_pos << obj[i]->GetPos().x() << " ";
        Y_pos << obj[i]->GetPos().y() << " ";
        Z_pos << obj[i]->GetPos().z() << " ";

        //Forces on bodies
        X_force << tempx.x() << " ";
        Y_force << tempx.y() << " ";
        Z_force << tempx.z() << " ";

        //Velocity of bodies
        X_vel << tempxx.x() << " ";
        Y_vel << tempxx.y() << " ";
        Z_vel << tempxx.z() << " ";

        //Rotation positions of bodies
        rott_0 << obj[i]->GetRot().e0() << " ";
        rott_1 << obj[i]->GetRot().e1() << " ";
        rott_2 << obj[i]->GetRot().e2() << " ";
        rott_3 << obj[i]->GetRot().e3() << " ";
    }

    //Storing the ball's position. Made in a loop for any future balls.
    for (int i = 0; i < Balls.size(); i++) {
        ball_p << Balls[i]->GetPos().x() << " ";
    }

    //Storing spring lengths and forces.
    for (int i = 0; i < Springs.size(); i++) {
        spring_length << Springs[i]->GetLength() << " ";
        spring_force << Springs[i]->GetForce() << " ";
    }

    //Create a new line.
    X_pos << "\n";
    Y_pos << "\n";
    Z_pos << "\n";

    X_force << "\n";
    Y_force << "\n";
    Z_force << "\n";

    X_vel << "\n";
    Y_vel << "\n";
    Z_vel << "\n";

    rott_0 << "\n";
    rott_1 << "\n";
    rott_2 << "\n";
    rott_3 << "\n";

    ball_p << "\n";
    spring_length << "\n";
    spring_force << "\n";
}

//========================================================================

// -----------------------------------------------------------------------------
// Create a floor for robot to live on
// -----------------------------------------------------------------------------
void AddFloor(ChSystemParallelNSC* my_system) {
    
    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    floor_mat->SetFriction(mu_f2);
    floor_mat->SetDampingF(mu_b2);
    floor_mat->SetRollingFriction(mu_r2);
    floor_mat->SetSpinningFriction(mu_s2);

    floor_mat->SetCompliance(C2); //Do we need the compliance values if they are so small? -EL
    floor_mat->SetComplianceT(Ct2);
    floor_mat->SetComplianceRolling(Cr2);
    floor_mat->SetComplianceSpinning(Cs2);
    
    //Floor dimensions
    double length = 8;
    double width = 8;
    double height = 0.1;

    auto floorBody = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
    floorBody->GetCollisionModel()->ClearModel();
    ChVector<> floor_dim(length / 2, height / 2, width / 2); //Dimensions in x,y,z. Double these and you get the actual dimensions.
    utils::AddBoxGeometry(floorBody.get(), floor_dim);
    floorBody->GetCollisionModel()->BuildModel();

    floorBody->SetPos(ChVector<>(0, -height / 2, 0));
    floorBody->SetMaterialSurface(floor_mat);
    floorBody->SetIdentifier(-1);
    floorBody->SetBodyFixed(true); //It is not moving!
    floorBody->SetCollide(true);

    // Attach a RGB color asset to the floor.
    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(0.2f, 0.25f, 0.25f));
    floorBody->AddAsset(color);
    my_system->Add(floorBody);
}

// -----------------------------------------------------------------------------
// Create bots with springs
// -----------------------------------------------------------------------------
void AddBots(ChSystemParallelNSC* my_system, 
    std::vector<std::shared_ptr<ChBody>>& obj, std::vector<int>& botcall, std::vector<int>& jamcall, std::vector<std::shared_ptr<ChLinkTSDA>>& Springs) {

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

    // Make the material
    auto bot_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    bot_mat->SetFriction(mu_f);
    bot_mat->SetDampingF(mu_b);
    bot_mat->SetRollingFriction(mu_r);
    bot_mat->SetSpinningFriction(mu_s);

    bot_mat->SetCompliance(C);                 //Do we need the compliance values if they are so small? -EL
    bot_mat->SetComplianceT(Ct);
    bot_mat->SetComplianceRolling(Cr);
    bot_mat->SetComplianceSpinning(Cs);

    std::vector<std::shared_ptr<ChBody>> bots; //Empty vector for bots to be stored

    double p1, p2, p3, p4, h;                  //Points that the spring will be attached to.
    p1 = 0;
    p2 = diameter / 2;
    p3 = 0;
    p4 = -diameter / 2;
    h = 0;

    for (int i = 0; i < nb; i++) {
        double theta = i * ((2 * CH_C_PI) / nb);
        double x_pos = R1 * cos(theta);
        double y_pos = (height/2); //As measured from COG of robot.
        double z_pos = R1 * sin(theta);

        auto bot = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>()); //Creating a bot using EasyBox. This already includes a collision model.
        bot->GetCollisionModel()->ClearModel();
        ChVector<> bot_dim(diameter / 2, height / 2, diameter / 2); //Dimensions of bot (x,y,z)
        utils::AddBoxGeometry(bot.get(), bot_dim);
        bot->GetCollisionModel()->BuildModel();

        bot->SetPos(ChVector<>(x_pos, y_pos, z_pos));   //Setting Bot position
        bot->SetMaterialSurface(bot_mat);               //Setting bot material
        bot->SetBodyFixed(false);                       //Make sure the bots can move around.
        bot->SetCollide(true);

        auto rotation = ChQuaternion<>();
        rotation.Q_from_AngAxis(-theta, ChVector<>(0, 1, 0));
        bot->SetRot(rotation);                          //Rotate the bot.

        //Mating the bots to the floor so they don't tip.
        auto pt = chrono_types::make_shared<ChLinkMatePlane>();
        pt->Initialize(my_system->SearchBodyID(-1), bot, true, ChVector<>(0, 0.1/2, 0), 
            ChVector<>(0, -height / 2, 0), ChVector<>(0, 1, 0), ChVector<>(0, -1, 0));
        pt->SetFlipped(false);
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
        auto spring = chrono_types::make_shared<ChLinkTSDA>();
        spring->SetName("spring");

        auto color_spring = std::make_shared<ChColorAsset>();
        color_spring->SetColor(ChColor(1, 0.75, 0));

        //Most of the springs are added in this loop.
        if (i >= 1) {
            spring->Initialize(bots[i - 1], bot, true, ChVector<>(p1, h, p2), ChVector<>(p3, h, p4), false);
            spring->SetSpringCoefficient(k);
            spring->SetRestLength(rl);
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
    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->SetName("spring");

    auto color_spring = std::make_shared<ChColorAsset>();
    color_spring->SetColor(ChColor(1, 0.75, 0));

    spring->Initialize(bots[nb - 1], bots[0], true, ChVector<>(p1, h, p2), ChVector<>(p3, h, p4), false);
    spring->SetSpringCoefficient(k);
    spring->SetRestLength(rl);
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

// -----------------------------------------------------------------------------
// Create the interior
// -----------------------------------------------------------------------------
void AddInterior(ChSystemParallelNSC* my_system, std::vector<std::shared_ptr<ChBody>>& obj, int& nt) {
    // Make the material
    auto par_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    par_mat->SetFriction(mu_f);
    par_mat->SetDampingF(mu_b);
    par_mat->SetRollingFriction(mu_r);
    par_mat->SetSpinningFriction(mu_s);

    par_mat->SetCompliance(C); //Do we need the compliance values if they are so small? -EL
    par_mat->SetComplianceT(Ct);
    par_mat->SetComplianceRolling(Cr);
    par_mat->SetComplianceSpinning(Cs);

    int ni = 0;             //Number of interiors (filled with loop below).
    for (int i = 0; i < num_ring; i++)
    {
        ni += in_ring[i];
    }
    nt = ni + nb; //Total number of bots and particles.

    for (int i = 0; i < num_ring; i++) {
        double R2 = (diameter * in_ring[i]) / (2 * CH_C_PI);

        for (int j = 0; j < in_ring[i]; j++) {
            double x_pos = R2 * cos(j * 2 * CH_C_PI / in_ring[i]);
            double y_pos = height / 2;
            double z_pos = R2 * sin(j * 2 * CH_C_PI / in_ring[i]);

            auto particle = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
            particle->GetCollisionModel()->ClearModel();
            utils::AddCylinderGeometry(particle.get(), diameter / 2, height / 2);
            particle->SetPos(ChVector<>(x_pos, y_pos, z_pos));
            particle->GetCollisionModel()->BuildModel();

            particle->SetBodyFixed(false);
            particle->SetCollide(true);
            particle->SetMaterialSurface(par_mat);

            auto color_particle = std::make_shared<ChColorAsset>();
            color_particle->SetColor(ChColor(0.996, 0.996, 0));
            particle->AddAsset(color_particle);

            auto pt = chrono_types::make_shared<ChLinkMatePlane>(); //Mating the bots to the floor so they don't tip.
            pt->Initialize(my_system->SearchBodyID(-1), particle, true, 
                ChVector<>(0, 0.1/2, 0), ChVector<>(0, -height / 2, 0), ChVector<>(0, 1, 0), ChVector<>(0, -1, 0));
            pt->SetFlipped(false);
            my_system->AddLink(pt); //Adding the link to the system.

            obj.push_back(particle);
            my_system->Add(particle);
        }
    }
}

// -----------------------------------------------------------------------------
// Create the ball
// -----------------------------------------------------------------------------
void AddBall(ChSystemParallelNSC* my_system, std::vector<std::shared_ptr<ChForce>>& forceb, std::vector<std::shared_ptr<ChBody>>& Balls) {
    auto ball_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ball_mat->SetFriction(mu_f2);
    ball_mat->SetDampingF(mu_b2);
    ball_mat->SetRollingFriction(mu_r2);
    ball_mat->SetSpinningFriction(mu_s2);

    ball_mat->SetCompliance(C2); //Do we need the compliance values if they are so small? -EL
    ball_mat->SetComplianceT(Ct2);
    ball_mat->SetComplianceRolling(Cr2);
    ball_mat->SetComplianceSpinning(Cs2);

    auto ball = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
    ball->GetCollisionModel()->ClearModel();
    utils::AddCylinderGeometry(ball.get(), Rb, height / 2);
    ball->SetPos(ChVector<>(rx, ry, rz));
    ball->GetCollisionModel()->BuildModel();

    ball->SetMaterialSurface(ball_mat);
    ball->SetBodyFixed(false);
    ball->SetCollide(true);

    //Creating force object on ball.
    auto myforceb = std::make_shared<ChForce>();
    myforceb->SetMode(ChForce().FORCE);
    ball->AddForce(myforceb);
    myforceb->SetDir(VECT_X);
    myforceb->SetVrelpoint(ChVector<>(rx, 0.03 * ry, rz));
    forceb.push_back(myforceb);

    //Make the ball blue.
    auto color_ball = std::make_shared<ChColorAsset>();
    color_ball->SetColor(ChColor(0, 1, 0));
    ball->AddAsset(color_ball);

    //Mating the ball to the floor so they don't tip.
    auto pt = chrono_types::make_shared<ChLinkMatePlane>();
    pt->Initialize(my_system->SearchBodyID(-1), ball, true, 
        ChVector<>(0, 0.1/2, 0), ChVector<>(0, -height / 2, 0), ChVector<>(0, 1, 0), ChVector<>(0, -1, 0));
    pt->SetFlipped(false);
    my_system->AddLink(pt); //Adding the link to the system.

    auto z2x = ChQuaternion<>();
    z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
    auto prismatic_ground_ball = std::make_shared<ChLinkLockPrismatic>();
    prismatic_ground_ball->Initialize(my_system->SearchBodyID(-1), ball, ChCoordsys<>(ChVector<>(5.5, 0, 0), z2x));
    my_system->AddLink(prismatic_ground_ball);

    Balls.push_back(ball);
    my_system->Add(ball);
}

// -----------------------------------------------------------------------------
// Create walls
// -----------------------------------------------------------------------------
void AddWall(ChSystemParallelNSC* my_system) {
    auto wall_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    wall_mat->SetFriction(mu_f2);
    wall_mat->SetDampingF(mu_b2);
    wall_mat->SetRollingFriction(mu_r2);
    wall_mat->SetSpinningFriction(mu_s2);

    wall_mat->SetCompliance(C2); //Do we need the compliance values if they are so small? -EL
    wall_mat->SetComplianceT(Ct2);
    wall_mat->SetComplianceRolling(Cr2);
    wall_mat->SetComplianceSpinning(Cs2);

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

    auto wall = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
    wall->GetCollisionModel()->ClearModel();
    ChVector<> wall_dim(wall_length / 2, wall_height / 2, wall_width / 2);
    utils::AddBoxGeometry(wall.get(), wall_dim);
    wall->GetCollisionModel()->BuildModel();

    wall->SetPos(ChVector<>(x_wall, y_wall, z_wall));
    wall->SetRot(Q_from_AngY(wall_rotate));
    wall->SetBodyFixed(true);
    wall->SetCollide(true);

    wall->SetMass(wall_mass);
    wall->SetInertiaXX(wall_inertia);
    wall->SetMaterialSurface(wall_mat);

    auto color_wall = std::make_shared<ChColorAsset>();
    color_wall->SetColor(ChColor(0.5, 0.5, 0.5));
    wall->AddAsset(color_wall);

    my_system->Add(wall);

    /* Note: Temporarily removed. Balls and bots are interferring.
    //Angled Walls
    //First angled wall.
    double z_wall_1 = 0.55;
    double wall_rotate_1 = CH_C_PI / 6;
    double wall_length_1 = 2;
    double x_wall_1 = -R1 - wall_width - 0.3;

    auto wall_1 = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
    wall_1->GetCollisionModel()->ClearModel();
    ChVector<> wall_1_dim(wall_length_1 / 2, wall_height / 2, wall_width / 2);
    utils::AddBoxGeometry(wall_1.get(), wall_1_dim);
    wall_1->GetCollisionModel()->BuildModel();

    wall_1->SetPos(ChVector<>(x_wall_1, y_wall, -z_wall_1));
    wall_1->SetRot(Q_from_AngY(wall_rotate_1));

    wall_1->SetMass(wall_mass);
    wall_1->SetInertiaXX(wall_inertia);
    wall_1->SetMaterialSurface(wall_mat);
    wall_1->SetCollide(true);
    wall_1->SetBodyFixed(true);

    wall_1->AddAsset(color_wall);

    my_system->Add(wall_1);

    //Second angled wall
    auto wall_2 = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelParallel>());
    wall_2->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(wall_2.get(), wall_1_dim);
    wall_2->GetCollisionModel()->BuildModel();

    wall_2->SetPos(ChVector<>(x_wall_1, y_wall, z_wall_1));
    wall_2->SetRot(Q_from_AngY(-wall_rotate_1));
    wall_2->SetBodyFixed(true);
    wall_2->SetMass(wall_mass);
    wall_2->SetInertiaXX(wall_inertia);
    wall_2->SetMaterialSurface(wall_mat);
    wall_2->SetCollide(true);

    wall_2->AddAsset(color_wall);

    my_system->Add(wall_2);
    */

}



// -----------------------------------------------------------------------------
// OpenGL creation
// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    //timer_total.start();                            // Start the timer
    SetChronoDataPath(CHRONO_DATA_DIR);             // Set path to Chrono data directory
    ChSystemParallelNSC my_system;            // Create a Chrono physical system

    // Set the solver and related settings
    //mphysicalSystem.SetSolverType(chrono::ChSolver::Type::PSSOR);

    int threads = 16;

    // Simulation parameters
    // ---------------------

    double gravity = 9.81;
    double time_step = 1e-3;
    double time_end = 2;

    double out_fps = 50;

    uint max_iteration = 300;
    real tolerance = 1e-3;

    // Set number of threads.
    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    CHOMPfunctions::SetNumThreads(threads);

    // Set gravitational acceleration
    my_system.Set_G_acc(ChVector<>(0, -gravity, 0));

    
    // Set solver parameters
    //No idea what these do to be honest. -EL
    my_system.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    my_system.GetSettings()->solver.max_iteration_normal = max_iteration / 3;
    my_system.GetSettings()->solver.max_iteration_sliding = max_iteration / 3;
    my_system.GetSettings()->solver.max_iteration_spinning = max_iteration/3;
    my_system.GetSettings()->solver.max_iteration_bilateral = max_iteration / 3;
    my_system.GetSettings()->solver.tolerance = tolerance;
    my_system.GetSettings()->solver.alpha = 0;
    my_system.GetSettings()->solver.contact_recovery_speed = 10000;
    my_system.ChangeSolverType(SolverType::APGD);
    my_system.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    my_system.GetSettings()->collision.collision_envelope = .1*diameter; //Increase this number if stability becomes an issue.
    my_system.GetSettings()->collision.bins_per_axis = vec3(8, 8, 2);

    //======================================================================
    // Adding assets to the system
    AddFloor(&my_system);
    AddBots(&my_system, obj, botcall, jamcall, Springs);
    AddInterior(&my_system,obj,nt);
    AddBall(&my_system,forceb,Balls);
    AddWall(&my_system);   

    //======================================================================

    //----------------------//
    //      Time Values     //
    //----------------------//
    double tstep = 0.005;   //Time Step
    double tset = 0.01;     //settling time
    double tend = 9;        //Length of simulation
    double tp = 7;          //Time to pull
    double tj = 5;          //Jaming starts
    double t = 0;           //Simulation time

    //Render everything
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Phase Sim Squares", &my_system);
    gl_window.SetCamera(ChVector<>(0, 2.5, .2), ChVector<>(0, 0, 0), ChVector<>(0, 1, 0), 0.05f, 0.05f);
    gl_window.SetRenderMode(opengl::SOLID);

    while (gl_window.Active()) {
        gl_window.DoStepDynamics(tstep);

        t += tstep;

        Controller(my_system, force, mag, botcall, tset, 
            springforce, nb, k, springlength, t, rl, rlmax, 
            Springs, forceb, jamcall, rlj, rljmax, tj, kj, mag2, magf, tp, obj, mag3);

        ExtractData(obj, nb, nt, springforce, springlength, 
            X_pos, Y_pos, Z_pos, X_force, Y_force, Z_force, 
            rott_0, rott_1, rott_2, rott_3, X_vel, Y_vel, Z_vel, 
            spring_length, spring_force, ball_p, Balls, t, Springs);

        gl_window.Render();
    }
    return 0;
}
