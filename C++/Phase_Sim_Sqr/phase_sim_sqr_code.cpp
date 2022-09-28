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
#include "chrono/physics/ChLinkSpring.h" //Added to include the spring.
#include "chrono/assets/ChPointPointDrawing.h" //Visualize a spring as a coil.
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"

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

//------------------------------------------//
//      Functions to be called on later     //
//------------------------------------------//
void Jamsprings(double& k, double& rl, double& rlmax, std::vector<std::shared_ptr<ChLinkSpring>> Springs, double t, double tj,  double kj, double rlj, double rljmax, double i, double jamcall) {
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

void setSpring(double k, double rl, double rlmax, std::vector<std::shared_ptr<ChLinkSpring>>& Springs, std::vector<double>& springforce, std::vector<double>& springlength, int i, double t) {
    double var1 = Springs[i]->Get_SpringLength();

    if (var1 < rl) {
        Springs[i]->Set_SpringF(0);
        double var2 = Springs[i]->Get_SpringF();
        springforce.push_back(var2);
    }

    if (var1 > rlmax) {
        Springs[i]->Set_SpringF(10 * k);
        double var2 = Springs[i]->Get_SpringF();
        springforce.push_back(var2);
    }

    else {
        Springs[i]->Set_SpringF(k);
        double var2 = Springs[i]->Get_SpringF();
        springforce.push_back(var2);
    }

    springlength.push_back(var1);
}

/* Skipping centroid function for now
void centroid(std::vector<std::shared_ptr<ChBody>> obj,) {

}
*/

void Controller(ChSystemNSC& my_system, std::vector<std::shared_ptr<ChForce>>& force, double mag, std::vector<int> botcall, double tset, std::vector<double>& springforce,
    double nb, double k, std::vector<double>& springlength, double t, double rl, double rlmax,
    std::vector<std::shared_ptr<ChLinkSpring>>& Springs, std::vector<std::shared_ptr<ChForce>>& forceb,std::vector<int> jamcall,
    double rlj, double rljmax, double tj, double kj, double mag2, double magf, double tp,std::vector<std::shared_ptr<ChBody>> obj,
    double mag3) {
    
    for (int ii = 0; ii < (nb - 1); ii++) {
        Jamsprings(k, rl, rlmax, Springs, t, tj, kj, rlj, rljmax, ii, jamcall[ii]);
        setSpring(k, rl, rlmax, Springs, springforce, springlength, ii,t);

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

void ExtractData(std::vector<std::shared_ptr<ChBody>> obj,int nb,int nt,
    std::vector<double>& springforce, std::vector<double>& springlength,
    ChStreamOutAsciiFile& X_pos, ChStreamOutAsciiFile& Y_pos, ChStreamOutAsciiFile& Z_pos,
    ChStreamOutAsciiFile& X_force, ChStreamOutAsciiFile& Y_force, ChStreamOutAsciiFile& Z_force,
    ChStreamOutAsciiFile& rott_0, ChStreamOutAsciiFile& rott_1, ChStreamOutAsciiFile& rott_2, ChStreamOutAsciiFile& rott_3,
    ChStreamOutAsciiFile& X_vel, ChStreamOutAsciiFile& Y_vel, ChStreamOutAsciiFile& Z_vel,
    ChStreamOutAsciiFile& spring_length, ChStreamOutAsciiFile& spring_force,
    ChStreamOutAsciiFile& ball_p,std::vector<std::shared_ptr<ChBody>> Balls,double t, std::vector<std::shared_ptr<ChLinkSpring>>& Springs) {
    
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
        spring_length << Springs[i]->Get_SpringLength() << " ";
        spring_force << Springs[i]->Get_SpringF() << " ";
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

template<typename T> //Creates the equivalent of numpy's arange function.
std::vector<T> arange(T start, T stop, T step = 1) {
    std::vector<T> values;
    for (T value = start; value < stop; value += step)
        values.push_back(value);
    return values;
}
//========================================================================

// -----------------------------------------------------------------------------
// Main Code
// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Create (if needed) output directory. 
    //You must MANUALLY CHANGE THIS
    const std::string out_dir = "C:/Users/17088/Documents/Soft Robotics Research/el-soro_chrono/C++/Phase_Sim_Sqr/Data";


    // Create a Chrono physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
        ChIrrApp application(&mphysicalSystem,                         // Target system
        L"FEA contacts",                    // Window title
        core::dimension2d<u32>(1200, 800),  // Window dimensions
        false,                              // Fullscreen?
        true,                               // Shadows?
        true,                               // Anti-Aliasing?
        video::EDT_OPENGL);                 // Graphics Driver

    //======================================================================

    // Creating the physical system.

    // -----------------------------------------------------------------------------
    // Create Floor
    // -----------------------------------------------------------------------------
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
    floorBody->SetPos(ChVector<>(0, -.1/2, 0));
    floorBody->SetMaterialSurface(floor_mat);
    floorBody->SetBodyFixed(true); //It is not moving!

    // Attach a RGB color asset to the floor.
    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(0.2f, 0.25f, 0.25f));
    floorBody->AddAsset(color);

    mphysicalSystem.Add(floorBody);

    // -----------------------------------------------------------------------------
    // Values for robots and interior
    // -----------------------------------------------------------------------------

    double nb = 100; // # of robots
    double diameter = 0.07; // Diameter of interior cylinder
    double R1 = ((diameter * nb) / (CH_C_PI * 2)) + .1;
    std::vector<int> in_ring = { 104,97,91,85,78,72,66,60,53,47,41,34,28,22,16,9,3 }; //Ring pattern of interior.
    int num_ring = in_ring.size();
    int ni=0; //Number of interiors (filled with loop below).
    for(int i = 0; i < num_ring; i++)
    {
        ni += in_ring[i];
    }
    int nt = ni + nb; //Total number of bots and particles.
    double nr = nb / ni; //Ratio of nb over ni.
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
    double volume = CH_C_PI * (0.25) * height * (pow(diameter,2));

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

    auto bot_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    float mu_f = 0.4f;
    bot_mat->SetFriction(mu_f);
    bot_mat->SetDampingF(0.01f);
    bot_mat->SetRollingFriction(0.1f);
    bot_mat->SetSpinningFriction(0.01f);

    bot_mat->SetCompliance(0.0001f); //Do we need the compliance values if they are so small? -EL
    bot_mat->SetComplianceT(0.0001f);
    bot_mat->SetComplianceRolling(0.0001f);
    bot_mat->SetComplianceSpinning(0.0001f);

    // -----------------------------------------------------------------------------
    // External Forces
    // Includes which bots are active
    // -----------------------------------------------------------------------------
    double mag = 0; //[N] - magnitude of external force applied at each bot
    double magf = (9.81) * mb * mu_f; //Force from frction an weight.
    double magd = 200; //Desired Force
    double mag2 = magd + magf; //Compensated force to account for friction.
    double mag3 = 3;
    std::vector<std::shared_ptr<ChForce>> forceb; //Empty vector to store forces on the ball
    std::vector<std::shared_ptr<ChForce>> force; //Empty vector to store force objects

    //Vectors with zeroes
    std::vector<int> jamcall(nb, 0);
    std::vector<int> botcall(nb, 0); 

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

    // -----------------------------------------------------------------------------
    // Create bots w/ springs
    // -----------------------------------------------------------------------------
    std::vector<std::shared_ptr<ChBody>> obj; //Empty matrix for bots and particles
    std::vector<std::shared_ptr<ChBodyEasyBox>> bots; //Create an empty vector for bots to be collected in.
    std::vector<std::shared_ptr<ChLinkSpring>> Springs; //An empty vector for springs to be collected in.
   
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

        auto bot = std::make_shared<ChBodyEasyBox>(diameter, height, diameter, rho_r,true,true); //Creating a bot using EasyBox. This already includes a collision model.
        bot->SetPos(ChVector<>(x_pos,y_pos,z_pos)); //Setting Bot position
        bot->SetMaterialSurface(bot_mat); //Setting bot material

        auto rotation = ChQuaternion<>();
        rotation.Q_from_AngAxis(-theta, ChVector<>(0, 1, 0));
        bot->SetRot(rotation); //Rotate the bot.
        
        bot->SetId(i); //Setting the Id for the bot.
        bot->SetBodyFixed(false); //Make sure the bots can move around.

        //Mating the bots to the floor so they don't tip.
        auto pt = chrono_types::make_shared<ChLinkMatePlane>(); 
        pt->Initialize(floorBody, bot, true, ChVector<>(0, 0, 0), ChVector<>(0, -height/2, 0), ChVector<>(0, -1, 0), ChVector<>(0, 1, 0));
        mphysicalSystem.AddLink(pt); //Adding the link to the system.

        auto color_bot = std::make_shared<ChColorAsset>();
        color_bot->SetColor(ChColor(0.44f, 0.11f, i*nb/255));
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
        color_spring->SetColor(ChColor(1,0.75,0));

        //Most of the springs are added in this loop.
        if (i >= 1) {
            spring->Initialize(bots[i - 1],bot, true, ChVector<>(p1, h, p2), ChVector<>(p3, h, p4), false);
            spring->Set_SpringF(k);
            spring->Set_SpringRestLength(rl);
            spring->AddAsset(color_spring);
            spring->AddAsset(std::make_shared<ChPointPointSpring>(0.01, 80, 15));
            mphysicalSystem.AddLink(spring);
            Springs.push_back(spring);
        }

        //Make the last bot green.
        if (i == nb-1) { 
            auto color_last_bot = std::make_shared<ChColorAsset>();
            color_last_bot->SetColor(ChColor(0, 1, 0));
            bot->AddAsset(color_last_bot);
        }
    }
    
    //Connects the last bot to the first one.
    auto spring = chrono_types::make_shared<ChLinkSpring>();
    spring->SetName("spring");

    auto color_spring = std::make_shared<ChColorAsset>();
    color_spring->SetColor(ChColor(1, 0.75, 0));
    
    spring->Initialize(bots[nb-1],bots[0], true, ChVector<>(p1, h, p2), ChVector<>(p3, h, p4), false);
    spring->Set_SpringF(k);
    spring->Set_SpringRestLength(rl);
    spring->AddAsset(color_spring);
    spring->AddAsset(std::make_shared<ChPointPointSpring>(0.01, 80, 15));
    mphysicalSystem.AddLink(spring);
    Springs.push_back(spring);
    
    //Add all the robots to the system.
    for (int i = 0; i < nb; i++) {
        auto robot = bots[i];
        mphysicalSystem.Add(robot);
    }
    // -----------------------------------------------------------------------------
    // Create the interior
    // -----------------------------------------------------------------------------
    for (int i = 0; i < num_ring; i++) {
        double R2 = (diameter * in_ring[i]) / (2 * CH_C_PI);

        for (int j = 0; j < in_ring[i]; j++) {
            double x_pos = R2 * cos(j * 2 * CH_C_PI / in_ring[i]); 
            double y_pos = height / 2; 
            double z_pos = R2 * sin(j * 2 * CH_C_PI / in_ring[i]);

            auto particle = std::make_shared<ChBodyEasyCylinder>(diameter/2,height,rho_p,true,true);
            particle->SetPos(ChVector<>(x_pos, y_pos, z_pos));
            particle->SetMaterialSurface(bot_mat);
            
            //Skipped creating a collision model. Collision model assumed from ChEasyBody.

            auto color_particle = std::make_shared<ChColorAsset>();
            color_particle->SetColor(ChColor(0.996, 0.996, 0));
            particle->AddAsset(color_particle);

            auto pt = chrono_types::make_shared<ChLinkMatePlane>(); //Mating the bots to the floor so they don't tip.
            pt->Initialize(floorBody, particle, true, ChVector<>(0, 0, 0), ChVector<>(0, -height / 2, 0), ChVector<>(0, -1, 0), ChVector<>(0, 1, 0));
            mphysicalSystem.AddLink(pt); //Adding the link to the system.
            
            obj.push_back(particle);
            mphysicalSystem.Add(particle);
        }
    }
    
    // -----------------------------------------------------------------------------
    // Create a ball to grab
    // -----------------------------------------------------------------------------
    
    //Empty vector to store ball bodies
    std::vector<std::shared_ptr<ChBody>> Balls; 
    
    auto ball = std::make_shared<ChBodyEasyCylinder>(Rb, height, rho_b,true,true);
    ball->SetPos(ChVector<>(rx, ry, rz));
    ball->SetMaterialSurface(floor_mat);
    ball->SetBodyFixed(false);

    //Creating force object on ball.
    auto myforceb = std::make_shared<ChForce>();
    myforceb->SetMode(ChForce().FORCE);
    ball->AddForce(myforceb);
    myforceb->SetDir(VECT_X);
    myforceb->SetVrelpoint(ChVector<>(rx,0.03*ry,rz));
    forceb.push_back(myforceb);
    
    //Make the ball blue.
    auto color_ball = std::make_shared<ChColorAsset>();
    color_ball->SetColor(ChColor(0, 0, 1));
    ball->AddAsset(color_ball);

    //Mating the bots to the floor so they don't tip.
    auto pt = chrono_types::make_shared<ChLinkMatePlane>(); 
    pt->Initialize(floorBody, ball, true, ChVector<>(0, 0, 0), ChVector<>(0, -height / 2, 0), ChVector<>(0, -1, 0), ChVector<>(0, 1, 0));
    mphysicalSystem.AddLink(pt); //Adding the link to the system.

    auto z2x = ChQuaternion<>();
    z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
    auto prismatic_ground_ball = std::make_shared<ChLinkLockPrismatic>();
    prismatic_ground_ball->Initialize(floorBody, ball, ChCoordsys<>(ChVector<>(5.5, 0, 0), z2x));
    mphysicalSystem.AddLink(prismatic_ground_ball);

    Balls.push_back(ball);
    mphysicalSystem.Add(ball);

    // -----------------------------------------------------------------------------
    // Make walls
    // -----------------------------------------------------------------------------
    
    double wall_rotate = CH_C_PI / 2;
    double wall_length = 5;
    double wall_height = 0.25;
    double wall_width = 0.1;
    double wall_mass = 05;
    auto wall_inertia = ChVector<>(1, 1, 1);

    //First wall.
    double z_wall =0 ;
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

    mphysicalSystem.Add(wall);

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

    mphysicalSystem.Add(wall_1);

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

    mphysicalSystem.Add(wall_2);
    // -----------------------------------------------------------------------------
    // Create empty .dat files to be filled
    // -----------------------------------------------------------------------------
    std::vector<double> springforce;
    std::vector<double> springlength;
    
    std::string templ=out_dir+"/templ.dat"; //Storing spring lengths
    std::string Fm=out_dir+"/Fm.dat"; //Storing the force in the springs.

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
    //======================================================================

    // Time values
    double tstep = 0.005;   //Time Step
    double tset = 0.01;     //settling time
    double tend = 9;        //Length of simulation
    double tp = 7;          //Time to pull
    double tj = 5;          //Jaming starts
    double t = 0;           //Simulation time (to be filled later)

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(2, 2, -5),     // Position
                                 core::vector3df(0, 1, 0));     // Aim
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

    // Adjust some settings:
    application.SetTimestep(tstep);
    application.SetTryRealtime(false);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();

        t += tstep;
        Controller(mphysicalSystem, force, mag, botcall, tset, 
            springforce, nb, k, springlength, t, rl, rlmax, 
            Springs, forceb, jamcall, rlj, rljmax, tj, kj, mag2, magf, tp, obj, mag3);
        
        ExtractData(obj, nb, nt, springforce, springlength, 
            X_pos, Y_pos, Z_pos, X_force, Y_force, Z_force, 
            rott_0, rott_1, rott_2, rott_3, X_vel, Y_vel, Z_vel, 
            spring_length, spring_force, ball_p, Balls,t,Springs);

        application.DoStep();
        application.EndScene();
        if (t > tend) {
            application.GetDevice()->closeDevice();
        }
    }

    return 0;
}
