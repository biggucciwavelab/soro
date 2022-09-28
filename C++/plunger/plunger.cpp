// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Qiyuan Zhou
// =============================================================================
// Model simulating a cylindrical plunger being pushed into media at a constant
// velocity. Paper for inspiration can be found at the link below:
// https://arxiv.org/abs/1910.03173
// =============================================================================

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include "chrono/core/ChGlobal.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_granular/api/ChApiGranularChrono.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_granular/utils/ChGranularJsonParser.h"

using namespace chrono;
using namespace chrono::granular;

void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file>" << std::endl;
}

void writeMeshFrames(std::ostringstream& outstream, ChBody& body, std::string obj_name, float mesh_scaling) {
    outstream << obj_name << ",";

    // Get frame position
    ChFrame<> body_frame = body.GetFrame_REF_to_abs();
    ChQuaternion<> rot = body_frame.GetRot();
    ChVector<> pos = body_frame.GetPos();

    // Get basis vectors
    ChVector<> vx = rot.GetXaxis();
    ChVector<> vy = rot.GetYaxis();
    ChVector<> vz = rot.GetZaxis();

    // Output in order
    outstream << pos.x() << ",";
    outstream << pos.y() << ",";
    outstream << pos.z() << ",";
    outstream << vx.x() << ",";
    outstream << vx.y() << ",";
    outstream << vx.z() << ",";
    outstream << vy.x() << ",";
    outstream << vy.y() << ",";
    outstream << vy.z() << ",";
    outstream << vz.x() << ",";
    outstream << vz.y() << ",";
    outstream << vz.z() << ",";
    outstream << mesh_scaling << "," << mesh_scaling << "," << mesh_scaling;
    outstream << "\n";
}

int main(int argc, char* argv[]) {
    sim_param_holder params;
    float plunger_vel;

    // Entry for .json file
    if (argc < 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage(argv[0]);
        return 1;
    }
    // Entry for plunger velocity
    if (argc < 3){
        cout<<"Remember that negative z direction is down\n";
        cout<<"Enter plunger velocity (cm/s): \n";
        std::cin>>plunger_vel;
        cout<<"Plunger velocity is: "<<plunger_vel<<"cm/s\n";
    } else{
        plunger_vel=atof(argv[2]);
        cout<<"Plunger velocity is: "<<plunger_vel<<"cm/s\n";
    }

    double iteration_step = params.step_size;

    ChGranularChronoTriMeshAPI apiSMC_TriMesh(params.sphere_radius, params.sphere_density,
                                              make_float3(params.box_X, params.box_Y, params.box_Z));

    ChSystemGranularSMC_trimesh& gran_sys = apiSMC_TriMesh.getGranSystemSMC_TriMesh();

    double fill_bottom = -params.box_Z / 2.0;
    double fill_top = params.box_Z / 2.0;

    chrono::utils::PDSampler<float> sampler(2.05 * params.sphere_radius);
    //chrono::utils::HCPSampler<float> sampler(2.05 * params.sphere_radius);
    //chrono::utils::GridSampler<float> sampler(2.05*params.sphere_radius);

    // leave a margin at edges of sampling
    ChVector<> hdims(params.box_X / 2 - 2.05*params.sphere_radius, params.box_Y / 2 - 2.05*params.sphere_radius, 0);
    ChVector<> center(0, 0, fill_bottom + 2.05*params.sphere_radius * params.sphere_radius);
    std::vector<ChVector<float>> body_points;

    // Shift up for bottom of box
    center.z() += 2.05 * params.sphere_radius;
    while (center.z() < fill_top) {
        std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }

    apiSMC_TriMesh.setElemsPositions(body_points);

    gran_sys.set_BD_Fixed(true);

    gran_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    gran_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    gran_sys.set_K_n_SPH2MESH(params.normalStiffS2M);

    gran_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    gran_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);
    gran_sys.set_Gamma_n_SPH2MESH(params.normalDampS2M);

    gran_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    gran_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    gran_sys.set_K_t_SPH2MESH(params.tangentStiffS2M);

    gran_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    gran_sys.set_Gamma_t_SPH2MESH(params.tangentDampS2M);

    gran_sys.set_Cohesion_ratio(params.cohesion_ratio);
    gran_sys.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);
    gran_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    gran_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);

    gran_sys.set_fixed_stepSize(params.step_size);
    gran_sys.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CHUNG);
    gran_sys.set_static_friction_coeff_SPH2SPH(params.static_friction_coeffS2S);
    gran_sys.set_static_friction_coeff_SPH2WALL(params.static_friction_coeffS2W);
    gran_sys.set_static_friction_coeff_SPH2MESH(params.static_friction_coeffS2M);

    // Import the mesh and set some parameters

    float cylinder_radius = 1;
    float cylinder_height = 10;
    float cylinder_density = 9;

    std::string mesh_filename("plunger_1cm.obj");
    std::vector<string> mesh_filenames(1, mesh_filename);
    std::vector<float3> mesh_translations(1, make_float3(0, 0, 0));
    std::vector<ChMatrix33<float>> mesh_rotscales(1, ChMatrix33<float>(cylinder_radius));
    
    float cylinder_mass = CH_C_PI * cylinder_radius * cylinder_radius * cylinder_height * cylinder_density;
    
    std::vector<float> mesh_masses(1, cylinder_mass);

    apiSMC_TriMesh.load_meshes(mesh_filenames, mesh_rotscales, mesh_translations, mesh_masses);

    gran_sys.setOutputMode(params.write_mode);
    gran_sys.setVerbose(params.verbose);
    filesystem::create_directory(filesystem::path(params.output_dir));

    unsigned int nSoupFamilies = gran_sys.getNumTriangleFamilies();
    std::cout << nSoupFamilies << " soup families" << std::endl;
    double* meshPosRot = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    gran_sys.initialize();

    // Create rigid ball_body simulation
    ChSystemSMC sys_ball;
    sys_ball.SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
    sys_ball.SetTimestepperType(ChTimestepper::Type::EULER_EXPLICIT);
    sys_ball.Set_G_acc(ChVector<>(0, 0, -981));

    double inertiaxy = (cylinder_mass/12.0) * (3*cylinder_radius*cylinder_radius+cylinder_height*cylinder_height);
    double inertiaz = cylinder_mass*cylinder_radius*cylinder_radius/2;
    ChVector<> ball_initial_pos(0, 0, fill_top + cylinder_height + 2 * params.sphere_radius);

    std::shared_ptr<ChBody> ball_body(sys_ball.NewBody());
    ball_body->SetMass(cylinder_mass);
    ball_body->SetInertiaXX(ChVector<>(inertiaxy, inertiaxy, inertiaz));
    ball_body->SetPos(ball_initial_pos);
    ball_body->SetBodyFixed(true);
    sys_ball.AddBody(ball_body);
    unsigned int out_fps = 500;
    std::cout << "Rendering at " << out_fps << "FPS" << std::endl;

    unsigned int out_steps = 1 / (out_fps * iteration_step);

    int currframe = 0;
    unsigned int curr_step = 0;

    char force_write[100];
    sprintf(force_write, "%s", params.output_dir.c_str());
    std::string mesh_force = std::string(force_write) + "/meshforce.csv";
    std::ofstream forces(mesh_force);
    std::ostringstream outforce;
    outforce << "fx,fy,fz,x,y,z\n";

    int count = 0;
    float set_time = 1.5; // Settling time before plunger goes in

    clock_t start = std::clock();
    for (float t = 0; t < params.time_end; t += iteration_step, curr_step++) {
        auto ball_pos = ball_body->GetPos();
        auto ball_rot = ball_body->GetRot();

        if(t>set_time){
            ball_body->SetBodyFixed(false);
            ball_body->SetPos_dt(ChVector<>(0,0,plunger_vel));
        }
        
        auto ball_vel = ball_body->GetPos_dt();
        auto ball_ang_vel = ball_body->GetWvel_loc();
        ball_ang_vel = ball_body->GetRot().GetInverse().Rotate(ball_ang_vel);

        meshPosRot[0] = ball_pos.x();
        meshPosRot[1] = ball_pos.y();
        meshPosRot[2] = ball_pos.z();
        meshPosRot[3] = ball_rot[0];
        meshPosRot[4] = ball_rot[1];
        meshPosRot[5] = ball_rot[2];
        meshPosRot[6] = ball_rot[3];

        meshVel[0] = ball_vel.x();
        meshVel[1] = ball_vel.y();
        meshVel[2] = ball_vel.z();
        meshVel[3] = ball_ang_vel.x();
        meshVel[4] = ball_ang_vel.y();
        meshVel[5] = ball_ang_vel.z();

        if(meshPosRot[2]<fill_bottom+10-(cylinder_height/2)){
            break;
        }

        gran_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);

        gran_sys.advance_simulation(iteration_step);
        sys_ball.DoStepDynamics(iteration_step);

        float ball_force[6];
        gran_sys.collectGeneralizedForcesOnMeshSoup(ball_force);

        ball_body->Empty_forces_accumulators();
        ball_body->Accumulate_force(ChVector<>(ball_force[0], ball_force[1], ball_force[2]), ball_pos, false);
        ball_body->Accumulate_torque(ChVector<>(ball_force[3], ball_force[4], ball_force[5]), false);

        // Export Forces
        if(meshPosRot[2]<5){
            outforce<<ball_force[0]/100000.f<<","<<ball_force[1]/100000.f<<","<<ball_force[2]/100000.f  // Forces
                    <<","<<meshPosRot[0]<<","<<meshPosRot[1]<<","<<meshPosRot[2]<<"\n";                 // Positions
        }
        
        if (curr_step % out_steps == 0 & meshPosRot[2]<5) {
            std::cout << "Rendering frame " << currframe << std::endl;
            char filename[100];
            sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
            gran_sys.writeFile(std::string(filename));

            std::string mesh_output = std::string(filename) + "_meshframes.csv";
            std::ofstream meshfile(mesh_output);
            std::ostringstream outstream;
            outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3,sx,sy,sz\n";
            writeMeshFrames(outstream, *ball_body, mesh_filename, cylinder_radius);
            meshfile << outstream.str();
            
            count+=1;
        }
        cout<<"Current simulation time: "<< std::setprecision(4) <<t<<"s\n";
    }
    forces << outforce.str();

    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

    std::cout << "Time: " << total_time << " seconds" << std::endl;

    delete[] meshPosRot;
    delete[] meshVel;

    return 0;
}
