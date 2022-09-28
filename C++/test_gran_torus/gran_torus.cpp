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
// Authors: Conlain Kelly
// =============================================================================
// Chrono::Granular simulation of a rectangular bed of granular material which
// is first let to settle and then compressed by advancing one of the box walls
// into the material.
// =============================================================================

#include <iostream>
#include <string>
#include <cmath>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_granular/api/ChApiGranularChrono.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/utils/ChGranularJsonParser.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/core/ChTimer.h"

using namespace chrono;
using namespace chrono::granular;

// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file>" << std::endl;
}

int main(int argc, char* argv[]) {
    ChTimer<double> timer;
    ChTimer<double> loop_time;
    timer.start();
    freopen("benchmark.txt", "w", stdout);  // Toggle logging to .txt file
    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage(argv[0]);
        return 1;
    }

    // Setup simulation
    ChSystemGranularSMC gran_sys(params.sphere_radius, params.sphere_density,
        make_float3(params.box_X, params.box_Y, params.box_Z));

    ChGranularSMC_API apiSMC;
    apiSMC.setGranSystem(&gran_sys);
    gran_sys.setPsiFactors(params.psi_T, params.psi_L);
    gran_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    gran_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    gran_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    gran_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);
    gran_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    gran_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    gran_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    gran_sys.set_static_friction_coeff_SPH2SPH(params.static_friction_coeffS2S);
    gran_sys.set_static_friction_coeff_SPH2WALL(params.static_friction_coeffS2W);
    gran_sys.set_Cohesion_ratio(params.cohesion_ratio);
    gran_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    gran_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    gran_sys.setOutputMode(params.write_mode);
    filesystem::create_directory(filesystem::path(params.output_dir));
    gran_sys.setMaxSafeVelocity_SU(1e8);

    float c = 150;       ///radius of large ring structure
    float a=50;        ///radius of torus tube
    float rad = params.sphere_radius;                   ///< Particle radius
    float padding = 1.05; //padding between particles

    std::vector<float3> pointsFloat3;
    for (float v = 0; v < CH_C_2PI- padding * 2 * atan(rad / a); v = v + padding * 2 * atan(rad / a)) {
        for (float u = 0; u < CH_C_2PI- padding * 2 * atan(rad / (c + a * cos(v))); u = u + padding* 2 * atan(rad / (c + a * cos(v)))) {
            
            // Point on torus
            auto x = cos(u) * (c + a * cos(v));
            auto y = sin(u) * (c + a * cos(v));
            auto z = a* sin(v) + 0;
            float3 point = { x,y,z };
            
            pointsFloat3.push_back(point);
            
        }
    }
    gran_sys.setParticlePositions(pointsFloat3);

    // Set the position of the BD
    gran_sys.set_BD_Fixed(true);
    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CHUNG);
    gran_sys.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
    gran_sys.set_fixed_stepSize(params.step_size);
    gran_sys.setVerbose(params.verbose);

    /*
    // start outside BD by 10 cm
    float plane_pos[3] = { -params.box_X / 2 - 10, 0, 0 };
    float plane_normal[3] = { 1, 0, 0 };

    size_t plane_bc_id = gran_sys.Create_BC_Plane(plane_pos, plane_normal, false);

    // Function prescibing the motion of the advancing plane.
    // Begins outside of the domain.
    std::function<double3(float)> plane_pos_func = [&params](float t) {
        double3 pos = { 0, 0, 0 };

        // move at 10 cm/s
        constexpr float vel = 10;

        // after 1 second the plane will be at the edge of the BD, and will continue in thereafter
        pos.x = vel * t;

        return pos;
    };
    */
    
    // Add a vibrating floor
    // start inside BD by 1 cm
    float plane_pos[3] = { 0, 0, -params.box_Z / 2 - 1 };
    float plane_normal[3] = { 0, 0, 1 };

    size_t plane_bc_id = gran_sys.Create_BC_Plane(plane_pos, plane_normal, false);

    // Function prescibing the motion of the advancing plane.
    // Begins outside of the domain.
    std::function<double3(float)> plane_pos_func = [&params](float t) {
        
        double3 pos = { 0, 0, 0 };
        float freq = 150; //[Hz]

        if (t < 1.f){ pos.z = 10 * t - params.box_Z / 2 - 1; }

        else { pos.z = 5 * sin(2 * CH_C_PI * freq * t-1) + 10 - params.box_Z / 2 - 1;}

        return pos;
    };
    
    gran_sys.initialize();

    gran_sys.set_BC_offset_function(plane_bc_id, plane_pos_func);
    
    int fps = 50;
    // assume we run for at least one frame
    float frame_step = 1.0f / fps;
    float curr_time = 0;
    int currframe = 0;

    char filename[100];
    sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
    gran_sys.writeFile(std::string(filename));

    std::cout << "frame step is " << frame_step << std::endl;

    // Run settling experiments
    while (curr_time < params.time_end) {
        loop_time.start();
        gran_sys.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        gran_sys.writeFile(std::string(filename));
        loop_time.stop();
        std::cout << "Time elapsed= " << loop_time.GetTimeMillisecondsIntermediate() << "ms" << "\n";
    }
    timer.stop();
    std::cout << "Time elapsed= " << timer.GetTimeSeconds() << "s" << "\n";
    return 0;
}