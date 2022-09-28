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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
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

//Empty vectors
std::vector<std::shared_ptr<ChBodyEasyCylinder>> objects;    // vector to store cylinder bodies

int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);
    
    // Create a Chrono physical system
    ChSystemNSC system;
    system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    system.SetSolverMaxIterations(200);
    // Use PSOR solver
    // Set gravity

    //Contact material
    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    material->SetFriction(0.5f);
    material->SetCompliance(1e-4);
    material->SetRestitution(0.8f);

    // Define colors
    auto mtexture = chrono_types::make_shared<ChTexture>(GetChronoDataFile("cubetexture_wood.png"));
    //auto sprVis = chrono_types::make_shared<ChPointPointSpring>(0.01f, 80.f, 15.f);


    // Spring constants
    float km = 500;
    float bm = 10;

    //Create floor
    if (true) {
        auto floor = chrono_types::make_shared<ChBodyEasyBox>(4.0f, 0.1f, 4.0f, 1000.f, true, true);
        floor->SetMaterialSurface(material);
        floor->SetBodyFixed(true);
        floor->AddAsset(mtexture);
        system.Add(floor);
    }
    
    // Two big bodies
    auto declan = chrono_types::make_shared<ChBodyEasyCylinder>(0.04725f, 0.05f, 2000.f, true, true);
    declan->SetPos(chrono::Vector(0.f, 0.5f, 0.f));
    declan->SetMaterialSurface(material);
    declan->SetBodyFixed(true);

    auto esteban = chrono_types::make_shared<ChBodyEasyCylinder>(0.04725f, 0.05f, 2000.f, true, true);
    esteban->SetPos(chrono::Vector(0.5f, 0.5f, 0.f));
    esteban->SetMaterialSurface(material);
    esteban->SetBodyFixed(true);

    system.Add(declan); system.Add(esteban);
    objects.push_back(declan);

    // Link them
    int ratioM = 5;


    for (int i = 0; i < ratioM; i++) {
        // Link to last object with springs

        // Initial position of particle
        float x = float(i) * 0.5 / (float(ratioM) + 1.f) + 0.012;
        float y = 0.5;
        float z = 0;

        // Create particles
        auto skinm = chrono_types::make_shared<ChBodyEasyCylinder>(0.01f, 0.04f, 2000.f, true, true);
        skinm->SetPos(chrono::Vector(x, y, z));
        skinm->SetMaterialSurface(material);
        skinm->SetNoGyroTorque(true);
        skinm->SetId(i+1);

        // Make the springs
        float p1 = 0; 
        float p2 = 0.02;
        float p3 = 0;
        float p4 = -0.02;
        float h = 0.01;

        auto ground1 = chrono_types::make_shared<ChLinkTSDA>();
        ground1->Initialize(objects[i], skinm, true, chrono::Vector(p1, h, p2), chrono::Vector(p3, h, p4), false, 0.0f);
        ground1->SetSpringCoefficient(km);
        ground1->SetDampingCoefficient(bm);
        //ground1->AddAsset(sprVis);
        system.Add(ground1);

        auto ground2 = chrono_types::make_shared<ChLinkTSDA>();
        ground2->Initialize(objects[i], skinm, true, chrono::Vector(p1, -h, p2), chrono::Vector(p3, -h, p4), false, 0.0f);
        ground2->SetSpringCoefficient(km);
        ground2->SetDampingCoefficient(bm);
        //ground1->AddAsset(sprVis);
        system.Add(ground2);

        if (i == ratioM - 1) {
            auto ground3 = chrono_types::make_shared<ChLinkTSDA>();
            ground3->Initialize(esteban, skinm, true, chrono::Vector(p1, h, p2), chrono::Vector(p3, h, p4), false, 0.0f);
            ground3->SetSpringCoefficient(km);
            ground3->SetDampingCoefficient(bm);
            //ground3->AddAsset(sprVis);
            system.Add(ground3);

            auto ground4 = chrono_types::make_shared<ChLinkTSDA>();
            ground4->Initialize(esteban, skinm, true, chrono::Vector(p1, h, p2), chrono::Vector(p3, h, p4), false, 0.0f);
            ground4->SetSpringCoefficient(km);
            ground4->SetDampingCoefficient(bm);
            //ground4->AddAsset(sprVis);
            system.Add(ground4);
        }
        objects.push_back(skinm);
        system.Add(skinm);
        std::cout << objects[i]->GetId() << std::endl;
    }

    //======================================================================
    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&system, L"A simple project template", core::dimension2d<u32>(800, 600),
        false);  // screen dimensions

// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0.75f,0.75,1.5f),
        core::vector3df(0, 1, 0));  // to change the position of camera
// application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Adjust some settings:
    application.SetTimestep(0.001);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
