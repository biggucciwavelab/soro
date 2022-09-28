#general code for comprssion testing of rings of regular 2D nodes (jammed or unjammed)

import pychrono as chrono 
import builtins
import numpy as np
import sys
import pychrono.irrlicht as chronoirr
import time
import os

def main(
    r_i=0.025,
    h=0.01,
    N_s=10,
    N_n=25,
    body_friction=1.2,
    ground_friction=0.05,
    block_friction=0.05,
    r=0.01,
    k=1,
    jammed=False,
    m_timestep=0.001,
    t_delay=0.1,
    csv_time_interval = 10.0,
):

    r_o = r_i / np.cos(np.pi / N_s)
    R_o = r_o / np.sin(np.pi / N_n)
    x_o = 0.049 if(jammed) else 0.051
    cam_y = 5*R_o
    acc = 0.5

    m_length = 1.0
    video_time_interval = int(1.0 / m_timestep / 240.0)  # create video at 20 fps
    m_visualization = "irrlicht"

    polygon_texture = chrono.ChTexture()
    polygon_texture.SetTextureFilename('.\data\octB.png')
    ground_texture = chrono.ChTexture()
    ground_texture.SetTextureFilename('.\data\ground.png')
    block_texture = chrono.ChTexture()
    block_texture.SetTextureFilename('.\data\octA.png')

    bodies = get_ring(N_s=N_s, r_i=r_i, h=h, density=2414.3, N_n=N_n, texture=polygon_texture)
    springs = get_ring_springs(bodies, x_o, r, k)
    ground = get_ground(8*R_o, 2*r_i, 6*R_o, 10000, ground_texture)
    block_fixed = get_block_fixed(R_o+2*r_o, r_o, h, 2*R_o, 10000, block_texture)
    block_moving = get_block_moving(R_o+2*r_o, r_o, h, 2*R_o, 10000, block_texture)

    """
    #constrain moving block to be parallel to fixed block (redundant bc I define moving block position at each time step later)
    link = chrono.ChLinkMateParallel()
    cA = chrono.ChVectorD(0, 0, 0)
    dA = chrono.ChVectorD(1, 0, 0)
    cB = chrono.ChVectorD(0, 0, 0)
    dB = chrono.ChVectorD(1, 0, 0)
    link.Initialize(block_moving, block_fixed, False, cA, cB, dA, dB)
    """
    for body in bodies:
        body.GetMaterialSurfaceNSC().SetFriction(body_friction)
        body.GetMaterialSurfaceNSC().SetRollingFriction(0.0)
    ground.GetMaterialSurfaceNSC().SetFriction(ground_friction)
    ground.GetMaterialSurfaceNSC().SetRollingFriction(0.0)
    block_fixed.GetMaterialSurfaceNSC().SetFriction(block_friction)
    block_fixed.GetMaterialSurfaceNSC().SetRollingFriction(0.0)
    block_moving.GetMaterialSurfaceNSC().SetFriction(block_friction)
    block_moving.GetMaterialSurfaceNSC().SetRollingFriction(0.0)

    my_system = chrono.ChSystemNSC()
    for body in ([block_fixed,block_moving, ground] + bodies + springs): #([block_fixed, block_moving, link, ground] + bodies + springs):
        my_system.Add(body)
    my_system.SetSolverTolerance(1e-9)
    solver = chrono.ChSolverBB()
    my_system.SetSolver(solver)
    solver.SetMaxIterations(3000)
    chrono.ChCollisionModel_SetDefaultSuggestedEnvelope(0.001)
    my_system.SetStep(m_timestep)
    cwd = os.getcwd()

    j = 'jammed' if(jammed) else 'unjammed'
    #csv_filename = 'results/Nsides_%d_Nnodes_%d_%s_%i.csv' %(N_s,N_n,j,int(time.time()))
    csv_filename = 'results/Nsides_%d_Nnodes_%d_UnajmmedObstacle_%i.csv' % (N_s, N_n, int(time.time()))
    f = open(csv_filename, 'w')
    snapshots_dir = csv_filename.replace('.csv', '_snapshots')

    if m_visualization == "irrlicht":
        #  Create an Irrlicht application to visualize the system
        myapplication = chronoirr.ChIrrApp(my_system, 'Ring of %d Polygons with %d Sides' %(N_s,N_n), chronoirr.dimension2du(1280, 720))
        myapplication.AddTypicalSky(chrono.GetChronoDataPath() + 'skybox/')
        myapplication.AddTypicalCamera(chronoirr.vector3df(R_o/2, cam_y, 0), chronoirr.vector3df(R_o/2, 0.0, 0.0))
        myapplication.AddLightWithShadow(chronoirr.vector3df(2, 5, 2), chronoirr.vector3df(2, 2, 2), 10, 2, 10, 120)
        myapplication.AddLightWithShadow(chronoirr.vector3df(2, 5, -2), chronoirr.vector3df(2, 2, 2), 10, 2, 10, 120)
        myapplication.SetSymbolscale(0.002)
        myapplication.SetShowInfos(False)
        #myapplication.SetContactsDrawMode(2)
        myapplication.AssetBindAll()
        myapplication.AssetUpdateAll()
        my_system.Set_G_acc(chrono.ChVectorD(0, -9.8, 0))

        ####################################################################################
        # video capture code
        # uncomment next line to activate, but if timestep different than Irrlicht default (0.01) simulation is unstable
        #myapplication.SetTimeStep(m_timestep)
        if video_time_interval is not None and isinstance(video_time_interval, int):
            if not os.path.exists(snapshots_dir):
                os.makedirs(snapshots_dir)
            os.chdir(snapshots_dir)
            myapplication.SetVideoframeSave(True)
            myapplication.SetVideoframeSaveInterval(video_time_interval)
        #####################################################################################

        t = 0
        i = 0
        z_max = 0.0
        z_min = 0.0
        z_diff = 0.0
        z_stop = 0.0
        #x_0 = block_moving.GetPos().x
        #y_0 = block_moving.GetPos().y
        z_0 = 0.0
        t_strain = m_length
        while (myapplication.GetDevice().run()):
            """
            # after the ring jams (t_delay) move blocks to be in contact with ring
            if (t > t_delay - m_timestep) and (t <= t_delay):
                for body in bodies:
                    if body.GetPos().z > z_max:
                        z_max = body.GetPos().z
                    if body.GetPos().z < z_min:
                        z_min = body.GetPos().z
                z_0 = z_min - 1.5 * r_o
                z_diff = z_max - z_min - r_o;
                t_strain = (strain_max * z_diff) / strain_rate#+ t_delay - m_timestep
                #print(z_0)
                #print(z_diff)
                #print(t_strain)
            
                block_fixed_updated = chrono.ChVectorD(block_fixed.GetPos().x,block_fixed.GetPos().y, z_max + 1.5 * r_o)
                block_moving_updated = chrono.ChVectorD(x_0, y_0, z_0)
                block_fixed.SetPos(block_fixed_updated)
                block_moving.SetPos(block_moving_updated)


            # compress ring by moving left block at constant rate
            if (t > t_delay) and (t < t_strain):
                block_moving.SetPos(chrono.ChVectorD(x_0, y_0, z_0 + strain_rate*t))
                z_stop = block_moving.GetPos().z
            if (t >= t_strain) and (t < t_strain + m_timestep):
                #z_stop = block_moving.GetPos().z
                print(z_stop)
                check = 0.0 if (z_diff == 0.0) else (z_stop - z_0)/z_diff
                print(check)
            if (t >= t_strain):
                block_moving.SetPos(chrono.ChVectorD(x_0, y_0, z_stop))
            # outputs
            moving_app_force = block_moving.Get_Xforce()
            moving_contact_force = block_moving.GetContactForce()
            fixed_contact_force = block_fixed.GetContactForce()
            if i % 10 == 0:
                strain = 0.0 if (z_diff == 0.0) else (block_moving.GetPos().z - z_0)/z_diff
                print(t, block_moving.GetPos().z, strain, moving_app_force.z)
                print("%f,%f,%f,%f,%f" % (t, block_moving.GetPos().z, strain,
                                             moving_contact_force.z, fixed_contact_force.z), file=f)
            """
            if (t > t_delay):
                my_system.Set_G_acc(chrono.ChVectorD(acc, -9.8, 0))
            if i % 100 == 0:
                print(t)
            myapplication.BeginScene()
            myapplication.DrawAll()
            myapplication.DoStep()
            myapplication.EndScene()
            t += m_timestep
            i += 1
            if (t >= m_length):
                break
    f.close()
    # change back to original directory so relative paths are correct for next run
    os.chdir(cwd)
    time.sleep(1)

def get_polygon_body(N_s=8, r_i=0.025, h=0.01, density=2414.3, texture=None):
    pt_vect = chrono.vector_ChVectorD()
    r_o = r_i/np.cos(np.pi/N_s)
    eta = np.random.rand() * 2*np.pi/N_s
    for i in range(N_s):
        pt_vect.push_back(chrono.ChVectorD(
            r_o*np.cos(eta + i*2*np.pi/N_s),
            0,
            r_o*np.sin(eta + i*2*np.pi/N_s)
        ))
        pt_vect.push_back(chrono.ChVectorD(
            r_o * np.cos(eta + i*2*np.pi / N_s),
            h,
            r_o * np.sin(eta + i*2*np.pi / N_s)
        ))
    body = chrono.ChBodyEasyConvexHull(pt_vect, density, True, True)
    if texture is not None:
        body.AddAsset(texture)
    return body

def get_ring(N_s=8, r_i=0.025, h=0.01, density=2414.3, N_n=8, texture=None):
    ring_bodies = []
    r_o = r_i/np.cos(np.pi/N_s)
    R_o = r_o/np.sin(np.pi/N_n)
    eta = np.random.rand() * 2*np.pi/N_n
    for i in range(N_n):
        body = get_polygon_body(N_s, r_i, h, density, texture)
        body.SetPos(chrono.ChVectorD(R_o*np.cos(eta + i*2*np.pi/N_n), h/2, R_o*np.sin(eta + i*2*np.pi/N_n)))
        ring_bodies.append(body)
    return ring_bodies

def get_ring_springs(bodies, d=0.049, r=0.1, k=100):
    springs = []
    for body_A, body_B in zip(bodies, (bodies[1:] + [bodies[0]])):
        spring = chrono.ChLinkSpring()
        spring.Initialize(body_A, body_B, True, chrono.ChVectorD(0.0, 0, 0.0), chrono.ChVectorD(0.0, 0, 0.0), True)
        spring.Set_SpringRestLength(d)
        spring.Set_SpringR(r)
        spring.Set_SpringK(k)
        springs.append(spring)
    return springs

def get_ground(x_s, y_s, z_s, density=10000, texture=None):
    ground = chrono.ChBodyEasyBox(x_s, y_s, z_s, density, True, True)
    ground.SetPos(chrono.ChVectorD(0, -y_s/2, 0))
    ground.SetBodyFixed(True)
    if texture is not None:
        ground.AddAsset(texture)
    return ground

def get_block_fixed(R_o, x_s, y_s, z_s, density=100000, texture=None):
    block_fixed = chrono.ChBodyEasyBox(x_s, y_s, z_s, density, True, True)
    block_fixed.SetPos(chrono.ChVectorD(R_o, y_s/2, z_s/2+R_o/2))
    block_fixed.SetBodyFixed(True)
    if texture is not None:
        block_fixed.AddAsset(texture)
    return block_fixed

def get_block_moving(R_o, x_s, y_s, z_s, density=100000, texture=None):
    block_moving = chrono.ChBodyEasyBox(x_s, y_s, z_s, density, True, True)
    block_moving.SetPos(chrono.ChVectorD(R_o, y_s/2, -(z_s/2+R_o/2)))
    block_moving.SetBodyFixed(True)
    if texture is not None:
        block_moving.AddAsset(texture)
    return block_moving

if __name__ == '__main__':
    main()

sys.exit()
