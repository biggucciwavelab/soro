from run_two_study import run_two_node_study, \
    add_spring_between_bodies, set_G, write_header_info_to_file, dump_info_to_file

import logging
import os
import time
import pychrono as chrono

def set_friction_params(my_system, exported_items, body_names, friction=0.4, rolling_friction=0.00):
    body = None
    for body_name in body_names:
        for item in exported_items:
            if item.GetName() == body_name:
                body = item
                break
        body.GetMaterialSurfaceNSC().SetFriction(friction)
        body.GetMaterialSurfaceNSC().SetRollingFriction(rolling_friction)

def set_G_2(my_system, exported_items, t):
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.8, t))

# For Debug Purposes
def output_spring_force(my_system, exported_items, t, spring_names=['Spring']):
    spring = None
    for spring_name in spring_names:
        for body in exported_items:
            if body.GetName() == spring_name:
                spring=body
                break
        print(t, spring_name, spring.GetDist(), spring.GetDist_dt(), spring.GetC_force())



def main():
    m_filenames = [
        'two_node_2250_study.py',
        'two_node_3375_study.py',
        'two_node_4500_study.py',
        'two_node_5625_study.py',
        'two_node_6750_study.py',
    ]

    preprocess_params_for_springs = [
        {'body_name_1': 'node_2250-1', 'body_name_2': 'node_2250-2'},
        {'body_name_1': 'node_3375-1', 'body_name_2': 'node_3375-2'},
        {'body_name_1': 'node_4500-1', 'body_name_2': 'node_4500-2'},
        {'body_name_1': 'node_5625-1', 'body_name_2': 'node_5625-2'},
        {'body_name_1': 'node_6750-1', 'body_name_2': 'node_6750-2'}
    ]

    m_timestep = 0.0001
    ks = [200]#[50, 100, 200]
    friction_paramss = [
        #{'friction': 0.1},#, 'rolling_friction': 0.0},
        #{'friction': 0.2},#, 'rolling_friction': 0.0},
        #{'friction': 0.4},#, 'rolling_friction': 0.0},
        {'friction': 0.8},#, 'rolling_friction': 0.0},
        {'friction': 1.2},#, 'rolling_friction': 0.0},
    ]
    cwd = os.getcwd()
    for filename, preprocess_params_for_spring in zip(m_filenames, preprocess_params_for_springs):
        for k in ks:
            for friction_params in friction_paramss:
                try:
                    del friction_params['body_names']
                except:
                    pass
                preprocess_params_for_spring["K"] = k
                csv_filename = './results/%s_k_%d_%s.csv' % (
                    filename.replace('.py', ''),
                    k,
                    str(friction_params).replace('{', '').replace(
                        '}', '').replace(':', '_').replace(',', '_').replace("'", "").replace(' ', '')
                )
                # CWD will be changed to this directory for snapshots from irrlicht
                snapshots_dir = csv_filename.replace('.csv', '_snapshots')

                f = open(csv_filename, 'w')
                body_names_to_write = [preprocess_params_for_spring[name] for name in ['body_name_1', 'body_name_2']]
                friction_params['body_names'] = body_names_to_write#['ground-1'] + body_names_to_write
                #print(friction_params, preprocess_params_for_spring)
                run_two_node_study(
                    m_filename=filename,
                    preprocess_params=[
                        preprocess_params_for_spring,
                        friction_params,
                        {'body_names': ['ground-1'], 'friction': 0.01},
                        {'f': f, 'body_names': body_names_to_write}
                    ],
                    preprocess_funcs=[
                        add_spring_between_bodies,
                        set_friction_params,
                        set_friction_params,
                        write_header_info_to_file
                    ],
                    iteration_postprocess_funcs=[
                        set_G_2,
                        dump_info_to_file,
                        #output_spring_force
                    ],
                    iteration_postprocess_params=[
                        {},
                        {'f': f, 'body_names': body_names_to_write, 'timestep_for_file': 0.001, 'm_timestep': m_timestep},
                        #{'spring_names': ['Spring_%s_%s' % tuple(body_names_to_write)]}
                    ],
                    m_timestep=m_timestep,
                    m_length=10.0,
                    #######################################################################################
                    video_save_interval= int(1.0/m_timestep/20.0),   # create video at 20 fps
                    #######################################################################################
                    snapshots_dir=snapshots_dir
                )
                f.close()
                # change back to original directory so relative paths are correct for next run
                os.chdir(cwd)
                time.sleep(1)

if __name__=='__main__':
    import sys
    logger = logging.getLogger('two_node_study')
    logger.setLevel(logging.INFO)
    handler = logging.StreamHandler(sys.stdout)
    logger.addHandler(handler)
    main()