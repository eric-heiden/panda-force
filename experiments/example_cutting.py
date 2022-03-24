import numpy as np
import time
import os

from tqdm.auto import tqdm, trange

from . import PandaZmqController

def main():
    import roboticstoolbox as rtb
    from roboticstoolbox.backends.Swift import Swift
    from spatialmath import SE3
    from scipy import interpolate

    hold = False
    wait = 7
    delay = 3.0
    knife = "slicing"

    model = PandaZmqController(connect=False)
    model.load_robot_model(knife)
    model.connect()
    model.error_recovery()
    
    model.set_knife(knife)

    print("current q:", model.get_state().q)
    
    factor = 2.0
    k = np.array([3000, 3000, 3000, 2500, 2500, 2000, 2000])
    model.set_joint_impedance(k * factor)
    model.set_collision_behavior()
    
    q_slicing_knife_above_cutting_board_11cm = np.array([
        1.13059481, -1.04005643, -1.77573664, -1.95988677, -1.15048631,  1.95949429,
   0.63239828
    ])
    
    q = q_slicing_knife_above_cutting_board_11cm

    diff = model.get_state().q - q
    print("diff:", diff)

    model.move_to_q(q, speed=0.1)
    diff = model.get_state().q - q
    print("diff:", diff)

    # model.plot_qs([q, q])

    os.chdir('assets/panda_robot')
    urdf_path = f'panda_arm_{knife}.urdf'
    urdf_content = open(urdf_path, 'r').read()
    urdf = rtb.tools.URDF.loadstr(urdf_content, urdf_path)
    robot = rtb.ERobot(urdf.elinks, name=urdf.name)
    robot.q = q
    os.chdir('../..')

    # pyplot = rtb.backends.PyPlot.PyPlot()
    # pyplot.launch()
    # pyplot.add(robot)
    
    CONTROL_FREQUENCY = 20
    CONTROL_DURATION = 10
    
    amplitude = 0.0 # 0.07
    knife_height = 0.11

    rest_phase = np.zeros((5, 3))
    # go slightly longer because the knife will not make a complete cut otherwise
    times = np.linspace(0, 1.06, int(CONTROL_DURATION * CONTROL_FREQUENCY))[:, None]
    cartesian_trajectory = np.hstack((
        np.sin(times * 20.0) * amplitude, times * 0.0, times * (-knife_height)))
    # smoothen the beginning a bit
    cartesian_trajectory[:CONTROL_FREQUENCY * 2] *= np.linspace(0, 1, CONTROL_FREQUENCY * 2)[:,None]
    
    # cartesian_trajectory = np.vstack((rest_phase, cartesian_trajectory, rest_phase))

    # plt.grid()
    # plt.plot(times, cartesian_trajectory)
    # plt.show()

    robot.ee_links = [robot.ee_links[0]]
    ee_tf = robot.fkine(q)
    # ee_target = SE3(0.0, 0.0, -0.12) @ ee_tf

    # start = time.time()
    # sol = robot.ikine_min(ee_target, q0=q, qlim=True)
    # print("IK time:", time.time() - start)
    # print("residual:", sol.residual)
    # linfit = interp1d([0, 1], np.vstack([q, sol.q]), axis=0)
    # qs = linfit(np.linspace(0, 1, 10000))

    qs = [q]
    for xyz in tqdm(cartesian_trajectory):
        ee_target = SE3(*xyz) @ ee_tf
        # sol = robot.ikine_min(ee_target, q0=qs[-1], qlim=True, ilimit=100, )
        sol = robot.ikine_LM(ee_target, q0=qs[-1], ilimit=100, )
        qs.append(sol.q)
    qs = np.array(qs[1:])

    model.plot_qs(qs)
    
    # model.preview_qs(qs, source_dt=1.0 / CONTROL_FREQUENCY)

    states = model.follow_qs(qs, source_dt=1.0 / CONTROL_FREQUENCY)
    model.save_states(states, f'{knife}_vertical_cutting_apple.json')
    model.plot_qs(states.q)

    backend = Swift()
    backend.launch()
    backend.add(robot)

    tcks = [interpolate.splrep(times, qs[:, i]) for i in range(7)]
    interpolated_qs = [interpolate.splev(
        np.linspace(0, 1, 10000), tcks[i]) for i in range(7)]
    interpolated_qs = np.array(interpolated_qs).T

    if delay > 0:
        time.sleep(delay)
    for q in tqdm(interpolated_qs):
        robot.q = q          # update the robot state
        backend.step()       # update visualization
        time.sleep(1e-3)
    if wait > 0:
        time.sleep(wait)
    if hold:
        backend.hold()


if __name__ == "__main__":
    main()
