import numpy as np
import json
import matplotlib.pyplot as plt
import time
import os
import munch
import copy

from tqdm.auto import tqdm


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class PandaZmqController:
    q0 = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi /
                  4, 0.0, np.pi / 2, np.pi / 4])

    def __init__(self, ip='172.16.0.1', port: int = 5555, connect: bool = True):
        import zmq
        self.ip = ip
        self.port = port
        self.socket = None
        self.robot_model = None
        self.context = zmq.Context()
        if connect:
            self.connect()

    def connect(self, robot_address="172.16.0.2"):
        import zmq
        if self.socket is None or self.socket.closed:
            print(f'Connecting to ZMQ socket {self.ip}:{self.port}...')
            self.socket = self.context.socket(zmq.REQ)
            self.socket.connect(f"tcp://{self.ip}:{self.port}")
            print(f'Connecting to Panda robot at {robot_address}...')
            s = self._request(f"connect {robot_address}")
            if s != "OK":
                raise Exception(f"Error connecting: {s}")
            print(f"Connected to robot at {self.ip}:{self.port}.")

    def _request(self, what: str) -> str:
        self.connect()
        self.socket.send_string(what)
        message = self.socket.recv()
        return message.decode("UTF-8")

    def _request_array(self, what: str):
        s = self._request(what)
        if not s.startswith("OK"):
            raise Exception(
                f"Error requesting array via {what.split()[0]}: {s}")
        s = s[len("OK "):]
        return np.fromstring(s.replace("[", "").replace("]", ""), sep=" ")

    def _request_nested_array(self, what: str):
        ps = self._request(what)
        if not ps.startswith("OK"):
            raise Exception(
                f"Error requesting nested array via {what.split()[0]}: {ps}")
        ps = ps[len("OK "):]
        ps = np.array([np.fromstring(
            s.replace("[", "").replace("]", ""), sep=" ") for s in ps.split("] [")])
        return ps

    @staticmethod
    def _array2string(x):
        return " ".join(map(lambda d: format(d, '.60g'), x))

    def error_recovery(self):
        s = self._request("error_recovery")
        if not s.startswith("OK"):
            raise Exception(f"Error during automatic error recovery: {s}")

    def set_knife(self, knife="slicing"):
        s = self._request(f"set_knife {knife}")
        if s != "OK":
            raise Exception(f"Error setting knife: {s}")

    def get_knife(self):
        s = self._request("get_knife")
        if not s.startswith("OK"):
            raise Exception(f"Error getting knife: {s}")
        return s[len("OK "):]

    def load_robot_model(self, knife=None):
        import roboticstoolbox as rtb
        if knife is None:
            knife = self.get_knife()
        os.chdir('assets/panda_robot')
        urdf_path = f'panda_arm_{knife}.urdf'
        urdf_content = open(urdf_path, 'r').read()
        urdf = rtb.tools.URDF.loadstr(urdf_content, urdf_path)
        self.robot_model = rtb.ERobot(urdf.elinks, name=urdf.name)
        os.chdir('../..')

    def save_states(self, states, filename):
        d = copy.deepcopy(states)
        for key, value in d.items():
            if isinstance(value, np.ndarray):
                d[key] = value.tolist()
        with open(filename, 'w') as f:
            json.dump(d, f)
        print(f"Saved states to {os.path.abspath(filename)}")

    def set_joint_impedance(self, k=[3000, 3000, 3000, 2500, 2500, 2000, 2000]):
        assert len(k) == 7, "Joint impedance must be a list of 7 values"
        k = np.array(k)
        assert np.all(k > 0), "Joint impedance must be positive"
        s = self._request(f"set_joint_impedance {self._array2string(k)}")
        if s != "OK":
            raise Exception(f"Error setting joint impedance: {s}")
        print(f"Joint impedance set to {k}")

    def set_collision_behavior(self,
                               lower_torque_thresholds=[
                                   100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
                               upper_torque_thresholds=[
                                   100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
                               lower_force_thresholds=[
                                   100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
                               upper_force_thresholds=[100.0, 100.0, 100.0, 100.0, 100.0, 100.0]):
        assert len(
            lower_torque_thresholds) == 7, "Lower torque thresholds must be a list of 7 values"
        assert len(
            upper_torque_thresholds) == 7, "Upper torque thresholds must be a list of 7 values"
        assert len(
            lower_force_thresholds) == 6, "Lower force thresholds must be a list of 6 values"
        assert len(
            upper_force_thresholds) == 6, "Upper force thresholds must be a list of 6 values"
        lower_torque_thresholds = np.array(lower_torque_thresholds)
        upper_torque_thresholds = np.array(upper_torque_thresholds)
        lower_force_thresholds = np.array(lower_force_thresholds)
        upper_force_thresholds = np.array(upper_force_thresholds)
        assert np.all(lower_torque_thresholds >
                      0), "Lower torque thresholds must be positive"
        assert np.all(upper_torque_thresholds >
                      0), "Upper torque thresholds must be positive"
        assert np.all(lower_force_thresholds >
                      0), "Lower force thresholds must be positive"
        assert np.all(upper_force_thresholds >
                      0), "Upper force thresholds must be positive"
        s = self._request(
            f"set_collision_behavior {self._array2string(lower_torque_thresholds)} {self._array2string(upper_torque_thresholds)} {self._array2string(lower_force_thresholds)} {self._array2string(upper_force_thresholds)}")
        if s != "OK":
            raise Exception(f"Error setting collision behavior: {s}")
        print(
            f"Collision behavior set to {lower_torque_thresholds} {upper_torque_thresholds} {lower_force_thresholds} {upper_force_thresholds}")

    def retrieve(self, filename):
        print(f"Retrieving log file {filename}")
        s = self._request(f"retrieve {filename}")
        if not s.startswith("OK"):
            raise Exception(f"Error retrieving: {s}")
        d = json.loads(s[len("OK "):])
        for key, value in d.items():
            if isinstance(value, list):
                d[key] = np.array(value)
        return munch.munchify(d)

    def record(self, num_steps=500, dt_in_ms=20):
        s = self._request(f"record {num_steps} {dt_in_ms}")
        if not s.startswith("OK"):
            raise Exception(f"Error recording: {s}")
        filename = s[len("OK "):]
        states = self.retrieve(filename)
        for key, value in states.items():
            if isinstance(value, list):
                states[key] = np.array(value[0])
        return states

    def get_state(self):
        data = self.record(1)
        for key, value in data.items():
            if isinstance(value, list):
                data[key] = value[0]
        return munch.munchify(data)

    def move_to_q(self, q, speed=0.5, prompt_confirmation=True, record_states=False, record_frequency=5):
        assert len(q) == 7, "q must be a 7-element array"
        assert speed >= 0.0 and speed <= 1.0, "speed factor must be between 0 and 1"
        s = self._request(
            f"move_to_q {self._array2string(q)} {speed:.60g} {int(prompt_confirmation)} {int(record_states)} {record_frequency}")
        if not s.startswith("OK"):
            raise Exception(f"Error moving to q: {s}")
        if record_states:
            return self.retrieve(s[len("OK "):])

    def interpolate(self, q_waypoints, source_dt=0.01, target_dt=0.002):
        assert all(len(
            q) == 7 for q in q_waypoints), "q_waypoints must be a list of 7-element arrays"
        wps = " ".join(map(lambda q: self._array2string(q), q_waypoints))
        result = self._request_nested_array(
            f"interpolate {len(q_waypoints)} {wps} {source_dt:.60g} {target_dt:.60g}")
        return result.T

    def follow_qs(self, q_waypoints, source_dt=2.0, record_frequency=5):
        assert all(len(
            q) == 7 for q in q_waypoints), "q_waypoints must be a list of 7-element arrays"
        wps = " ".join(map(lambda q: self._array2string(q), q_waypoints))
        s = self._request(
            f"follow_qs {len(q_waypoints)} {wps} {source_dt:.60g} {record_frequency}")
        if s.startswith("FAIL"):
            split = s.split()
            filename = split[1]
            error_msg = " ".join(split[2:])
            print(f'{bcolors.FAIL}Error following qs: {error_msg}{bcolors.ENDC}')
            return self.retrieve(filename)
        elif not s.startswith("OK"):
            raise Exception(f"Error following qs: {s}")
        return self.retrieve(s[len("OK "):])

    def follow_cartesian_vel(self, cart_vel_waypoints, source_dt=0.1, record_frequency=5):
        assert all(len(
            q) == 6 for q in cart_vel_waypoints), "cart_vel_waypoints must be a list of 6-element arrays"
        wps = " ".join(
            map(lambda q: self._array2string(q), cart_vel_waypoints))
        s = self._request(
            f"follow_cartesian_vel {len(cart_vel_waypoints)} {wps} {source_dt:.60g} {record_frequency}")
        if s.startswith("FAIL"):
            split = s.split()
            filename = split[1]
            error_msg = " ".join(split[2:])
            print(
                f'{bcolors.FAIL}Error following Cartesian vel: {error_msg}{bcolors.ENDC}')
            return self.retrieve(filename)
        elif not s.startswith("OK"):
            raise Exception(f"Error following Cartesian vel: {s}")
        return self.retrieve(s[len("OK "):])

    def _setup_robot_model(self, q=None, knife=None):
        import roboticstoolbox as rtb

        if q is None:
            state = self.get_state()
            q = state["q"]
        if knife is None:
            knife = self.get_knife()

        os.chdir('assets/panda_robot')
        urdf_path = f'panda_arm_{knife}.urdf'
        urdf_content = open(urdf_path, 'r').read()
        urdf = rtb.tools.URDF.loadstr(urdf_content, urdf_path)
        robot = rtb.ERobot(urdf.elinks, name=urdf.name)
        robot.ee_links = [robot.ee_links[0]]
        robot.q = q
        os.chdir('../..')
        return robot

    def _setup_visualization(self, q=None, knife=None):
        from roboticstoolbox.backends.Swift import Swift
        robot = self._setup_robot_model(q=q, knife=knife)
        backend = Swift()
        backend.launch()
        backend.add(robot)
        return backend, robot

    def preview_q(self, q=None, knife=None, hold=False, wait=7):
        backend, robot = self._setup_visualization(q, knife)
        if wait > 0:
            time.sleep(wait)
        if hold:
            backend.hold()

    def preview_qs(self, qs, source_dt=0.01, target_dt=0.002, knife=None, hold=False, wait=7, delay=5.0):
        iqs = self.interpolate(qs, source_dt, target_dt)
        backend, robot = self._setup_visualization(qs[0], knife)
        if delay > 0:
            time.sleep(delay)
        for q in iqs:
            robot.q = q      # update the robot state
            backend.step()   # update visualization
            time.sleep(target_dt)
        if wait > 0:
            time.sleep(wait)
        if hold:
            backend.hold()

    def preview_move_to_q(self, q, knife=None, hold=False, wait=7, delay=5.0):
        q0 = self.get_state().q
        qs = self.interpolate([q0, q0, q, q], 0.01, 0.002)
        self.preview_qs(qs, knife=knife, hold=hold, wait=wait, delay=delay)

    def plot_qs(self, qs, times=None, show_limits=True):
        if times is None:
            times = np.arange(len(qs))
        else:
            times = np.array(times) - times[0]
        qs = np.array(qs)
        ncols = 4
        nrows = 2
        fig, axes = plt.subplots(
            ncols=ncols,
            nrows=nrows,
            constrained_layout=True,
            figsize=(ncols * 3.5, nrows * 3.5),
            squeeze=False,
            sharex=True,
        )
        if len(qs) < 20:
            marker = '.'
        else:
            marker = None
        for dim in range(ncols * nrows):
            ax = axes[dim // ncols, dim % ncols]
            if dim >= 7:
                ax.axis("off")
                continue
            ax.plot(times, qs[:, dim], marker=marker, color="C0", linewidth=2)
            ax.grid()
            ax.set_title("$\\mathbf{q}[%i]$" % dim)
            if show_limits and self.robot_model is not None and self.robot_model.qlim is not None:
                ax.axhline(
                    self.robot_model.qlim[0, dim], linestyle='--', color="k")
                ax.axhline(
                    self.robot_model.qlim[1, dim], linestyle='--', color="k")
        plt.show()

    def relative_xyz_to_qs(self, rel_xyz_waypoints, initial_q=None, consider_joint_limits=False, max_iter=100):
        assert all(len(
            q) == 3 for q in rel_xyz_waypoints), "rel_xyz_waypoints must be a list of 3-element arrays"

        from spatialmath import SE3
        robot = self._setup_robot_model(q=initial_q)
        ee_tf = robot.fkine(robot.q)
        qs = [robot.q]
        for xyz in tqdm(rel_xyz_waypoints):
            ee_target = SE3(*xyz) @ ee_tf
            if consider_joint_limits:
                sol = robot.ikine_min(
                    ee_target, q0=qs[-1], qlim=True, ilimit=max_iter)
            else:
                sol = robot.ikine_LM(ee_target, q0=qs[-1], ilimit=max_iter)
            qs.append(sol.q)
        qs = np.array(qs[1:])
        return qs
