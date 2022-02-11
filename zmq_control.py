import zmq
import numpy as np
import json
import matplotlib.pyplot as plt
import time
import os
import munch


class RobotConnector:
    q0 = np.array([0.0, -np.pi/4, 0.0, -3 * np.pi/4, 0.0, np.pi/2, np.pi/4])

    q_slicing_above_cutting_board = np.array([
        -0.15426344996377042, 0.17191124297868524,
        0.02710628974630387, -2.805887122037118,
        -0.06784833805872718, 3.0057289532061104,
        0.7289365255381608])

    q_slicing_above_cutting_board_sideways_left = np.array([
        1.0488755670665233, -1.5550153837063028,
        -1.6894963620169121, -2.0950458705592574,
        -1.6710747315496837, 1.686909673478868,
        0.8274693135027815])

    # cutting board to the left of the robot on top of a box
    q_slicing_above_cutting_board_sideways_left_higher = np.array([
        0.9387033627179631, -1.4958561291778296,
        -1.496686039880605, -2.239123061628293,
        -1.5187597279879779, 1.5900584277995513,
        0.8654174339953385])

    q_slicing_above_cutting_board_sideways_right = np.array([
        2.0526782641599604, 1.3710241951691478,
        -1.3712464959964414, -2.126852119732135,
        1.4426313187180542, 1.8592795748180808,
        0.7966741719692945
    ])

    def __init__(self, ip='172.16.0.1', port: int = 5555):
        self.context = zmq.Context()
        self.ip = ip
        self.port = port
        self.socket = None
        self.connect()

    def connect(self, robot_address="172.16.0.2"):
        if self.socket is None or self.socket.closed:
            self.socket = self.context.socket(zmq.REQ)
            self.socket.connect(f"tcp://{self.ip}:{self.port}")
            s = self.request(f"connect {robot_address}")
            if s != "OK":
                raise Exception(f"Error connecting: {s}")
            print(f"Connected to robot at {self.ip}:{self.port}.")

    def request(self, what: str) -> str:
        self.connect()
        self.socket.send_string(what)
        message = self.socket.recv()
        return message.decode("UTF-8")

    def request_array(self, what: str):
        s = self.request(what)
        if not s.startswith("OK"):
            raise Exception(
                f"Error requesting array via {what.split()[0]}: {s}")
        s = s[len("OK "):]
        return np.fromstring(s.replace("[", "").replace("]", ""), sep=" ")

    def request_nested_array(self, what: str):
        ps = self.request(what)
        if not ps.startswith("OK"):
            raise Exception(
                f"Error requesting nested array via {what.split()[0]}: {ps}")
        ps = ps[len("OK "):]
        ps = np.array([np.fromstring(
            s.replace("[", "").replace("]", ""), sep=" ") for s in ps.split("] [")])
        return ps

    @staticmethod
    def array2string(x):
        return " ".join(map(lambda d: format(d, '.60g'), x))

    def error_recovery(self):
        s = self.request("error_recovery")
        if not s.startswith("OK"):
            raise Exception(f"Error during automatic error recovery: {s}")

    def set_knife(self, knife="slicing"):
        s = self.request(f"set_knife {knife}")
        if s != "OK":
            raise Exception(f"Error setting knife: {s}")

    def get_knife(self):
        s = self.request("get_knife")
        if not s.startswith("OK"):
            raise Exception(f"Error getting knife: {s}")
        return s[len("OK "):]

    def retrieve(self, filename):
        print(f"Retrieving log file {filename}")
        s = self.request(f"retrieve {filename}")
        if not s.startswith("OK"):
            raise Exception(f"Error retrieving: {s}")
        return json.loads(s[len("OK "):])

    def record(self, num_steps=500, dt_in_ms=20):
        s = self.request(f"record {num_steps} {dt_in_ms}")
        if not s.startswith("OK"):
            raise Exception(f"Error recording: {s}")
        filename = s[len("OK "):]
        states = self.retrieve(filename)
        for key, value in states.items():
            if isinstance(value, list):
                states[key] = np.array(value[0])
        return munch.munchify(states)

    def get_state(self):
        data = self.record(1)
        for key, value in data.items():
            if isinstance(value, list):
                data[key] = value[0]
        return munch.munchify(data)

    def move_to_q(self, q, speed=0.5, prompt_confirmation=True, record_states=False, record_frequency=5):
        assert len(q) == 7, "q must be a 7-element array"
        assert speed >= 0.0 and speed <= 1.0, "speed factor must be between 0 and 1"
        s = self.request(
            f"move_to_q {self.array2string(q)} {speed:.60g} {int(prompt_confirmation)} {int(record_states)} {record_frequency}")
        if not s.startswith("OK"):
            raise Exception(f"Error moving to q: {s}")
        if record_states:
            return self.retrieve(s[len("OK "):])

    def interpolate(self, q_waypoints, source_dt=0.01, target_dt=0.002):
        assert all(len(
            q) == 7 for q in q_waypoints), "q_waypoints must be a list of 7-element arrays"
        wps = " ".join(map(lambda q: self.array2string(q), q_waypoints))
        result = self.request_nested_array(
            f"interpolate {len(q_waypoints)} {wps} {source_dt:.60g} {target_dt:.60g}")
        return result.T

    def follow_qs(self, q_waypoints, source_dt=2.0, record_frequency=5):
        assert all(len(
            q) == 7 for q in q_waypoints), "q_waypoints must be a list of 7-element arrays"
        wps = " ".join(map(lambda q: self.array2string(q), q_waypoints))
        s = self.request(
            f"follow_qs {len(q_waypoints)} {wps} {source_dt:.60g} {record_frequency}")
        if not s.startswith("OK"):
            raise Exception(f"Error following qs: {s}")
        return self.retrieve(s[len("OK "):])

    def follow_cartesian_vel(self, cart_vel_waypoints, source_dt=0.1, record_frequency=5):
        assert all(len(
            q) == 6 for q in cart_vel_waypoints), "cart_vel_waypoints must be a list of 6-element arrays"
        wps = " ".join(map(lambda q: self.array2string(q), cart_vel_waypoints))
        s = self.request(
            f"follow_cartesian_vel {len(cart_vel_waypoints)} {wps} {source_dt:.60g} {record_frequency}")
        if not s.startswith("OK"):
            raise Exception(f"Error following Cartesian vel: {s}")
        return self.retrieve(s[len("OK "):])

    def _setup_visualization(self, q=None, knife=None):
        import roboticstoolbox as rtb
        from roboticstoolbox.backends.Swift import Swift

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
        robot.q = q
        os.chdir('../..')

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
            robot.q = q          # update the robot state
            backend.step()       # update visualization
            time.sleep(target_dt)
        if wait > 0:
            time.sleep(wait)
        if hold:
            backend.hold()

    def preview_move_to_q(self, q, knife=None, hold=False, wait=7, delay=5.0):
        q0 = self.get_state().q
        qs = self.interpolate([q0, q0, q, q], 0.01, 0.002)
        self.preview_qs(qs, knife=knife, hold=hold, wait=wait, delay=delay)

    def plot_qs(self, qs, times=None):
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
        for dim in range(ncols * nrows):
            ax = axes[dim // ncols, dim % ncols]
            if dim >= 7:
                ax.axis("off")
                continue
            ax.plot(times, qs[:, dim])
            ax.grid()
            ax.set_title("$\\mathbf{q}[%i]$" % dim)
        plt.show()


def main():
    robot = RobotConnector()
    robot.error_recovery()
    robot.set_knife("slicing")

    state = robot.get_state()
    print(state.q)

    # # robot.preview_q(q=RobotConnector.q0)
    # # states = robot.move_to_q(RobotConnector.q0, record_states=True)
    # # robot.plot_qs(states["q"], states["time"])
    # robot.move_to_q(RobotConnector.q0, record_states=False)
    # waypoints = [
    #     RobotConnector.q0,
    #     RobotConnector.q0 + np.array([0, 0, 0, 0.2, 0, 0, 0.1]),
    #     RobotConnector.q0 + np.array([-0.3, 0, 0, 0.2, 0, 0.3, -0.1]),
    # ]
    # # robot.plot_qs(robot.interpolate(waypoints, source_dt=1.0, target_dt=0.01))
    # robot.preview_qs(waypoints, source_dt=1.0, target_dt=0.01)
    # # states = robot.follow_qs(waypoints, source_dt=3.0)
    # # robot.plot_qs(states["q"], states["time"])

    # robot.preview_move_to_q(robot.q_slicing_above_cutting_board_sideways_left_higher)

    robot.move_to_q(robot.q_slicing_above_cutting_board_sideways_left_higher,
                    speed=0.1, record_states=False)

    # slicing_amplitude = 0.05
    # downward_velocity = 0.01
    # waypoints = [
    #     (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    #     (-slicing_amplitude, 0.0, -downward_velocity, 0.0, 0.0, 0.0),
    #     (slicing_amplitude, 0.0,-downward_velocity, 0.0, 0.0, 0.0),
    #     (-slicing_amplitude, 0.0, -downward_velocity, 0.0, 0.0, 0.0),
    #     (slicing_amplitude, 0.0,-downward_velocity, 0.0, 0.0, 0.0),
    #     (-slicing_amplitude, 0.0, -downward_velocity, 0.0, 0.0, 0.0),
    #     (slicing_amplitude, 0.0,-downward_velocity, 0.0, 0.0, 0.0),
    #     (-slicing_amplitude, 0.0, -downward_velocity, 0.0, 0.0, 0.0),
    #     (slicing_amplitude, 0.0,-downward_velocity, 0.0, 0.0, 0.0),
    #     (-slicing_amplitude, 0.0, -downward_velocity, 0.0, 0.0, 0.0),
    #     (slicing_amplitude, 0.0,-downward_velocity, 0.0, 0.0, 0.0),
    #     (-slicing_amplitude, 0.0, -downward_velocity, 0.0, 0.0, 0.0),
    #     (slicing_amplitude, 0.0,-downward_velocity, 0.0, 0.0, 0.0),
    #     (-slicing_amplitude, 0.0, -downward_velocity, 0.0, 0.0, 0.0),
    #     (slicing_amplitude, 0.0,-downward_velocity, 0.0, 0.0, 0.0),
    #     (-slicing_amplitude, 0.0, -downward_velocity, 0.0, 0.0, 0.0),
    #     (slicing_amplitude, 0.0,-downward_velocity, 0.0, 0.0, 0.0),
    #     (-slicing_amplitude, 0.0, -downward_velocity, 0.0, 0.0, 0.0),
    #     (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    #     (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    # ]
    # states = robot.follow_cartesian_vel(waypoints, source_dt=0.5)
    # robot.plot_qs(states["q"], states["time"])


    # print(states)
    # print(robot.get_state()["q"])
    # recording = robot.record()
    # plt.title("q")
    # times = np.array(recording["time"])
    # times -= times[0]
    # plt.plot(times, recording["q"])
    # plt.show()
if __name__ == "__main__":
    main()
