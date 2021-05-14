# import UDPComms
import numpy as np
import time
from src.State import BehaviorState, State
from src.Command import Command
from src.Utilities import deadband, clipped_first_order_filter

# from mjoy import MyJoy
from vjoy import VJoy
import threading

MESSAGE_RATE = 20


class JoystickInterface:
    def __init__(
        self, config, udp_port=8830, udp_publisher_port=8840,
    ):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_activate_toggle = 0

        self.message_rate = 50
        # self.udp_handle = UDPComms.Subscriber(udp_port, timeout=0.3)
        # self.udp_publisher = UDPComms.Publisher(udp_publisher_port,"192.168.50.255")

        self.joystick = VJoy(callback=None)

        reader = threading.Thread(target=self.joystick.start_open_loop)
        reader.daemon = True
        reader.start()

    def get_command(self, state, do_print=False):
        try:
            # msg = self.udp_handle.get()
            msg = self.get_joystick()
            command = Command()
            # print(msg)
            ####### Handle discrete commands ########
            # Check if requesting a state transition to trotting, or from trotting to resting
            gait_toggle = msg["R1"]
            command.trot_event = (
                gait_toggle == 1 and self.previous_gait_toggle == 0)

            # Check if requesting a state transition to hopping, from trotting or resting
            hop_toggle = msg["x"]
            command.hop_event = (
                hop_toggle == 1 and self.previous_hop_toggle == 0)

            activate_toggle = msg["L1"]
            command.activate_event = (
                activate_toggle == 1 and self.previous_activate_toggle == 0)

            # Update previous values for toggles and state
            self.previous_gait_toggle = gait_toggle
            self.previous_hop_toggle = hop_toggle
            self.previous_activate_toggle = activate_toggle

            ####### Handle continuous commands ########
            x_vel = msg["ly"] * self.config.max_x_velocity
            y_vel = msg["lx"] * -self.config.max_y_velocity
            command.horizontal_velocity = np.array([x_vel, y_vel])
            command.yaw_rate = msg["rx"] * -self.config.max_yaw_rate

            message_rate = msg["message_rate"]
            message_dt = 1.0 / message_rate

            pitch = msg["ry"] * self.config.max_pitch
            deadbanded_pitch = deadband(
                pitch, self.config.pitch_deadband
            )
            pitch_rate = clipped_first_order_filter(
                state.pitch,
                deadbanded_pitch,
                self.config.max_pitch_rate,
                self.config.pitch_time_constant,
            )
            command.pitch = state.pitch + message_dt * pitch_rate

            height_movement = msg["dpady"]
            command.height = state.height - message_dt * \
                self.config.z_speed * height_movement

            roll_movement = - msg["dpadx"]
            command.roll = state.roll + message_dt * self.config.roll_speed * roll_movement

            return command

        except Exception as e:
            if do_print:
                print(e)
            return Command()

    def get_joystick(self):
        buttons = self.joystick.buttons
        axes = self.joystick.axes
        left_y = -axes[1]/32767.0
        right_y = -axes[4]/32767.0
        right_x = axes[3]/32767.0
        left_x = axes[0]/32767.0

        L2 = axes[2]/32767.0
        R2 = axes[5]/32767.0

        R1 = buttons[5]
        L1 = buttons[4]

        square = buttons[2]
        x = buttons[0]
        circle = buttons[1]
        triangle = buttons[3]

        dpadx = axes[6]/32767.0
        dpady = -axes[7]/32767.0

        msg = {
            "ly": left_y,
            "lx": left_x,
            "rx": right_x,
            "ry": right_y,
            "L2": L2,
            "R2": R2,
            "R1": R1,
            "L1": L1,
            "dpady": dpady,
            "dpadx": dpadx,
            "x": x,
            "square": square,
            "circle": circle,
            "triangle": triangle,
            "message_rate": MESSAGE_RATE,
        }
        return msg

    def set_color(self, color):
        joystick_msg = {"ps4_color": color}
        # self.udp_publisher.send(joystick_msg)
