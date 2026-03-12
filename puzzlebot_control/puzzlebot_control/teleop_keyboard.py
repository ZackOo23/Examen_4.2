#!/usr/bin/env python3
"""
Keyboard Teleop for PuzzleBot
──────────────────────────────
Arrow keys / WASD to drive. Press 1-5 to switch controller.
Press G to send goal for autonomous mode.

FIX: Cuando se presiona G se activa autonomous_mode=True y el teleop
     deja de publicar /cmd_vel para no pisar los comandos del controlador.
     Cualquier tecla de movimiento (WASD/flechas/espacio) cancela el modo
     autónomo y regresa el control manual.
"""
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
import json


USAGE = """
╔══════════════════════════════════════════════════════════════╗
║            PuzzleBot Keyboard Teleop                        ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  Movement (nonholonomic — no lateral sliding):               ║
║                                                              ║
║         ↑ / W         Forward                                ║
║    ← / A     → / D    Turn Left / Right                      ║
║         ↓ / S         Backward                               ║
║                                                              ║
║  Commands:                                                   ║
║    SPACE   Emergency stop  (cancela modo autónomo)           ║
║    G       Send goal (2.0, 1.5) → autonomous control         ║
║    1       → PID controller                                  ║
║    2       → SMC controller                                  ║
║    3       → ISMC controller                                 ║
║    4       → CTC controller                                  ║
║    5       → Port-Hamiltonian controller                     ║
║    P       Toggle perturbations                              ║
║    R       Reset simulation                                  ║
║    Q       Quit                                              ║
║                                                              ║
║  NOTA: En modo autónomo el teleop NO publica /cmd_vel.       ║
║        Cualquier tecla de movimiento cancela el modo.        ║
╚══════════════════════════════════════════════════════════════╝
"""

KEY_MAP = {
    '\x1b[A': 'up', '\x1b[B': 'down', '\x1b[C': 'right', '\x1b[D': 'left',
    'w': 'up', 's': 'down', 'a': 'left', 'd': 'right',
    'W': 'up', 'S': 'down', 'A': 'left', 'D': 'right',
    ' ': 'stop', 'q': 'quit', 'Q': 'quit',
    'g': 'goal', 'G': 'goal',
    'p': 'perturb', 'P': 'perturb',
    'r': 'reset', 'R': 'reset',
    '1': 'pid', '2': 'smc', '3': 'ismc', '4': 'ctc', '5': 'ph',
}

CTRL_NAMES = {
    'pid': 'PID', 'smc': 'SMC', 'ismc': 'ISMC',
    'ctc': 'CTC', 'ph': 'Port-Hamiltonian',
}

# Teclas que implican control manual → cancelan modo autónomo
MANUAL_ACTIONS = {'up', 'down', 'left', 'right', 'stop'}

LINEAR_STEP  = 0.05
ANGULAR_STEP = 0.15
MAX_V = 0.5
MAX_W = 3.0


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.cmd_pub     = self.create_publisher(Twist,       '/cmd_vel',           10)
        self.goal_pub    = self.create_publisher(PoseStamped, '/goal_pose',          10)
        self.switch_pub  = self.create_publisher(String,      '/switch_controller',  10)
        self.terrain_pub = self.create_publisher(String,      '/terrain_config',     10)
        self.reset_pub   = self.create_publisher(String,      '/sim_reset',          10)
        self.gz_reset    = self.create_client(Empty,          '/reset_simulation')

        self.v = 0.0
        self.w = 0.0
        self.perturb_on = True

        # FIX: flag de modo autónomo
        self.autonomous_mode = False

    def _safe_pub(self, pub, msg):
        try:
            if rclpy.ok():
                pub.publish(msg)
        except Exception:
            pass

    def _cancel_autonomous(self):
        """Cancela modo autónomo y regresa control manual."""
        if self.autonomous_mode:
            self.autonomous_mode = False
            print('\r[MANUAL] Modo autónomo cancelado — control manual activo.    ')

    def run(self):
        print(USAGE)
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    ch = sys.stdin.read(1)
                    if ch == '\x1b':
                        ch += sys.stdin.read(2)
                    action = KEY_MAP.get(ch)
                else:
                    action = None
                    # Decaimiento solo en modo manual
                    if not self.autonomous_mode:
                        self.v *= 0.92
                        self.w *= 0.88

                # FIX: si la acción es manual, cancelar modo autónomo primero
                if action in MANUAL_ACTIONS:
                    self._cancel_autonomous()

                if action == 'quit':
                    break

                elif action == 'up':
                    self.v = min(self.v + LINEAR_STEP, MAX_V)

                elif action == 'down':
                    self.v = max(self.v - LINEAR_STEP, -MAX_V)

                elif action == 'left':
                    self.w = min(self.w + ANGULAR_STEP, MAX_W)

                elif action == 'right':
                    self.w = max(self.w - ANGULAR_STEP, -MAX_W)

                elif action == 'stop':
                    self.v = 0.0
                    self.w = 0.0
                    # Manda stop explícito aunque estemos en manual
                    self._safe_pub(self.cmd_pub, Twist())

                elif action == 'goal':
                    # FIX: detener robot, mandar goal y activar modo autónomo
                    self.v = 0.0
                    self.w = 0.0
                    self._safe_pub(self.cmd_pub, Twist())   # stop explícito

                    msg = PoseStamped()
                    msg.header.stamp    = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'odom'
                    msg.pose.position.x = 2.0
                    msg.pose.position.y = 1.5
                    msg.pose.orientation.w = 1.0
                    self._safe_pub(self.goal_pub, msg)

                    self.autonomous_mode = True
                    print('\r[GOAL] → (2.0, 1.5) enviado. Modo AUTÓNOMO activo.     ')
                    print('\r[INFO] El teleop no publicará /cmd_vel hasta que muevas el robot.')

                elif action == 'reset':
                    self._cancel_autonomous()
                    self._safe_pub(self.reset_pub, String(data='{}'))
                    if self.gz_reset.service_is_ready():
                        self.gz_reset.call_async(Empty.Request())
                    self.v = 0.0
                    self.w = 0.0
                    self._safe_pub(self.cmd_pub, Twist())
                    print('\r[RESET] Gazebo + controllers reset.                      ')

                elif action == 'perturb':
                    self.perturb_on = not self.perturb_on
                    cfg = json.dumps({'enabled': self.perturb_on})
                    self._safe_pub(self.terrain_pub, String(data=cfg))
                    st = 'ON' if self.perturb_on else 'OFF'
                    print(f'\r[TERRAIN] Perturbations: {st}                           ')

                elif action in CTRL_NAMES:
                    name = CTRL_NAMES[action]
                    self._safe_pub(self.switch_pub, String(data=name))
                    print(f'\r[SWITCH] Controller → {name}                            ')

                # FIX: solo publica /cmd_vel si NO estamos en modo autónomo
                if not self.autonomous_mode:
                    cmd = Twist()
                    cmd.linear.x  = float(self.v)
                    cmd.angular.z = float(self.w)
                    self._safe_pub(self.cmd_pub, cmd)

                # Status line
                mode_str = 'AUTO' if self.autonomous_mode else 'MANUAL'
                print(
                    f'\r  [{mode_str}]  v={self.v:+.3f} m/s  ω={self.w:+.3f} rad/s  ',
                    end='', flush=True
                )

                try:
                    rclpy.spin_once(self, timeout_sec=0)
                except Exception:
                    break

        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self._safe_pub(self.cmd_pub, Twist())
            print('\n[STOP] Teleop ended.')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.run()
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
