#!/usr/bin/env python3
"""
hardware_bridge.py — Puzzlebot ROS2 ↔ ESP32 serial bridge (fixed)

Fixes aplicados:
  FIX 3: threading.Lock protege wl_meas / wr_meas entre el thread serial y el timer.
  FIX 4: Comentario explícito de convención Motor A = izquierda, Motor B = derecha.
          Ajusta swap_motors=True si tu cableado está al revés.
"""

import math
import serial
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from tf_transformations import quaternion_from_euler


class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')

        # ===== Parameters =====
        self.declare_parameter('port',                '/dev/ttyUSB0')
        self.declare_parameter('baudrate',            115200)
        self.declare_parameter('wheel_radius',        0.03)    # m
        self.declare_parameter('wheel_base',          0.18)    # m
        self.declare_parameter('encoder_cpr',         910.0)
        self.declare_parameter('cmd_vel_topic',       '/cmd_vel')
        self.declare_parameter('odom_topic',          '/odom_hw')
        self.declare_parameter('joint_state_topic',   '/joint_states_hw')
        self.declare_parameter('publish_joint_states', True)
        # FIX 4: si Motor A es físicamente la rueda DERECHA, pon swap_motors=True
        self.declare_parameter('swap_motors',         False)

        self.port               = self.get_parameter('port').value
        self.baudrate           = int(self.get_parameter('baudrate').value)
        self.r                  = float(self.get_parameter('wheel_radius').value)
        self.L                  = float(self.get_parameter('wheel_base').value)
        self.cpr                = float(self.get_parameter('encoder_cpr').value)
        self.swap_motors        = bool(self.get_parameter('swap_motors').value)
        cmd_vel_topic           = self.get_parameter('cmd_vel_topic').value
        odom_topic              = self.get_parameter('odom_topic').value
        joint_state_topic       = self.get_parameter('joint_state_topic').value
        self.publish_joint_states = bool(self.get_parameter('publish_joint_states').value)

        # ===== State (protegido por state_lock) =====
        # FIX 3: un solo lock cubre TODAS las variables compartidas entre threads
        self.state_lock = threading.Lock()

        self.wl_meas    = 0.0
        self.wr_meas    = 0.0
        self.left_ticks  = 0
        self.right_ticks = 0
        self.left_pos    = 0.0
        self.right_pos   = 0.0

        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        # ===== Publishers / Subscribers =====
        self.cmd_sub = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_callback, 10
        )
        self.odom_pub  = self.create_publisher(Odometry,          odom_topic,  10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/wheel_debug', 10)

        if self.publish_joint_states:
            self.js_pub = self.create_publisher(JointState, joint_state_topic, 10)
        else:
            self.js_pub = None

        # ===== Serial =====
        self.serial_lock = threading.Lock()
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        time.sleep(2.0)   # Espera reset del ESP32 tras abrir el puerto

        self.reader_thread = threading.Thread(
            target=self.serial_reader_loop, daemon=True
        )
        self.reader_thread.start()

        self.timer = self.create_timer(0.02, self.publish_state)  # 50 Hz

        self.get_logger().info(
            f'Hardware bridge OK  port={self.port}  baud={self.baudrate}'
            f'  swap_motors={self.swap_motors}'
        )

    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Convención:  Motor A = izquierda,  Motor B = derecha
        # wl = (2v - wL) / 2r      wr = (2v + wL) / 2r
        wl_ref = (2.0 * v - w * self.L) / (2.0 * self.r)
        wr_ref = (2.0 * v + w * self.L) / (2.0 * self.r)

        # FIX 4: si los motores están invertidos en el hardware, intercambia aquí
        if self.swap_motors:
            wl_ref, wr_ref = wr_ref, wl_ref

        # CMD,wA_ref,wB_ref   (A = izquierda, B = derecha)
        line = f'CMD,{wl_ref:.4f},{wr_ref:.4f}\n'
        with self.serial_lock:
            try:
                self.ser.write(line.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')

    # ------------------------------------------------------------------
    def serial_reader_loop(self):
        """Thread dedicado a leer MEAS desde la ESP32."""
        while rclpy.ok():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # MEAS,wA,wB,ticksA,ticksB,posA,posB
                parts = line.split(',')
                if parts[0] != 'MEAS' or len(parts) < 7:
                    continue

                wA = float(parts[1])
                wB = float(parts[2])
                tA = int(parts[3])
                tB = int(parts[4])
                pA = float(parts[5])
                pB = float(parts[6])

                # FIX 3: escritura atómica bajo lock
                with self.state_lock:
                    if self.swap_motors:
                        self.wl_meas    = wB
                        self.wr_meas    = wA
                        self.left_ticks  = tB
                        self.right_ticks = tA
                        self.left_pos    = pB
                        self.right_pos   = pA
                    else:
                        self.wl_meas    = wA
                        self.wr_meas    = wB
                        self.left_ticks  = tA
                        self.right_ticks = tB
                        self.left_pos    = pA
                        self.right_pos   = pB

            except (ValueError, IndexError) as e:
                self.get_logger().warn(f'Parse error: {e}  raw={line!r}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().warn(f'Unexpected serial error: {e}')
                time.sleep(0.05)

    # ------------------------------------------------------------------
    def publish_state(self):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        # FIX 3: lectura atómica — snapshot de todas las variables juntas
        with self.state_lock:
            wl = self.wl_meas
            wr = self.wr_meas
            lt = self.left_ticks
            rt = self.right_ticks
            lp = self.left_pos
            rp = self.right_pos

        # Odometría diferencial
        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L

        self.theta += w * dt
        self.x     += v * math.cos(self.theta) * dt
        self.y     += v * math.sin(self.theta) * dt

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.theta)

        # --- Odometry msg ---
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        # --- JointState msg ---
        if self.js_pub is not None:
            js = JointState()
            js.header.stamp = now.to_msg()
            js.name         = ['left_wheel_joint', 'right_wheel_joint']
            js.position     = [lp, rp]
            js.velocity     = [wl, wr]
            self.js_pub.publish(js)

        # --- Debug msg: [wl, wr, v, w] ---
        dbg = Float32MultiArray()
        dbg.data = [float(wl), float(wr), float(v), float(w)]
        self.debug_pub.publish(dbg)


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Parar motores al cerrar
        with node.serial_lock:
            try:
                node.ser.write(b'CMD,0.0,0.0\n')
                time.sleep(0.05)
            except Exception:
                pass
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()