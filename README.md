## Puzzlebot Simulation and Real Implementation Control With ROS2
Control de robot diferencial (Puzzlebot) con cinco estrategias de control, análisis de estabilidad de Lyapunov, retratos de fase y dashboard en tiempo real.

Video demostrativo:
https://youtu.be/B1MJl_FetrQ?si=gFaJjcNQtSFqrsaN

---

**Materia:** TE3001B - Fundamentacion de Robótica

**Profesor:** Nezih Nieto Gutiérrez

**Institución:** Tecnológico de Monterrey

**Equipo:** 2 - REPO

**Fecha:** 12 de Marzo 2026

---

# Controladores implementados
| # | Controlador | Descripción |
|---|------------|----------------|
| 1 | PID | Control proporcional-integral-derivativo clásico sobre error de distancia y ángulo |
| 2 | SMC | Control por modos deslizantes, robusto ante perturbaciones, con superficie de deslizamiento s=0 |
| 3 | ISMC | SMC integral — inicia directamente sobre la superficie, elimina la fase de alcance |
| 4 | CTC | Control por par calculado, linealiza la dinámica y aplica control lineal |
| 5 | Port-Hamiltonian | Control basado en energía que moldea la función Hamiltoniana para convergencia al equilibrio |

---

# Resultados — Simulación en Gazebo
Las pruebas se realizaron con setpoint en (2.0 m, 1.5 m) desde el origen, usando el simulador Gazebo con el plugin diff_drive.
Dashboard en tiempo real (localhost:8080)

El dashboard muestra en vivo:
<img width="1484" height="620" alt="image" src="https://github.com/user-attachments/assets/94737eac-f06c-4f84-aa34-fee8330a0a0c" />

---

Lyapunov V(t) — función de energía, debe decrecer monotónicamente a 0

Trayectoria XY — camino recorrido vs. setpoint

Error de distancia ‖e‖(t) y error de ángulo θ_e(t)

Velocidades v(t), ω(t) comandadas

Superficies deslizantes s_v, s_w (relevantes para SMC e ISMC)

Retratos de fase en espacio de distancia, ángulo y velocidad

---

# Observaciones por controlador
PID:

El controlador más simple. Converge suavemente al setpoint con oscilación mínima. La función de Lyapunov desciende de forma consistente. Sensible al tuning de ganancias pero predecible en comportamiento.

SMC:

Presenta una fase de alcance inicial visible en el retrato de fase antes de llegar a la superficie s=0, seguida de deslizamiento directo al origen. Más robusto que PID ante perturbaciones de terreno.

ISMC:

Variante del SMC que inicia directamente sobre la superficie deslizante gracias al término integral, eliminando la fase de alcance. Convergencia más directa que SMC clásico.

CTC:

Linealiza la dinámica del robot mediante cancelación de no linealidades. Comportamiento similar a un controlador lineal una vez linealizado. Requiere modelo preciso del robot.

Port-Hamiltonian:

Enfoque basado en energía. La función Hamiltoniana actúa como Lyapunov natural del sistema. Convergencia suave con geometría de atractor bien definida en el espacio de velocidades.

# Comparativa de desempeño
| Controlador | Convergencia | Robustez ante perturbaciones | Complejidad |
|-------------|------------|-----------------------|-----------|
| PID | Media | Baja | Baja |
| SMC | Rápida | Alta | Media |
| ISMC | Rápida | Alta | Media |
| CTC | Media | Media | Alta |
| Port-Hamiltonian | Suave | Media | Alta |

---

Conclusión: 

Para el caso de navegación a un setpoint en condiciones nominales, el PID ofrece el mejor balance entre simplicidad y desempeño. Para escenarios con perturbaciones de terreno, SMC e ISMC son preferibles por su robustez inherente. El controlador Port-Hamiltonian destaca en fundamento teórico y garantías formales de estabilidad.

---

# Requisitos

```bash
bashsudo apt install -y \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-tf-transformations \
  ros-${ROS_DISTRO}-rviz2
```

```bash
pip3 install tf-transformations numpy matplotlib
```

# Instalación

```bash
bashmkdir -p ~/pb_ws/src
cp -r puzzlebot_control ~/pb_ws/src/
cd ~/pb_ws
colcon build --packages-select puzzlebot_control
source install/setup.bash
```

# Terminal 1 - Gazebo

```bash
ros2 launch puzzlebot_control gazebo.launch.py
```

# Terminal 2 — hardware_bridge.py

```bash
ros2 run puzzlebot_control hardware_bridge --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200 -p wheel_radius:=0.03 -p wheel_base:=0.18
```

# Terminal 3 - Teleop_keyboard
```bash
ros2 run puzzlebot_control teleop_keyboard
```
En teleop_keyboard.py, se presiona 1–5 para seleccionar controlador, luego G para enviar el goal a (2.0, 1.5). Todos funcionales

# Terminal 3 — Dashboard

```bash
xdg-open http://localhost:8080
```
Muy lento. Es necesario reiniciar cada que se trabe

# Trayectorias personalizadas

```bash
bashros2 topic pub /trajectory nav_msgs/msg/Path "{
  header: {frame_id: 'odom'},
  poses: [
    {pose: {position: {x: 1.0, y: 0.0}}},
    {pose: {position: {x: 2.0, y: 1.0}}},
    {pose: {position: {x: 0.0, y: 2.0}}}
  ]
}" --once
```
También se pueden dibujar waypoints directamente en el dashboard haciendo clic en la gráfica XY.
---

# Referencia
T. Ferguson, A. Donaire, C. Renton, R. H. Middleton,
"A port-Hamiltonian approach to the control of nonholonomic systems,"
arXiv:1801.06954v1, 2018.

# Autores

Manuel Ferro Sánchez

Zacbe Ortega Obregón

Fabricio Banda Hernández

Alexandro Kurt Cárdenas Pérez

Fernando Proal Sifuentes
