# segway_ros2_ws

Digital twin et stack ROS2 d'un Segway miniature à deux roues auto-équilibré.

## Architecture du workspace

```
segway_ws/
├── src/
│   ├── segway_description/   # URDF, launch, rviz
│   ├── segway_control/       # Contrôleur d'équilibre (PID/LQR)
│   └── segway_bringup/       # Launch fichiers système complets
├── build/
├── install/
└── log/
```

---

## Matériel

### Châssis

| Paramètre | Valeur |
|---|---|
| Profondeur (x) | 70 mm |
| Largeur inter-roues (y) | 210 mm |
| Hauteur châssis | 155 mm |
| Largeur totale avec roues | 264 mm |
| Hauteur totale sol | 187.5 mm |
| Plateaux | 3 mm alu, 5 niveaux |
| Colonnes | Ø6 mm, longueur 139 mm |

### Roues

| Paramètre | Valeur |
|---|---|
| Diamètre | 65 mm |
| Largeur | 27 mm |

### Motoréducteurs — KIMISS DC 12V 130 RPM (×2)

| Paramètre | Valeur |
|---|---|
| Tension nominale | 12V |
| Vitesse moteur à vide | ~1000 RPM |
| Rapport de réduction | ~1:70 |
| Vitesse roue à vide | 130 RPM |
| Vitesse max linéaire | ~0.44 m/s |
| Encodeur | Hall effect, 11 CPR |
| Résolution roue (quadrature ×4) | ~3080 counts/tour |
| Arbre | 4 mm |

### Électronique

| Composant | Modèle | Dimensions | Masse |
|---|---|---|---|
| Calculateur haut niveau | Radxa Zero 3W | 65×37 mm | ~15g |
| Microcontrôleur | STM32 Nucleo-64 | 70×53 mm | ~20g |
| IMU | MPU-6050 (GY-521) | 20×16 mm | ~3g |
| Pont en H | L298N ou TB6612 | 43×43 mm | ~15g |
| Batterie | LiPo 3S — 160×48×26 mm | 160×48×26 mm | ~180g |
| Convertisseurs | 2× step-down (5V RPi, 3.3V STM32) | 22×17 mm | ~20g |

### Manette

DualShock 4 (PS4) via Bluetooth 5.4 (intégré Radxa Zero 3W)

---

## Stack logicielle

| Couche | Techno |
|---|---|
| OS Radxa | Ubuntu 22.04 Server (headless) |
| ROS2 | Humble Hawksbill |
| Simulation | Gazebo Fortress |
| Visualisation | RViz2 (PC développement) |

### Architecture ROS2

```
DS4 (BT)
   │
   ▼
[joy_node] → /joy
   │
   ▼
[teleop_twist_joy] → /cmd_vel
   │
   ▼
[balance_controller] ←── /imu/data (MPU-6050 @ 100Hz)
   │
   ▼
[diff_drive] → /wheel_cmd
   │
   ▼
STM32 Nucleo (UART) → Pont en H → Moteurs
                    ← Encodeurs
```

---

## Étages du châssis

```
z = +155 mm  ┌─────────────────┐  IMU MPU-6050
             │                 │
z = +121 mm  ├─────────────────┤  Radxa Zero 3W + STM32 Nucleo-64
             │                 │
z = +081 mm  ├─────────────────┤  2× convertisseurs tension
             │                 │
z = +061 mm  ├─────────────────┤  LiPo 3S (160×48×26 mm)
             │                 │
z = +035 mm  ├─────────────────┤  Pont en H
             │                 │
z = +015 mm  ├─────────────────┤  1er plateau
             │                 │
z =    0 mm  │  ←── axe roues ─┤  Motoréducteurs (centrés sur z=0)
             │                 │
z = -015 mm  └─────────────────┘
```

---

## Installation

```bash
# Dépendances
sudo apt install ros-humble-xacro \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-joy \
                 ros-humble-teleop-twist-joy

# Build
cd ~/segway_ws
colcon build
source install/setup.bash
```

### Visualiser le robot dans RViz2

```bash
ros2 launch segway_description display.launch.py
```

### Vérifier le URDF

```bash
ros2 run xacro xacro src/segway_description/urdf/segway.urdf.xacro
```

---

## Manette DS4 — mapping

| Contrôle | Action |
|---|---|
| L2 maintenu | Dead man's switch (sécurité) |
| Stick gauche Y | Vitesse avance / recul |
| Stick droit X | Rotation |
| R2 | Boost ×2 |
| Croix | Reset IMU |
| Options | Emergency stop |

### Pairing Bluetooth DS4

```bash
bluetoothctl
power on
agent on
scan on
# Maintenir Share + PS jusqu'au clignotement rapide
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
```

---

## Statut

- [x] URDF / digital twin
- [ ] Package ROS2 (launch, CMakeLists)
- [ ] Contrôleur d'équilibre
- [ ] Interface STM32 (UART)
- [ ] Tests simulation Gazebo
- [ ] Build physique
---

## Commandes

# Terminal 1 : Gazebo
ros2 launch segway_gazebo gazebo.launch.py

# Terminal 2 : filtre IMU
ros2 launch segway_control imu_filter.launch.py

# Terminal 3 : PID
ros2 launch segway_control balance_controller.launch.py


# Terminal 4 : contoller
ros2 run controller_manager spawner joint_state_broadcaster \
  -c /segway/controller_manager

ros2 run controller_manager spawner wheel_effort_controller \
  -c /segway/controller_manager

---

## Auteur

alain-31 — [github.com/alain-31/segway_ros2_ws](https://github.com/alain-31/segway_ros2_ws)

