# segway_robot — ROS2 workspace

Robot auto-équilibré à deux roues (segway).

## Matériel

| Composant    | Détail            |
|--------------|-------------------|
| CPU haut     | Raspberry Pi      |
| CPU bas      | STM32             |
| Motorisation | Moto-réducteurs   |
| Driver       | Pont en H         |
| Batterie     | LiPo 3S (~11.1 V) |
| IMU          | MPU-6050          |
| Roues        | ø68 × 27 mm       |

## Dimensions (phase conception)

| Paramètre          | Valeur   |
|--------------------|----------|
| Profondeur (x)     | 50 mm    |
| Entraxe roues      | 180 mm   |
| Hauteur châssis    | 150 mm   |
| Hauteur sol→sommet | ≈ 184 mm |
| Étages             | 6 × 25 mm|

## Structure des packages

```
src/
├── segway_description/   # URDF, Xacro, RViz  ← ce package
├── segway_control/       # ros2_control, diff_drive      (à venir)
├── segway_bringup/       # launch de haut niveau         (à venir)
├── segway_gazebo/        # simulation Gazebo             (à venir)
├── segway_balance/       # contrôleur LQR/PID            (à venir)
└── segway_firmware/      # interface micro-ROS STM32     (à venir)
```

## Build & visualisation

```bash
# 1. Dépendances
rosdep install --from-paths src --ignore-src -r -y

# 2. Build
colcon build --symlink-install
source install/setup.bash

# 3. Visualisation RViz
ros2 launch segway_description display.launch.py
```

## Roadmap

- [x] URDF / Xacro multi-étages (segway_description)
- [ ] ros2_control + diff_drive (segway_control)
- [ ] Simulation Gazebo (segway_gazebo)
- [ ] Contrôleur d'équilibre LQR (segway_balance)
- [ ] Interface micro-ROS STM32 (segway_firmware)
