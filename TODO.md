# TODO — Migration Segway Gazebo vers effort control

## 0. Sauvegarde de la version actuelle

- [ ] Vérifier que la version actuelle stabilise bien le robot avec `diff_drive` et boucle externe désactivée.
- [ ] Committer cette version stable.
- [ ] Ajouter un tag Git, par exemple `stable-balance-diffdrive`.
- [ ] Créer une branche dédiée, par exemple `feature/effort-control`.

## 1. Modification de l'URDF

- [ ] Supprimer ou commenter le plugin `libgazebo_ros_diff_drive.so`.
- [ ] Ajouter le plugin `libgazebo_ros2_control.so`.
- [ ] Ajouter une section `<ros2_control>` dans l'URDF.
- [ ] Déclarer `left_wheel_joint` avec `command_interface="effort"`.
- [ ] Déclarer `right_wheel_joint` avec `command_interface="effort"`.
- [ ] Garder les `state_interface` `position` et `velocity` pour les deux roues.
- [ ] Conserver provisoirement les paramètres de contact roue/sol actuels (`mu=1.0`, `kp=100000`, `kd=10`).

## 2. Configuration ROS 2 Control

- [ ] Créer un fichier `controllers.yaml`.
- [ ] Ajouter `joint_state_broadcaster`.
- [ ] Ajouter un contrôleur d'effort pour les deux roues.
- [ ] Vérifier le nom exact du contrôleur disponible avec `ros2 control list_controller_types`.
- [ ] Configurer les joints `left_wheel_joint` et `right_wheel_joint` dans le contrôleur d'effort.
- [ ] Définir une fréquence de mise à jour raisonnable, par exemple `200 Hz`.

## 3. Modification du launch Gazebo

- [ ] Charger le fichier `controllers.yaml` au démarrage de Gazebo.
- [ ] Lancer le `joint_state_broadcaster` avec un spawner.
- [ ] Lancer le contrôleur d'effort avec un spawner.
- [ ] Vérifier que les contrôleurs passent à l'état `active`.
- [ ] Vérifier que les topics de commande d'effort sont bien créés.

## 4. Test manuel des roues

- [ ] Démarrer Gazebo sans activer le PID d'équilibre.
- [ ] Publier manuellement un effort positif sur les deux roues.
- [ ] Vérifier que les roues tournent dans le bon sens.
- [ ] Publier manuellement un effort négatif sur les deux roues.
- [ ] Vérifier que les roues tournent dans le sens inverse.
- [ ] Tester un effort nul et vérifier que la commande est bien annulée.
- [ ] Noter l'ordre de grandeur des efforts utiles.

## 5. Modification du `balance_controller`

- [ ] Remplacer la publication `geometry_msgs::msg::Twist` sur `/segway/cmd_vel`.
- [ ] Publier une commande d'effort pour les deux roues.
- [ ] Convertir la sortie du PID d'angle en `left_effort` et `right_effort`.
- [ ] Ajouter une saturation explicite sur l'effort moteur.
- [ ] Supprimer provisoirement la compensation de deadband liée à `cmd_vel`.
- [ ] Garder la boucle externe désactivée.

## 6. Test de la boucle interne seule

- [ ] Tester avec `pitch_setpoint = 0`.
- [ ] Vérifier que le robot peut tenir debout quelques secondes.
- [ ] Ajuster uniquement les gains de la boucle interne si nécessaire.
- [ ] Logger `pitch`, `pitch_setpoint`, `effort_left`, `effort_right`, `vx` et `pos_x`.
- [ ] Vérifier que le signe de l'effort est correct.
- [ ] Vérifier que la saturation n'est pas atteinte en permanence.

## 7. Test avec consignes d'inclinaison fixes

- [ ] Tester `pitch_setpoint = +0.01 rad`.
- [ ] Tester `pitch_setpoint = -0.01 rad`.
- [ ] Tester `pitch_setpoint = +0.02 rad` si le comportement est stable.
- [ ] Tester `pitch_setpoint = -0.02 rad` si le comportement est stable.
- [ ] Observer la relation entre `pitch_setpoint` et `vx`.
- [ ] Vérifier que le comportement est reproductible.

## 8. Réactivation de la boucle externe

- [ ] Réactiver la boucle vitesse avec `Ki = 0` et `Kd = 0`.
- [ ] Commencer avec un `Kp` vitesse très faible.
- [ ] Limiter fortement `pitch_setpoint`, par exemple à `±0.02 rad`.
- [ ] Tester d'abord `vx_setpoint = 0.0 m/s`.
- [ ] Vérifier que la boucle externe réduit la dérive sans provoquer d'emballement.
- [ ] Tester ensuite `vx_setpoint = 0.03 m/s`.
- [ ] Tester `vx_setpoint = 0.05 m/s` uniquement si le test précédent est stable.

## 9. Analyse comparative

- [ ] Comparer le comportement avec l'ancienne version `diff_drive`.
- [ ] Vérifier si le bang-bang lié à la deadband a disparu.
- [ ] Vérifier si le réglage de la boucle externe est plus lisible.
- [ ] Décider si l'architecture `effort control` devient la nouvelle base du projet.
- [ ] Si oui, merger la branche après nettoyage.
- [ ] Si non, revenir à la version taggée `stable-balance-diffdrive`.
