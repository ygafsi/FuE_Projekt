# FuE Projekt - Progression actuelle

## 1. Construction du Docker

Depuis le dossier du projet :

```bash
cd ~/fue_projekt
./start_docker_original.sh
```

Si problème de droits Docker :

```bash
sudo ./start_docker_original.sh
```




cd /home/robot/ros2_ws
colcon build
source install/setup.bash
ros2 run screwdriver_driver kl_tcg_driver




























---

## 2. Vérification ROS2

Dans le container :

```bash
cd /home/robot/ros2_ws

colcon build

source install/setup.bash

ros2 topic list
```

Résultat obtenu :

```text
/parameter_events
/rosout
```

---

## 3. Création du package ROS2

```bash
cd /home/robot/ros2_ws/src

ros2 pkg create screwdriver_driver \
  --build-type ament_python \
  --dependencies rclpy std_msgs
```

---

## 4. Compilation du package

```bash
cd /home/robot/ros2_ws

colcon build

source install/setup.bash
```

---

## 5. Création du premier node

Fichier créé :

```text
ros2_ws/src/screwdriver_driver/screwdriver_driver/kl_tcg_driver.py
```

Node de test :

* publie sur `/screwdriver/status`
* envoie le message `driver alive`

---

## 6. Modification de setup.py

Ajout :

```python
'kl_tcg_driver = screwdriver_driver.kl_tcg_driver:main'
```

---

## 7. Lancement du node

```bash
ros2 run screwdriver_driver kl_tcg_driver
```

Résultat obtenu :

```text
[INFO] [....] [kl_tcg_driver]: KL-TCG driver node started
```

---

## 8. Vérification du topic

Dans un deuxième terminal :

```bash
sudo docker exec -it ros2_bachelor bash

source /home/robot/ros2_ws/install/setup.bash

ros2 topic echo /screwdriver/status
```

Résultat :

```text
data: driver alive
```

---

## 9. Installation de pyserial

```bash
sudo apt update

sudo apt install -y python3-serial
```

Ajout également dans le Dockerfile :

```dockerfile
python3-pip \
python3-serial && \
```

---

## État actuel

Le driver ROS2 fonctionne.

Prochaine étape :

Connexion RS232 avec le contrôleur KL-TCG et lecture des premières trames.
