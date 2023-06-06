# sycobot_mouvement
Ce package fait l'interface entre l'utilisateur et le déplacement des robots

Prérequis:
- Installer ROS noetic
- Télécharger le git rubyrhod_ws dans le dossier src avec ses submodules (https://www.vogella.com/tutorials/GitSubmodules/article.html#cloning-a-repository-that-contains-submodules)
- Supprimer les packages robotiq concernant gripper 3f
- Compiler le code avec catkin_make
- Le robot MiR doit etre allumer et coneceté au wifi sycobotmachine
- Le doosan doit etre connecter au reseau d'une maniere fillaire
- Le raspberry Pi doit etre connecter a sycomachine avec les variable d'environement connecté au roscore maitre: http://wiki.ros.org/ROS/NetworkSetup
- Se connecter au wifi sycobotmachine
- Ne pas executer les tests dans une machine virtuelle (wsl, docker...), Doosan ne l'accepte pas

Procédure de test:
- Executer: **roslaunch mir_driver mir.launch mir_hostname:=192.168.100.209**
- Executer: **rosrun behaviors mir_test**

Validation: Tout les test doivent etre vert.

Test de grasp a portée du bras (V1):  
- Sur le raspberry Pi:  
  - **rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0**
- Sur le PC:  
  - **roslaunch dsr_control dsr_moveit.launch mode:=real host:=192.168.100.50**
  - **ROS_NAMESPACE=/dsr01m1013 rosrun utility grasp.py robot_description:=/dsr01m1013/robot_description**
  - **rosrun behaviors grasp_test.py**
 
Test des grasps lointain complexe:

- Sur le raspberry Pi:
  - **rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0**
- Sur le PC:
  - **roslaunch dsr_control dsr_moveit.launch mode:=real host:=192.168.100.50**
  - **roslaunch mir_driver mir.launch mir_hostname:=192.168.100.209**
  - **roslaunch behaviors tf.launch**
  - **ROS_NAMESPACE=/dsr01m1013 rosrun utility grasp_complexe.py robot_description:=/dsr01m1013/robot_description**
  - **rosrun behaviors grasp_complexe_test.py**

Test AR Tags:
- Sur le pc leeloo: 
  - **rostest behaviors main.test**
- Visualiser le resultat:
  - **rosrun rviz rviz**
  - modifier "Fixed Frame": camera_link
  - ajouter "/camera/color/image_raw" 
  - ajouter "visualization_marker"

Test des grasp v3 avec moveit:
Sur le raspberry Pi:
- **rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0**
  Sur le PC:
- **roslaunch dsr_control dsr_moveit.launch mode:=real host:=192.168.100.50**
- **roslaunch mir_driver mir.launch mir_hostname:=192.168.100.209**
- **roslaunch behaviors tf.launch**
- **ROS_NAMESPACE=/dsr01m1013 rosrun utility grasp_v3.py robot_description:=/dsr01m1013/robot_description**
- **rosrun behaviors grasp_v3_test.py**

Test HBBA filtrage topic
- roslaunch behaviors simulation.launch
- roslaunch dsr_control dsr_moveit.launch
- rviz
- roslaunch behaviors basic_hbba_test.launch
- rostopic pub /scenario_1 std_msgs/Bool "data: true"
- rostopic pub /perception_output std_msgs/Bool "data: true"

Test Niveau de batterie
on the leelooo run :
- roslaunch mir_driver mir.launch mir_hostname:=192.168.100.209

then run the node:
- rosrun perceptions proc_battery_level.py

- rostopic echo /proc_battery_level

Test arbitration topic
- roslaunch arbitration_test.launch
- rostopic echo test
Test Docking
- rosrun behaviors bhvr_docking.py
- rostopic pub /proc_battery_level std_msgs/Float64 "data: 0.0"
