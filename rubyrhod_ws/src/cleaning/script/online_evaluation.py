#!/usr/bin/env python3
import rospkg
from rospy_message_converter import json_message_converter
import json

# Fonction pour convertir un JSON en message ROS
def convert_to_ros_message(json_data, message_type):
    return json_message_converter.convert_json_to_ros_message(message_type, json_data)


if __name__ == '__main__':
    # Chemin vers le fichier contenant les JSON
    file_path = rospkg.RosPack().get_path('cleaning') + '/data/telemetrie/telemetrie_20230718-162155.json'  # Remplacez par le bon chemin

    # Ouvrir le fichier et traiter chaque ligne
    result = {}
    actual_guard = tuple()
    with open(file_path, 'r') as file:
        for line in file:
            # Charger le JSON à partir de la ligne
            json_data = json.loads(line)

            # Vérifier le type de message dans le JSON et convertir en conséquence

            if 'position' in json_data:

                ros_message = convert_to_ros_message(line, 'sensor_msgs/JointState')
                actual_guard = tuple(ros_message.position)
                result[actual_guard] = {[]}
            elif 'pose' in json_data:
                ros_message = convert_to_ros_message(line, 'geometry_msgs/PoseStamped')
                result[actual_guard] = result.get(actual_guard).append(ros_message.pose.position)
            else:
                # Gérer le cas où le type de message n'est pas reconnu
                print("Type de message non reconnu.")
                continue

            # Faites ce que vous voulez avec le message ROS ici, par exemple l'imprimer
            print(ros_message)