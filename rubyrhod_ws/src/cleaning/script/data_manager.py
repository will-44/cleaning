#!/usr/bin/env python3

import glob
import json
import pickle

import open3d as o3d
import rospy
import numpy as np


class DataManager:

    def __init__(self):
        rospy.loginfo('{nodeName} : DataMenager init'.format(nodeName = rospy.get_name()))

    def save_pcd_list(self, pcd_list, file_name_prefix):
        """
        Save a list of N pcd to N file of pcd
        :param pcd_list: the list of pcd
        :param file_name_prefix: the path and name files
        :return: write N files
        """
        for i, pcd in enumerate(pcd_list):
            file_name = f"{file_name_prefix}_{str(i).zfill(3)}.pcd"
            # rospy.loginfo(file_name)
            o3d.io.write_point_cloud(file_name, pcd)

    def load_pcd_list(self, file_name_prefix):
        """
        Load  N pcd files to a list of N pcd
        :param file_name_prefix: the path and name files
        :return: a list of N pcd
        """
        pcd_list = []
        file_names = glob.glob(f"{file_name_prefix}_*.pcd")
        nb_pcd = len(file_names)
        for index in range(nb_pcd):
            file_name = f"{file_name_prefix}_{str(index).zfill(3)}.pcd"
            pcd = o3d.io.read_point_cloud(file_name)
            pcd_list.append(pcd)
        return pcd_list

    def save_var_pickle(self, var2save, path):
        """
        Save a "native" variable with the lib pickle to the path.
        :param var2save: The variable to save
        :param path: the path and name of the file
        :return: none
        """
        with open(path, 'wb') as f:
            pickle.dump(var2save, f)

    def load_var_pickle(self, path):
        with open(path, 'rb') as f:
            res = pickle.load(f)
        return res

    def open_json(self, path):
        """
        Open a json file and return the data
        :param path: the path of the file
        :return: the data
        """
        with open(path) as json_file:
            data = json.load(json_file)
        return data

    def convert_np_arrays(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return obj

    def save_dict_to_json(self, input_dict, json_file_path):
        # Convert tuple keys to strings and handle NumPy arrays
        converted_dict = {str(key): self.convert_np_arrays(value) for key, value in input_dict.items()}

        # Save the dictionary to a JSON file
        with open(json_file_path, 'w') as json_file:
            json.dump(converted_dict, json_file, indent=2)

    def load_json_to_dict(self, json_file_path):
        with open(json_file_path, 'r') as json_file:
            loaded_dict = json.load(json_file)

        # Convert keys back to tuples and handle NumPy arrays
        converted_dict = {eval(key): self.convert_np_arrays(value) for key, value in loaded_dict.items()}

        return converted_dict
