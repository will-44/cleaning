#!/usr/bin/env python3

import rospy
from doosan import Doosan

# This file is not use anymore

# old position
# pos = [[-88.10, -30.50, -78.36, -6.87, -120.64, 180.16], # 1
#        [-88.10, 16.74, -97.02, -5.05, -107.5, 180.16],
#        [-87.93, -6.79, -89.64, -4.81, -115.41, 180.16],
#        [-87.02, -28.66, -73.48, -3.12, -122.58, 180.16],
#        [-86.59, -54.56, -58.39, 1.14, -133.17, 180.16],   # 5
#        [-50.3, -0.72, -88.66, -14.61, -118.09, 191.29],
#        [-60.66, -17.62, -81.19, -7.57, -115.02, 191.28],
#        [-65.64, -41.76, -66.55, -9.96, -129.51, 191.47],
#        [-62.90, -54.89, -62.10, -10.38, -133.19, 191.54],
#        [-132.60, -14.65, -71.52, 19.61, -122.88, 159.90],   #10
#        [-121.23, -37.49, -63.67, -0.09, -123.98, 159.92],
#        [-111.89, -60.24, -55.49, 1.32, -131.38, 156.30],
#        [-107.95, -66.93, -57.76, 4.23, -133.28, 142.55],
#        [-82.62, -37.20, -60.59, -5.53, -123.84, 120.69],
#        [-74.78, -59.20, -45.47, -12.34, -133.24, 120.66],   # 15
#        [-82.41, -14.97, -79.56, -1.11, -113.99, 120.25],
#        [-82.40, 3.54, -85.26, -1.11, -114.01, 120.25],
#        [-82.40, 20.82, -85.29, 0.24, -120.97, 120.25],
#        [-43.06, -4.80, -81.55, -8.83, -123.02, 120.28],
#        [-52.17, -26.88, -72.14, -11.66, -119.21, 120.28],
#        [-59.27, -43.92, -65.13, -5.66, -131.73, 120.28]]

# Pos joint list for calibration with zivid and matrox
pos = [[-88.24, -24.68, -85.76, -2.6, -112.79, -178.05],
       [-89.58, -48.09, -80.54, -2.49, -119.67, -178.03],
       [-86.46, 11.37, -117.63, -2.77, -84.43, -176.09],
       [-88.23, -22.33, -96.61, -2.61, -104.4, -177.19],
       [-125.54, -31.94, -89.73, 17.37, -112.44, -197.34],
       [-119.08, -55.71, -66.7, 13.04, -131.42, -197.76],
       [-130.24, -10.82, -115.12, 18.58, -90.44, -203.07],
       [-38.02, -2.68, -122.63, -11.77, -86.44, -130.16],
       [-50.15, -20.32, -107.69, -6.59, -95.35, -147.58],
       [-62.49, -48.11, -85.57, -1.3, -116.09, -152.76]]

if __name__ == '__main__':
    #  This file generates yaml and txt files to run the calibration with Zivid sdk and Matrox

    rospy.init_node('calibration_node', anonymous=True)
    arm = Doosan()
    nb_pos = 0
    # Generate for all position the rotation and translation matrix in mm and deg for matrox
    for pt in pos:
        nb_pos = nb_pos + 1
        print("new pose" + str(nb_pos))
        pt = [coord * (3.1415 / 180) for coord in pt]
        arm.go_to_j(pt)
        rot_mat = arm.get_current_rotm()
        posx = arm.get_pos_x()
        posx = arm.MRadToMMDeg(posx)
        trans_mat = [
            [rot_mat.rot_matrix[0].data[0], rot_mat.rot_matrix[0].data[1], rot_mat.rot_matrix[0].data[2], posx[0]],
            [rot_mat.rot_matrix[1].data[0], rot_mat.rot_matrix[1].data[1], rot_mat.rot_matrix[1].data[2], posx[1]],
            [rot_mat.rot_matrix[2].data[0], rot_mat.rot_matrix[2].data[1], rot_mat.rot_matrix[2].data[2], posx[2]],
            [0, 0, 0, 1]]

        print(trans_mat[0])
        print(trans_mat[1])
        print(trans_mat[2])
        print(trans_mat[3])

        l1 = "%YAML:1.0\n"
        l2 = "---\n"
        l3 = "PoseState: !!opencv-matrix\n"
        l4 = "    rows: 4\n"
        l5 = "    cols: 4\n"
        l6 = "    dt: d\n"
        l7 = "    data:\n"
        l8 = "        [\n"
        l9 = "            "
        l10 = "        ]\n"
        if (nb_pos < 10):
            path_transfo = r'/home/sycobot/Document/calibration/poses/pos0' + str(nb_pos) + '.yaml'
        else:
            path_transfo = r'/home/sycobot/Document/calibration/poses/pos' + str(nb_pos) + '.yaml'
        with open(path_transfo, 'w') as file:
            file.write(l1)
            file.write(l2)
            file.write(l3)
            file.write(l4)
            file.write(l5)
            file.write(l6)
            file.write(l7)
            file.write(l8)
            file.write(l9 + str(trans_mat[0][0]) + ",\n")
            file.write(l9 + str(trans_mat[0][1]) + ",\n")
            file.write(l9 + str(trans_mat[0][2]) + ",\n")
            file.write(l9 + str(trans_mat[0][3]) + ",\n")
            file.write(l9 + str(trans_mat[1][0]) + ",\n")
            file.write(l9 + str(trans_mat[1][1]) + ",\n")
            file.write(l9 + str(trans_mat[1][2]) + ",\n")
            file.write(l9 + str(trans_mat[1][3]) + ",\n")
            file.write(l9 + str(trans_mat[2][0]) + ",\n")
            file.write(l9 + str(trans_mat[2][1]) + ",\n")
            file.write(l9 + str(trans_mat[2][2]) + ",\n")
            file.write(l9 + str(trans_mat[2][3]) + ",\n")
            file.write(l9 + str(trans_mat[3][0]) + ",\n")
            file.write(l9 + str(trans_mat[3][1]) + ",\n")
            file.write(l9 + str(trans_mat[3][2]) + ",\n")
            file.write(l9 + str(trans_mat[3][3]) + "\n")
            file.write(l10)
        if (nb_pos < 10):
            path_joint = r'/home/sycobot/Document/calibration/posx/pos0' + str(nb_pos) + '.txt'
        else:
            path_joint = r'/home/sycobot/Document/calibration/posx/pos' + str(nb_pos) + '.txt'
        with open(path_joint, 'w') as file:
            file.write(str(posx))

        input("coucou ! Press enter !")
