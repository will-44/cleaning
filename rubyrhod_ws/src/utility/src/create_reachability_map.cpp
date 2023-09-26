// The spheres and poses are fused in a single dataset, instead of two datasets for sphere and poses
#include "../includes/mir.h"
#include "../includes/sphere_discretization.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseArray.h"
#include <map>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <sstream>
#include <iostream>
#include "moveit_msgs/GetPositionIK.h"
#include "../includes/progressbar.hpp"
#include <visualization_msgs/Marker.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>

//struct stat st;

typedef std::multimap< const std::vector< double >*, const std::vector< double >* > MultiMapPtr;
typedef std::map< const std::vector< double >*, double > MapVecDoublePtr;
typedef std::multimap< std::vector< double >, std::vector< double > > MultiMap;
typedef std::map< std::vector< double >, double > MapVecDouble;
typedef std::vector<std::vector<double> > VectorOfVectors;
struct stat st;
typedef std::vector<std::pair< std::vector< double >, const std::vector< double >* > > MultiVector;
//typedef std::multimap< const std::vector< double >*, const std::vector< double >* > MultiMap;




static bool isIKSuccess( ros::NodeHandle n, ros::Publisher pub, const std::vector<double> &pose, std::vector<double> &joints, int& numOfSolns)
{

    ros::ServiceClient client = n.serviceClient<moveit_msgs::GetPositionIK>("/dsr01m1013/compute_ik");
    moveit_msgs::GetPositionIK srv;
    srv.request.ik_request.group_name = "arm";
    srv.request.ik_request.ik_link_name = "link6";
    std::vector<std::string> joint_name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    srv.request.ik_request.robot_state.joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    std::vector<double> joint_position = {0, 0, 0, 0, 0, 0};
    srv.request.ik_request.robot_state.joint_state.position = {0, 0, 0, 0, 0, 0};
    srv.request.ik_request.pose_stamped.pose.position.x = pose[0];
    srv.request.ik_request.pose_stamped.pose.position.y = pose[1];
    srv.request.ik_request.pose_stamped.pose.position.z = pose[2];
    srv.request.ik_request.pose_stamped.pose.orientation.x = pose[3];
    srv.request.ik_request.pose_stamped.pose.orientation.y = pose[4];
    srv.request.ik_request.pose_stamped.pose.orientation.z = pose[5];
    srv.request.ik_request.pose_stamped.pose.orientation.w = pose[6];
    srv.request.ik_request.avoid_collisions = false;
    srv.request.ik_request.timeout.nsec = 1000;
//    ROS_INFO("position: %f, %f, %f, %f, %f, %f, %f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]);


    ros::Time starsrv = ros::Time::now();

    if (client.call(srv) && srv.response.error_code.val == 1)
    {

        joints = srv.response.solution.joint_state.position;
        double dif = ros::Duration( ros::Time::now() - starsrv).toNSec();
//        ROS_INFO("Elasped time is %.2lf NanoSeconds.", dif);
        return srv.response.error_code.val;

    }
    else
    {
//        visualization_msgs::Marker points;
//        points.type = visualization_msgs::Marker::ARROW;
//        points.header.frame_id = "map";
//        points.pose.position.x = pose[0];
//        points.pose.position.y = pose[1];
//        points.pose.position.z = pose[2];
//        points.pose.orientation.x = pose[3];
//        points.pose.orientation.y = pose[4];
//        points.pose.orientation.z = pose[5];
//        points.pose.orientation.w = pose[6];
//        points.scale.x = 0.5;
//        points.scale.y = 0.05;
//        points.scale.z = 0.05;
//        points.color.a = 1.0; // Don't forget to set the alpha!
//        points.color.r = 0.0;
//        points.color.g = 1.0;
//        points.color.b = 0.0;
//        pub.publish(points);

//        ROS_ERROR("Failed to call service add_two_ints");
        double dif = ros::Duration( ros::Time::now() - starsrv).toNSec();
//        ROS_INFO("Elasped time is %.2lf NanoSeconds.", dif);
        return false;
    }

    return false;

}





bool isFloat(std::string s)
{
    std::istringstream iss(s);
    float dummy;
    iss >> std::noskipws >> dummy;
    return iss && iss.eof();  // Result converted to bool
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "workspace");
    ros::NodeHandle n;
    ros::Time startit = ros::Time::now();
    float resolution = 0.08;  //previous 0.08
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate loop_rate(10);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = cloud.makeShared();

    int count = 0;

    while (ros::ok())
    {
        unsigned char max_depth = 16;
        unsigned char minDepth = 0;

        // A box of radius 1 is created. It will be the size of the robot+1.5. Then the box is discretized by voxels of
        // specified resolution

        // TODO resolution will be user argument
        // The center of every voxels are stored in a vector

        sphere_discretization::SphereDiscretization sd;
        float r = 1;
        octomap::point3d origin = octomap::point3d(0, 0, 0);  // This point will be the base of the robot
        octomap::OcTree *tree = sd.generateBoxTree(origin, r, resolution);
        std::vector< octomap::point3d > new_data;
        ROS_INFO("Creating the box and discretizing with resolution: %f", resolution);
        int sphere_count = 0;
        for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
        {
            sphere_count++;
        }
        new_data.reserve(sphere_count);
        for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
        {
            new_data.push_back(it.getCoordinate());
        }

        ROS_INFO("Total no of spheres now: %lu", new_data.size());
        ROS_INFO("Please hold ON. Spheres are discretized and all of the poses are checked for Ik solutions. May take some "
                 "time");

        // A sphere is created in every voxel. The sphere may be created by default or other techniques.
        // TODO Other techniques need to be modified. the user can specifiy which technique they want to use
        // TODO The sphere discretization parameter and rotation of every poses will be taken as argument. If the final
        // joints can rotate (0, 2pi) we dont need to rotate the poses.
        // Every discretized points on spheres are converted to pose and all the poses are saved in a multimap with their
        // corresponding sphere centers
        // If the resolution is 0.01 the programs not responds
        //TODO seperate raduise and resolutiion
        float radius = resolution/2;

        VectorOfVectors sphere_coord;
        sphere_coord.resize( new_data.size() );

        MultiVector pose_col;
        pose_col.reserve( new_data.size() * 50);

        for (int i = 0; i < new_data.size(); i++)
        {
            static std::vector< geometry_msgs::Pose > pose;
            sd.convertPointToVector(new_data[i], sphere_coord[i]);

            sd.make_sphere_poses(new_data[i], radius, pose);
            for (int j = 0; j < pose.size(); j++)
            {
                static std::vector< double > point_on_sphere;
                sd.convertPoseToVector(pose[j], point_on_sphere);
                pose_col.push_back( std::make_pair(point_on_sphere, &sphere_coord[i]));
            }
        }

        // Every pose is checked for IK solutions. The reachable poses and the their corresponsing joint solutions are
        // stored. Only the First joint solution is stored. We may need this solutions in the future. Otherwise we can show
        // the robot dancing with the joint solutions in a parallel thread
        // TODO Support for more than 6DOF robots needs to be implemented.

        // Kinematics k;

        MultiMapPtr pose_col_filter;
        VectorOfVectors ik_solutions;
        ik_solutions.reserve( pose_col.size() );
        progressbar bar(pose_col.size());
        for (MultiVector::iterator it = pose_col.begin(); it != pose_col.end(); ++it)
        {
            static std::vector< double > joints(6);
            int solns;
//            TODO fct inverse kinematics

            if (isIKSuccess(n, marker_pub, it->first, joints, solns))
            {
//                ROS_INFO("joint = %f", joints[1] );
                pose_col_filter.insert( std::make_pair( it->second, &(it->first)));
                ik_solutions.push_back(joints);
            }
            bar.update();
        }

        ROS_INFO("Total number of poses: %lu", pose_col.size());
        ROS_INFO("Total number of reachable poses: %lu", pose_col_filter.size());

        // The centers of reachable spheres are stored in a map. This data will be utilized in visualizing the spheres in
        // the visualizer.
        // TODO there are several maps are implemented. We can get rid of few maps and run large loops. The complexity of
        // accessing map is Olog(n)

        MapVecDoublePtr sphere_color;
        int nb_valide_pose = 0;

        for (MultiMapPtr::iterator it = pose_col_filter.begin(); it != pose_col_filter.end(); ++it)
        {
            const std::vector<double>* sphere_coord    = it->first;
            //const std::vector<double>* point_on_sphere = it->second;

            // Reachability Index D=R/N*100;
            float d = float(pose_col_filter.count(sphere_coord)) / (pose_col.size() / new_data.size()) * 100;
//            ROS_INFO("Indice D: %f", d);
            if(d >= 5.0){
                sphere_color.insert(std::make_pair(it->first, double(d)));
            }

        }
        // create the reachability map
//        cloud.width = sphere_color.size();
//        cloud.height = 1;
        cloud.is_dense = false;
        cloud.resize (cloud.width * cloud.height);

        for (auto & it : sphere_color)
        {
            float x = it.first[0][0];
            float y = it.first[0][1];
            float z = it.first[0][2];
            cloud.push_back(pcl::PointXYZ(x, y, z));
        }

        ROS_INFO("No of spheres reachable: %lu", sphere_color.size());

//        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//        viewer.showCloud(cloud_ptr);
        pcl::io::savePCDFileASCII ("/home/will/cleaning/test_pcd.pcd", cloud);
//        while (!viewer.wasStopped ())
//        {
//
//        }



        // Creating maps now
//TODO Saving map to dataset
//        hdf5_dataset::Hdf5Dataset h5(filename);
//        h5.saveReachMapsToDataset(pose_col_filter, sphere_color, resolution);

        double dif = ros::Duration( ros::Time::now() - startit).toSec();
        ROS_INFO("Elasped time is %.2lf seconds.", dif);
        ROS_INFO("Completed");
        ros::spinOnce();
        // sleep(10000);
        return 1;
        loop_rate.sleep();
        count;
    }
    return 0;
}