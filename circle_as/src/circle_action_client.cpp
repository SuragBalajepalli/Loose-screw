#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <circle_action/circular_pathAction.h>
#include <arm7dof_fk_ik/arm7dof_kinematics.h>
#include <joint_space_planner/joint_space_planner.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h> 
//#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/PCLPointCloud2.h>
#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs
#include <circle_action/perceptionAction.h>

std::vector<vector<Eigen::VectorXd> > g_path_solns;
geometry_msgs::Pose g_recieved_pose;
Arm7dof_IK_solver arm7dof_ik_solver;
//JointSpacePlanner joint_space_planner;
bool g_pose_recieved = false; 
bool g_recieved_patch = false;
double g_surface_z;

void perception_doneCallBack(const actionlib::SimpleClientGoalState& state, const circle_action::perceptionResultConstPtr& result) {
	ROS_INFO("perception complete; Pose recieved");
	g_recieved_pose=result->nut_pose;
	g_pose_recieved = true;
}

void path_doneCallBack(const actionlib::SimpleClientGoalState& state, const circle_action::circular_pathResultConstPtr& result) {
	ROS_INFO("Path recieved");
	for(int i=0;i<=result->path.poses.size();i++)
	{
	geometry_msgs::PoseStamped recieved_pose;
	recieved_pose=result->path.poses[i];
	std::vector<Vectorq7x1> soln;
	Eigen::Affine3d aff;
	int no_of_solns;
	//generate stuff from path
	aff = Eigen::Translation3d(recieved_pose.pose.position.x,recieved_pose.pose.position.y,recieved_pose.pose.position.z) *
       Eigen::Quaterniond(recieved_pose.pose.orientation.w,recieved_pose.pose.orientation.x,recieved_pose.pose.orientation.y,recieved_pose.pose.orientation.z);
 	
 	no_of_solns= arm7dof_ik_solver.ik_solns_sampled_qs0(aff,soln);// what do i do with this?// can i call the ik function here? so many questions// why didnt i learn c++ properly
 	//g_path_solns.push_back(soln);
 	//joint_space_planner.compute_optimal_path(g_path_solns);//how to use "get" method to get member var optimal_path?
 	//why did i Make the path global?
 	//can this client connect to another server? why not?
 	//send this stuff to robot motion node

    
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclSelectedPoints_ptr;
void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    ROS_INFO("RECEIVED NEW PATCH");

    pcl::fromROSMsg(*cloud, *g_pclSelectedPoints_ptr);
    ROS_INFO("patch dimensions: %d * %d points", g_pclSelectedPoints_ptr->width, g_pclSelectedPoints_ptr->height);
    

    //ROS_INFO("frame_id = %s",pclSelectedPoints_ptr_->header.frame_id);
    //std::cout<<"frame_id ="<<g_pclSelectedPoints_ptr->header.frame_id<<endl;
    ROS_INFO_STREAM("frame_id = "<<g_pclSelectedPoints_ptr->header.frame_id<<endl);
    
    Eigen::MatrixXf points_mat;
    Eigen::Vector3f cloud_pt;
    //populate points_mat from cloud data;

    int npts = g_pclSelectedPoints_ptr->points.size();
    points_mat.resize(3, npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        cloud_pt = g_pclSelectedPoints_ptr->points[i].getVector3fMap();
        points_mat.col(i) = cloud_pt;
    }   

    Eigen::Vector3f centroid = Eigen::MatrixXf::Zero(3, 1); // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
    // compute the centroid of the selected points
     for (int ipt = 0; ipt < npts; ipt++) {
        centroid += points_mat.col(ipt); //add all the column vectors together
    }
    centroid /= npts; //divide by the number of points to get the centroid   
    ROS_INFO("centroid of selected points is: (%f, %f, %f)",centroid(0),centroid(1),centroid(2));
         /**/
    g_recieved_patch=true;
    g_surface_z=centroid(2);
}



int main (int argc, char** argv) {
	ros::init(argc, argv, "circle_action_client_node");
	circle_action::circular_pathGoal path_goal;
	circle_action::perceptionGoal perception_goal;
	Eigen::Vector4f plane_parameters;
	actionlib::SimpleActionClient<circle_action::circular_pathAction> path_ac("whats his name", true);
	actionlib::SimpleActionClient<circle_action::perceptionAction> perception_ac("whats his name2", true);
    ros::NodeHandle nh;
	g_path_solns.clear();

	 ros::Subscriber selected_points_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/selected_points", 1, selectCB);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclSelectedPoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    g_pclSelectedPoints_ptr = pclSelectedPoints_ptr;
    
    //loop to test for new selected-points inputs and compute and display corresponding planar fits 
    while (!g_recieved_patch) {
    	ROS_INFO(" select a patch of points to find the selected-points centroid...");
        ros::spinOnce(); //callback does all the work
        ros::Duration(0.1).sleep();
    }


	perception_goal.surface_z=g_surface_z;
	//first do perception, get pose, put in global from done function, then send as goal to path
	perception_ac.sendGoal(perception_goal, perception_doneCallBack);
	bool server_exists = path_ac.waitForServer(ros::Duration(5.0));
	while(!server_exists) {
		ROS_INFO("waiting for server");
	}
	ROS_INFO("connection success");
	//stuff to find pose of  nut
	while(!g_pose_recieved) {
		ros::Duration(0.1).sleep();
	}
	path_ac.sendGoal(path_goal, &path_doneCallBack);
	bool finished_before_timeout = path_ac.waitForResult(ros::Duration(5.0));
	while(!finished_before_timeout) {
		ROS_INFO("waiting");
	}

}
