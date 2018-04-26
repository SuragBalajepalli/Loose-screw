#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm7dof_fk_ik/arm7dof_kinematics.h>
#include <joint_space_planner/joint_space_planner.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h> 
//#include <pcl/ros/conversions.h>
#include <nav_msgs/Path.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/PCLPointCloud2.h>
#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs
#include <screw_loose/perceptionAction.h>
#include <actionlib/client/terminal_state.h>
#include <arm7dof_traj_as/arm7dof_traj_as.h>
#include<arm7dof_traj_as/trajAction.h>
#include <xform_utils/xform_utils.h>




std::vector<vector<Eigen::VectorXd> > path_options;
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclSelectedPoints_ptr;
geometry_msgs::Pose g_recieved_pose;
Arm7dof_IK_solver arm7dof_ik_solver;
//JointSpacePlanner joint_space_planner;
bool g_pose_recieved = false; 
bool g_recieved_patch = false;
XformUtils xformUtils;
double g_surface_z;


geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double psi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}

double convertPlanarQuaternion2Psi(geometry_msgs::Quaternion quaternion) {
	double psi;
	psi=atan2(quaternion.z,quaternion.w);
	return psi;
}


void make_circular_path_around_provided_pose (geometry_msgs::Pose center_pose, nav_msgs::Path &circular_path) {
	Eigen::Affine3d tf_wrench, pose_center, pose_wrench;
	geometry_msgs::Pose wrench_pose;
	geometry_msgs::PoseStamped desired_pose;
	circular_path.poses.clear();
	circular_path.header.seq=0;
    circular_path.header.stamp=ros::Time::now();
    circular_path.header.frame_id== "world";
    double amp = 1.0;
    double phase = convertPlanarQuaternion2Psi(center_pose.orientation);
    double omega = 1.0;
    double dt = 0.1;
    double x, y, z;
    z = 0;
    double ang_min = 0;
    double ang_max=6.28;
    double ang_i=ang_min;
    int i=0;

    //nut pose to affine, multiply with wrench transform, back to pose, check xform utils
	//but for debug:
	wrench_pose.position.x=0;
	wrench_pose.position.y=0;
	wrench_pose.position.z=0;
	wrench_pose.orientation.x=0;
	wrench_pose.orientation.y=0;
	wrench_pose.orientation.z=0;
	wrench_pose.orientation.w=1;
	//
	//first pose above nut
	desired_pose.header.seq=i;
	desired_pose.header.stamp=ros::Time::now();
    desired_pose.header.frame_id="world";
    desired_pose.pose.position.x=wrench_pose.position.x;
    desired_pose.pose.position.y=wrench_pose.position.y;
    desired_pose.pose.position.z=wrench_pose.position.z+0.2;//start with 20cm above surface
    desired_pose.pose.orientation=convertPlanarPsi2Quaternion(-phase);
	circular_path.poses.push_back(desired_pose);
	i++;
	//second pose to insert wrench, maybe need to refine?
	desired_pose.header.seq=i;
	desired_pose.header.stamp=ros::Time::now();
    desired_pose.header.frame_id="world";
    desired_pose.pose.position.x=wrench_pose.position.x;
    desired_pose.pose.position.y=wrench_pose.position.y;
    desired_pose.pose.position.z=wrench_pose.position.z;//start with 20cm above surface
    desired_pose.pose.orientation=convertPlanarPsi2Quaternion(-phase);
	circular_path.poses.push_back(desired_pose);
	i++;
	while(phase<ang_max) {
    		if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
        	phase += omega*dt;
        	x = amp *sin(phase);
        	y = amp *cos(phase);
        	desired_pose.header.seq=i;
    		desired_pose.header.stamp=ros::Time::now();
    		desired_pose.header.frame_id="world";
    		desired_pose.pose.position.x=center_pose.position.x+x;
    		desired_pose.pose.position.y=center_pose.position.y+y;
    		desired_pose.pose.orientation=convertPlanarPsi2Quaternion(-phase); 
         	circular_path.poses.push_back(desired_pose);
        	i++;
    		
        }

    desired_pose.header.seq=i;
	desired_pose.header.stamp=ros::Time::now();
    desired_pose.header.frame_id="world";
    desired_pose.pose.position.x=desired_pose.pose.position.x;
    desired_pose.pose.position.y=desired_pose.pose.position.y;
    desired_pose.pose.position.z=desired_pose.pose.position.z+0.2;//start with 20cm above surface
    desired_pose.pose.orientation=convertPlanarPsi2Quaternion(-phase);
	circular_path.poses.push_back(desired_pose);
	i++;

	ROS_INFO("Debug: Path contains %d poses", i);

}

bool find_optimal_path_from_path_message (nav_msgs::Path path, std::vector<Eigen::VectorXd> &optimal_path) {
	Eigen::VectorXd weights;
	path_options.clear();
	optimal_path.clear();
	int no_of_poses= path.poses.size();
	ROS_INFO("Debug, got %d poses", no_of_poses );
	weights.resize(7);
    for (int j = 0; j < 7; j++) {
       	weights(j) = 1.0;
    }

	for(int i=0;i<no_of_poses;i++) {
		Eigen::VectorXd node;
		std::vector<Eigen::VectorXd> single_layer_nodes;
		geometry_msgs::PoseStamped recieved_pose;
		recieved_pose=path.poses[i];
		std::vector<Vectorq7x1> soln;
		Eigen::Affine3d aff;
		int no_of_solns;
		ROS_INFO("Checking for %d pose",i);
	//aff = Eigen::Translation3d(recieved_pose.pose.position.x,recieved_pose.pose.position.y,recieved_pose.pose.position.z) *
      // Eigen::Quaterniond(recieved_pose.pose.orientation.w,recieved_pose.pose.orientation.x,recieved_pose.pose.orientation.y,recieved_pose.pose.orientation.z); //replace with func from xfor utils
 	
 		aff = xformUtils.transformPoseToEigenAffine3d(recieved_pose);
 		no_of_solns= arm7dof_ik_solver.ik_solns_sampled_qs0(aff,soln);
 		
 		ROS_INFO("no of solns %d on %d pose", no_of_solns, i);

 		if (no_of_solns>0) {
 		single_layer_nodes.clear();
 		single_layer_nodes.resize(no_of_solns);
 		ROS_INFO("Debug: resize done");
        	for (int isoln = 0; isoln < no_of_solns; isoln++) {
    			ROS_INFO("Debug: Inside for loop");
    			node = soln[isoln];
            	single_layer_nodes[isoln] = node;
            	}
        ROS_INFO("Debug: Trying to push back");
    	path_options.push_back(single_layer_nodes);
        }
        ROS_INFO("---------------------------------------------------------------------------");
 	}
 	int nlayers= path_options.size();
 	if(!nlayers==0)
 	{
 	optimal_path.resize(nlayers);
 	{
 		ROS_INFO("Debug: out of iterative loop");
 		JointSpacePlanner jsp(path_options, weights);
 		ROS_INFO("Debug: Before getting solution");
 		jsp.get_soln(optimal_path);
 		ROS_INFO("Debug: optimal path contains %d poses", nlayers);
		
	}
	return true;
	}
	else 
		{
			ROS_WARN("No solutions");
			return false;
	}
}


void armDoneCb(const actionlib::SimpleClientGoalState& state,
        const arm7dof_traj_as::trajResultConstPtr& result) {
    ROS_INFO("armDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
}

void perception_doneCallBack(const actionlib::SimpleClientGoalState& state, const screw_loose::perceptionResultConstPtr& result) {
	ROS_INFO("perception complete; Pose recieved");
	g_recieved_pose=result->nut_pose;
	g_pose_recieved = true;
}

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
	ros::init(argc, argv, "screw_action_client_node");
	ros::NodeHandle nh;
	std::vector<Eigen::VectorXd> optimal_path;
	nav_msgs::Path g_path;

	optimal_path.clear();
	arm7dof_traj_as::trajGoal arm_goal;
	screw_loose::perceptionGoal perception_goal;
	Eigen::VectorXd q_pre_pose;
    Eigen::VectorXd q_vec;
    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory;
    Arm7dof_traj_streamer arm7dof_traj_streamer(&nh);
	actionlib::SimpleActionClient<arm7dof_traj_as::trajAction> arm_action_client("trajActionServer", true);
	actionlib::SimpleActionClient<screw_loose::perceptionAction> perception_ac("whatshisname2", true);

    q_pre_pose.resize(7);
    q_pre_pose << 0, 1, 0, -2, 0, 1, 0; 
    path_options.clear();
    
    ros::Subscriber selected_points_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/selected_points", 1, selectCB);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclSelectedPoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    g_pclSelectedPoints_ptr = pclSelectedPoints_ptr;
    
    while(ros::ok()) {
    while (!g_recieved_patch) {
    	ROS_INFO(" select a patch of points to find the selected-points centroid...");
        ros::spinOnce(); //callback does all the work
        ros::Duration(0.1).sleep();
    }


    bool server_exists = perception_ac.waitForServer(ros::Duration(5.0));
	while(!server_exists) {
		ROS_INFO("waiting for server");
	}
	ROS_INFO("connection success");

	perception_goal.surface_z=g_surface_z;
	//first do perception, get pose, put in global from done function, then send as goal to path
	perception_ac.sendGoal(perception_goal, perception_doneCallBack);
	

	while (!g_pose_recieved) {
    	ROS_INFO(" waiting for nut pose");
        ros::spinOnce(); //callback does all the work
        ros::Duration(0.1).sleep();
    }

    g_path.poses.clear();//gotta clear these inside functions
	des_trajectory.points.clear();
	optimal_path.clear();
    


	
	make_circular_path_around_provided_pose(g_recieved_pose, g_path);

	bool found_optimal_path=find_optimal_path_from_path_message(g_path, optimal_path);
	optimal_path.push_back(q_pre_pose);
	if(found_optimal_path) {
	q_vec = arm7dof_traj_streamer.get_q_vec_Xd(); 
    cout << "arm current state:" << q_vec.transpose() << endl;
    
     

    cout << "stuffing traj: " << endl;
    //convert from vector of 7dof poses to trajectory message  
    arm7dof_traj_streamer.stuff_trajectory(optimal_path, des_trajectory); 
    	

    int traj_size = des_trajectory.points.size();
    ROS_INFO("pts in traj %d", traj_size);
	ROS_INFO("waiting for arm server: ");
    bool server_exists2 = arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists2) {
        ROS_WARN("waiting on server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists2 = arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to arm action server"); // if here, then we connected to the server;  
    arm_goal.trajectory=des_trajectory;
    ROS_INFO("sending goals to arm: ");
    arm_action_client.sendGoal(arm_goal, &armDoneCb); 
	}
    bool finished_before_timeout2=false;
    while (!finished_before_timeout2) {
    	ROS_INFO("debug:waiting");
    finished_before_timeout2 = arm_action_client.waitForResult(ros::Duration(10.0));
    }
	


}	
}
