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
#include <screw_loose_test/perceptionAction.h>
#include <actionlib/client/terminal_state.h>
#include <arm7dof_traj_as/arm7dof_traj_as.h>
#include<arm7dof_traj_as/trajAction.h>
#include <xform_utils/xform_utils.h>
#include <control_msgs/FollowJointTrajectoryAction.h>




std::vector<vector<Eigen::VectorXd> > path_options;
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclSelectedPoints_ptr;
geometry_msgs::Pose g_recieved_pose, g_end_pose;
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


void create_cutting_traj(geometry_msgs::Pose start_pose, geometry_msgs::Pose end_pose, nav_msgs::Path &cutting_path) {
	cutting_path.poses.clear();
	cutting_path.header.seq=0;
	cutting_path.header.stamp=ros::Time::now();
	cutting_path.header.frame_id= "world";
	double dx,dy,dz;
	geometry_msgs::PoseStamped desired_pose;
	desired_pose.pose = start_pose;
	desired_pose.header.seq=0;
	desired_pose.header.frame_id="world";
	desired_pose.header.stamp=ros::Time::now();
	cutting_path.poses.push_back(desired_pose);
	double cut_precision; //WHY AM I NOT USING
	int steps = 100; //NEED TO ADJUST
	dx=(end_pose.position.x - start_pose.position.x)/steps;
	dy = (end_pose.position.y - start_pose.position.y)/steps;
	dz = (end_pose.position.z - start_pose.position.z)/steps;
	for(int i=1; i < steps+1; i++) {  //UGLY LOOPING FOR CORRECT SEQUENCE. BUT DOES SEQUENCE MATTER
		//desired_pose.header.seq = i;
		desired_pose.header.frame_id = "world";
		desired_pose.header.stamp= ros::Time::now();
		desired_pose.pose.position.x +=dx;
		desired_pose.pose.position.y +=dy;
		desired_pose.pose.position.z +=dz;
		cutting_path.poses.push_back(desired_pose);
		//GOTTA FIX ORIENTATION TO POINT GRIPPER IN THE DIRECTION OF NORMAL TO LINE BETWEEN TWO POSES. HOW WILL I DO THAT
	}
    desired_pose.pose=end_pose;
    cutting_path.poses.push_back(desired_pose);

}


void make_circular_path_around_provided_pose (geometry_msgs::Pose center_pose, nav_msgs::Path &circular_path) {
	Eigen::Affine3d tf_wrench, pose_center, pose_wrench;
	geometry_msgs::Pose wrench_pose;
	geometry_msgs::PoseStamped desired_pose;
	circular_path.poses.clear();
	circular_path.header.seq=0;
    circular_path.header.stamp=ros::Time::now();
    circular_path.header.frame_id = "world";
    double amp = 1.0;
    
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
	wrench_pose.position.z=1.5;
	wrench_pose.orientation.x=0;
	wrench_pose.orientation.y=1;
	wrench_pose.orientation.z=0;
	wrench_pose.orientation.w=0;
	double phase = convertPlanarQuaternion2Psi(wrench_pose.orientation);
	//
	//first pose above nut
	desired_pose.header.seq=i;
	desired_pose.header.stamp=ros::Time::now();
    desired_pose.header.frame_id="world";
    desired_pose.pose.position.x=wrench_pose.position.x;
    desired_pose.pose.position.y=wrench_pose.position.y;
    desired_pose.pose.position.z=wrench_pose.position.z-0.2;//start with 20cm above surface
    desired_pose.pose.orientation=convertPlanarPsi2Quaternion(-phase);
	circular_path.poses.push_back(desired_pose);
	i++;
	//second pose to insert wrench, maybe need to refine?
	desired_pose.header.seq=i;
	desired_pose.header.stamp=ros::Time::now();
    desired_pose.header.frame_id="world";
    desired_pose.pose.position.x=wrench_pose.position.x;
    desired_pose.pose.position.y=wrench_pose.position.y;
    desired_pose.pose.position.z=wrench_pose.position.z;
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
    		desired_pose.pose.position.x=wrench_pose.position.x+x;
    		desired_pose.pose.position.y=wrench_pose.position.y+y;
    		desired_pose.pose.orientation=convertPlanarPsi2Quaternion(-phase+1.57); 
         	circular_path.poses.push_back(desired_pose);
        	i++;
    		
        }

    desired_pose.header.seq=i;
	desired_pose.header.stamp=ros::Time::now();
    desired_pose.header.frame_id="world";
    desired_pose.pose.position.x=desired_pose.pose.position.x;
    desired_pose.pose.position.y=desired_pose.pose.position.y;
    desired_pose.pose.position.z=desired_pose.pose.position.z-0.2;//start with 20cm above surface
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











int main (int argc, char** argv) {
	ros::init(argc, argv, "screw_action_client_node");
	ros::NodeHandle nh;
	std::vector<Eigen::VectorXd> optimal_path;
	nav_msgs::Path g_path;
	int choice;
	optimal_path.clear();
	control_msgs::FollowJointTrajectoryGoal arm_goal;
	screw_loose_test::perceptionGoal perception_goal;
	Eigen::VectorXd q_pre_pose;
    Eigen::VectorXd q_vec;
    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory;
    Arm7dof_traj_streamer arm7dof_traj_streamer(&nh);
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_action_client("/arm7dof/joint_trajectory_controller/follow_joint_trajectory/", true);
	

    q_pre_pose.resize(7);
    q_pre_pose << 0, 1, 0, -2, 0, 1, 0; 
    path_options.clear();
    
    g_recieved_pose.position.x=-2;
    g_recieved_pose.position.y=0.5;
    g_recieved_pose.position.z=1.5;
    g_recieved_pose.orientation.x=0;
    g_recieved_pose.orientation.y=0;
    g_recieved_pose.orientation.z=0;
    g_recieved_pose.orientation.w=1;

    g_end_pose.position.x=2;
    g_end_pose.position.y=0.5;
    g_end_pose.position.z=1.5;
    g_end_pose.orientation.x=0;
    g_end_pose.orientation.y=0;
    g_end_pose.orientation.z=0;
    g_end_pose.orientation.w=1;
    
    while(ros::ok()) {
    g_path.poses.clear();//gotta clear these inside functions
	des_trajectory.points.clear();
	optimal_path.clear();

    	ROS_INFO(" For circular motion, press 1, for cutting motion, press 2");
    	cin>>choice;
    switch (choice) {
    	case 1:
    		make_circular_path_around_provided_pose(g_recieved_pose, g_path);
    		break;
    	case 2:
    		create_cutting_traj(g_recieved_pose,g_end_pose, g_path);
    		break;
    	default:
    		ROS_INFO("How can you mess up with just two options");

    }

    
    //SEG FAULT POSSIBLE HERE


	
	

	bool found_optimal_path=find_optimal_path_from_path_message(g_path, optimal_path);
	//optimal_path.push_back(q_pre_pose);
	if(found_optimal_path) {
	q_vec = arm7dof_traj_streamer.get_q_vec_Xd(); 
    cout << "arm current state:" << q_vec.transpose() << endl;
    
     

    cout << "stuffing traj: " << endl;
    //convert from vector of 7dof poses to trajectory message  
    arm7dof_traj_streamer.stuff_trajectory(optimal_path, des_trajectory); 
    	

    int traj_size = des_trajectory.points.size();
    ROS_INFO("pts in traj %d", traj_size);
	ROS_INFO("waiting for arm server: ");
    bool server_exists = arm_action_client.waitForServer(ros::Duration(10.0));
    while (!server_exists) {
        ROS_WARN("waiting on server...");
        ros::spinOnce();
        ros::Duration(10.0).sleep();
        server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to arm action server"); // if here, then we connected to the server;  
    arm_goal.trajectory=des_trajectory;
    ROS_INFO("sending goals to arm: ");
    arm_action_client.sendGoal(arm_goal); 
	}
    bool finished_before_timeout2=false;
    while (!finished_before_timeout2) {
    	ROS_INFO("debug:waiting");
    finished_before_timeout2 = arm_action_client.waitForResult(ros::Duration(10.0));
    }
	


}	
}
