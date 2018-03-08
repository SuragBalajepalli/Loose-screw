#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl_utils/pcl_utils.h>
#include <xform_utils/xform_utils.h>
#include <circle_action/perceptionAction.h>


Eigen::Affine3f g_affine_sensor_wrt_base;




class Perception {
private:

	ros::NodeHandle nh;
	actionlib::SimpleActionServer<circle_action::perceptionAction> perception_as;

	circle_action::perceptionGoal goal;
	circle_action::perceptionResult result;
	circle_action::perceptionFeedback fdbk;

	PclUtils pclUtils;
	tf::TransformListener* tfListener;

	
	bool find_nut(float, geometry_msgs::PoseStamped&); //function to find nuts. 
	
	//more functions if needed
	bool found_surfaces;

public:
	Perception();
	~Perception(void);

	void executeCb(const actionlib::SimpleActionServer<circle_action::perceptionAction>::GoalConstPtr& goal);
	//Xform_utils xform_utils;//unused?

};

Perception::Perception():
perception_as(nh,"perception_action_service", boost::bind(&Perception::executeCb,this,_1),false), pclUtils(&nh) {
	ROS_INFO("in constructor of perception");

	perception_as.start();
	tfListener = new tf::TransformListener;
	found_surfaces=false;
}

//class necessary? anyway.

bool Perception::find_nut(float surface_z, geometry_msgs::PoseStamped &nut_pose) {
	Eigen::Vector3f plane_normal;
	float x_max,x_min,y_max,y_min, nut_height_min, nut_height_max, eps;
	double plane_dist;
	Eigen::Vector3f major_axis;
	Eigen::Vector3f centroid;
	bool found_nut = true;
	double nut_height=42; // gotta change according to requirement
	found_nut = pclUtils.find_plane_fit(x_max,x_min,y_max,y_min,surface_z+nut_height_min,surface_z+nut_height_max,eps,plane_normal,plane_dist,major_axis,centroid);

	if (plane_normal(2)<0) plane_normal(2) *= -1.0;
	Eigen::Matrix3f R;
	Eigen::Vector3f y_vec;
	R.col(0) = major_axis;
	R.col(1) = plane_normal.cross(major_axis);
	R.col(2) = plane_normal;
	Eigen::Quaternionf quat(R);
	nut_pose.header.frame_id = "world";
	nut_pose.pose.position.x = centroid(0);
	nut_pose.pose.position.y = centroid(1);
	nut_pose.pose.position.z= centroid(2)-0.5*nut_height;
	//unsure of the following, might need to change based on geometry of nut and wrench
	nut_pose.pose.orientation.x = quat.x();
	nut_pose.pose.orientation.y = quat.y();
	nut_pose.pose.orientation.z = quat.z();
	//unsurety ends
	return found_nut;
}

void Perception::executeCb(const actionlib::SimpleActionServer<circle_action::perceptionAction>::GoalConstPtr& goal) {
	geometry_msgs::PoseStamped nut_pose;
	double surface_z;
	bool found_nut=false;
	surface_z=goal->surface_z;
	found_nut=find_nut(surface_z, nut_pose);
	result.nut_pose=nut_pose.pose;
	perception_as.setSucceeded(result);

	}




int main(int argc, char** argv) {
	ros::init(argc,argv,"nut_perception_node");
	tf::TransformListener tfListener;
	tf::StampedTransform stf_sensor_wrt_base;
	ROS_INFO("waiting for tf between sensor and base");
	bool tferr = true;
	while (tferr) {
		tferr = false;
		try {
			tfListener.lookupTransform("base","sensor",ros::Time(0), stf_sensor_wrt_base);//change according to sensor name. or topic remapping?
		} catch (tf::TransformException &exception) {
			ROS_WARN("%s; retrying..", exception.what());
			tferr = true;
			ros::Duration(0.5).sleep();
			ros::spinOnce();
			//above lines should be replaced if the transform is static and can be hardcoded into the node
		}
	}
	ROS_INFO("recieved a transform");
	ROS_INFO("spinning, not literally");

	while(ros::ok()) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();

	}
return 0;
}
