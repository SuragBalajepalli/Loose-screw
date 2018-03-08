#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <circle_action/circular_pathAction.h>


geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double psi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}

class CircularPathActionServer {
private:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<circle_action::circular_pathAction> as_;
	circle_action::circular_pathGoal goal_;
	circle_action::circular_pathResult result_;

public:
	CircularPathActionServer();

	~CircularPathActionServer(void) {
	}
	void executeCB(const actionlib::SimpleActionServer<circle_action::circular_pathAction>::GoalConstPtr& goal);
	};
CircularPathActionServer::CircularPathActionServer() : as_(nh_,"circular_path_action", boost::bind(&CircularPathActionServer::executeCB,this,_1),false)
{
	ROS_INFO("In constructor of action server, trying to initialize stuff");
	nav_msgs::Path desired_motion;
    
    
	as_.start();
}

void CircularPathActionServer::executeCB(const actionlib::SimpleActionServer<circle_action::circular_pathAction>::GoalConstPtr& goal) {
	geometry_msgs::PoseStamped desired_pose;
	result_.path.header.seq=0;
    result_.path.header.stamp=ros::Time::now();
    result_.path.header.frame_id== "world";
    double amp = 1.0;
    double phase = 0.0;
    double omega = 1.0;
    double dt = 0.01;
    double x, y, z;
    z = 0;
    int i=0;
    
	while(i<goal->turns) {
    		if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
        	phase += omega*dt;
        	x = amp * sin(phase);
        	y = amp * cos(phase);
        	desired_pose.header.seq=i;
    		desired_pose.header.stamp=ros::Time::now();
    		desired_pose.header.frame_id="world";
    		desired_pose.pose.position.x=x;
    		desired_pose.pose.position.y=y;
    		desired_pose.pose.orientation=convertPlanarPsi2Quaternion(-phase); 
         	result_.path.poses.push_back(desired_pose);	// since path, should append?
        	//somehow check if path message is populated
        	as_.setSucceeded(result_);
        	i++;
        }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "circular_pose_as"); // name this node 
    CircularPathActionServer as_object;
    while (ros::ok()) {
    	
			//WHAT CAN I DO HERE      
           }
           return 0;
}
