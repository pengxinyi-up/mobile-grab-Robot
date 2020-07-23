#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"


bool getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
double& x, double& y, double& yaw,
const ros::Time& t, const std::string& base_link)
{
	// Get the robot‘s pose
	tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
	tf::Vector3(0,0,0)), t, base_link );
	try
	{
		tf_ = new tf::TransformListener();
		tf_->transformPose(odom_frame_id_, ident, odom_pose);
	}
	catch(tf::TransformException e)
	{
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return false;
	}
		x = odom_pose.getOrigin().x();
		y = odom_pose.getOrigin().y();
	double pitch,roll;
	odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

return true;
}
