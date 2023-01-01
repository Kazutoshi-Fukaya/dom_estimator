#ifndef DOM_ESTIMATOR_H_
#define DOM_ESTIMATOR_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>
// #include

#include <sstream>
#include <fstream>

#include "ros_utils/objects_data_subscribers.h"
#include "utils/database/database.h"

namespace dom_estimator
{
class DomEstimator
{
public:
    DomEstimator();
	~DomEstimator();
    void process();

private:
	void load_object_param();	// yaml file
	void load_objects();		// csv file
	void setup_object_texts();	// object texts
	void setup_time_text();		// time text

	// update
	void update();

	// publish
	void visualize_object();
	void publish_object_texts();
	void publish_time_text();
	void publish_msg();

	geometry_msgs::Pose get_pose_msg(double x,double y);
	std_msgs::ColorRGBA get_color_msg(double r,double g,double b,double a);
	std_msgs::ColorRGBA get_background_color();
	std::vector<std::string> split(std::string& input,char delimiter);
	double get_time();

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

	// publisher
	ros::Publisher markers_pub_;
	std::vector<ros::Publisher> object_text_pubs_;
	ros::Publisher time_pub_;

	// database
	Database* database_;

	// data subscribers
	ObjectsDataSubscribers* objects_data_subs_;

	// buffer
	ros::Time start_time_;
	std::vector<jsk_rviz_plugins::OverlayText> object_texts_;
	jsk_rviz_plugins::OverlayText time_text_;
	int update_count_;

	// params
	std::string MAP_FRAME_ID_;
	bool IS_DEBUG_;
	int HZ_;
	double UPDATE_INTERVAL_;
};
} // namespace dom_estimator

#endif  // DOM_ESTIMATOR_H_
