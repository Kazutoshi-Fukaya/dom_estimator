#ifndef DOM_ESTIMATOR_H_
#define DOM_ESTIMATOR_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>

#include <sstream>
#include <fstream>

#include "multi_localizer_msgs/ObjectMap.h"
#include "dom_estimator_msgs/Doms.h"
#include "object_identifier_msgs/ObjectPositionsWithID.h"

#include "objects_data_subscribers/objects_data_subscribers.h"
#include "utils/database/database.h"
#include "utils/dom_recorder/dom_recorder.h"

namespace dom_estimator
{
class DomEstimator
{
public:
    DomEstimator();
    ~DomEstimator();
    void process();

private:
    void load_object_param();   // yaml file
    void load_objects();        // csv file
    void setup_object_texts();  // object texts
    void setup_time_text();     // time text

    // save
    void save_objects(std::string record_file);        // save csv

    // update
    void update();

    // publish
    void visualize_object();
    void publish_object_texts();
    void publish_time_text();
    void publish_object();
    void publish_msg();

    geometry_msgs::Pose get_pose_msg(double x,double y);
    std_msgs::ColorRGBA get_color_msg(double r,double g,double b,double a);
    std_msgs::ColorRGBA get_background_color();
    std::vector<std::string> split(std::string& input,char delimiter);

    std::string get_date();
    double get_time();

    // callback
    void ops_with_id_callback(const object_identifier_msgs::ObjectPositionsWithIDConstPtr& msg);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // publisher
    ros::Publisher markers_pub_;
    std::vector<ros::Publisher> object_text_pubs_;
    ros::Publisher time_pub_;
    ros::Publisher dom_pub_;
    ros::Publisher object_map_pub_;

    // subscriber
    ros::Subscriber ops_with_id_in_;

    // database
    Database* database_;

    // recorder
    DomRecorder* recorder_;

    // data subscribers
    // ObjectsDataSubscribers* objects_data_subs_;

    // buffer
    ros::Time start_time_;
    std::vector<jsk_rviz_plugins::OverlayText> object_texts_;
    jsk_rviz_plugins::OverlayText time_text_;
    int update_count_;
    int dom_count_;
    int time_count_;

    // params
    std::string MAP_FRAME_ID_;
    bool IS_DEBUG_;
    bool IS_RECORD_;
    bool IS_OBSERVED_SITUATION_;
    bool UPDATE_DOM_;
    int HZ_;
    int RECORD_INTERVAL_;
    double UPDATE_INTERVAL_;
    // double DOM_INTERVAL_;
};
} // namespace dom_estimator

#endif  // DOM_ESTIMATOR_H_
