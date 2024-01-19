#include "dom_estimator/dom_estimator.h"

using namespace dom_estimator;

DomEstimator::DomEstimator() :
    private_nh_("~"),
    database_(new Database()),
    // objects_data_subs_(new ObjectsDataSubscribers(nh_,private_nh_,database_)),
    start_time_(ros::Time::now()),
    update_count_(0), dom_count_(0),
    time_count_(0)
{
    private_nh_.param("MAP_FRAME_ID",MAP_FRAME_ID_,{std::string("map")});
    private_nh_.param("IS_DEBUG",IS_DEBUG_,{false});
    private_nh_.param("IS_RECORD",IS_RECORD_,{false});
    private_nh_.param("HZ",HZ_,{10});
    private_nh_.param("UPDATE_INTERVAL",UPDATE_INTERVAL_,{300.0});
    // private_nh_.param("DOM_INTERVAL",DOM_INTERVAL_,{100.0});

    if(IS_RECORD_) recorder_ = new DomRecorder();

    // database
    load_object_param();
    load_objects();
    // database_->print_contents();

    // text
    setup_object_texts();
    setup_time_text();

    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("objects",1);
    time_pub_ = nh_.advertise<jsk_rviz_plugins::OverlayText>("time",1);
    dom_pub_ = nh_.advertise<dom_estimator_msgs::Doms>("dom",1);
    object_map_pub_ = nh_.advertise<multi_localizer_msgs::ObjectMap>("object_map",1);
}

DomEstimator::~DomEstimator()
{
    if(IS_DEBUG_){
        database_->print_contents();
    }

    if(IS_RECORD_){
        std::string record_path;
        private_nh_.param("RECORD_PATH",record_path,{std::string("")});
        recorder_->set_path(record_path + "dom/");
        recorder_->output_data();
        save_objects(record_path + "objects/" + get_date() + ".csv");
    }
}

void DomEstimator::ops_with_id_callback(const object_identifier_msgs::ObjectPositionsWithIDConstPtr& msg)
{
    double time = msg->header.stamp.toSec();
    for(const auto & data : msg->object_positions_with_id){
        database_->add_object(data.id,data.x,data.y,time,data.probability); //probability = credibility
    }
}

void DomEstimator::load_object_param()
{
    std::string yaml_file_name;
    private_nh_.param("YAML_FILE_NAME",yaml_file_name,{std::string("object_params")});
    XmlRpc::XmlRpcValue object_params;
    if(!private_nh_.getParam(yaml_file_name.c_str(),object_params)){
        ROS_WARN("Cloud not load %s", yaml_file_name.c_str());
        return;
    }

    ROS_ASSERT(object_params.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < object_params.size(); i++){
        if(!object_params[i]["id"].valid() || !object_params[i]["name"].valid() || !object_params[i]["condition"].valid() ||
           !object_params[i]["r"].valid() || !object_params[i]["g"].valid() || !object_params[i]["b"].valid() ||
           !object_params[i]["dist_th"].valid() || !object_params[i]["dom"].valid()){
            ROS_WARN("%s is valid", yaml_file_name.c_str());
            return;
        }
        if(object_params[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
           object_params[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
           object_params[i]["condition"].getType() == XmlRpc::XmlRpcValue::TypeString &&
           object_params[i]["r"].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
           object_params[i]["g"].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
           object_params[i]["b"].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
           object_params[i]["dist_th"].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
           object_params[i]["dom"].getType() == XmlRpc::XmlRpcValue::TypeDouble){
            int id = static_cast<int>(object_params[i]["id"]);
            std::string name = static_cast<std::string>(object_params[i]["name"]);
            std::string condition = static_cast<std::string>(object_params[i]["condition"]);
            double r = static_cast<double>(object_params[i]["r"]);
            double g = static_cast<double>(object_params[i]["g"]);
            double b = static_cast<double>(object_params[i]["b"]);
            double dist_th = static_cast<double>(object_params[i]["dist_th"]);
            double dom = static_cast<double>(object_params[i]["dom"]);
            ObjectParam* object_param (new ObjectParam(name,condition,Color(r,g,b)));
            // Objects* objects (new Objects(id,name,condition,dom,dist_th));
            // database_->insert(std::map<ObjectParam*,Objects*>::value_type(object_param,objects));
            ObjectWithID* object_with_id (new ObjectWithID(id,name,condition,dom,dist_th));
            database_->insert(std::map<ObjectParam*,ObjectWithID*>::value_type(object_param,object_with_id));
            // std::cout << "id: " << id << std::endl;

            if(IS_RECORD_){
                std::vector<DomRecord> doms;
                // recorder_->insert(std::map<std::string,std::vector<DomRecord>>::value_type(name,doms));
                recorder_->insert(std::map<int,std::vector<DomRecord>>::value_type(id,doms));
            }
        }
    }
    // std::cout << "size: " << database_->size() << std::endl;
}

void DomEstimator::load_objects()
{
    std::string objects_file;
    private_nh_.param("OBJECTS_FILE",objects_file,{std::string("")});
    ROS_INFO("load file: %s", objects_file.c_str());
    static std::ifstream ifs(objects_file);
    std::string line;
    while(std::getline(ifs,line)){
        std::vector<std::string> strvec = split(line,',');
        try{
            // std::string name = static_cast<std::string>(strvec[0]);
            // // double time = static_cast<double>(std::stod(strvec[1]));
            // double x = static_cast<double>(std::stod(strvec[2]));
            // double y = static_cast<double>(std::stod(strvec[3]));

            int id = static_cast<int>(std::stoi(strvec[0]));
            std::string name = static_cast<std::string>(strvec[1]);
            double time = static_cast<double>(std::stod(strvec[2]));
            double dom = static_cast<double>(std::stod(strvec[3]));     // unused?
            double x = static_cast<double>(std::stod(strvec[4]));
            double y = static_cast<double>(std::stod(strvec[5]));
            // database_->add_init_object(name,x,y);   // time = 0.0, credibility = 1.0
            database_->add_init_object(id,x,y);   // time = 0.0, credibility = 1.0
            // std::cout << "load: " << name << std::endl;
        }
        catch(const std::invalid_argument& ex){
            ROS_ERROR("invalid: %s", ex.what());
        }
        catch(const std::out_of_range& ex){
            ROS_ERROR("out of range: %s", ex.what());
        }
    }
    ifs.close();
}

void DomEstimator::setup_object_texts()
{
    // is database empty?
    if(database_->empty()){
        ROS_ERROR("Database is empty!");
        return;
    }

    // text position
    int width, height, left, top;
    private_nh_.param("OBJECT_TEXT_WIDTH",width,{350});
    private_nh_.param("OBJECT_TEXT_HEIGHT",height,{25});
    private_nh_.param("OBJECT_TEXT_LEFT",left,{0});
    private_nh_.param("OBJECT_TEXT_TOP",top,{25});

    // text font
    std::string font;
    int line_width, text_size;
    private_nh_.param("OBJECT_TEXT_FONT",font,{std::string("Ubuntu")});
    private_nh_.param("OBJECT_TEXT_LINE_WIDTH",line_width,{1});
    private_nh_.param("OBJECT_TEXT_TEXT_SIZE",text_size,{14});

    int index = 0;
    for(auto it = database_->begin(); it != database_->end(); it++, index++){
        // publisher
        ros::Publisher object_text_pub = nh_.advertise<jsk_rviz_plugins::OverlayText>(it->first->name,1);
        object_text_pubs_.emplace_back(object_text_pub);

        // text
        jsk_rviz_plugins::OverlayText text;
        text.action = jsk_rviz_plugins::OverlayText::ADD;
        text.width = width;
        text.height = height;
        if(index >= 5){
            text.left = left + width;
            text.top = top*(index - 5);
        }
        else{
            text.left = left;
            text.top = top*index;
        }
        text.font = font;
        text.line_width = line_width;
        text.text_size = text_size;
        text.text = it->first->name;
        text.bg_color = get_background_color();
        text.fg_color = get_color_msg(it->first->color.r,it->first->color.g,it->first->color.b,1.0);
        object_texts_.emplace_back(text);
        // index++;
    }
}

void DomEstimator::setup_time_text()
{
    // default
    time_text_.action = jsk_rviz_plugins::OverlayText::ADD;
    time_text_.bg_color = get_background_color();

    // text position
    int width, height, left, top;
    private_nh_.param("TIME_TEXT_WIDTH",width,{230});
    private_nh_.param("TIME_TEXT_HEIGHT",height,{25});
    private_nh_.param("TIME_TEXT_LEFT",left,{1300});
    private_nh_.param("TIME_TEXT_TOP",top,{0});
    time_text_.width = width;
    time_text_.height = height;
    time_text_.left = left;
    time_text_.top = top;

    // text font
    std::string font;
    int line_width, text_size;
    private_nh_.param("TIME_TEXT_FONT",font,{std::string("Ubuntu")});
    private_nh_.param("TIME_TEXT_LINE_WIDTH",line_width,{1});
    private_nh_.param("TIME_TEXT_TEXT_SIZE",text_size,{14});
    time_text_.font = font;
    time_text_.line_width = line_width;
    time_text_.text_size = text_size;

    // text color
    double color_r, color_g, color_b, color_a;
    private_nh_.param("TIME_TEXT_COLOR_R",color_r,{0.0});
    private_nh_.param("TIME_TEXT_COLOR_G",color_g,{0.0});
    private_nh_.param("TIME_TEXT_COLOR_B",color_b,{0.0});
    private_nh_.param("TIME_TEXT_COLOR_A",color_a,{1.0});
    time_text_.fg_color = get_color_msg(color_r,color_g,color_b,color_a);
}

void DomEstimator::save_objects(std::string record_file)
{
    std::ofstream ofs(record_file);
    for(auto it = database_->begin(); it != database_->end();it++){
        for(auto sit = it->second->begin(); sit != it->second->end(); sit++){
            ofs << it->second->name.c_str() << ","
                << sit->time << "," 
                << sit->x <<  "," 
                << sit->y << std::endl;
        }
    }
    ofs.close();
}

void DomEstimator::update()
{
    // update database (for object)
    database_->update_objects();

    // object update (for time)
    if(get_time() > UPDATE_INTERVAL_*(update_count_ + 1)){
        database_->time_update();
        update_count_++;

        // delete all markers
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker marker;
        marker.header.frame_id = MAP_FRAME_ID_;
        marker.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.emplace_back(marker);
        markers_pub_.publish(markers);
    }

    // update dom
    // if(get_time() > DOM_INTERVAL_*(dom_count_ + 1)){
        // database_->update_dom(get_time());
        // dom_count_++;
    // }
    database_->update_dom(get_time());
}

// void DomEstimator::visualize_object()
// {
//     ros::Time now_time = ros::Time::now();
//     visualization_msgs::MarkerArray markers;
//     int marker_id = 0;
//     for(auto it = database_->begin(); it != database_->end(); it++){
//         for(auto sit = it->second->begin(); sit != it->second->end(); sit++){
//             // marker
//             visualization_msgs::Marker marker;
//             marker.id = marker_id;
//             marker.header.frame_id = MAP_FRAME_ID_;
//             marker.header.stamp = now_time;
//             marker.type = visualization_msgs::Marker::CUBE;
//             marker.action = visualization_msgs::Marker::ADD;
//             marker.lifetime = ros::Duration();
//             marker.ns = it->first->name;
//             marker.scale.x = 0.4;
//             marker.scale.y = 0.4;
//             marker.scale.z = 0.1;
//             marker.pose = get_pose_msg(sit->x,sit->y);
//             marker.color.r = it->first->color.r;
//             marker.color.g = it->first->color.g;
//             marker.color.b = it->first->color.b;
//             marker.color.a = 0.2;
//             if(sit->has_observed){
//                 marker.color.a += 0.8*sit->credibility + 0.2;
//             }
//             markers.markers.emplace_back(marker);
//             marker_id++;
//         }
//     }
//     markers_pub_.publish(markers);
// }

void DomEstimator::visualize_object()
{
    ros::Time now_time = ros::Time::now();
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;
    for(auto it = database_->begin(); it != database_->end(); it++){
        // marker
        visualization_msgs::Marker marker;
        marker.id = marker_id;
        marker.header.frame_id = MAP_FRAME_ID_;
        marker.header.stamp = now_time;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();
        marker.ns = it->first->name;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.1;
        marker.pose = get_pose_msg(it->second->x,it->second->y);
        marker.color.r = it->first->color.r;
        marker.color.g = it->first->color.g;
        marker.color.b = it->first->color.b;
        marker.color.a = 0.2;
        if(it->second->has_observed){
            marker.color.a += 0.8*it->second->credibility + 0.2;
        }
        markers.markers.emplace_back(marker);
        marker_id++;
    }
    markers_pub_.publish(markers);
}

void DomEstimator::publish_object_texts()
{
    for(size_t i = 0; i < object_text_pubs_.size(); i++){
        for(auto it = database_->begin(); it != database_->end(); it++){
            std::string topic_name = "/" + it->first->name;
            if(topic_name == object_text_pubs_[i].getTopic()){
                std::string text_msg = it->first->name + "\t (dom: " + std::to_string(it->second->dom) + ")";
                object_texts_[i].text = text_msg;
            }
        }
        object_text_pubs_[i].publish(object_texts_[i]);
    }

    for(auto it = database_->begin(); it != database_->end(); it++){
        if(IS_RECORD_){
            // if(time_count_*30.0 < get_time()){
            if(time_count_%300 == 0){
                // std::string name = it->first->name;
                int id = it->second->id;
                double time = get_time();
                double dom = it->second->dom;
                // recorder_->add_data(name,DomRecord(time,dom));
                recorder_->add_data(id,DomRecord(time,dom));
                // time_count_++;
            }
        }
    }
    time_count_++;
}

void DomEstimator::publish_time_text()
{
    time_text_.text =  std::string("Time: ") + std::to_string(get_time());
    time_pub_.publish(time_text_);
}

void DomEstimator::publish_object()
{
    dom_estimator_msgs::Doms doms;
    multi_localizer_msgs::ObjectMap object_map;
    ros::Time now_time = ros::Time::now();
    doms.header.stamp = now_time;
    object_map.header.frame_id = MAP_FRAME_ID_;
    object_map.header.stamp = now_time;
    for(auto it = database_->begin(); it != database_->end(); it++){
        // dom
        dom_estimator_msgs::Dom dom;
        dom.name = it->second->name;
        dom.dom = it->second->dom;
        dom.object_size = it->second->size();
        dom.appearance_count = it->second->appearance_count;
        dom.disappearance_count = it->second->disappearance_count;
        dom.observations_count = it->second->observations_count;
        doms.doms.emplace_back(dom);

        // object data
        for(auto sit = it->second->begin(); sit != it->second->end(); sit++){
            multi_localizer_msgs::ObjectData data;
            data.name = it->second->name;
            data.credibility = sit->credibility;
            data.time = get_time();
            data.x = sit->x;
            data.y = sit->y;
            object_map.data.emplace_back(data);
        }
    }
    dom_pub_.publish(doms);
    object_map_pub_.publish(object_map);
}

void DomEstimator::publish_msg()
{
    visualize_object();
    publish_object_texts();
    publish_time_text();
    publish_object();
}

geometry_msgs::Pose DomEstimator::get_pose_msg(double x,double y)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    tf2::Quaternion tf_q;
    tf_q.setRPY(0.0,0.0,0.0);
    pose.orientation.w = tf_q.w();
    pose.orientation.x = tf_q.x();
    pose.orientation.y = tf_q.y();
    pose.orientation.z = tf_q.z();

    return pose;
}

std_msgs::ColorRGBA DomEstimator::get_color_msg(double r,double g,double b,double a)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
}

std_msgs::ColorRGBA DomEstimator::get_background_color()
{
    // background color is white
    return get_color_msg(1.0,1.0,1.0,1.0);
}

std::vector<std::string> DomEstimator::split(std::string& input,char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while(std::getline(stream,field,delimiter)) result.emplace_back(field);
    return result;
}

std::string DomEstimator::get_date()
{
    time_t t = time(nullptr);
    const tm* localTime = localtime(&t);
    std::stringstream s;
    s << localTime->tm_year + 1900;
    s << std::setw(2) << std::setfill('0') << localTime->tm_mon + 1;
    s << std::setw(2) << std::setfill('0') << localTime->tm_mday;
    s << std::setw(2) << std::setfill('0') << localTime->tm_hour;
    s << std::setw(2) << std::setfill('0') << localTime->tm_min;
    s << std::setw(2) << std::setfill('0') << localTime->tm_sec;

    return s.str();
}

double DomEstimator::get_time() { return (ros::Time::now() - start_time_).toSec(); }

void DomEstimator::process()
{
    std::cout << "=== Main Process ===" << std::endl;
    ros::Rate rate(HZ_);
    while(ros::ok()){
        publish_msg();  // publish msg
        update();       // update
        ros::spinOnce();
        rate.sleep();
    }
}
