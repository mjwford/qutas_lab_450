#include <ros/ros.h>

#include <area_map/area_map.h>

#include <nav_msgs/OccupancyGrid.h>

AreaMap::AreaMap() :
	nh_("~"),
	topic_map_("grid"),
	param_frame_id_("map"),
	param_map_width_(10),
	param_map_height_(10),
	param_map_resolution_(0.1),
	param_map_boarder_(false),
	param_num_obs_(0) {

	//Load node parameters
	nh_.param("topic_map", topic_map_, topic_map_);
	nh_.param("frame_id", param_frame_id_, param_frame_id_);

	nh_.param("map/width", param_map_width_, param_map_width_);
	nh_.param("map/height", param_map_height_, param_map_height_);
	nh_.param("map/resolution", param_map_resolution_, param_map_resolution_);
	nh_.param("map/boarder", param_map_boarder_, param_map_boarder_);

	nh_.param("obstacles/number", param_num_obs_, param_num_obs_);

	for( int i = 0; i < param_num_obs_; i++ ) {
		obstacles_t obs;
		std::string str;

		nh_.getParam("obstacles/obs_" + std::to_string(i) + "/type", str);

		ROS_INFO("Loading obstacle %i (%s)", i, str.c_str());

		if(str == "square") {
			obs.type = OBS_SQUARE;
		} else if(str == "circle") {
			obs.type = OBS_CIRCLE;
		} else {
			ROS_ERROR("Unknown obstacle type: %s", str.c_str());
		}

		nh_.getParam("obstacles/obs_" + std::to_string(i) + "/size", obs.size);
		nh_.getParam("obstacles/obs_" + std::to_string(i) + "/position/x", obs.x);
		nh_.getParam("obstacles/obs_" + std::to_string(i) + "/position/y", obs.y);

		obs_.push_back(obs);
	}

	//Setup publisher
	pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>(topic_map_, 1, true);

	//Generate Map
	generate_map();

	ROS_INFO("Generated map with %i obstacles", param_num_obs_);

	//Publish map data (latched)
	pub_map_.publish(msg_out_);
}

AreaMap::~AreaMap() {
}

void AreaMap::generate_map( void ) {
	ros::Time stamp = ros::Time::now();

	msg_out_.header.frame_id = param_frame_id_;
	msg_out_.header.stamp = stamp;

	msg_out_.info.map_load_time = stamp;
	msg_out_.info.resolution = param_map_resolution_;
	msg_out_.info.width = param_map_width_;
	msg_out_.info.height = param_map_height_;

	msg_out_.info.origin.position.x = -param_map_resolution_ * (param_map_width_ / 2);
	msg_out_.info.origin.position.y = -param_map_resolution_ * (param_map_height_ / 2);
	msg_out_.info.origin.position.z = 0.0;
	msg_out_.info.origin.orientation.w = 1.0;
	msg_out_.info.origin.orientation.x = 0.0;
	msg_out_.info.origin.orientation.y = 0.0;
	msg_out_.info.origin.orientation.z = 0.0;

	//Load in base map
	for( int i = 0; i < param_map_width_; i++ ) {
		for( int j = 0; j < param_map_height_; j++ ) {
			int8_t spot = 0;

			//Draw in the boarder
			if( param_map_boarder_ &&
				( ( i == 0 ) || (j == 0 ) || ( i == ( param_map_width_ - 1 ) ) || ( j == ( param_map_height_ - 1 ) ) ) ) {
				spot = 100;
			}

			msg_out_.data.push_back(spot);
		}
	}

	//Load in obstacles
	for( int k = 0; k < obs_.size(); k++ ) {
		switch(obs_[k].type) {
			case OBS_SQUARE: {
				for( int y = -obs_[k].size; y < obs_[k].size + 1; y++ ) {
					for( int x = -obs_[k].size; x < obs_[k].size + 1; x++ ) {
						int dx = obs_[k].x + x;
						int dy = obs_[k].y + y;

						if ( (dx >= 0) &&
							 (dx < param_map_width_) &&
							 (dy >= 0) &&
							 (dy < param_map_height_) ) {
								msg_out_.data[dx + (dy*param_map_width_)] = 100;
						}
					}
				}

				break;
			}
			case OBS_CIRCLE: {
				for( int y = -obs_[k].size; y < obs_[k].size + 1; y++ ) {
					for( int x = -obs_[k].size; x < obs_[k].size + 1; x++ ) {
						int dx = obs_[k].x + x;
						int dy = obs_[k].y + y;
						int r = obs_[k].size;

						if ( (dx >= 0) &&
							 (dx < param_map_width_) &&
							 (dy >= 0) &&
							 (dy < param_map_height_) &&
							 ( (x*x + y*y) <= (r*r + r*0.25) ) ) {
								msg_out_.data[dx + (dy*param_map_width_)] = 100;
						}
					}
				}

				break;
			}
			default: {
				ROS_ERROR("Cannot load obstacle: %i", k);
			}
		}
	}
}
