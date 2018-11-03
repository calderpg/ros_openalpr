#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <ros_openalpr/PlateCandidate.h>
#include <ros_openalpr/PlateResult.h>
#include <ros_openalpr/PlatesResults.h>
#include <alpr.h>

class RosOpenALPR
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher plates_pub_;
  alpr::Alpr recognizer_;

public:
  RosOpenALPR(
      const ros::NodeHandle& nh, const std::string& image_topic,
      const std::string& plates_topic, const std::string& region_code,
      const std::string& config_path, const std::string& runtime_path,
      const int32_t num_plates_to_return, const std::string& region_pattern)
    : nh_(nh), it_(nh), recognizer_(region_code, config_path, runtime_path)
  {
    // Make sure OpenALPR is loaded properly
    recognizer_.setTopN(num_plates_to_return);
    recognizer_.setDefaultRegion(region_pattern);
    if (recognizer_.isLoaded())
    {
      ROS_INFO_NAMED(ros::this_node::getName(), "OpenALPR loaded");
    }
    else
    {
      ROS_FATAL_NAMED(ros::this_node::getName(), "Failed to load OpenALPR");
    }
    // Wire up ROS connections
    image_sub_ =
        it_.subscribe(image_topic, 1, &RosOpenALPR::ImageCallback, this);
    plates_pub_ =
        nh_.advertise<ros_openalpr::PlatesResults>(plates_topic, 1, false);
    // Check what transport we're using for camera images
    const std::string transport_in = image_sub_.getTransport();
    ROS_INFO_NAMED(ros::this_node::getName(),
                   "Subscribed using %s for transport", transport_in.c_str());
  }

  void Loop(const double loop_frequency)
  {
    ros::Rate loop_rate(loop_frequency);
    while (ros::ok())
    {
      loop_rate.sleep();
      ros::spinOnce();
    }
  }

  void ImageCallback(const sensor_msgs::ImageConstPtr& image_msg)
  {
    const int32_t bytes_per_pixel = static_cast<int32_t>(image_msg->step)
                                    / static_cast<int32_t>(image_msg->width);
    const alpr::AlprResults results =
        recognizer_.recognize(const_cast<uint8_t*>(image_msg->data.data()),
                              bytes_per_pixel,
                              static_cast<int32_t>(image_msg->width),
                              static_cast<int32_t>(image_msg->height),
                              std::vector<alpr::AlprRegionOfInterest>());
    ros_openalpr::PlatesResults results_msg;
    // Set header information
    results_msg.header.frame_id = image_msg->header.frame_id;
    results_msg.image_time = image_msg->header.stamp;
    results_msg.header.stamp = ros::Time::now();
    // Add detected plates
    for (size_t plate_idx = 0; plate_idx < results.plates.size(); plate_idx++)
    {
      const alpr::AlprPlateResult& plate_result = results.plates.at(plate_idx);
      ros_openalpr::PlateResult plate_result_msg;
      // Set detection information
      plate_result_msg.processing_time =
          ros::Duration(plate_result.processing_time_ms * 0.001);
      plate_result_msg.plate_country_code = plate_result.country;
      plate_result_msg.requested_num_candidates = plate_result.requested_topn;
      plate_result_msg.top_left_x = plate_result.plate_points[0].x;
      plate_result_msg.top_left_y = plate_result.plate_points[0].y;
      plate_result_msg.top_right_x = plate_result.plate_points[1].x;
      plate_result_msg.top_right_y = plate_result.plate_points[1].y;
      plate_result_msg.bottom_right_x = plate_result.plate_points[2].x;
      plate_result_msg.bottom_right_y = plate_result.plate_points[2].y;
      plate_result_msg.bottom_left_x = plate_result.plate_points[3].x;
      plate_result_msg.bottom_left_y = plate_result.plate_points[3].y;
      // Add plate candidates
      for (size_t candidate_idx = 0;
           candidate_idx < plate_result.topNPlates.size(); candidate_idx++)
      {
        const alpr::AlprPlate& plate_candidate =
            plate_result.topNPlates.at(candidate_idx);
        ros_openalpr::PlateCandidate plate_candidate_msg;
        plate_candidate_msg.confidence = plate_candidate.overall_confidence;
        plate_candidate_msg.plate_number = plate_candidate.characters;
        plate_result_msg.plate_candidates.push_back(plate_candidate_msg);
      }
      results_msg.plates.push_back(plate_result_msg);
    }
    // Add regions of interest
    for (size_t roi_idx = 0; roi_idx < results.regionsOfInterest.size();
         roi_idx++)
    {
      const alpr::AlprRegionOfInterest& roi =
          results.regionsOfInterest.at(roi_idx);
      sensor_msgs::RegionOfInterest roi_msg;
      roi_msg.do_rectify = false;
      roi_msg.x_offset = static_cast<uint32_t>(roi.x);
      roi_msg.y_offset = static_cast<uint32_t>(roi.y);
      roi_msg.width = static_cast<uint32_t>(roi.width);
      roi_msg.height = static_cast<uint32_t>(roi.height);
      results_msg.regions_of_interest.push_back(roi_msg);
    }
    plates_pub_.publish(results_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_openalpr_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Load parameters
  const std::string image_topic =
      nhp.param(std::string("image_topic"), std::string("image"));
  const std::string plates_topic =
      nhp.param(std::string("plates_topic"), std::string("plates"));
  const std::string region_code =
      nhp.param(std::string("region_code"), std::string("us"));
  const std::string config_path =
      nhp.param(std::string("config_path"), std::string(""));
  const std::string runtime_path =
      nhp.param(std::string("runtime_path"), std::string(""));
  const int32_t num_plates_to_return =
      nhp.param(std::string("num_plates_to_return"), 10);
  const std::string region_pattern =
      nhp.param(std::string("region_pattern"), std::string(""));
  const double loop_rate = nhp.param(std::string("loop_rate"), 128.0);
  // Make interface
  RosOpenALPR ros_alpr(
      nh, image_topic, plates_topic, region_code, config_path, runtime_path,
      num_plates_to_return, region_pattern);
  // Run
  ros_alpr.Loop(loop_rate);
  return 0;
}

