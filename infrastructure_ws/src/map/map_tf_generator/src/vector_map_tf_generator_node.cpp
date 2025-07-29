// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <memory>
#include <string>

class VectorMapTFGeneratorNode : public rclcpp::Node
{
public:
  explicit VectorMapTFGeneratorNode(const rclcpp::NodeOptions & options)
  : Node("vector_map_tf_generator", options)
  {
    map_frame_ = declare_parameter("map_frame", "map");
    viewer_frame_ = declare_parameter("viewer_frame", "base_link");

    sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
      "vector_map", rclcpp::QoS{1}.transient_local(),
      std::bind(&VectorMapTFGeneratorNode::onVectorMap, this, std::placeholders::_1));

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  }

private:
  std::string map_frame_;
  std::string viewer_frame_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;

  void onVectorMap(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
  {
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);
    std::vector<double> points_x;
    std::vector<double> points_y;
    std::vector<double> points_z;

    for (const lanelet::Point3d & point : lanelet_map_ptr_->pointLayer) {
      const double point_x = point.x();
      const double point_y = point.y();
      const double point_z = point.z();
      points_x.push_back(point_x);
      points_y.push_back(point_y);
      points_z.push_back(point_z);
    }
    const double coordinate_x =
      std::accumulate(points_x.begin(), points_x.end(), 0.0) / points_x.size();
    const double coordinate_y =
      std::accumulate(points_y.begin(), points_y.end(), 0.0) / points_y.size();
    const double coordinate_z =
      std::accumulate(points_z.begin(), points_z.end(), 0.0) / points_z.size();

    geometry_msgs::msg::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = this->now();
    static_transformStamped.header.frame_id = map_frame_;
    static_transformStamped.child_frame_id = viewer_frame_;
    //roadside
    // static_transformStamped.transform.translation.x = -30.3648;
    // static_transformStamped.transform.translation.y = -25.5845;
    // static_transformStamped.transform.translation.z = 1.81929;
 
    //roadside diaotou
    // static_transformStamped.transform.translation.x =  -46.6814;
    // static_transformStamped.transform.translation.y = 7.43737;
    // static_transformStamped.transform.translation.z = 1.08176;

    //car
    // static_transformStamped.transform.translation.x = -28.12824663355757;
    // static_transformStamped.transform.translation.y = -22.94452397747907;
    // static_transformStamped.transform.translation.z = -1.4373740381447144;
    
    //roadside 2
    // static_transformStamped.transform.translation.x = -41.7748;
    // static_transformStamped.transform.translation.y = 16.2617;
    // static_transformStamped.transform.translation.z = 1.13644;

    //DG
    static_transformStamped.transform.translation.x = -33.8221;
    static_transformStamped.transform.translation.y = -10.7672;
    static_transformStamped.transform.translation.z = 2.63644;

    tf2::Quaternion quat;
    tf2::Matrix3x3 basis;

    //roadside diaotou
    //basis.setValue(0.372915,  -0.927859,  0.00520842 ,0.92787,    0.372899 , -0.00402609 ,0.00179338, 0.00633418 ,  0.999992  );
    
    //roadside
    // basis.setValue(0.788131,  -0.615334,  -0.0152771  ,0.615516,    0.787744  , 0.0249288 ,-0.00330502, -0.0290502 ,  0.999577   );
    
    //roadside2
    //basis.setValue(0.747671,-0.663973,0.0119094,0.663996,0.747741,0.002344,-0.0104615,0.00615519,0.999941);

    //DG
    basis.setValue(0.743145,-0.669131,0.0,0.669131,0.743145,0.0,0.0,0.0,1.0);



    tf2Scalar r, p, y;
    basis.getRPY(r,p,y);
    quat.setRPY(r, p, y);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();

    //car
    // static_transformStamped.transform.rotation.x = -0.017524184867755373;
    // static_transformStamped.transform.rotation.y = 0.01657190176724868;
    // static_transformStamped.transform.rotation.z = 0.5247388890807094;
    // static_transformStamped.transform.rotation.w = 0.850921484805079;

    static_broadcaster_->sendTransform(static_transformStamped);

    RCLCPP_INFO_STREAM(
      get_logger(), "broadcast static tf. map_frame:"
                      << map_frame_ << ", viewer_frame:" << viewer_frame_ << ", x:" << coordinate_x
                      << ", y:" << coordinate_y << ", z:" << coordinate_z);
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(VectorMapTFGeneratorNode)
