/* Custom costmap pugin for marking predicted waypoints directly onto the costmap */

#include "my_costmap_plugin/path_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_path_costmap_plugin {

    PathLayer:PathLayer()
    : temp(0)
    {

    }

    void PathLayer::onInitialize() {
        auto node = node_.lock(); 
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + "." + "enabled", enabled_);
        
    }
}