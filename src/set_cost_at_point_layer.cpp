// set_cost_at_point_layer.cpp

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <costmap_2d/layer.hpp>
#include <costmap_2d/layered_costmap.hpp>

namespace costmap2d_plugin_test
{

class SetCostAtPointLayer : public costmap_2d::Layer
{
public:
  SetCostAtPointLayer()
    : x_(0.0), y_(0.0), cost_(255)
  {}

  virtual void onInitialize()
  {
    declareParameter("x", rclcpp::ParameterValue(0.0));
    declareParameter("y", rclcpp::ParameterValue(0.0));
    declareParameter("cost", rclcpp::ParameterValue(255));

    get_parameter("x", x_);
    get_parameter("y", y_);
    int cost;
    get_parameter("cost", cost);
    cost_ = static_cast<unsigned char>(cost);

    current_ = true;
    RCLCPP_INFO(node_->get_logger(), "SetCostAtPointLayer initialized");
  }

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y,
                            double* max_x, double* max_y)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    *min_x = std::min(*min_x, x_);
    *min_y = std::min(*min_y, y_);
    *max_x = std::max(*max_x, x_);
    *max_y = std::max(*max_y, y_);
  }

  virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
                           int min_i, int min_j, int max_i, int max_j)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    unsigned int mx, my;
    if (master_grid.worldToMap(x_, y_, mx, my))
    {
      master_grid.setCost(mx, my, cost_);
    }
  }

  virtual bool isClearable() { return false; }

private:
  double x_;
  double y_;
  unsigned char cost_;
  std::mutex mutex_;
};

}  // namespace costmap2d_plugin_test

PLUGINLIB_EXPORT_CLASS(costmap2d_plugin_test::SetCostAtPointLayer, costmap_2d::Layer)
