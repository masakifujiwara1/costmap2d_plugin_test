// set_cost_at_point_layer.cpp

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

namespace costmap2d_plugin_test
{

class SetCostAtPointLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SetCostAtPointLayer()
    : nav2_costmap_2d::CostmapLayer(), x_(0.0), y_(0.0), cost_(255)
  {}

  virtual void onInitialize() override
  {
    nav2_costmap_2d::CostmapLayer::onInitialize();

    auto node = node_;
    if (node)
    {
      declareParameter("x", rclcpp::ParameterValue(0.0));
      declareParameter("y", rclcpp::ParameterValue(0.0));
      declareParameter("cost", rclcpp::ParameterValue(255));

      node->get_parameter(name_ + "." + "x", x_);
      node->get_parameter(name_ + "." + "y", y_);
      int cost;
      node->get_parameter(name_ + "." + "cost", cost);
      cost_ = static_cast<unsigned char>(cost);

      RCLCPP_INFO(logger_, "SetCostAtPointLayer initialized");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("SetCostAtPointLayer"), "Unable to lock node");
    }

    current_ = true;
  }

  virtual void updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                            double* min_x, double* min_y,
                            double* max_x, double* max_y) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    *min_x = std::min(*min_x, x_);
    *min_y = std::min(*min_y, y_);
    *max_x = std::max(*max_x, x_);
    *max_y = std::max(*max_y, y_);
  }

  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                           int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    unsigned int mx, my;
    if (master_grid.worldToMap(x_, y_, mx, my))
    {
      master_grid.setCost(mx, my, cost_);
    }
  }

  virtual bool isClearable() override { return false; }

  virtual void reset() override
  {
    // 必要に応じて実装内容を追加
  }

private:
  double x_;
  double y_;
  unsigned char cost_;
  std::mutex mutex_;
};

}  // namespace costmap2d_plugin_test

PLUGINLIB_EXPORT_CLASS(costmap2d_plugin_test::SetCostAtPointLayer, nav2_costmap_2d::Layer)
