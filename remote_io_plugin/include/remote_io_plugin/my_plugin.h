/*
  Copyright 2016 Lucas Walter
*/
#ifndef REMOTE_IO_PLUGIN_MY_PLUGIN_H
#define REMOTE_IO_PLUGIN_MY_PLUGIN_H

//#include <rqt_gui_cpp/plugin.h>
#include "/opt/ros/humble/include/rqt_gui_cpp/rqt_gui_cpp/plugin.h"
#include <ui_my_plugin.h>
#include <chrono>


#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

#include "rclcpp/rclcpp.hpp"

#include <QWidget>

namespace remote_io_plugin
{

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);


public slots:

  void gripper_trigger();
  void dispenser_trigger();    

  void update_gripper_time_ON();
  void update_vaccum_time_ON();

  void update_workcell_no();
  void gripper_suction_no();

  void clear_pins();

  void set_pin_on();
  void set_pin_off();

private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;

  int vaccum_on_time;
  int dispenser_on_time;


  void set_output(int pin_no, bool pin_value); // int label_size
  void io_status_cb(const control_msgs::msg::DynamicJointState::SharedPtr msg);
  void initialise_multi_array();

  //Trigger label offset
  void trigger_label_printer(int workcell);
  void set_gripper(int workcell, bool operation, int suction);


  void set_status_msg(std::string msg);
  void set_io_status_msg(std::string input_status_msg, std::string output_status_msg);


  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  double input_status[16] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double output_status[16] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  std_msgs::msg::Float64MultiArray output_msg;
  std_msgs::msg::MultiArrayDimension dim1;
  std_msgs::msg::MultiArrayDimension dim2;

  int workcell;
  int suction_value;


};
}  // namespace rqt_example_cpp
#endif  // RQT_EXAMPLE_CPP_MY_PLUGIN_H
