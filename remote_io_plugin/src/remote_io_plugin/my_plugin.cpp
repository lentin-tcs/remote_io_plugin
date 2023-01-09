/*
  Copyright 2016 Lucas Walter
*/

#include "remote_io_plugin/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace remote_io_plugin
{

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0),
  vaccum_on_time(0),
  dispenser_on_time(0),
  workcell(2),
  suction_value(2)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  node_ = this->node_ ;

  connect(ui_.pushButton_2, &QPushButton::released, this, &MyPlugin::gripper_trigger);
  connect(ui_.pushButton_3, &QPushButton::released, this, &MyPlugin::dispenser_trigger);

  //Vaccum time ON
  connect(ui_.spinBox_2, QOverload<int>::of(&QSpinBox::valueChanged), this, &MyPlugin::update_vaccum_time_ON);
  //Gripper time ON
  connect(ui_.spinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MyPlugin::update_gripper_time_ON);

  //Updating workcell no
  connect(ui_.spinBox_3, QOverload<int>::of(&QSpinBox::valueChanged), this, &MyPlugin::update_workcell_no);
  //Updating gripper suction
  connect(ui_.spinBox_4, QOverload<int>::of(&QSpinBox::valueChanged), this, &MyPlugin::gripper_suction_no);
  connect(ui_.pushButton_4, &QPushButton::released, this, &MyPlugin::clear_pins);

  connect(ui_.pushButton, &QPushButton::released, this, &MyPlugin::set_pin_on);
  connect(ui_.pushButton_5, &QPushButton::released, this, &MyPlugin::clear_pins);


  //publisher_ = this->node_->create_publisher<std_msgs::msg::String>("topic", 10);
  publisher_ = this->node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/hardware/gpio_command_controller/commands", 1);

  subscription_ = this->node_->create_subscription<control_msgs::msg::DynamicJointState>(
        "/hardware/gpio_command_controller/gpio_states", 1, std::bind(&MyPlugin::io_status_cb, this, std::placeholders::_1));


}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here

}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}


void MyPlugin::dispenser_trigger()
{
  std::cout << "Dispenser Triggered" << std::endl;
  ui_.label_7->setText("Triggered");
  this->trigger_label_printer(ui_.spinBox_3->value());

}


void MyPlugin::update_gripper_time_ON()
{

  std::cout << "Gripper: " << std::endl;
  std::cout << ui_.spinBox->value() << std::endl;
  dispenser_on_time = ui_.spinBox->value();

  ;
}

void MyPlugin::update_vaccum_time_ON()
{
  std::cout << "Vaccum: " << std::endl;
  std::cout << ui_.spinBox_2->value() << std::endl;
  vaccum_on_time = ui_.spinBox_2->value();
}

void MyPlugin::clear_pins()
{
      for(int i = 0;  i < 32; i++)
      {
        this->set_output(i,false);
      }
      this->set_status_msg("Cleared all PINS");
}

/////////////////////////////////////////////////////

void MyPlugin::update_workcell_no()
{
  std::cout << "Workcell : " << std::endl;
  std::cout << ui_.spinBox_3->value() << std::endl;
  workcell = ui_.spinBox_3->value();
}

void MyPlugin::gripper_suction_no()
{
  std::cout << "Gripper : " << std::endl;
  std::cout << ui_.spinBox_4->value() << std::endl;
  suction_value = ui_.spinBox_4->value();
}


/////////////////////////////////////////////////////////////////////////

void MyPlugin::set_output(int pin_no, bool pin_value) // int label_size
  {
    int max_attempt = 10;
    while(max_attempt)
    {
      if(pin_value == output_status[pin_no])
      {break;}
      output_msg.data.clear();
      for(int i = 0;  i < 32; i++)
      { 
        if(i < 16)
        {output_msg.data.push_back(0);}
        else
        {
          if(i == (pin_no + 16))
          {output_msg.data.push_back(pin_value);}
          else
          {output_msg.data.push_back(output_status[i - 16]);}
        }
      }
      publisher_->publish(output_msg);
      --max_attempt;
      std::this_thread::sleep_for(std::chrono::microseconds(10*1000));
    }
  }


void MyPlugin::io_status_cb(const control_msgs::msg::DynamicJointState::SharedPtr msg)
  {
    
    std::string input_msg = "Input_status : ";
    std::string output_msg = "Output_status : ";

    for(int i = 0; i < 16; i++) {
      input_status[i] = msg->interface_values[0].values[i];
      input_msg+=std::to_string(input_status[i])+",";
    }
    for(int i = 0; i < 16; i++) {
      output_status[i] = msg->interface_values[1].values[i];
      output_msg+=std::to_string(output_status[i])+",";

    }
    // RCLCPP_WARN(this->get_logger(), "hi!");
    //Show the I/O status
    this->set_io_status_msg(input_msg,output_msg);

  }

void MyPlugin::initialise_multi_array()
  {
    output_msg.layout.dim.clear();
    dim1.label = "gpio_num";
    dim1.size = 2;
    dim1.stride = 32;
    output_msg.layout.dim.push_back(dim1);
    dim2.label = "io_num";
    dim2.size = 16;
    dim2.stride = 16;
    output_msg.layout.dim.push_back(dim2);
    output_msg.layout.data_offset = 0;
  }


void MyPlugin::trigger_label_printer(int workcell)
{


          if(workcell == 2)
          {
            this->set_output(4,true);  //Set 4th PIN(ZEBRA MOTOR RUN) to true
            std::this_thread::sleep_for(std::chrono::microseconds(500*1000));

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Input status 5: %f", input_status[5]);
            if(input_status[5])
            {
            this->set_output(2,true);  //Set 2nd(ZEBRA START PRINT) PIN to true
            std::this_thread::sleep_for(std::chrono::microseconds(20*1000));
            this->set_output(2,false);  //Set 2nd PIN to true
            std::this_thread::sleep_for(std::chrono::microseconds(3000*1000));

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Input status 7: %f", input_status[7]);
            if(input_status[7])
            {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "label is at blower. everything ok");
            this->set_status_msg("label is at blower. everything ok");



            }
            else
            {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "label is not in the blower");
            this->set_status_msg("label is not in the blower");


            }
            }
            else
            {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "printer is not ready");
            this->set_status_msg("printer is not ready");

            }
          }
          else if(workcell == 3)
          {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Input status 3: %f", input_status[3]);
            if(!input_status[3])
            {
              this->set_output(8,true);  //Set 2nd(ZEBRA START PRINT) PIN to true
              std::this_thread::sleep_for(std::chrono::microseconds(100*1000));  //100 ms
              this->set_output(8,false);
              std::this_thread::sleep_for(std::chrono::microseconds(1000*1000));  //1000 ms
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Input status 3: %f", input_status[3]);
            if(input_status[3])
            {
              if(input_status[1])  //RET position
              {
                this->set_output(7,true);  //Set 2nd(ZEBRA START PRINT) PIN to true
                std::this_thread::sleep_for(std::chrono::microseconds(50*1000));  //100 ms
                this->set_output(7,false);  //Set 2nd PIN to true

                std::this_thread::sleep_for(std::chrono::microseconds(2000*1000));  //wait for 0.5 sec

                if(input_status[2])
                {
                  this->set_output(0,true);  //Set 2nd(ZEBRA START PRINT) PIN to true
                  std::this_thread::sleep_for(std::chrono::microseconds(50*1000));  //100 ms
                  this->set_output(0,false);

                  std::this_thread::sleep_for(std::chrono::microseconds(2000*1000));  //wait for 0.5 sec

                  if(input_status[0])
                  {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cylinder is at EXT position with label on the blower");
                    this->set_status_msg("cylinder is at EXT position with label on the blower");

                    this->set_output(1,true);  //Set 2nd(ZEBRA START PRINT) PIN to true
                    std::this_thread::sleep_for(std::chrono::microseconds(50*1000));  //100 ms
                    this->set_output(1,false);
                  }
                  else
                  {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cyliner did not reach EXT position");
                    this->set_status_msg("cyliner did not reach EXT position");

                  }
                }
                else
                {
                   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "label is not at the blower");
                   this->set_status_msg("label is not at the blower");


                }
              }
              else
              {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cyliner is not at RET position");
                this->set_status_msg("cyliner is not at RET position");

              }
            }
            else
            {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "label is not ready");
            this->set_status_msg("label is not ready");

            }
          }
          else
          {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "wrong workcell input");
            this->set_status_msg("wrong workcell input");

          }


}

void MyPlugin::set_status_msg(std::string msg)
{
    QString str = QString::fromUtf8(msg.c_str());
    ui_.label_12->setText(str);

}

void MyPlugin::set_io_status_msg(std::string input_status_msg, std::string output_status_msg)
{
    QString str = QString::fromUtf8(output_status_msg.c_str());
    ui_.label_13->setText(str);

    QString str1 = QString::fromUtf8(input_status_msg.c_str());
    ui_.label_16->setText(str1);
   

}

void MyPlugin::gripper_trigger()
{

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper Trigger");

    int workcell = ui_.spinBox_3->value();
    bool operation = ui_.radioButton->isChecked();  //1 is for pick and 0 for place
    int suction = ui_.spinBox_4->value();

    this->set_gripper(workcell, operation, suction);
}

void MyPlugin::set_gripper(int workcell, bool operation, int suction)
{


    if (workcell == 2)
    {
      //Pick
      if (operation == 1.0)
      {

        ui_.label_6->setText("Pick");
  
        switch (suction)
        {
        case (1):
          this->set_output(9, true);
          break;
        case (2):
          this->set_output(9, true);
          this->set_output(10, true);
          break;
        case (3):
          this->set_output(9, true);
          this->set_output(10, true);
          this->set_output(11, true);
          break;
        default:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Invalid Label size.");
          this->set_status_msg("Invalid Label size.");
          break;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Label picked Successfully.");
        this->set_status_msg("Label picked Successfully. ");
 
      }
      // Place

      
      else if (operation == 0.0)
      {

        ui_.label_6->setText("Place");
      
        this->set_output(9, false);
        this->set_output(10, false);
        this->set_output(11, false);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Label placed Successfully.");
        this->set_status_msg("Label placed successfully. ");
     
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Invalid Goal request.");
        this->set_status_msg("Invalid Goal request. ");

      }
    }

    
    else if (workcell == 3)
    {
      if (operation == 1.0)
      {

        ui_.label_6->setText("Pick");
    
        switch (suction)
        {
        case (1):
          this->set_output(4, true);
          break;
        case (2):
          this->set_output(4, true);
          this->set_output(5, true);
          break;
        case (3):
          this->set_output(4, true);
          this->set_output(5, true);
          this->set_output(6, true);
          break;
        default:
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Invalid Label size.");
          this->set_status_msg("Invalid Label size.");
          break;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Label picked Successfully.");
        this->set_status_msg("Label picked Successfully.");

      }
      // Place
      else if (operation == 0.0)
      {

        ui_.label_6->setText("Place");
      
        this->set_output(4, false);
        this->set_output(5, false);
        this->set_output(6, false);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Label placed Successfully.");
        this->set_status_msg("Label placed Successfully.");

      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Invalid Goal request.");
        this->set_status_msg("Invalid Goal request.");

      }
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "wrong workcell input");
      this->set_status_msg("wrong workcell input.");

    }


  ;
}

void MyPlugin::set_pin_on()
{
    int pin_no = ui_.spinBox_5->value();
    this->set_output(pin_no, true);

    this->set_status_msg("Set pin no to true: "+ std::to_string(pin_no));

}
void MyPlugin::set_pin_off()
{

    int pin_no_1 = ui_.spinBox_5->value();
    this->set_output(pin_no_1, false);

    this->set_status_msg("Set pin no to false: "+ std::to_string(pin_no_1));

}


} 

/////////////////////////////////////////////////////////////////////////////
// PLUGINLIB_DECLARE_CLASS(rqt_example_cpp, MyPlugin, rqt_example_cpp::MyPlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(remote_io_plugin::MyPlugin, rqt_gui_cpp::Plugin)
