#include <elfin_ethercat_driver/elfin_ethercat_driver.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr elfin_ethercat_node = rclcpp::Node::make_shared("elfin_ethercat_node");
    elfin_ethercat_driver::EtherCatManager em("enp2s0");
    elfin_ethercat_driver::ElfinEtherCATDriver ed(&em, "elfin", elfin_ethercat_node);
    RCLCPP_INFO(elfin_ethercat_node->get_logger(),"elfin_ethercat_node start");
    rclcpp::spin(elfin_ethercat_node);
}