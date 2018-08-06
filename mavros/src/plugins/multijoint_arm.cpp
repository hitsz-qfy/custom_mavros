/**
 * @brief Altitude plugin
 * @file altitude.cpp
 * @author Andreas Antener <andreas@uaventure.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Andreas Antener <andreas@uaventure.com>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/MultijointArm.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief MultijointArm plugin.
 */
class MultijointArmPlugin : public plugin::PluginBase {
public:
	MultijointArmPlugin() : PluginBase(),
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		nh.param<std::string>("frame_id", frame_id, "map");
		multijoint_arm_sub = nh.subscribe("/out/multijoint_arm", 10, &MultijointArmPlugin::multijoint_arm_cb , this);

		mavros_multijoint_arm_pub = nh.advertise<mavros_msgs::MultijointArm>("multijoint_arm", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			make_handler(&MultijointArmPlugin::handle_multijoint_arm),
		};
	}

private:
	ros::NodeHandle nh;
	std::string frame_id;

	ros::Publisher mavros_multijoint_arm_pub;
	ros::Subscriber multijoint_arm_sub;
	//@Description: this function subcribe mavlink message from PX4 and convert it to ros message
	void handle_multijoint_arm(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MULTIJOINT_ARM &multijoint_arm_)
	{
		auto ros_msg = boost::make_shared<mavros_msgs::MultijointArm>();
		ros_msg->header = m_uas->synchronized_header(frame_id, multijoint_arm_.timestamp);

		ros_msg->joint_id = multijoint_arm_.joint_id;
		ros_msg->joint_mass = multijoint_arm_.joint_mass;
		ros_msg->joint_angle = multijoint_arm_.joint_angle;

		mavros_multijoint_arm_pub.publish(ros_msg);
	}

	//@Description: this callback function listen ros topic and convert it to mavlink message to PX4.
	void multijoint_arm_cb(const mavros_msgs::MultijointArm & ros_msg){
		mavlink::common::msg::MULTIJOINT_ARM multi_arm;

		multi_arm.timestamp = m_uas->get_tgt_system();
		multi_arm.joint_id = ros_msg.joint_id;
		multi_arm.joint_mass = ros_msg.joint_mass;
		multi_arm.joint_angle = ros_msg.joint_angle;

		UAS_FCU(m_uas)->send_message_ignore_drop(multi_arm);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::MultijointArmPlugin, mavros::plugin::PluginBase)
