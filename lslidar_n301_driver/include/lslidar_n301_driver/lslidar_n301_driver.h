/*
 * This file is part of lslidar_n301 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LSLIDAR_N301_DRIVER_H
#define LSLIDAR_N301_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <lslidar_n301_msgs/LslidarN301Packet.h>
#include <std_msgs/Byte.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/types.h>
#include <sys/stat.h>	
#include "input.h"
namespace lslidar_n301_driver {

//static uint16_t UDP_PORT_NUMBER = 8080;
//static uint16_t PACKET_SIZE = 1206;

class LslidarN301Driver {
public:

    LslidarN301Driver(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~LslidarN301Driver();

    bool initialize();
    bool polling();
	void difopPoll();
	void serialPoll();
	int lidarFileSave(char *ip,int lidar_serial_number,int port,std::string msg);
	char *getCurrentTime(char *curret_time);
  
    typedef boost::shared_ptr<LslidarN301Driver> LslidarN301DriverPtr;
    typedef boost::shared_ptr<const LslidarN301Driver> LslidarN301DriverConstPtr;

private:

    bool loadParameters();
    bool createRosIO();
    bool openUDPPort();
    int getPacket(lslidar_n301_msgs::LslidarN301PacketPtr& msg);
	int getDifopPacket(lslidar_n301_msgs::LslidarN301PacketPtr& msg);
	bool SendPacketToLidar(bool power_switch);
	
    boost::shared_ptr<Input> msop_input_;
    boost::shared_ptr<Input> difop_input_;

    // Ethernet relate variables
    std::string device_ip_string;
    in_addr device_ip;
    int UDP_PORT_NUMBER;
	int UDP_DIFOP_PORT_NUMBER;
	int agreement_type;
    int socket_id;
	int socket_id_difop;
	int lidar_serial_number;
	bool first_time;
	bool add_multicast;
	bool serial_switch;
	bool timeout_flags;
	unsigned char difop_data[1206];
	std::string group_ip;
	std::string serial_device;

    // ROS related variables
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string frame_id;
    ros::Publisher packet_pub;
	ros::Publisher type_pub;
	ros::Publisher difop_output_;
	
	ros::Publisher read_pub;
	ros::Publisher write_sub;
	
	boost::shared_ptr<boost::thread> difop_thread;
	boost::shared_ptr<boost::thread> serial_thread;
	
    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;

};

typedef LslidarN301Driver::LslidarN301DriverPtr LslidarN301DriverPtr;
typedef LslidarN301Driver::LslidarN301DriverConstPtr LslidarN301DriverConstPtr;

} // namespace lslidar_driver

#endif // _LSLIDAR_N301_DRIVER_H_
