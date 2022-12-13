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

#include <lslidar_n301_decoder/lslidar_n301_decoder.h>

using namespace std;

namespace lslidar_n301_decoder {
LslidarN301Decoder::LslidarN301Decoder(
        ros::NodeHandle& n, ros::NodeHandle& pn):
    nh(n),
    pnh(pn),
    publish_point_cloud(false),
    use_gps_ts(false),
    is_first_sweep(true),
	first_gps(true),
    last_azimuth(0.0),
    sweep_start_time(0.0),
    packet_start_time(0.0),
	error_time(0),
	correct_time(0),
	scan_size(16),
    sweep_data(new lslidar_n301_msgs::LslidarN301Sweep()){
    return;
}

bool LslidarN301Decoder::loadParameters() {
    pnh.param<int>("point_num", point_num, 300000);
    pnh.param<double>("min_range", min_range, 0.3);
    pnh.param<double>("max_range", max_range, 100.0);
    pnh.param<int>("start_angle", start_angle,0);
    pnh.param<int>("end_angle", end_angle, 360);
    pnh.param<double>("frequency", frequency, 10.0);
    pnh.param<bool>("publish_point_cloud", publish_point_cloud, false);
    pnh.param<bool>("use_gps_ts", use_gps_ts, false);
    pnh.param<string>("fixed_frame_id", fixed_frame_id, "map");
    pnh.param<string>("child_frame_id", child_frame_id, "lslidar");
    pnh.param<bool>("filter_scan_point", filter_scan_point, false);
	pnh.param<bool>("gps_correct", gps_correct, true);
    angle_base = M_PI*2 / point_num;
    pnh.param<int>("agreement_type", agreement_type, 1);
    while(end_angle<0)          end_angle +=360;
    while(end_angle>360)        end_angle -=360;
    while(start_angle<0)        start_angle +=360;
    while(start_angle>360)      start_angle -=360;
    if(start_angle>end_angle)   end_angle +=360;
    ROS_WARN("Using GPS timestamp or not %d", use_gps_ts);
    return true;
}

bool LslidarN301Decoder::createRosIO() {
	//type_sub = nh.subscribe("agreement_type", 100, &LslidarN301Decoder::TypeReceive, this);
	difop_sub_ = nh.subscribe("lslidar_packet_difop", 10, &LslidarN301Decoder::processDifop, (LslidarN301Decoder*)this);
    packet_sub = nh.subscribe<lslidar_n301_msgs::LslidarN301Packet>(
                "lslidar_packet", 100, &LslidarN301Decoder::packetCallback, this, ros::TransportHints().tcpNoDelay(true));
    sweep_pub = nh.advertise<lslidar_n301_msgs::LslidarN301Sweep>(
                "lslidar_sweep", 10);
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
                "lslidar_point_cloud", 10);
    scan_pub = nh.advertise<sensor_msgs::LaserScan>(
                "scan", 100);
	device_pub = nh.advertise<lslidar_n301_msgs::LslidarN301Difop>(
                "difop_information", 100);

    return true;
}

bool LslidarN301Decoder::initialize() {
    if (!loadParameters()) {
        ROS_ERROR("Cannot load all required parameters...");
        return false;
    }

    if (!createRosIO()) {
        ROS_ERROR("Cannot create ROS I/O...");
        return false;
    }
    TypeReceive();
    // Fill in the altitude for each scan.
    for (size_t scan_idx = 0; scan_idx < 16; ++scan_idx) {
        size_t remapped_scan_idx = scan_idx%2 == 0 ? scan_idx/2 : scan_idx/2+8;
        sweep_data->scans[remapped_scan_idx].altitude = scan_altitude[scan_idx];
    }

    // Create the sin and cos table for different azimuth values.
    for (size_t i = 0; i < 6300; ++i) {
        double angle = static_cast<double>(i) / 1000.0;
        cos_azimuth_table[i] = cos(angle);
        sin_azimuth_table[i] = sin(angle);
    }

    return true;
}

void LslidarN301Decoder::TypeReceive() {
			printf("agreement_type = %d\n",agreement_type);
			if(agreement_type == 2)
			{
				first_gps = true;
                scan_size = 1;
			}
			else
			{
				DISTANCE_RESOLUTION = 0.004;
                DSR_TOFFSET       = 1;   // [µs]
	            FIRING_TOFFSET    = 15;  // [µs]
	            scan_size = 15;
				//ROS_INFO("The agreement type is: %d", agreement_type);
			}
}

void LslidarN301Decoder::processDifop(const lslidar_n301_msgs::LslidarN301Packet::ConstPtr& difop_msg)
{
	  const uint8_t* data = &difop_msg->data[0];
	  
	  Difop_data = lslidar_n301_msgs::LslidarN301DifopPtr(
							new lslidar_n301_msgs::LslidarN301Difop());
							
	  // check header
	  if (data[0] != 0xa5 || data[1] != 0xff || data[2] != 0x00 || data[3] != 0x5a)
	  {
			return;
	  }
	
	if(data[160] == 0x11 && data[161] == 0x22 && data[162] == 0x33 && data[163] == 0x44)
	{
		Difop_data->rpm = (data[8]<<8) | data[9];
		Difop_data->temperature = data[80]*256 + data[81] - 50;
	}
	else{
	     // rpm
		 Difop_data->rpm = (data[20]<<8) | data[21];
		 Difop_data->temperature = (data[28]*256 + data[29]) * 330 / 4096 - 50;
		 
		 /*if(data[31] == 1)
			Difop_data->power_state = "power on";
		 else
			Difop_data->power_state = "power down";*/
	}
	
	device_pub.publish(Difop_data);
}

bool LslidarN301Decoder::checkPacketValidity(const RawPacket* packet) {
    for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
        if (packet->blocks[blk_idx].header != UPPER_BANK) {
            ROS_WARN("Skip invalid LS-16 packet: block %lu header is %x",
                     blk_idx, packet->blocks[blk_idx].header);
            printf("%x,%x,%x,%X",packet->blocks[blk_idx].data[0],packet->blocks[blk_idx].data[1],packet->blocks[blk_idx].data[2],packet->blocks[blk_idx].data[3]);
            return false;
        }
    }
    return true;
}


void LslidarN301Decoder::publishPointCloud() {

    VPointCloud::Ptr point_cloud(new VPointCloud());
    double timestamp = sweep_data->header.stamp.toSec();
    point_cloud->header.stamp =
            pcl_conversions::toPCL(sweep_data->header).stamp;
    point_cloud->header.frame_id = child_frame_id;
    point_cloud->height = 1;
		
    for (size_t i = 0; i < scan_size; ++i) {
        const lslidar_n301_msgs::LslidarN301Scan& scan = sweep_data->scans[i];
        // The first and last point in each scan is ignored, which
        // seems to be corrupted based on the received data.
        // TODO: The two end points should be removed directly
        //    in the scans.
        if (scan.points.size() == 0) continue;

        for (size_t j = 1; j < scan.points.size()-1; ++j) {
            VPoint point;
            point.timestamp = timestamp - (scan.points.size()-1 - j)*0.05;
            point.x = scan.points[j].x;
            point.y = scan.points[j].y;
            point.z = scan.points[j].z;
            point.intensity = scan.points[j].intensity;
            point_cloud->points.push_back(point);
            ++point_cloud->width;
        }
    }

    //  	if(point_cloud->width > 2000)
    {
        point_cloud_pub.publish(point_cloud);
    }
	
    return;
}

void LslidarN301Decoder::publishScan()
{
    sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);

    if(sweep_data->scans[0].points.size() <= 1)
        return;
	
	uint16_t scan_sizes = 1;
    int scan_num = 0;		   
    scan_num = ceil(point_num*(end_angle - start_angle)/360 + 1);

    scan->header.frame_id = child_frame_id;
	
	if (use_gps_ts){
		scan->header.stamp = sweep_data->header.stamp;
	}
	else{
		scan->header.stamp = ros::Time::now();
	}
		
    scan->header.stamp = sweep_data->header.stamp;  // timestamp will obtained from sweep data stamp
	
    //scan->time_increment = 0.00005;
    //scan->scan_time= 0.1;
	if(end_angle >360){
        scan->angle_min = 2.0*M_PI*(start_angle-360)/360;
        scan->angle_max = 2.0*M_PI*(end_angle-360)/360;    
    }
    else{
        scan->angle_min = 2.0*M_PI*start_angle/360;
        scan->angle_max = 2.0*M_PI*end_angle/360;
    }
    scan->angle_increment = (scan->angle_max - scan->angle_min)/scan_num;
    scan->range_min = min_range;
    scan->range_max = max_range;
    scan->ranges.reserve(scan_num);
    scan->ranges.assign(scan_num, std::numeric_limits<float>::infinity());
    scan->intensities.reserve(scan_num);
    scan->intensities.assign(scan_num, std::numeric_limits<float>::infinity());
		
	if(agreement_type)
		scan_sizes = 15;

    for(uint16_t j=0; j < scan_sizes; j++)
    {
        for(uint16_t i = 0; i < sweep_data->scans[j].points.size(); i++)
        {
            int point_idx = round(sweep_data->scans[j].points[i].azimuth / angle_base);
            int start_num = start_angle * point_num / 360;
            int end_num = end_angle * point_num / 360;
            if(end_angle>360 && point_idx < (end_num-point_num))          point_idx += point_num;
            point_idx = point_idx - start_num;
            if (point_idx >= 0 && point_idx < scan_num){   
                if (sweep_data->scans[j].points[i].distance == 0.0) {
                    scan->ranges[point_idx] = std::numeric_limits<float>::infinity();
                    scan->intensities[point_idx] = 0;
                }
                else{
                    scan->ranges[point_idx] = sweep_data->scans[j].points[i].distance;
                    scan->intensities[point_idx] = sweep_data->scans[j].points[i].intensity;
                }
            }
        }
    }
    scan_pub.publish(scan);
}

point_struct LslidarN301Decoder::getMeans(std::vector<point_struct> clusters)
{
    point_struct tmp;
    int num = clusters.size();
    if (num == 0)
    {
        tmp.distance = std::numeric_limits<float>::infinity();
        tmp.intensity = std::numeric_limits<float>::infinity();
    }
    else
    {
        double mean_distance = 0, mean_intensity = 0;

        for (int i = 0; i < num; i++)
        {
            mean_distance += clusters[i].distance;
            mean_intensity += clusters[i].intensity;
        }

        tmp.distance = mean_distance / num;
        tmp.intensity = mean_intensity / num;
    }
    return tmp;
}

 uint64_t LslidarN301Decoder::get_gps_stamp(struct tm t){

   uint64_t ptime =static_cast<uint64_t>(timegm(&t));
   return ptime;
}

void LslidarN301Decoder::decodePacket(const RawPacket* packet) {

    // Compute the azimuth angle for each firing.
    for (size_t fir_idx = 0; fir_idx < FIRINGS_PER_PACKET; fir_idx+=2) {
        size_t blk_idx = fir_idx / 2;
        firings[fir_idx].firing_azimuth = rawAzimuthToDouble(
                    packet->blocks[blk_idx].rotation);                  //弧度值
    }

    // Interpolate the azimuth values
    for (size_t fir_idx = 1; fir_idx < FIRINGS_PER_PACKET; fir_idx+=2) {
        size_t lfir_idx = fir_idx - 1;
        size_t rfir_idx = fir_idx + 1;

        double azimuth_diff;
        if (fir_idx == FIRINGS_PER_PACKET - 1) {
            lfir_idx = fir_idx - 3;
            rfir_idx = fir_idx - 1;
        }

        azimuth_diff = firings[rfir_idx].firing_azimuth -
                firings[lfir_idx].firing_azimuth;
        azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 2*M_PI : azimuth_diff;
        azimuth_diff = azimuth_diff > 2*M_PI ? azimuth_diff - 2*M_PI : azimuth_diff;

        firings[fir_idx].firing_azimuth =
              firings[fir_idx-1].firing_azimuth + azimuth_diff/2.0;

        firings[fir_idx].firing_azimuth = firings[fir_idx].firing_azimuth > 2*M_PI ?
              firings[fir_idx].firing_azimuth-2*M_PI : firings[fir_idx].firing_azimuth;

        firings[fir_idx].firing_azimuth = firings[fir_idx].firing_azimuth < 0 ?
              firings[fir_idx].firing_azimuth + 2*M_PI:firings[fir_idx].firing_azimuth;
    }

    // Fill in the distance and intensity for each firing.
    for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
        const RawBlock& raw_block = packet->blocks[blk_idx];

        for (size_t blk_fir_idx = 0; blk_fir_idx < FIRINGS_PER_BLOCK; ++blk_fir_idx){
            size_t fir_idx = blk_idx*FIRINGS_PER_BLOCK + blk_fir_idx;

            double azimuth_diff = 0.0;
            if (fir_idx < FIRINGS_PER_PACKET - 1){
                azimuth_diff = firings[fir_idx+1].firing_azimuth -
                        firings[fir_idx].firing_azimuth;
            }else{
                azimuth_diff = firings[fir_idx].firing_azimuth -
                        firings[fir_idx-1].firing_azimuth;
            }
            azimuth_diff = azimuth_diff > 2*M_PI ? azimuth_diff - 2*M_PI : azimuth_diff;
            azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 2*M_PI : azimuth_diff;
			

            for (size_t scan_fir_idx = 0; scan_fir_idx < scan_size; ++scan_fir_idx){
                size_t byte_idx = RAW_SCAN_SIZE * (
                            15*blk_fir_idx + scan_fir_idx);
				
				if( blk_fir_idx == 1)
					byte_idx = byte_idx + 3;
				// printf("scan_fir_idx =%d\n",scan_fir_idx);
                // printf("byte_idx =%d\n",byte_idx);
                // Azimuth
                firings[fir_idx].azimuth[scan_fir_idx] =  firings[fir_idx].firing_azimuth +
                        (scan_fir_idx*DSR_TOFFSET/FIRING_TOFFSET) * azimuth_diff;
                firings[fir_idx].azimuth[scan_fir_idx] = firings[fir_idx].azimuth[scan_fir_idx] > 2*M_PI ?
                      firings[fir_idx].azimuth[scan_fir_idx] - 2*M_PI : firings[fir_idx].azimuth[scan_fir_idx];
                firings[fir_idx].azimuth[scan_fir_idx] = firings[fir_idx].azimuth[scan_fir_idx] < 0 ?
                      firings[fir_idx].azimuth[scan_fir_idx] + 2*M_PI : firings[fir_idx].azimuth[scan_fir_idx];
                // for(int i = 0;i<96;i++)
                //     printf("raw_block.data[%d]=%x\n",i,raw_block.data[i]);
                // Distance
                TwoBytes raw_distance;
                raw_distance.bytes[0] = raw_block.data[byte_idx];
                raw_distance.bytes[1] = raw_block.data[byte_idx+1];
                //printf("raw_distance.bytes[0] = %x raw_distance.bytes[1] = %x\n",raw_distance.bytes[0],raw_distance.bytes[1]);

                firings[fir_idx].distance[scan_fir_idx] = static_cast<double>(
                            raw_distance.distance) * DISTANCE_RESOLUTION;

                // Intensity
                firings[fir_idx].intensity[scan_fir_idx] = static_cast<double>(
                            raw_block.data[byte_idx+2]);
            }

            //extract gps_time
			if(agreement_type)
			{
				pTime.tm_year = raw_block.data[45]+2000-1900;
				pTime.tm_mon = raw_block.data[46]-1;
				pTime.tm_mday = raw_block.data[47];
				pTime.tm_hour = raw_block.data[93];
				pTime.tm_min = raw_block.data[94];
				pTime.tm_sec = raw_block.data[95];
			}
			else
			{
				if(raw_block.data[45])
				{
					pTime.tm_year = raw_block.data[45]+2000-1900;
					pTime.tm_mon = raw_block.data[46]-1;
					pTime.tm_mday = raw_block.data[47];
					pTime.tm_hour = raw_block.data[93];
					pTime.tm_min = raw_block.data[94];
					pTime.tm_sec = raw_block.data[95];
				}
				else
				{
					pTime.tm_year = raw_block.data[90]+2000-1900;
					pTime.tm_mon = raw_block.data[91]-1;
					pTime.tm_mday = raw_block.data[92];
					pTime.tm_hour = raw_block.data[93];
					pTime.tm_min = raw_block.data[94];
					pTime.tm_sec = raw_block.data[95];
				}
			}
        }
    }

    // resolve the timestamp in the end of packet
    packet_timestamp = (packet->time_stamp_yt[0]  +
                        packet->time_stamp_yt[1] * pow(2, 8) +
                        packet->time_stamp_yt[2] * pow(2, 16) +
                        packet->time_stamp_yt[3] * pow(2, 24)) * 1e3;
    // ROS_DEBUG("nsec part: %lu", packet_timestamp);

	
    return;
}

void LslidarN301Decoder::packetCallback(
        const lslidar_n301_msgs::LslidarN301PacketConstPtr& msg) {

    // Convert the msg to the raw packet type.
    const RawPacket* raw_packet = (const RawPacket*) (&(msg->data[0]));
	
    // Check if the packet is valid+
    if (!checkPacketValidity(raw_packet)) return;
	
    // Decode the packet
    decodePacket(raw_packet);

    // Find the start of a new revolution
    //    If there is one, new_sweep_start will be the index of the start firing,
    //    otherwise, new_sweep_start will be FIRINGS_PER_PACKET. FIRINGS_PER_PACKET = 24
    size_t new_sweep_start = 0;
    do {

        if (fabs(firings[new_sweep_start].firing_azimuth - last_azimuth) > M_PI) break;
        else {
            last_azimuth = firings[new_sweep_start].firing_azimuth;
            ++new_sweep_start;
        }
    } while (new_sweep_start < FIRINGS_PER_PACKET);

    // the first sweep will be discarded. We will wait for the
    // second sweep in order to find the 0 azimuth angle.
    size_t start_fir_idx = 0;
    size_t end_fir_idx = new_sweep_start;
    if (is_first_sweep &&
            new_sweep_start == FIRINGS_PER_PACKET) {
        // The first sweep has not ended yet.
        return;
    } else {
        if (is_first_sweep) {
            is_first_sweep = false;
            start_fir_idx = new_sweep_start;
            end_fir_idx = FIRINGS_PER_PACKET;
            sweep_start_time = get_gps_stamp(pTime);
        }
    }

    int start_packet;
    int end_packet;
    if(filter_scan_point){
        start_packet = start_fir_idx;
        end_packet = end_fir_idx;
    }else{
        start_packet = 0;
        end_packet = FIRINGS_PER_PACKET;
    }
	
	if (end_fir_idx != FIRINGS_PER_PACKET || last_azimuth > 2*M_PI - 1)
	{
		sweep_data1 = lslidar_n301_msgs::LslidarN301SweepPtr(
							new lslidar_n301_msgs::LslidarN301Sweep());
	}
	
    for (size_t fir_idx = start_packet; fir_idx < end_packet; ++fir_idx) {
        for (size_t scan_idx = 0; scan_idx < scan_size; ++scan_idx) {
			if(firings[fir_idx].intensity[scan_idx] < 1) continue;
			if(firings[fir_idx].intensity[scan_idx] < 3)  firings[fir_idx].intensity[scan_idx] = 15;
            // Check if the point is valid.
            //if (!isPointInRange(firings[fir_idx].distance[scan_idx])) continue;

            // Convert the point to xyz coordinate
            size_t table_idx = floor(firings[fir_idx].azimuth[scan_idx]*1000.0+0.5);

            double cos_azimuth = cos_azimuth_table[table_idx];
            double sin_azimuth = sin_azimuth_table[table_idx];

            double x = firings[fir_idx].distance[scan_idx] * sin_azimuth;
            double y = firings[fir_idx].distance[scan_idx] * cos_azimuth;
            double z = 0;

            double x_coord = y;
            double y_coord = -x;
            double z_coord = z;

            // get timestamp from gps and hardware instead
            double time = sweep_end_time_gps*1.0 + sweep_end_time_hardware*1e-9;

            // Remap the index of the scan
            int remapped_scan_idx = scan_idx%2 == 0 ? scan_idx/2 : scan_idx/2+8;
			
		    //lslidar_n301_msgs::LslidarN301Point& new_point;
			if ((end_fir_idx != FIRINGS_PER_PACKET || last_azimuth > 2*M_PI - 1)&& firings[fir_idx].azimuth[scan_idx] < M_PI )
			{
					sweep_data1->scans[remapped_scan_idx].points.push_back(
                        lslidar_n301_msgs::LslidarN301Point());
						
					lslidar_n301_msgs::LslidarN301Point& new_point =		// new_point 为push_back最后一个的引用
						sweep_data1->scans[remapped_scan_idx].points[
						sweep_data1->scans[remapped_scan_idx].points.size()-1];
						
					// Pack the data into point msg
					new_point.time = time;
					new_point.x = x_coord;
					new_point.y = y_coord;
					new_point.z = z_coord;
					new_point.azimuth =2*M_PI - firings[fir_idx].azimuth[scan_idx];
					new_point.distance = firings[fir_idx].distance[scan_idx];
					new_point.intensity = firings[fir_idx].intensity[scan_idx];
			}
			else
			{
					sweep_data->scans[remapped_scan_idx].points.push_back(
								lslidar_n301_msgs::LslidarN301Point());

					lslidar_n301_msgs::LslidarN301Point& new_point =		// new_point 为push_back最后一个的引用
							sweep_data->scans[remapped_scan_idx].points[
							sweep_data->scans[remapped_scan_idx].points.size()-1];
							
					// Pack the data into point msg
					new_point.time = time;
					new_point.x = x_coord;
					new_point.y = y_coord;
					new_point.z = z_coord;
					new_point.azimuth =2*M_PI - firings[fir_idx].azimuth[scan_idx];
					new_point.distance = firings[fir_idx].distance[scan_idx];
					new_point.intensity = firings[fir_idx].intensity[scan_idx];
			}
        }
    }

    packet_start_time += FIRING_TOFFSET * (end_fir_idx-start_fir_idx);
	
    // A new sweep begins
    if (end_fir_idx != FIRINGS_PER_PACKET) {
		
        // Publish the last revolution
        sweep_end_time_gps = get_gps_stamp(pTime);
        sweep_end_time_hardware = packet_timestamp%1000000000;

		//GPS time correction function
		if (use_gps_ts && gps_correct)
		{
			if(first_gps)
			{
				last_timestamp = sweep_end_time_gps * 1000000000 + sweep_end_time_hardware - 100000000;
				first_gps = false;
				correct_gps = true;
			}
			
			if(correct_time >= 100)
				correct_gps = false;
			
			if(error_time >= 50 && correct_gps)  
			{
				error_time = 0;
				last_timestamp = sweep_end_time_gps * 1000000000 + sweep_end_time_hardware - 100000000;
			}
				
			uint64_t Diff_calcule = sweep_end_time_gps * 1000000000 + sweep_end_time_hardware - last_timestamp;
			if(Diff_calcule < 30000000 || Diff_calcule > 300000000 || last_timestamp < 1600000000000000000)
			{
				correct_time = 0;
				error_time++;
				sweep_end_time_gps = (last_timestamp - last_timestamp % 1000000000) / 1000000000;

				 if (fabs(sweep_end_time_gps * 1000000000 + sweep_end_time_hardware - last_timestamp) > 500000000) 
				{
					sweep_end_time_gps = sweep_end_time_gps + 1;
					Diff_calcule = sweep_end_time_gps * 1000000000 + sweep_end_time_hardware - last_timestamp;
				}
			}
			else
			{
				correct_time++;
				error_time = 0;
			}
			if(error_time >= 300)
			{
				error_time = 0;
				first_gps = true;
			}
		}
		
        sweep_data->header.frame_id = "sweep";
        if (use_gps_ts){
            sweep_data->header.stamp = ros::Time(sweep_end_time_gps, sweep_end_time_hardware);
        }
        else{
            sweep_data->header.stamp = ros::Time::now();
        }
		
		/*if(sweep_end_time_gps * 1000000000 + sweep_end_time_hardware - last_timestamp > 300000000)
		{
			ROS_WARN("last time stamp is %ld", last_timestamp);
			ROS_WARN("time stamp is %ld", sweep_end_time_gps * 1000000000 + sweep_end_time_hardware);
		}
		else if(sweep_end_time_gps * 1000000000 + sweep_end_time_hardware - last_timestamp < 30000000)
		{
			ROS_WARN("last time stamp is %ld", last_timestamp);
			ROS_WARN("time stamp is %ld", sweep_end_time_gps * 1000000000 + sweep_end_time_hardware);
		}*/
		last_timestamp = sweep_end_time_gps * 1000000000 + sweep_end_time_hardware;
		
        sweep_pub.publish(sweep_data);
		
        if (publish_point_cloud) publishPointCloud();
		
        publishScan();
			
        sweep_data = lslidar_n301_msgs::LslidarN301SweepPtr(
                    new lslidar_n301_msgs::LslidarN301Sweep());
					
		sweep_data = sweep_data1;
        // Prepare the next revolution
        sweep_start_time = sweep_end_time_gps * 1.0 + sweep_end_time_hardware*1e-9;

        packet_start_time = 0.0;
        last_azimuth = firings[FIRINGS_PER_PACKET-1].firing_azimuth;

        start_fir_idx = end_fir_idx;
        end_fir_idx = FIRINGS_PER_PACKET;

        int start_packet;
        int end_packet;
        if(filter_scan_point){
            start_packet = start_fir_idx;
            end_packet = end_fir_idx;
        }else{
            start_packet = 0;
            end_packet = FIRINGS_PER_PACKET;
        }


        for (size_t fir_idx = start_packet; fir_idx < end_packet; ++fir_idx) {
            for (size_t scan_idx = 0; scan_idx < scan_size; ++scan_idx) {
				if(firings[fir_idx].intensity[scan_idx] < 3) continue;
				//if(firings[fir_idx].intensity[scan_idx] < 3)  firings[fir_idx].intensity[scan_idx] = 15;
                // Check if the point is valid.
               // if (!isPointInRange(firings[fir_idx].distance[scan_idx])) continue;

                // Convert the point to xyz coordinate
                size_t table_idx = floor(firings[fir_idx].azimuth[scan_idx]*1000.0+0.5);

                double cos_azimuth = cos_azimuth_table[table_idx];
                double sin_azimuth = sin_azimuth_table[table_idx];

                double x = firings[fir_idx].distance[scan_idx] * sin_azimuth;
                double y = firings[fir_idx].distance[scan_idx] * cos_azimuth;
                double z = 0;

                double x_coord = y;
                double y_coord = -x;
                double z_coord = z;

                // get timestamp from gps and hardware instead
                double time = sweep_end_time_gps*1.0 + sweep_end_time_hardware*1e-9;

                // Remap the index of the scan
                int remapped_scan_idx = scan_idx%2 == 0 ? scan_idx/2 : scan_idx/2+8;
                sweep_data->scans[remapped_scan_idx].points.push_back(
                            lslidar_n301_msgs::LslidarN301Point());
                lslidar_n301_msgs::LslidarN301Point& new_point =
                        sweep_data->scans[remapped_scan_idx].points[
                        sweep_data->scans[remapped_scan_idx].points.size()-1];

                // Pack the data into point msg
                new_point.time = time;
                new_point.x = x_coord;
                new_point.y = y_coord;
                new_point.z = z_coord;
                new_point.azimuth =2*M_PI - firings[fir_idx].azimuth[scan_idx];
                new_point.distance = firings[fir_idx].distance[scan_idx];
                new_point.intensity = firings[fir_idx].intensity[scan_idx];
            }
        }

        packet_start_time += FIRING_TOFFSET * (end_fir_idx-start_fir_idx);
    }
    return;
}

} // end namespace lslidar_n301_decoder

