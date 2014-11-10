#ifndef __ORIENTATION_NODE_H
#define __ORIENTATION_NODE_H

#include <string>

namespace s8 {
	namespace orientation_node {
		const std::string NODE_NAME =        	"s8_orientation_node";

		const std::string TOPIC_IMU =        	"/imu/data";
		const std::string TOPIC_ORIENTATION =	"/s8/orientation";
	}
}

#endif
