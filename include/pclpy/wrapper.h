#ifndef _PCLPY_WRAPPER_H
#define _PCLPY_WRAPPER_H

#include <boost/python.hpp>
#include <string>

#include <ros/serialization.h>

template <typename M>
M from_python(const std::string str_msg) {
	size_t serial_size = str_msg.size();
	boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
	for( size_t i = 0; i < serial_size; ++i )
		buffer[i] = str_msg[i];
	ros::serialization::IStream stream(buffer.get(), serial_size);
	M msg;
	ros::serialization::Serializer<M>::read(stream, msg);
	return msg;
}

template <typename M>
std::string to_python(const M& msg) {
	size_t serial_size = ros::serialization::serializationLength(msg);
	boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
	ros::serialization::OStream stream(buffer.get(), serial_size);
	ros::serialization::serialize(stream, msg);
	std::string str_msg;
	str_msg.reserve(serial_size);
	for( size_t i = 0; i < serial_size; ++i )
		str_msg.push_back(buffer[i]);
	return str_msg;
}

#endif // _PCLPY_WRAPPER_H
