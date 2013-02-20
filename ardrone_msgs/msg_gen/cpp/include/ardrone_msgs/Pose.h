/* Auto-generated by genmsg_cpp for file /home/jara/electric_workspace/ardrone_msgs/msg/Pose.msg */
#ifndef ARDRONE_MSGS_MESSAGE_POSE_H
#define ARDRONE_MSGS_MESSAGE_POSE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ardrone_msgs/Vector3.h"
#include "ardrone_msgs/Vector3.h"

namespace ardrone_msgs
{
template <class ContainerAllocator>
struct Pose_ {
  typedef Pose_<ContainerAllocator> Type;

  Pose_()
  : real()
  , pixels()
  , priority(false)
  {
  }

  Pose_(const ContainerAllocator& _alloc)
  : real(_alloc)
  , pixels(_alloc)
  , priority(false)
  {
  }

  typedef  ::ardrone_msgs::Vector3_<ContainerAllocator>  _real_type;
   ::ardrone_msgs::Vector3_<ContainerAllocator>  real;

  typedef  ::ardrone_msgs::Vector3_<ContainerAllocator>  _pixels_type;
   ::ardrone_msgs::Vector3_<ContainerAllocator>  pixels;

  typedef uint8_t _priority_type;
  uint8_t priority;


private:
  static const char* __s_getDataType_() { return "ardrone_msgs/Pose"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "c4a280f87b43aa759c89cab10eed497e"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Vector3 real\n\
\n\
Vector3 pixels\n\
\n\
bool priority\n\
\n\
================================================================================\n\
MSG: ardrone_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, real);
    ros::serialization::serialize(stream, pixels);
    ros::serialization::serialize(stream, priority);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, real);
    ros::serialization::deserialize(stream, pixels);
    ros::serialization::deserialize(stream, priority);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(real);
    size += ros::serialization::serializationLength(pixels);
    size += ros::serialization::serializationLength(priority);
    return size;
  }

  typedef boost::shared_ptr< ::ardrone_msgs::Pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_msgs::Pose_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Pose
typedef  ::ardrone_msgs::Pose_<std::allocator<void> > Pose;

typedef boost::shared_ptr< ::ardrone_msgs::Pose> PosePtr;
typedef boost::shared_ptr< ::ardrone_msgs::Pose const> PoseConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::ardrone_msgs::Pose_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::ardrone_msgs::Pose_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace ardrone_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ardrone_msgs::Pose_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ardrone_msgs::Pose_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ardrone_msgs::Pose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c4a280f87b43aa759c89cab10eed497e";
  }

  static const char* value(const  ::ardrone_msgs::Pose_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc4a280f87b43aa75ULL;
  static const uint64_t static_value2 = 0x9c89cab10eed497eULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_msgs::Pose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ardrone_msgs/Pose";
  }

  static const char* value(const  ::ardrone_msgs::Pose_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ardrone_msgs::Pose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Vector3 real\n\
\n\
Vector3 pixels\n\
\n\
bool priority\n\
\n\
================================================================================\n\
MSG: ardrone_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const  ::ardrone_msgs::Pose_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::ardrone_msgs::Pose_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ardrone_msgs::Pose_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.real);
    stream.next(m.pixels);
    stream.next(m.priority);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Pose_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ardrone_msgs::Pose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::ardrone_msgs::Pose_<ContainerAllocator> & v) 
  {
    s << indent << "real: ";
s << std::endl;
    Printer< ::ardrone_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.real);
    s << indent << "pixels: ";
s << std::endl;
    Printer< ::ardrone_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.pixels);
    s << indent << "priority: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.priority);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ARDRONE_MSGS_MESSAGE_POSE_H
