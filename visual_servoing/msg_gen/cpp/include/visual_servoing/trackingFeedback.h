/* Auto-generated by genmsg_cpp for file /home/marc/electric_workspace/visual_servoing/msg/trackingFeedback.msg */
#ifndef VISUAL_SERVOING_MESSAGE_TRACKINGFEEDBACK_H
#define VISUAL_SERVOING_MESSAGE_TRACKINGFEEDBACK_H
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


namespace visual_servoing
{
template <class ContainerAllocator>
struct trackingFeedback_ {
  typedef trackingFeedback_<ContainerAllocator> Type;

  trackingFeedback_()
  : working(false)
  , vel_x(0.0)
  , vel_y(0.0)
  , centre_x(0)
  , centre_y(0)
  {
  }

  trackingFeedback_(const ContainerAllocator& _alloc)
  : working(false)
  , vel_x(0.0)
  , vel_y(0.0)
  , centre_x(0)
  , centre_y(0)
  {
  }

  typedef uint8_t _working_type;
  uint8_t working;

  typedef float _vel_x_type;
  float vel_x;

  typedef float _vel_y_type;
  float vel_y;

  typedef int32_t _centre_x_type;
  int32_t centre_x;

  typedef int32_t _centre_y_type;
  int32_t centre_y;


private:
  static const char* __s_getDataType_() { return "visual_servoing/trackingFeedback"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "ee22359594f93668d00814bfda66b0cc"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#feedback definition\n\
bool working\n\
float32 vel_x\n\
float32 vel_y\n\
int32 centre_x\n\
int32 centre_y\n\
\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, working);
    ros::serialization::serialize(stream, vel_x);
    ros::serialization::serialize(stream, vel_y);
    ros::serialization::serialize(stream, centre_x);
    ros::serialization::serialize(stream, centre_y);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, working);
    ros::serialization::deserialize(stream, vel_x);
    ros::serialization::deserialize(stream, vel_y);
    ros::serialization::deserialize(stream, centre_x);
    ros::serialization::deserialize(stream, centre_y);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(working);
    size += ros::serialization::serializationLength(vel_x);
    size += ros::serialization::serializationLength(vel_y);
    size += ros::serialization::serializationLength(centre_x);
    size += ros::serialization::serializationLength(centre_y);
    return size;
  }

  typedef boost::shared_ptr< ::visual_servoing::trackingFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::visual_servoing::trackingFeedback_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct trackingFeedback
typedef  ::visual_servoing::trackingFeedback_<std::allocator<void> > trackingFeedback;

typedef boost::shared_ptr< ::visual_servoing::trackingFeedback> trackingFeedbackPtr;
typedef boost::shared_ptr< ::visual_servoing::trackingFeedback const> trackingFeedbackConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::visual_servoing::trackingFeedback_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::visual_servoing::trackingFeedback_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace visual_servoing

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::visual_servoing::trackingFeedback_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::visual_servoing::trackingFeedback_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::visual_servoing::trackingFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ee22359594f93668d00814bfda66b0cc";
  }

  static const char* value(const  ::visual_servoing::trackingFeedback_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xee22359594f93668ULL;
  static const uint64_t static_value2 = 0xd00814bfda66b0ccULL;
};

template<class ContainerAllocator>
struct DataType< ::visual_servoing::trackingFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "visual_servoing/trackingFeedback";
  }

  static const char* value(const  ::visual_servoing::trackingFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::visual_servoing::trackingFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#feedback definition\n\
bool working\n\
float32 vel_x\n\
float32 vel_y\n\
int32 centre_x\n\
int32 centre_y\n\
\n\
\n\
";
  }

  static const char* value(const  ::visual_servoing::trackingFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::visual_servoing::trackingFeedback_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::visual_servoing::trackingFeedback_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.working);
    stream.next(m.vel_x);
    stream.next(m.vel_y);
    stream.next(m.centre_x);
    stream.next(m.centre_y);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct trackingFeedback_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::visual_servoing::trackingFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::visual_servoing::trackingFeedback_<ContainerAllocator> & v) 
  {
    s << indent << "working: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.working);
    s << indent << "vel_x: ";
    Printer<float>::stream(s, indent + "  ", v.vel_x);
    s << indent << "vel_y: ";
    Printer<float>::stream(s, indent + "  ", v.vel_y);
    s << indent << "centre_x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.centre_x);
    s << indent << "centre_y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.centre_y);
  }
};


} // namespace message_operations
} // namespace ros

#endif // VISUAL_SERVOING_MESSAGE_TRACKINGFEEDBACK_H

