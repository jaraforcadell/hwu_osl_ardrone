/* Auto-generated by genmsg_cpp for file /home/marc/electric_workspace/visual_servoing/msg/Tag_poseResult.msg */
#ifndef VISUAL_SERVOING_MESSAGE_TAG_POSERESULT_H
#define VISUAL_SERVOING_MESSAGE_TAG_POSERESULT_H
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
struct Tag_poseResult_ {
  typedef Tag_poseResult_<ContainerAllocator> Type;

  Tag_poseResult_()
  {
  }

  Tag_poseResult_(const ContainerAllocator& _alloc)
  {
  }


private:
  static const char* __s_getDataType_() { return "visual_servoing/Tag_poseResult"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    return size;
  }

  typedef boost::shared_ptr< ::visual_servoing::Tag_poseResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::visual_servoing::Tag_poseResult_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Tag_poseResult
typedef  ::visual_servoing::Tag_poseResult_<std::allocator<void> > Tag_poseResult;

typedef boost::shared_ptr< ::visual_servoing::Tag_poseResult> Tag_poseResultPtr;
typedef boost::shared_ptr< ::visual_servoing::Tag_poseResult const> Tag_poseResultConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::visual_servoing::Tag_poseResult_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::visual_servoing::Tag_poseResult_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace visual_servoing

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::visual_servoing::Tag_poseResult_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::visual_servoing::Tag_poseResult_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::visual_servoing::Tag_poseResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::visual_servoing::Tag_poseResult_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::visual_servoing::Tag_poseResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "visual_servoing/Tag_poseResult";
  }

  static const char* value(const  ::visual_servoing::Tag_poseResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::visual_servoing::Tag_poseResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
";
  }

  static const char* value(const  ::visual_servoing::Tag_poseResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::visual_servoing::Tag_poseResult_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::visual_servoing::Tag_poseResult_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Tag_poseResult_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::visual_servoing::Tag_poseResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::visual_servoing::Tag_poseResult_<ContainerAllocator> & v) 
  {
  }
};


} // namespace message_operations
} // namespace ros

#endif // VISUAL_SERVOING_MESSAGE_TAG_POSERESULT_H

