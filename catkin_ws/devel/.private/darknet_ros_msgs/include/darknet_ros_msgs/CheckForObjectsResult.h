// Generated by gencpp from file darknet_ros_msgs/CheckForObjectsResult.msg
// DO NOT EDIT!


#ifndef DARKNET_ROS_MSGS_MESSAGE_CHECKFOROBJECTSRESULT_H
#define DARKNET_ROS_MSGS_MESSAGE_CHECKFOROBJECTSRESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <darknet_ros_msgs/BoundingBoxes.h>

namespace darknet_ros_msgs
{
template <class ContainerAllocator>
struct CheckForObjectsResult_
{
  typedef CheckForObjectsResult_<ContainerAllocator> Type;

  CheckForObjectsResult_()
    : id(0)
    , boundingBoxes()  {
    }
  CheckForObjectsResult_(const ContainerAllocator& _alloc)
    : id(0)
    , boundingBoxes(_alloc)  {
  (void)_alloc;
    }



   typedef int16_t _id_type;
  _id_type id;

   typedef  ::darknet_ros_msgs::BoundingBoxes_<ContainerAllocator>  _boundingBoxes_type;
  _boundingBoxes_type boundingBoxes;




  typedef boost::shared_ptr< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> const> ConstPtr;

}; // struct CheckForObjectsResult_

typedef ::darknet_ros_msgs::CheckForObjectsResult_<std::allocator<void> > CheckForObjectsResult;

typedef boost::shared_ptr< ::darknet_ros_msgs::CheckForObjectsResult > CheckForObjectsResultPtr;
typedef boost::shared_ptr< ::darknet_ros_msgs::CheckForObjectsResult const> CheckForObjectsResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace darknet_ros_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'darknet_ros_msgs': ['/root/catkin_ws/src/darknet_ros/darknet_ros_msgs/msg', '/root/catkin_ws/devel/.private/darknet_ros_msgs/share/darknet_ros_msgs/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b7095776bbf2dd52b93a4ce86a7902c0";
  }

  static const char* value(const ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb7095776bbf2dd52ULL;
  static const uint64_t static_value2 = 0xb93a4ce86a7902c0ULL;
};

template<class ContainerAllocator>
struct DataType< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "darknet_ros_msgs/CheckForObjectsResult";
  }

  static const char* value(const ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Result definition\n\
int16 id\n\
darknet_ros_msgs/BoundingBoxes boundingBoxes\n\
\n\
\n\
================================================================================\n\
MSG: darknet_ros_msgs/BoundingBoxes\n\
Header header\n\
BoundingBox[] boundingBoxes\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: darknet_ros_msgs/BoundingBox\n\
string Class\n\
float64 probability\n\
int64 xmin\n\
int64 ymin\n\
int64 xmax\n\
int64 ymax\n\
\n\
";
  }

  static const char* value(const ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.boundingBoxes);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CheckForObjectsResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::darknet_ros_msgs::CheckForObjectsResult_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int16_t>::stream(s, indent + "  ", v.id);
    s << indent << "boundingBoxes: ";
    s << std::endl;
    Printer< ::darknet_ros_msgs::BoundingBoxes_<ContainerAllocator> >::stream(s, indent + "  ", v.boundingBoxes);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DARKNET_ROS_MSGS_MESSAGE_CHECKFOROBJECTSRESULT_H
