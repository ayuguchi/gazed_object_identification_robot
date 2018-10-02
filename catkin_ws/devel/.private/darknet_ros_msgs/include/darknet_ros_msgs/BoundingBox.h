// Generated by gencpp from file darknet_ros_msgs/BoundingBox.msg
// DO NOT EDIT!


#ifndef DARKNET_ROS_MSGS_MESSAGE_BOUNDINGBOX_H
#define DARKNET_ROS_MSGS_MESSAGE_BOUNDINGBOX_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace darknet_ros_msgs
{
template <class ContainerAllocator>
struct BoundingBox_
{
  typedef BoundingBox_<ContainerAllocator> Type;

  BoundingBox_()
    : Class()
    , probability(0.0)
    , xmin(0)
    , ymin(0)
    , xmax(0)
    , ymax(0)  {
    }
  BoundingBox_(const ContainerAllocator& _alloc)
    : Class(_alloc)
    , probability(0.0)
    , xmin(0)
    , ymin(0)
    , xmax(0)
    , ymax(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _Class_type;
  _Class_type Class;

   typedef double _probability_type;
  _probability_type probability;

   typedef int64_t _xmin_type;
  _xmin_type xmin;

   typedef int64_t _ymin_type;
  _ymin_type ymin;

   typedef int64_t _xmax_type;
  _xmax_type xmax;

   typedef int64_t _ymax_type;
  _ymax_type ymax;




  typedef boost::shared_ptr< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> const> ConstPtr;

}; // struct BoundingBox_

typedef ::darknet_ros_msgs::BoundingBox_<std::allocator<void> > BoundingBox;

typedef boost::shared_ptr< ::darknet_ros_msgs::BoundingBox > BoundingBoxPtr;
typedef boost::shared_ptr< ::darknet_ros_msgs::BoundingBox const> BoundingBoxConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8434cf5c7ed632005efa1b05bb45ca2a";
  }

  static const char* value(const ::darknet_ros_msgs::BoundingBox_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8434cf5c7ed63200ULL;
  static const uint64_t static_value2 = 0x5efa1b05bb45ca2aULL;
};

template<class ContainerAllocator>
struct DataType< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> >
{
  static const char* value()
  {
    return "darknet_ros_msgs/BoundingBox";
  }

  static const char* value(const ::darknet_ros_msgs::BoundingBox_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string Class\n\
float64 probability\n\
int64 xmin\n\
int64 ymin\n\
int64 xmax\n\
int64 ymax\n\
\n\
";
  }

  static const char* value(const ::darknet_ros_msgs::BoundingBox_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Class);
      stream.next(m.probability);
      stream.next(m.xmin);
      stream.next(m.ymin);
      stream.next(m.xmax);
      stream.next(m.ymax);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BoundingBox_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::darknet_ros_msgs::BoundingBox_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::darknet_ros_msgs::BoundingBox_<ContainerAllocator>& v)
  {
    s << indent << "Class: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.Class);
    s << indent << "probability: ";
    Printer<double>::stream(s, indent + "  ", v.probability);
    s << indent << "xmin: ";
    Printer<int64_t>::stream(s, indent + "  ", v.xmin);
    s << indent << "ymin: ";
    Printer<int64_t>::stream(s, indent + "  ", v.ymin);
    s << indent << "xmax: ";
    Printer<int64_t>::stream(s, indent + "  ", v.xmax);
    s << indent << "ymax: ";
    Printer<int64_t>::stream(s, indent + "  ", v.ymax);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DARKNET_ROS_MSGS_MESSAGE_BOUNDINGBOX_H
