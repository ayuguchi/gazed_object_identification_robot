// Generated by gencpp from file openface_ros/Faces.msg
// DO NOT EDIT!


#ifndef OPENFACE_ROS_MESSAGE_FACES_H
#define OPENFACE_ROS_MESSAGE_FACES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <openface_ros/Face.h>

namespace openface_ros
{
template <class ContainerAllocator>
struct Faces_
{
  typedef Faces_<ContainerAllocator> Type;

  Faces_()
    : faces()  {
    }
  Faces_(const ContainerAllocator& _alloc)
    : faces(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::openface_ros::Face_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::openface_ros::Face_<ContainerAllocator> >::other >  _faces_type;
  _faces_type faces;




  typedef boost::shared_ptr< ::openface_ros::Faces_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::openface_ros::Faces_<ContainerAllocator> const> ConstPtr;

}; // struct Faces_

typedef ::openface_ros::Faces_<std::allocator<void> > Faces;

typedef boost::shared_ptr< ::openface_ros::Faces > FacesPtr;
typedef boost::shared_ptr< ::openface_ros::Faces const> FacesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::openface_ros::Faces_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::openface_ros::Faces_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace openface_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'openface_ros': ['/root/catkin_ws/src/openface_ros/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::openface_ros::Faces_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::openface_ros::Faces_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::openface_ros::Faces_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::openface_ros::Faces_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::openface_ros::Faces_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::openface_ros::Faces_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::openface_ros::Faces_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9efd106cd1ef1598e44e5a94142562d9";
  }

  static const char* value(const ::openface_ros::Faces_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9efd106cd1ef1598ULL;
  static const uint64_t static_value2 = 0xe44e5a94142562d9ULL;
};

template<class ContainerAllocator>
struct DataType< ::openface_ros::Faces_<ContainerAllocator> >
{
  static const char* value()
  {
    return "openface_ros/Faces";
  }

  static const char* value(const ::openface_ros::Faces_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::openface_ros::Faces_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Face[] faces\n\
================================================================================\n\
MSG: openface_ros/Face\n\
std_msgs/Header header\n\
\n\
geometry_msgs/Vector3 left_gaze\n\
geometry_msgs/Vector3 right_gaze\n\
\n\
geometry_msgs/Pose head_pose\n\
\n\
geometry_msgs/Point[] landmarks_3d\n\
geometry_msgs/Point[] landmarks_2d\n\
\n\
#openface_ros/ActionUnit[] action_units\n\
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
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::openface_ros::Faces_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::openface_ros::Faces_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.faces);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Faces_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::openface_ros::Faces_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::openface_ros::Faces_<ContainerAllocator>& v)
  {
    s << indent << "faces[]" << std::endl;
    for (size_t i = 0; i < v.faces.size(); ++i)
    {
      s << indent << "  faces[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::openface_ros::Face_<ContainerAllocator> >::stream(s, indent + "    ", v.faces[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPENFACE_ROS_MESSAGE_FACES_H
