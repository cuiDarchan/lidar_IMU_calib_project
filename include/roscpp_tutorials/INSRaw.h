// Generated by gencpp from file roscpp_tutorials/INSRaw.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_INSRAW_H
#define ROSCPP_TUTORIALS_MESSAGE_INSRAW_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/MsgHeader.h>
#include <roscpp_tutorials/Quaterniond.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct INSRaw_
{
  typedef INSRaw_<ContainerAllocator> Type;

  INSRaw_()
    : header()
    , measured_timestamp()
    , position()
    , linear_velocity()
    , orientation()  {
      position.assign(0.0);

      linear_velocity.assign(0.0);
  }
  INSRaw_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , measured_timestamp()
    , position()
    , linear_velocity()
    , orientation(_alloc)  {
  (void)_alloc;
      position.assign(0.0);

      linear_velocity.assign(0.0);
  }



   typedef  ::roscpp_tutorials::MsgHeader_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef ros::Time _measured_timestamp_type;
  _measured_timestamp_type measured_timestamp;

   typedef boost::array<double, 3>  _position_type;
  _position_type position;

   typedef boost::array<float, 3>  _linear_velocity_type;
  _linear_velocity_type linear_velocity;

   typedef  ::roscpp_tutorials::Quaterniond_<ContainerAllocator>  _orientation_type;
  _orientation_type orientation;





  typedef boost::shared_ptr< ::roscpp_tutorials::INSRaw_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::INSRaw_<ContainerAllocator> const> ConstPtr;

}; // struct INSRaw_

typedef ::roscpp_tutorials::INSRaw_<std::allocator<void> > INSRaw;

typedef boost::shared_ptr< ::roscpp_tutorials::INSRaw > INSRawPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::INSRaw const> INSRawConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::INSRaw_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::INSRaw_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace roscpp_tutorials

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'roscpp_tutorials': ['/home/cui-dell/catkin_ws/src/roscpp_tutorials/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::INSRaw_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::INSRaw_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::INSRaw_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::INSRaw_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::INSRaw_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::INSRaw_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::INSRaw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b308e8e04c1cc03adda524e9dda63fb3";
  }

  static const char* value(const ::roscpp_tutorials::INSRaw_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb308e8e04c1cc03aULL;
  static const uint64_t static_value2 = 0xdda524e9dda63fb3ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::INSRaw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/INSRaw";
  }

  static const char* value(const ::roscpp_tutorials::INSRaw_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::INSRaw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "  MsgHeader header\n\
\n\
  time measured_timestamp\n\
\n\
  float64[3] position\n\
\n\
  float32[3] linear_velocity\n\
\n\
  Quaterniond orientation\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/MsgHeader\n\
uint64 timestamp\n\
uint64 sequence_num\n\
uint16 module_name\n\
uint16 status\n\
uint64 is_debag\n\
uint64 measured_timestamp\n\
uint8[3] version\n\
uint64 token\n\
uint64 token_timestamp\n\
string detail\n\
\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/Quaterniond\n\
float64 w\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::roscpp_tutorials::INSRaw_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::INSRaw_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.measured_timestamp);
      stream.next(m.position);
      stream.next(m.linear_velocity);
      stream.next(m.orientation);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct INSRaw_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::INSRaw_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::INSRaw_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "measured_timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.measured_timestamp);
    s << indent << "position[]" << std::endl;
    for (size_t i = 0; i < v.position.size(); ++i)
    {
      s << indent << "  position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position[i]);
    }
    s << indent << "linear_velocity[]" << std::endl;
    for (size_t i = 0; i < v.linear_velocity.size(); ++i)
    {
      s << indent << "  linear_velocity[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.linear_velocity[i]);
    }
    s << indent << "orientation: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::Quaterniond_<ContainerAllocator> >::stream(s, indent + "  ", v.orientation);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_INSRAW_H
