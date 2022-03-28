// Generated by gencpp from file roscpp_tutorials/RadarRaw.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_RADARRAW_H
#define ROSCPP_TUTORIALS_MESSAGE_RADARRAW_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/MsgHeader.h>
#include <roscpp_tutorials/RadarRawObject.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct RadarRaw_
{
  typedef RadarRaw_<ContainerAllocator> Type;

  RadarRaw_()
    : header()
    , measured_timestamp()
    , meas_counter(0)
    , interface_version(0)
    , objects()
    , object_num(0)
    , sensor_id(0)  {
    }
  RadarRaw_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , measured_timestamp()
    , meas_counter(0)
    , interface_version(0)
    , objects()
    , object_num(0)
    , sensor_id(0)  {
  (void)_alloc;
      objects.assign( ::roscpp_tutorials::RadarRawObject_<ContainerAllocator> (_alloc));
  }



   typedef  ::roscpp_tutorials::MsgHeader_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef ros::Time _measured_timestamp_type;
  _measured_timestamp_type measured_timestamp;

   typedef int32_t _meas_counter_type;
  _meas_counter_type meas_counter;

   typedef uint8_t _interface_version_type;
  _interface_version_type interface_version;

   typedef boost::array< ::roscpp_tutorials::RadarRawObject_<ContainerAllocator> , 256>  _objects_type;
  _objects_type objects;

   typedef uint8_t _object_num_type;
  _object_num_type object_num;

   typedef uint8_t _sensor_id_type;
  _sensor_id_type sensor_id;





  typedef boost::shared_ptr< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> const> ConstPtr;

}; // struct RadarRaw_

typedef ::roscpp_tutorials::RadarRaw_<std::allocator<void> > RadarRaw;

typedef boost::shared_ptr< ::roscpp_tutorials::RadarRaw > RadarRawPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::RadarRaw const> RadarRawConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::RadarRaw_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "df0b4c53cc837fe138eaab42618cc754";
  }

  static const char* value(const ::roscpp_tutorials::RadarRaw_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdf0b4c53cc837fe1ULL;
  static const uint64_t static_value2 = 0x38eaab42618cc754ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/RadarRaw";
  }

  static const char* value(const ::roscpp_tutorials::RadarRaw_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "MsgHeader header\n\
\n\
time measured_timestamp\n\
\n\
int32 meas_counter\n\
uint8 interface_version\n\
RadarRawObject[256] objects\n\
uint8 object_num\n\
uint8 sensor_id\n\
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
MSG: roscpp_tutorials/RadarRawObject\n\
uint8 id\n\
float32 longitude_dist\n\
float32 lateral_dist\n\
float32 longitude_vel\n\
float32 lateral_vel\n\
uint8 dyn_prop\n\
float32 rcs\n\
float32 longitude_dist_rms\n\
float32 longitude_vel_rms\n\
float32 lateral_dist_rms\n\
float32 lateral_vel_rms\n\
float32 lateral_accel_rms\n\
float32 longitude_accel_rms\n\
float32 oritation_angle_rms\n\
uint8 meas_state\n\
float32 prob_of_exist\n\
float32 longitude_accel\n\
uint8 object_class\n\
float32 lateral_accel\n\
float32 oritation_angle\n\
float32 object_length\n\
float32 object_width\n\
";
  }

  static const char* value(const ::roscpp_tutorials::RadarRaw_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.measured_timestamp);
      stream.next(m.meas_counter);
      stream.next(m.interface_version);
      stream.next(m.objects);
      stream.next(m.object_num);
      stream.next(m.sensor_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RadarRaw_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::RadarRaw_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::RadarRaw_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "measured_timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.measured_timestamp);
    s << indent << "meas_counter: ";
    Printer<int32_t>::stream(s, indent + "  ", v.meas_counter);
    s << indent << "interface_version: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.interface_version);
    s << indent << "objects[]" << std::endl;
    for (size_t i = 0; i < v.objects.size(); ++i)
    {
      s << indent << "  objects[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::RadarRawObject_<ContainerAllocator> >::stream(s, indent + "    ", v.objects[i]);
    }
    s << indent << "object_num: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.object_num);
    s << indent << "sensor_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sensor_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_RADARRAW_H