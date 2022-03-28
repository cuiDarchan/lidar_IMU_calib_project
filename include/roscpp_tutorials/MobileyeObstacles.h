// Generated by gencpp from file roscpp_tutorials/MobileyeObstacles.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_MOBILEYEOBSTACLES_H
#define ROSCPP_TUTORIALS_MESSAGE_MOBILEYEOBSTACLES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/MobileyeObstaclesGo.h>
#include <roscpp_tutorials/MobileyeObstacle.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct MobileyeObstacles_
{
  typedef MobileyeObstacles_<ContainerAllocator> Type;

  MobileyeObstacles_()
    : measured_timestamp()
    , timestamp(0)
    , application_version(0)
    , active_version_number_section(0)
    , left_close_rang_cut_in(false)
    , right_close_rang_cut_in(false)
    , go()
    , protocol_version(0)
    , close_car(false)
    , failsafe(0)
    , obstacles()
    , obstacle_num(0)  {
    }
  MobileyeObstacles_(const ContainerAllocator& _alloc)
    : measured_timestamp()
    , timestamp(0)
    , application_version(0)
    , active_version_number_section(0)
    , left_close_rang_cut_in(false)
    , right_close_rang_cut_in(false)
    , go(_alloc)
    , protocol_version(0)
    , close_car(false)
    , failsafe(0)
    , obstacles()
    , obstacle_num(0)  {
  (void)_alloc;
      obstacles.assign( ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> (_alloc));
  }



   typedef ros::Time _measured_timestamp_type;
  _measured_timestamp_type measured_timestamp;

   typedef uint8_t _timestamp_type;
  _timestamp_type timestamp;

   typedef uint8_t _application_version_type;
  _application_version_type application_version;

   typedef uint8_t _active_version_number_section_type;
  _active_version_number_section_type active_version_number_section;

   typedef uint8_t _left_close_rang_cut_in_type;
  _left_close_rang_cut_in_type left_close_rang_cut_in;

   typedef uint8_t _right_close_rang_cut_in_type;
  _right_close_rang_cut_in_type right_close_rang_cut_in;

   typedef  ::roscpp_tutorials::MobileyeObstaclesGo_<ContainerAllocator>  _go_type;
  _go_type go;

   typedef uint8_t _protocol_version_type;
  _protocol_version_type protocol_version;

   typedef uint8_t _close_car_type;
  _close_car_type close_car;

   typedef uint8_t _failsafe_type;
  _failsafe_type failsafe;

   typedef boost::array< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> , 20>  _obstacles_type;
  _obstacles_type obstacles;

   typedef uint8_t _obstacle_num_type;
  _obstacle_num_type obstacle_num;





  typedef boost::shared_ptr< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> const> ConstPtr;

}; // struct MobileyeObstacles_

typedef ::roscpp_tutorials::MobileyeObstacles_<std::allocator<void> > MobileyeObstacles;

typedef boost::shared_ptr< ::roscpp_tutorials::MobileyeObstacles > MobileyeObstaclesPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::MobileyeObstacles const> MobileyeObstaclesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace roscpp_tutorials

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'roscpp_tutorials': ['/home/cui-dell/catkin_ws/src/roscpp_tutorials/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "27de254c5d3530be72efc7545c44e18f";
  }

  static const char* value(const ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x27de254c5d3530beULL;
  static const uint64_t static_value2 = 0x72efc7545c44e18fULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/MobileyeObstacles";
  }

  static const char* value(const ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time measured_timestamp\n\
uint8 timestamp\n\
uint8 application_version\n\
uint8 active_version_number_section\n\
bool left_close_rang_cut_in\n\
bool right_close_rang_cut_in\n\
MobileyeObstaclesGo go\n\
uint8 protocol_version\n\
bool close_car\n\
uint8 failsafe\n\
MobileyeObstacle[20] obstacles\n\
uint8 obstacle_num\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/MobileyeObstaclesGo\n\
uint16 STOP = 0\n\
uint16 GO = 1\n\
uint16 NOT_CALCULATED = 15\n\
================================================================================\n\
MSG: roscpp_tutorials/MobileyeObstacle\n\
uint8 id\n\
float32 position_x\n\
float32 position_y\n\
uint8 blinker_info\n\
uint8 cut_in_and_out\n\
float32 velocity_x\n\
uint8 type\n\
uint8 status\n\
bool brake_lights\n\
uint8 valid\n\
float32 length\n\
float32 width\n\
uint8 age\n\
uint8 lane\n\
bool cipv\n\
float32 radar_position_x\n\
float32 radar_velocity_x\n\
uint8 radar_match_confidence\n\
uint8 matched_radar_id\n\
float32 angle_rate\n\
float32 scale_change\n\
float32 acceleration_x\n\
bool replaced\n\
float32 angle\n\
";
  }

  static const char* value(const ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.measured_timestamp);
      stream.next(m.timestamp);
      stream.next(m.application_version);
      stream.next(m.active_version_number_section);
      stream.next(m.left_close_rang_cut_in);
      stream.next(m.right_close_rang_cut_in);
      stream.next(m.go);
      stream.next(m.protocol_version);
      stream.next(m.close_car);
      stream.next(m.failsafe);
      stream.next(m.obstacles);
      stream.next(m.obstacle_num);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MobileyeObstacles_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::MobileyeObstacles_<ContainerAllocator>& v)
  {
    s << indent << "measured_timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.measured_timestamp);
    s << indent << "timestamp: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.timestamp);
    s << indent << "application_version: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.application_version);
    s << indent << "active_version_number_section: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.active_version_number_section);
    s << indent << "left_close_rang_cut_in: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.left_close_rang_cut_in);
    s << indent << "right_close_rang_cut_in: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.right_close_rang_cut_in);
    s << indent << "go: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::MobileyeObstaclesGo_<ContainerAllocator> >::stream(s, indent + "  ", v.go);
    s << indent << "protocol_version: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.protocol_version);
    s << indent << "close_car: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.close_car);
    s << indent << "failsafe: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.failsafe);
    s << indent << "obstacles[]" << std::endl;
    for (size_t i = 0; i < v.obstacles.size(); ++i)
    {
      s << indent << "  obstacles[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> >::stream(s, indent + "    ", v.obstacles[i]);
    }
    s << indent << "obstacle_num: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.obstacle_num);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_MOBILEYEOBSTACLES_H