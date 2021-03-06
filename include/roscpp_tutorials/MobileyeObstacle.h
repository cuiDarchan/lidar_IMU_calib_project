// Generated by gencpp from file roscpp_tutorials/MobileyeObstacle.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_MOBILEYEOBSTACLE_H
#define ROSCPP_TUTORIALS_MESSAGE_MOBILEYEOBSTACLE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct MobileyeObstacle_
{
  typedef MobileyeObstacle_<ContainerAllocator> Type;

  MobileyeObstacle_()
    : id(0)
    , position_x(0.0)
    , position_y(0.0)
    , blinker_info(0)
    , cut_in_and_out(0)
    , velocity_x(0.0)
    , type(0)
    , status(0)
    , brake_lights(false)
    , valid(0)
    , length(0.0)
    , width(0.0)
    , age(0)
    , lane(0)
    , cipv(false)
    , radar_position_x(0.0)
    , radar_velocity_x(0.0)
    , radar_match_confidence(0)
    , matched_radar_id(0)
    , angle_rate(0.0)
    , scale_change(0.0)
    , acceleration_x(0.0)
    , replaced(false)
    , angle(0.0)  {
    }
  MobileyeObstacle_(const ContainerAllocator& _alloc)
    : id(0)
    , position_x(0.0)
    , position_y(0.0)
    , blinker_info(0)
    , cut_in_and_out(0)
    , velocity_x(0.0)
    , type(0)
    , status(0)
    , brake_lights(false)
    , valid(0)
    , length(0.0)
    , width(0.0)
    , age(0)
    , lane(0)
    , cipv(false)
    , radar_position_x(0.0)
    , radar_velocity_x(0.0)
    , radar_match_confidence(0)
    , matched_radar_id(0)
    , angle_rate(0.0)
    , scale_change(0.0)
    , acceleration_x(0.0)
    , replaced(false)
    , angle(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _id_type;
  _id_type id;

   typedef float _position_x_type;
  _position_x_type position_x;

   typedef float _position_y_type;
  _position_y_type position_y;

   typedef uint8_t _blinker_info_type;
  _blinker_info_type blinker_info;

   typedef uint8_t _cut_in_and_out_type;
  _cut_in_and_out_type cut_in_and_out;

   typedef float _velocity_x_type;
  _velocity_x_type velocity_x;

   typedef uint8_t _type_type;
  _type_type type;

   typedef uint8_t _status_type;
  _status_type status;

   typedef uint8_t _brake_lights_type;
  _brake_lights_type brake_lights;

   typedef uint8_t _valid_type;
  _valid_type valid;

   typedef float _length_type;
  _length_type length;

   typedef float _width_type;
  _width_type width;

   typedef uint8_t _age_type;
  _age_type age;

   typedef uint8_t _lane_type;
  _lane_type lane;

   typedef uint8_t _cipv_type;
  _cipv_type cipv;

   typedef float _radar_position_x_type;
  _radar_position_x_type radar_position_x;

   typedef float _radar_velocity_x_type;
  _radar_velocity_x_type radar_velocity_x;

   typedef uint8_t _radar_match_confidence_type;
  _radar_match_confidence_type radar_match_confidence;

   typedef uint8_t _matched_radar_id_type;
  _matched_radar_id_type matched_radar_id;

   typedef float _angle_rate_type;
  _angle_rate_type angle_rate;

   typedef float _scale_change_type;
  _scale_change_type scale_change;

   typedef float _acceleration_x_type;
  _acceleration_x_type acceleration_x;

   typedef uint8_t _replaced_type;
  _replaced_type replaced;

   typedef float _angle_type;
  _angle_type angle;





  typedef boost::shared_ptr< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> const> ConstPtr;

}; // struct MobileyeObstacle_

typedef ::roscpp_tutorials::MobileyeObstacle_<std::allocator<void> > MobileyeObstacle;

typedef boost::shared_ptr< ::roscpp_tutorials::MobileyeObstacle > MobileyeObstaclePtr;
typedef boost::shared_ptr< ::roscpp_tutorials::MobileyeObstacle const> MobileyeObstacleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1b36a03591cc7189b32e4dbd21d100b1";
  }

  static const char* value(const ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1b36a03591cc7189ULL;
  static const uint64_t static_value2 = 0xb32e4dbd21d100b1ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/MobileyeObstacle";
  }

  static const char* value(const ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 id\n\
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

  static const char* value(const ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.position_x);
      stream.next(m.position_y);
      stream.next(m.blinker_info);
      stream.next(m.cut_in_and_out);
      stream.next(m.velocity_x);
      stream.next(m.type);
      stream.next(m.status);
      stream.next(m.brake_lights);
      stream.next(m.valid);
      stream.next(m.length);
      stream.next(m.width);
      stream.next(m.age);
      stream.next(m.lane);
      stream.next(m.cipv);
      stream.next(m.radar_position_x);
      stream.next(m.radar_velocity_x);
      stream.next(m.radar_match_confidence);
      stream.next(m.matched_radar_id);
      stream.next(m.angle_rate);
      stream.next(m.scale_change);
      stream.next(m.acceleration_x);
      stream.next(m.replaced);
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MobileyeObstacle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::MobileyeObstacle_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id);
    s << indent << "position_x: ";
    Printer<float>::stream(s, indent + "  ", v.position_x);
    s << indent << "position_y: ";
    Printer<float>::stream(s, indent + "  ", v.position_y);
    s << indent << "blinker_info: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.blinker_info);
    s << indent << "cut_in_and_out: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cut_in_and_out);
    s << indent << "velocity_x: ";
    Printer<float>::stream(s, indent + "  ", v.velocity_x);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
    s << indent << "brake_lights: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.brake_lights);
    s << indent << "valid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.valid);
    s << indent << "length: ";
    Printer<float>::stream(s, indent + "  ", v.length);
    s << indent << "width: ";
    Printer<float>::stream(s, indent + "  ", v.width);
    s << indent << "age: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.age);
    s << indent << "lane: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.lane);
    s << indent << "cipv: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cipv);
    s << indent << "radar_position_x: ";
    Printer<float>::stream(s, indent + "  ", v.radar_position_x);
    s << indent << "radar_velocity_x: ";
    Printer<float>::stream(s, indent + "  ", v.radar_velocity_x);
    s << indent << "radar_match_confidence: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.radar_match_confidence);
    s << indent << "matched_radar_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.matched_radar_id);
    s << indent << "angle_rate: ";
    Printer<float>::stream(s, indent + "  ", v.angle_rate);
    s << indent << "scale_change: ";
    Printer<float>::stream(s, indent + "  ", v.scale_change);
    s << indent << "acceleration_x: ";
    Printer<float>::stream(s, indent + "  ", v.acceleration_x);
    s << indent << "replaced: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.replaced);
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_MOBILEYEOBSTACLE_H
