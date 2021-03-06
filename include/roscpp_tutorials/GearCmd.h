// Generated by gencpp from file roscpp_tutorials/GearCmd.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_GEARCMD_H
#define ROSCPP_TUTORIALS_MESSAGE_GEARCMD_H


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
struct GearCmd_
{
  typedef GearCmd_<ContainerAllocator> Type;

  GearCmd_()
    : gear_location(0)
    , drive_gear_position(0)
    , gear_box_mode(0)
    , gearbox_switch_enable(false)  {
    }
  GearCmd_(const ContainerAllocator& _alloc)
    : gear_location(0)
    , drive_gear_position(0)
    , gear_box_mode(0)
    , gearbox_switch_enable(false)  {
  (void)_alloc;
    }



   typedef uint16_t _gear_location_type;
  _gear_location_type gear_location;

   typedef uint16_t _drive_gear_position_type;
  _drive_gear_position_type drive_gear_position;

   typedef uint16_t _gear_box_mode_type;
  _gear_box_mode_type gear_box_mode;

   typedef uint8_t _gearbox_switch_enable_type;
  _gearbox_switch_enable_type gearbox_switch_enable;





  typedef boost::shared_ptr< ::roscpp_tutorials::GearCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::GearCmd_<ContainerAllocator> const> ConstPtr;

}; // struct GearCmd_

typedef ::roscpp_tutorials::GearCmd_<std::allocator<void> > GearCmd;

typedef boost::shared_ptr< ::roscpp_tutorials::GearCmd > GearCmdPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::GearCmd const> GearCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::GearCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::GearCmd_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::GearCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::GearCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::GearCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::GearCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::GearCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::GearCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::GearCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "deb9daf23cdeb898ea48bdb1c2f9b1e9";
  }

  static const char* value(const ::roscpp_tutorials::GearCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdeb9daf23cdeb898ULL;
  static const uint64_t static_value2 = 0xea48bdb1c2f9b1e9ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::GearCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/GearCmd";
  }

  static const char* value(const ::roscpp_tutorials::GearCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::GearCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint16 gear_location\n\
    \n\
uint16 drive_gear_position\n\
    \n\
uint16 gear_box_mode\n\
    \n\
bool gearbox_switch_enable\n\
";
  }

  static const char* value(const ::roscpp_tutorials::GearCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::GearCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.gear_location);
      stream.next(m.drive_gear_position);
      stream.next(m.gear_box_mode);
      stream.next(m.gearbox_switch_enable);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GearCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::GearCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::GearCmd_<ContainerAllocator>& v)
  {
    s << indent << "gear_location: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.gear_location);
    s << indent << "drive_gear_position: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.drive_gear_position);
    s << indent << "gear_box_mode: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.gear_box_mode);
    s << indent << "gearbox_switch_enable: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gearbox_switch_enable);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_GEARCMD_H
