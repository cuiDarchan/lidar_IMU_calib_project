// Generated by gencpp from file roscpp_tutorials/VehicleSignalCmd.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_VEHICLESIGNALCMD_H
#define ROSCPP_TUTORIALS_MESSAGE_VEHICLESIGNALCMD_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/LightCmd.h>
#include <roscpp_tutorials/WiperCmd.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct VehicleSignalCmd_
{
  typedef VehicleSignalCmd_<ContainerAllocator> Type;

  VehicleSignalCmd_()
    : light_cmd()
    , wiper_cmd()
    , horn_status(0)  {
    }
  VehicleSignalCmd_(const ContainerAllocator& _alloc)
    : light_cmd(_alloc)
    , wiper_cmd(_alloc)
    , horn_status(0)  {
  (void)_alloc;
    }



   typedef  ::roscpp_tutorials::LightCmd_<ContainerAllocator>  _light_cmd_type;
  _light_cmd_type light_cmd;

   typedef  ::roscpp_tutorials::WiperCmd_<ContainerAllocator>  _wiper_cmd_type;
  _wiper_cmd_type wiper_cmd;

   typedef uint16_t _horn_status_type;
  _horn_status_type horn_status;





  typedef boost::shared_ptr< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> const> ConstPtr;

}; // struct VehicleSignalCmd_

typedef ::roscpp_tutorials::VehicleSignalCmd_<std::allocator<void> > VehicleSignalCmd;

typedef boost::shared_ptr< ::roscpp_tutorials::VehicleSignalCmd > VehicleSignalCmdPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::VehicleSignalCmd const> VehicleSignalCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f91491ef6c8550e6678f8fedc5de168a";
  }

  static const char* value(const ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf91491ef6c8550e6ULL;
  static const uint64_t static_value2 = 0x678f8fedc5de168aULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/VehicleSignalCmd";
  }

  static const char* value(const ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "    LightCmd light_cmd\n\
    WiperCmd wiper_cmd\n\
    uint16  horn_status\n\
================================================================================\n\
MSG: roscpp_tutorials/LightCmd\n\
uint16 beam_status\n\
uint16 turn_signal\n\
uint16 brake_light_status\n\
================================================================================\n\
MSG: roscpp_tutorials/WiperCmd\n\
uint16 wiper_status    \n\
uint16 wiper_speed\n\
";
  }

  static const char* value(const ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.light_cmd);
      stream.next(m.wiper_cmd);
      stream.next(m.horn_status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VehicleSignalCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::VehicleSignalCmd_<ContainerAllocator>& v)
  {
    s << indent << "light_cmd: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::LightCmd_<ContainerAllocator> >::stream(s, indent + "  ", v.light_cmd);
    s << indent << "wiper_cmd: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::WiperCmd_<ContainerAllocator> >::stream(s, indent + "  ", v.wiper_cmd);
    s << indent << "horn_status: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.horn_status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_VEHICLESIGNALCMD_H
