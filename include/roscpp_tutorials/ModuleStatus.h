// Generated by gencpp from file roscpp_tutorials/ModuleStatus.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_MODULESTATUS_H
#define ROSCPP_TUTORIALS_MESSAGE_MODULESTATUS_H


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
struct ModuleStatus_
{
  typedef ModuleStatus_<ContainerAllocator> Type;

  ModuleStatus_()
    {
    }
  ModuleStatus_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





  enum {
    OFF = 0u,
    OK = 1u,
    CONTROL_ERROR = 1000u,
    CONTROL_INIT_ERROR = 1001u,
    CONTROL_COMPUTE_ERROR = 1002u,
    CANBUS_ERROR = 2000u,
    CAN_CLIENT_ERROR_BASE = 2100u,
    CAN_CLIENT_ERROR_OPEN_DEVICE_FAILED = 2101u,
    CAN_CLIENT_ERROR_FRAME_NUM = 2102u,
    CAN_CLIENT_ERROR_SEND_FAILED = 2103u,
    CAN_CLIENT_ERROR_RECV_FAILED = 2104u,
    LOCALIZATION_ERROR = 3000u,
    LOCALIZATION_ERROR_MSG = 3100u,
    LOCALIZATION_ERROR_LIDAR = 3200u,
    LOCALIZATION_ERROR_INTEG = 3300u,
    LOCALIZATION_ERROR_GNSS = 3400u,
    PERCEPTION_ERROR = 4000u,
    PERCEPTION_ERROR_TF = 4001u,
    PERCEPTION_ERROR_PROCESS = 4002u,
    PERCEPTION_FATAL = 4003u,
    PREDICTION_ERROR = 5000u,
    PLANNING_ERROR = 6000u,
    HDMAP_DATA_ERROR = 7000u,
    ROUTING_ERROR = 8000u,
    ROUTING_ERROR_REQUEST = 8001u,
    ROUTING_ERROR_RESPONSE = 8002u,
    ROUTING_ERROR_NOT_READY = 8003u,
    END_OF_INPUT = 9000u,
    HTTP_LOGIC_ERROR = 10000u,
    HTTP_RUNTIME_ERROR = 10001u,
    RELATIVE_MAP_ERROR = 11000u,
    RELATIVE_MAP_NOT_READY = 11001u,
    DRIVER_ERROR_GNSS = 12000u,
    DRIVER_ERROR_VELODYNE = 13000u,
    NAVI_GENERATOR_ERROR_DATABASE = 15000u,
    OBSTACLE_GENERATING_ERROR = 16000u,
  };


  typedef boost::shared_ptr< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> const> ConstPtr;

}; // struct ModuleStatus_

typedef ::roscpp_tutorials::ModuleStatus_<std::allocator<void> > ModuleStatus;

typedef boost::shared_ptr< ::roscpp_tutorials::ModuleStatus > ModuleStatusPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::ModuleStatus const> ModuleStatusConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6179f092dad35ba483170a2a09ef9b39";
  }

  static const char* value(const ::roscpp_tutorials::ModuleStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6179f092dad35ba4ULL;
  static const uint64_t static_value2 = 0x83170a2a09ef9b39ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/ModuleStatus";
  }

  static const char* value(const ::roscpp_tutorials::ModuleStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint16 OFF = 0\n\
## No error reutrns on success.\n\
uint16 OK = 1\n\
\n\
## Control module error codes start from here.\n\
uint16 CONTROL_ERROR = 1000\n\
uint16 CONTROL_INIT_ERROR = 1001\n\
uint16 CONTROL_COMPUTE_ERROR = 1002\n\
\n\
## Canbus module error codes start from here.\n\
uint16 CANBUS_ERROR = 2000\n\
uint16 CAN_CLIENT_ERROR_BASE = 2100\n\
uint16 CAN_CLIENT_ERROR_OPEN_DEVICE_FAILED = 2101\n\
uint16 CAN_CLIENT_ERROR_FRAME_NUM = 2102\n\
uint16 CAN_CLIENT_ERROR_SEND_FAILED = 2103\n\
uint16 CAN_CLIENT_ERROR_RECV_FAILED = 2104\n\
\n\
## Localization module error codes start from here.\n\
uint16 LOCALIZATION_ERROR = 3000\n\
uint16 LOCALIZATION_ERROR_MSG = 3100\n\
uint16 LOCALIZATION_ERROR_LIDAR = 3200\n\
uint16 LOCALIZATION_ERROR_INTEG = 3300\n\
uint16 LOCALIZATION_ERROR_GNSS = 3400\n\
\n\
## Perception module error codes start from here.\n\
uint16 PERCEPTION_ERROR = 4000\n\
uint16 PERCEPTION_ERROR_TF = 4001\n\
uint16 PERCEPTION_ERROR_PROCESS = 4002\n\
uint16 PERCEPTION_FATAL = 4003\n\
\n\
## Prediction module error codes start from here.\n\
uint16 PREDICTION_ERROR = 5000\n\
\n\
## Planning module error codes start from here\n\
uint16 PLANNING_ERROR = 6000\n\
\n\
## HDMap module error codes start from here\n\
uint16 HDMAP_DATA_ERROR = 7000\n\
\n\
## Routing module error codes\n\
uint16 ROUTING_ERROR = 8000\n\
uint16 ROUTING_ERROR_REQUEST = 8001\n\
uint16 ROUTING_ERROR_RESPONSE = 8002\n\
uint16 ROUTING_ERROR_NOT_READY = 8003\n\
\n\
## Indicates an input has been exhausted.\n\
uint16 END_OF_INPUT = 9000\n\
\n\
## HTTP request error codes.\n\
uint16 HTTP_LOGIC_ERROR = 10000\n\
uint16 HTTP_RUNTIME_ERROR = 10001\n\
\n\
## Relative Map error codes.\n\
uint16 RELATIVE_MAP_ERROR = 11000  ## general relative map error code\n\
uint16 RELATIVE_MAP_NOT_READY = 11001\n\
\n\
## Driver error codes.\n\
uint16 DRIVER_ERROR_GNSS = 12000\n\
uint16 DRIVER_ERROR_VELODYNE = 13000\n\
\n\
## Navi Generator error codes.\n\
uint16 NAVI_GENERATOR_ERROR_DATABASE = 15000\n\
\n\
## Obstacle generating\n\
uint16 OBSTACLE_GENERATING_ERROR = 16000\n\
";
  }

  static const char* value(const ::roscpp_tutorials::ModuleStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ModuleStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::ModuleStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::roscpp_tutorials::ModuleStatus_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_MODULESTATUS_H
