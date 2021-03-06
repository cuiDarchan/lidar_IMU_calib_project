// Generated by gencpp from file roscpp_tutorials/LocalizationStatus.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_LOCALIZATIONSTATUS_H
#define ROSCPP_TUTORIALS_MESSAGE_LOCALIZATIONSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/MeasureState.h>
#include <roscpp_tutorials/MeasureState.h>
#include <roscpp_tutorials/MeasureState.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct LocalizationStatus_
{
  typedef LocalizationStatus_<ContainerAllocator> Type;

  LocalizationStatus_()
    : fusion_status()
    , gnss_status()
    , lidar_status()  {
    }
  LocalizationStatus_(const ContainerAllocator& _alloc)
    : fusion_status(_alloc)
    , gnss_status(_alloc)
    , lidar_status(_alloc)  {
  (void)_alloc;
    }



   typedef  ::roscpp_tutorials::MeasureState_<ContainerAllocator>  _fusion_status_type;
  _fusion_status_type fusion_status;

   typedef  ::roscpp_tutorials::MeasureState_<ContainerAllocator>  _gnss_status_type;
  _gnss_status_type gnss_status;

   typedef  ::roscpp_tutorials::MeasureState_<ContainerAllocator>  _lidar_status_type;
  _lidar_status_type lidar_status;





  typedef boost::shared_ptr< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> const> ConstPtr;

}; // struct LocalizationStatus_

typedef ::roscpp_tutorials::LocalizationStatus_<std::allocator<void> > LocalizationStatus;

typedef boost::shared_ptr< ::roscpp_tutorials::LocalizationStatus > LocalizationStatusPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::LocalizationStatus const> LocalizationStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b3774761b8efc46fad4f1ad7d625f286";
  }

  static const char* value(const ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb3774761b8efc46fULL;
  static const uint64_t static_value2 = 0xad4f1ad7d625f286ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/LocalizationStatus";
  }

  static const char* value(const ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "MeasureState fusion_status\n\
MeasureState gnss_status\n\
MeasureState lidar_status\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/MeasureState\n\
uint8 NOT_VALID = 0\n\
uint8 NOT_STABLE = 1\n\
uint8 OK = 2\n\
uint8 VALID = 3\n\
";
  }

  static const char* value(const ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.fusion_status);
      stream.next(m.gnss_status);
      stream.next(m.lidar_status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LocalizationStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::LocalizationStatus_<ContainerAllocator>& v)
  {
    s << indent << "fusion_status: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::MeasureState_<ContainerAllocator> >::stream(s, indent + "  ", v.fusion_status);
    s << indent << "gnss_status: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::MeasureState_<ContainerAllocator> >::stream(s, indent + "  ", v.gnss_status);
    s << indent << "lidar_status: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::MeasureState_<ContainerAllocator> >::stream(s, indent + "  ", v.lidar_status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_LOCALIZATIONSTATUS_H
