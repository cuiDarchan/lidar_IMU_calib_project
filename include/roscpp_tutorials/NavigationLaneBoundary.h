// Generated by gencpp from file roscpp_tutorials/NavigationLaneBoundary.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_NAVIGATIONLANEBOUNDARY_H
#define ROSCPP_TUTORIALS_MESSAGE_NAVIGATIONLANEBOUNDARY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/LaneBoundaryType.h>
#include <roscpp_tutorials/Curved.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct NavigationLaneBoundary_
{
  typedef NavigationLaneBoundary_<ContainerAllocator> Type;

  NavigationLaneBoundary_()
    : type()
    , segments()  {
    }
  NavigationLaneBoundary_(const ContainerAllocator& _alloc)
    : type(_alloc)
    , segments(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::roscpp_tutorials::LaneBoundaryType_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::LaneBoundaryType_<ContainerAllocator> >::other >  _type_type;
  _type_type type;

   typedef  ::roscpp_tutorials::Curved_<ContainerAllocator>  _segments_type;
  _segments_type segments;





  typedef boost::shared_ptr< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> const> ConstPtr;

}; // struct NavigationLaneBoundary_

typedef ::roscpp_tutorials::NavigationLaneBoundary_<std::allocator<void> > NavigationLaneBoundary;

typedef boost::shared_ptr< ::roscpp_tutorials::NavigationLaneBoundary > NavigationLaneBoundaryPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::NavigationLaneBoundary const> NavigationLaneBoundaryConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8bdac294b32f1bcea53f423d695f7819";
  }

  static const char* value(const ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8bdac294b32f1bceULL;
  static const uint64_t static_value2 = 0xa53f423d695f7819ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/NavigationLaneBoundary";
  }

  static const char* value(const ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >
{
  static const char* value()
  {
    return "LaneBoundaryType[] type\n\
Curved segments\n\
================================================================================\n\
MSG: roscpp_tutorials/LaneBoundaryType\n\
    uint16 shape\n\
    float32 start_s\n\
    float32 end_s\n\
================================================================================\n\
MSG: roscpp_tutorials/Curved\n\
Vec3d[] points\n\
\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/Vec3d\n\
float32[3] point\n\
\n\
";
  }

  static const char* value(const ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.segments);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavigationLaneBoundary_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator>& v)
  {
    s << indent << "type[]" << std::endl;
    for (size_t i = 0; i < v.type.size(); ++i)
    {
      s << indent << "  type[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::LaneBoundaryType_<ContainerAllocator> >::stream(s, indent + "    ", v.type[i]);
    }
    s << indent << "segments: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::Curved_<ContainerAllocator> >::stream(s, indent + "  ", v.segments);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_NAVIGATIONLANEBOUNDARY_H
