// Generated by gencpp from file roscpp_tutorials/Quaternionf.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_QUATERNIONF_H
#define ROSCPP_TUTORIALS_MESSAGE_QUATERNIONF_H


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
struct Quaternionf_
{
  typedef Quaternionf_<ContainerAllocator> Type;

  Quaternionf_()
    : w(0.0)
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  Quaternionf_(const ContainerAllocator& _alloc)
    : w(0.0)
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef float _w_type;
  _w_type w;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> const> ConstPtr;

}; // struct Quaternionf_

typedef ::roscpp_tutorials::Quaternionf_<std::allocator<void> > Quaternionf;

typedef boost::shared_ptr< ::roscpp_tutorials::Quaternionf > QuaternionfPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::Quaternionf const> QuaternionfConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::Quaternionf_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6b94d9692e392e2b3b71fa994a2cf858";
  }

  static const char* value(const ::roscpp_tutorials::Quaternionf_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6b94d9692e392e2bULL;
  static const uint64_t static_value2 = 0x3b71fa994a2cf858ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/Quaternionf";
  }

  static const char* value(const ::roscpp_tutorials::Quaternionf_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 w\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const ::roscpp_tutorials::Quaternionf_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.w);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Quaternionf_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::Quaternionf_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::Quaternionf_<ContainerAllocator>& v)
  {
    s << indent << "w: ";
    Printer<float>::stream(s, indent + "  ", v.w);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_QUATERNIONF_H
