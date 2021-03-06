// Generated by gencpp from file roscpp_tutorials/MsgHeader.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_MSGHEADER_H
#define ROSCPP_TUTORIALS_MESSAGE_MSGHEADER_H


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
struct MsgHeader_
{
  typedef MsgHeader_<ContainerAllocator> Type;

  MsgHeader_()
    : timestamp(0)
    , sequence_num(0)
    , module_name(0)
    , status(0)
    , is_debag(0)
    , measured_timestamp(0)
    , version()
    , token(0)
    , token_timestamp(0)
    , detail()  {
      version.assign(0);
  }
  MsgHeader_(const ContainerAllocator& _alloc)
    : timestamp(0)
    , sequence_num(0)
    , module_name(0)
    , status(0)
    , is_debag(0)
    , measured_timestamp(0)
    , version()
    , token(0)
    , token_timestamp(0)
    , detail(_alloc)  {
  (void)_alloc;
      version.assign(0);
  }



   typedef uint64_t _timestamp_type;
  _timestamp_type timestamp;

   typedef uint64_t _sequence_num_type;
  _sequence_num_type sequence_num;

   typedef uint16_t _module_name_type;
  _module_name_type module_name;

   typedef uint16_t _status_type;
  _status_type status;

   typedef uint64_t _is_debag_type;
  _is_debag_type is_debag;

   typedef uint64_t _measured_timestamp_type;
  _measured_timestamp_type measured_timestamp;

   typedef boost::array<uint8_t, 3>  _version_type;
  _version_type version;

   typedef uint64_t _token_type;
  _token_type token;

   typedef uint64_t _token_timestamp_type;
  _token_timestamp_type token_timestamp;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _detail_type;
  _detail_type detail;





  typedef boost::shared_ptr< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> const> ConstPtr;

}; // struct MsgHeader_

typedef ::roscpp_tutorials::MsgHeader_<std::allocator<void> > MsgHeader;

typedef boost::shared_ptr< ::roscpp_tutorials::MsgHeader > MsgHeaderPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::MsgHeader const> MsgHeaderConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::MsgHeader_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4ee0ea975012446d4f22e7f8b8f7ea75";
  }

  static const char* value(const ::roscpp_tutorials::MsgHeader_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4ee0ea975012446dULL;
  static const uint64_t static_value2 = 0x4f22e7f8b8f7ea75ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/MsgHeader";
  }

  static const char* value(const ::roscpp_tutorials::MsgHeader_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 timestamp\n\
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
";
  }

  static const char* value(const ::roscpp_tutorials::MsgHeader_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.sequence_num);
      stream.next(m.module_name);
      stream.next(m.status);
      stream.next(m.is_debag);
      stream.next(m.measured_timestamp);
      stream.next(m.version);
      stream.next(m.token);
      stream.next(m.token_timestamp);
      stream.next(m.detail);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MsgHeader_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::MsgHeader_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.timestamp);
    s << indent << "sequence_num: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.sequence_num);
    s << indent << "module_name: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.module_name);
    s << indent << "status: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.status);
    s << indent << "is_debag: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.is_debag);
    s << indent << "measured_timestamp: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.measured_timestamp);
    s << indent << "version[]" << std::endl;
    for (size_t i = 0; i < v.version.size(); ++i)
    {
      s << indent << "  version[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.version[i]);
    }
    s << indent << "token: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.token);
    s << indent << "token_timestamp: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.token_timestamp);
    s << indent << "detail: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.detail);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_MSGHEADER_H
