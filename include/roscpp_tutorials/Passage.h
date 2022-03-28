// Generated by gencpp from file roscpp_tutorials/Passage.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_PASSAGE_H
#define ROSCPP_TUTORIALS_MESSAGE_PASSAGE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/PathPoint.h>
#include <roscpp_tutorials/LaneSampleAssociation.h>
#include <roscpp_tutorials/LaneSampleAssociation.h>
#include <roscpp_tutorials/PassageType.h>
#include <roscpp_tutorials/PassageSpeed.h>
#include <roscpp_tutorials/NavigationLaneBoundary.h>
#include <roscpp_tutorials/NavigationLaneBoundary.h>
#include <roscpp_tutorials/CurveSign.h>
#include <roscpp_tutorials/PolygonSign.h>
#include <roscpp_tutorials/Signal.h>
#include <roscpp_tutorials/ParkingSpace.h>
#include <roscpp_tutorials/PassageTurn.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct Passage_
{
  typedef Passage_<ContainerAllocator> Type;

  Passage_()
    : id()
    , path()
    , left_samples()
    , right_samples()
    , types()
    , speed_limit()
    , left_boundary()
    , right_boundary()
    , curve_signs()
    , polygon_signs()
    , signals()
    , is_on_passage(false)
    , can_exit(false)
    , change_lane_type(0)
    , remain_length(0.0)
    , stop_for_destination(false)
    , predecessor_id()
    , sucessor_id()
    , left_neighbor_id()
    , right_neighbor_id()
    , parking_spaces()
    , turns()  {
    }
  Passage_(const ContainerAllocator& _alloc)
    : id(_alloc)
    , path(_alloc)
    , left_samples(_alloc)
    , right_samples(_alloc)
    , types(_alloc)
    , speed_limit(_alloc)
    , left_boundary(_alloc)
    , right_boundary(_alloc)
    , curve_signs(_alloc)
    , polygon_signs(_alloc)
    , signals(_alloc)
    , is_on_passage(false)
    , can_exit(false)
    , change_lane_type(0)
    , remain_length(0.0)
    , stop_for_destination(false)
    , predecessor_id(_alloc)
    , sucessor_id(_alloc)
    , left_neighbor_id(_alloc)
    , right_neighbor_id(_alloc)
    , parking_spaces(_alloc)
    , turns(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _id_type;
  _id_type id;

   typedef std::vector< ::roscpp_tutorials::PathPoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::PathPoint_<ContainerAllocator> >::other >  _path_type;
  _path_type path;

   typedef std::vector< ::roscpp_tutorials::LaneSampleAssociation_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::LaneSampleAssociation_<ContainerAllocator> >::other >  _left_samples_type;
  _left_samples_type left_samples;

   typedef std::vector< ::roscpp_tutorials::LaneSampleAssociation_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::LaneSampleAssociation_<ContainerAllocator> >::other >  _right_samples_type;
  _right_samples_type right_samples;

   typedef std::vector< ::roscpp_tutorials::PassageType_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::PassageType_<ContainerAllocator> >::other >  _types_type;
  _types_type types;

   typedef std::vector< ::roscpp_tutorials::PassageSpeed_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::PassageSpeed_<ContainerAllocator> >::other >  _speed_limit_type;
  _speed_limit_type speed_limit;

   typedef  ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator>  _left_boundary_type;
  _left_boundary_type left_boundary;

   typedef  ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator>  _right_boundary_type;
  _right_boundary_type right_boundary;

   typedef std::vector< ::roscpp_tutorials::CurveSign_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >::other >  _curve_signs_type;
  _curve_signs_type curve_signs;

   typedef std::vector< ::roscpp_tutorials::PolygonSign_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::PolygonSign_<ContainerAllocator> >::other >  _polygon_signs_type;
  _polygon_signs_type polygon_signs;

   typedef std::vector< ::roscpp_tutorials::Signal_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::Signal_<ContainerAllocator> >::other >  _signals_type;
  _signals_type signals;

   typedef uint8_t _is_on_passage_type;
  _is_on_passage_type is_on_passage;

   typedef uint8_t _can_exit_type;
  _can_exit_type can_exit;

   typedef uint16_t _change_lane_type_type;
  _change_lane_type_type change_lane_type;

   typedef float _remain_length_type;
  _remain_length_type remain_length;

   typedef uint8_t _stop_for_destination_type;
  _stop_for_destination_type stop_for_destination;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _predecessor_id_type;
  _predecessor_id_type predecessor_id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _sucessor_id_type;
  _sucessor_id_type sucessor_id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _left_neighbor_id_type;
  _left_neighbor_id_type left_neighbor_id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _right_neighbor_id_type;
  _right_neighbor_id_type right_neighbor_id;

   typedef std::vector< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >::other >  _parking_spaces_type;
  _parking_spaces_type parking_spaces;

   typedef std::vector< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> >::other >  _turns_type;
  _turns_type turns;





  typedef boost::shared_ptr< ::roscpp_tutorials::Passage_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::Passage_<ContainerAllocator> const> ConstPtr;

}; // struct Passage_

typedef ::roscpp_tutorials::Passage_<std::allocator<void> > Passage;

typedef boost::shared_ptr< ::roscpp_tutorials::Passage > PassagePtr;
typedef boost::shared_ptr< ::roscpp_tutorials::Passage const> PassageConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::Passage_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::Passage_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::Passage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::Passage_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::Passage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::Passage_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::Passage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::Passage_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::Passage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9ee50ee975694006b8eed20c8e252939";
  }

  static const char* value(const ::roscpp_tutorials::Passage_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9ee50ee975694006ULL;
  static const uint64_t static_value2 = 0xb8eed20c8e252939ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::Passage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/Passage";
  }

  static const char* value(const ::roscpp_tutorials::Passage_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::Passage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string id\n\
PathPoint[] path\n\
LaneSampleAssociation[] left_samples\n\
LaneSampleAssociation[] right_samples\n\
PassageType[] types\n\
PassageSpeed[] speed_limit\n\
NavigationLaneBoundary left_boundary\n\
NavigationLaneBoundary right_boundary\n\
CurveSign[] curve_signs\n\
PolygonSign[] polygon_signs\n\
Signal[] signals\n\
bool is_on_passage\n\
bool can_exit\n\
uint16 change_lane_type\n\
float32 remain_length\n\
bool stop_for_destination\n\
string predecessor_id\n\
string sucessor_id\n\
string left_neighbor_id\n\
string right_neighbor_id\n\
ParkingSpace[] parking_spaces\n\
PassageTurn[] turns\n\
================================================================================\n\
MSG: roscpp_tutorials/PathPoint\n\
Point3d position\n\
float32 theta\n\
float32 kappa\n\
float32 s\n\
float32 dkappa\n\
float32 ddkappa\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/Point3d\n\
	float32 x  # in meters or m/s\n\
	float32 y  # in meters or m/s\n\
	float32 z  # height in meters or m/s\n\
================================================================================\n\
MSG: roscpp_tutorials/LaneSampleAssociation\n\
float32 start_s\n\
float32 end_s\n\
float32 width\n\
================================================================================\n\
MSG: roscpp_tutorials/PassageType\n\
    uint16 type\n\
    float32 start_s\n\
    float32 end_s\n\
================================================================================\n\
MSG: roscpp_tutorials/PassageSpeed\n\
    float32 speed_limit\n\
    float32 start_s\n\
    float32 end_s\n\
================================================================================\n\
MSG: roscpp_tutorials/NavigationLaneBoundary\n\
LaneBoundaryType[] type\n\
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
\n\
================================================================================\n\
MSG: roscpp_tutorials/CurveSign\n\
    string id\n\
    uint16 type\n\
    string other_type\n\
    Curved segments\n\
    float32 start_s\n\
    float32 end_s\n\
================================================================================\n\
MSG: roscpp_tutorials/PolygonSign\n\
    string id\n\
    uint16 type\n\
    string other_type\n\
    Curved points\n\
    float32 start_s\n\
    float32 end_s\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/Signal\n\
    string id\n\
    uint16 type\n\
    float32[3] position\n\
    Curved roi_polygon\n\
    Curved stop_line\n\
    float32 start_s\n\
    float32 end_s\n\
================================================================================\n\
MSG: roscpp_tutorials/ParkingSpace\n\
string id\n\
uint16 type\n\
Curved polygon\n\
float32 heading\n\
float32 start_s\n\
float32 end_s\n\
================================================================================\n\
MSG: roscpp_tutorials/PassageTurn\n\
    uint16 turn_type\n\
    float32 start_s\n\
    float32 end_s\n\
";
  }

  static const char* value(const ::roscpp_tutorials::Passage_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::Passage_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.path);
      stream.next(m.left_samples);
      stream.next(m.right_samples);
      stream.next(m.types);
      stream.next(m.speed_limit);
      stream.next(m.left_boundary);
      stream.next(m.right_boundary);
      stream.next(m.curve_signs);
      stream.next(m.polygon_signs);
      stream.next(m.signals);
      stream.next(m.is_on_passage);
      stream.next(m.can_exit);
      stream.next(m.change_lane_type);
      stream.next(m.remain_length);
      stream.next(m.stop_for_destination);
      stream.next(m.predecessor_id);
      stream.next(m.sucessor_id);
      stream.next(m.left_neighbor_id);
      stream.next(m.right_neighbor_id);
      stream.next(m.parking_spaces);
      stream.next(m.turns);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Passage_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::Passage_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::Passage_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.id);
    s << indent << "path[]" << std::endl;
    for (size_t i = 0; i < v.path.size(); ++i)
    {
      s << indent << "  path[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::PathPoint_<ContainerAllocator> >::stream(s, indent + "    ", v.path[i]);
    }
    s << indent << "left_samples[]" << std::endl;
    for (size_t i = 0; i < v.left_samples.size(); ++i)
    {
      s << indent << "  left_samples[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::LaneSampleAssociation_<ContainerAllocator> >::stream(s, indent + "    ", v.left_samples[i]);
    }
    s << indent << "right_samples[]" << std::endl;
    for (size_t i = 0; i < v.right_samples.size(); ++i)
    {
      s << indent << "  right_samples[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::LaneSampleAssociation_<ContainerAllocator> >::stream(s, indent + "    ", v.right_samples[i]);
    }
    s << indent << "types[]" << std::endl;
    for (size_t i = 0; i < v.types.size(); ++i)
    {
      s << indent << "  types[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::PassageType_<ContainerAllocator> >::stream(s, indent + "    ", v.types[i]);
    }
    s << indent << "speed_limit[]" << std::endl;
    for (size_t i = 0; i < v.speed_limit.size(); ++i)
    {
      s << indent << "  speed_limit[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::PassageSpeed_<ContainerAllocator> >::stream(s, indent + "    ", v.speed_limit[i]);
    }
    s << indent << "left_boundary: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >::stream(s, indent + "  ", v.left_boundary);
    s << indent << "right_boundary: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::NavigationLaneBoundary_<ContainerAllocator> >::stream(s, indent + "  ", v.right_boundary);
    s << indent << "curve_signs[]" << std::endl;
    for (size_t i = 0; i < v.curve_signs.size(); ++i)
    {
      s << indent << "  curve_signs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::CurveSign_<ContainerAllocator> >::stream(s, indent + "    ", v.curve_signs[i]);
    }
    s << indent << "polygon_signs[]" << std::endl;
    for (size_t i = 0; i < v.polygon_signs.size(); ++i)
    {
      s << indent << "  polygon_signs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::PolygonSign_<ContainerAllocator> >::stream(s, indent + "    ", v.polygon_signs[i]);
    }
    s << indent << "signals[]" << std::endl;
    for (size_t i = 0; i < v.signals.size(); ++i)
    {
      s << indent << "  signals[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::Signal_<ContainerAllocator> >::stream(s, indent + "    ", v.signals[i]);
    }
    s << indent << "is_on_passage: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_on_passage);
    s << indent << "can_exit: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.can_exit);
    s << indent << "change_lane_type: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.change_lane_type);
    s << indent << "remain_length: ";
    Printer<float>::stream(s, indent + "  ", v.remain_length);
    s << indent << "stop_for_destination: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.stop_for_destination);
    s << indent << "predecessor_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.predecessor_id);
    s << indent << "sucessor_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.sucessor_id);
    s << indent << "left_neighbor_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.left_neighbor_id);
    s << indent << "right_neighbor_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.right_neighbor_id);
    s << indent << "parking_spaces[]" << std::endl;
    for (size_t i = 0; i < v.parking_spaces.size(); ++i)
    {
      s << indent << "  parking_spaces[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >::stream(s, indent + "    ", v.parking_spaces[i]);
    }
    s << indent << "turns[]" << std::endl;
    for (size_t i = 0; i < v.turns.size(); ++i)
    {
      s << indent << "  turns[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::PassageTurn_<ContainerAllocator> >::stream(s, indent + "    ", v.turns[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_PASSAGE_H
