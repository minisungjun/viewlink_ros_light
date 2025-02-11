// Generated by gencpp from file kari_dronecop_rd_payload_mgmt/payload_vu8_lidar_chunk.msg
// DO NOT EDIT!


#ifndef KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_VU8_LIDAR_CHUNK_H
#define KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_VU8_LIDAR_CHUNK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kari_dronecop_rd_payload_mgmt
{
template <class ContainerAllocator>
struct payload_vu8_lidar_chunk_
{
  typedef payload_vu8_lidar_chunk_<ContainerAllocator> Type;

  payload_vu8_lidar_chunk_()
    : range(0.0)
    , angle(0.0)
    , xrel(0.0)
    , yrel(0.0)
    , amplitude(0.0)
    , segment_status(false)
    , error_code(0)  {
    }
  payload_vu8_lidar_chunk_(const ContainerAllocator& _alloc)
    : range(0.0)
    , angle(0.0)
    , xrel(0.0)
    , yrel(0.0)
    , amplitude(0.0)
    , segment_status(false)
    , error_code(0)  {
  (void)_alloc;
    }



   typedef float _range_type;
  _range_type range;

   typedef float _angle_type;
  _angle_type angle;

   typedef float _xrel_type;
  _xrel_type xrel;

   typedef float _yrel_type;
  _yrel_type yrel;

   typedef float _amplitude_type;
  _amplitude_type amplitude;

   typedef uint8_t _segment_status_type;
  _segment_status_type segment_status;

   typedef int8_t _error_code_type;
  _error_code_type error_code;





  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> const> ConstPtr;

}; // struct payload_vu8_lidar_chunk_

typedef ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<std::allocator<void> > payload_vu8_lidar_chunk;

typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk > payload_vu8_lidar_chunkPtr;
typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk const> payload_vu8_lidar_chunkConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator2> & rhs)
{
  return lhs.range == rhs.range &&
    lhs.angle == rhs.angle &&
    lhs.xrel == rhs.xrel &&
    lhs.yrel == rhs.yrel &&
    lhs.amplitude == rhs.amplitude &&
    lhs.segment_status == rhs.segment_status &&
    lhs.error_code == rhs.error_code;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kari_dronecop_rd_payload_mgmt

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> >
{
  static const char* value()
  {
    return "112ecc50c133becb9d4ef127fa329694";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x112ecc50c133becbULL;
  static const uint64_t static_value2 = 0x9d4ef127fa329694ULL;
};

template<class ContainerAllocator>
struct DataType< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kari_dronecop_rd_payload_mgmt/payload_vu8_lidar_chunk";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Payload TmTc in ROS env., Vu8 LiDAR\n"
"float32 range\n"
"float32 angle\n"
"float32 xrel\n"
"float32 yrel\n"
"float32 amplitude\n"
"bool segment_status\n"
"int8 error_code\n"
;
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.range);
      stream.next(m.angle);
      stream.next(m.xrel);
      stream.next(m.yrel);
      stream.next(m.amplitude);
      stream.next(m.segment_status);
      stream.next(m.error_code);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct payload_vu8_lidar_chunk_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kari_dronecop_rd_payload_mgmt::payload_vu8_lidar_chunk_<ContainerAllocator>& v)
  {
    s << indent << "range: ";
    Printer<float>::stream(s, indent + "  ", v.range);
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
    s << indent << "xrel: ";
    Printer<float>::stream(s, indent + "  ", v.xrel);
    s << indent << "yrel: ";
    Printer<float>::stream(s, indent + "  ", v.yrel);
    s << indent << "amplitude: ";
    Printer<float>::stream(s, indent + "  ", v.amplitude);
    s << indent << "segment_status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.segment_status);
    s << indent << "error_code: ";
    Printer<int8_t>::stream(s, indent + "  ", v.error_code);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_VU8_LIDAR_CHUNK_H
