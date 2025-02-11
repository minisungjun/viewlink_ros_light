// Generated by gencpp from file kari_dronecop_rd_payload_mgmt/payload_decepter_status.msg
// DO NOT EDIT!


#ifndef KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_DECEPTER_STATUS_H
#define KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_DECEPTER_STATUS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace kari_dronecop_rd_payload_mgmt
{
template <class ContainerAllocator>
struct payload_decepter_status_
{
  typedef payload_decepter_status_<ContainerAllocator> Type;

  payload_decepter_status_()
    : header()
    , dct_status_mode(0)
    , rsn_status_ack(0)
    , rsn_control_target_id(0)
    , dct_neutralize_result(0)
    , rsn_control_dlatitude(0.0)
    , rsn_control_dlongitude(0.0)
    , rsn_control_daltitude(0.0)
    , rsn_control_speedxy(0.0)
    , rsn_control_speedx(0.0)
    , rsn_control_speedy(0.0)
    , rsn_control_speedz(0.0)
    , dct_status_error(0)  {
    }
  payload_decepter_status_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , dct_status_mode(0)
    , rsn_status_ack(0)
    , rsn_control_target_id(0)
    , dct_neutralize_result(0)
    , rsn_control_dlatitude(0.0)
    , rsn_control_dlongitude(0.0)
    , rsn_control_daltitude(0.0)
    , rsn_control_speedxy(0.0)
    , rsn_control_speedx(0.0)
    , rsn_control_speedy(0.0)
    , rsn_control_speedz(0.0)
    , dct_status_error(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _dct_status_mode_type;
  _dct_status_mode_type dct_status_mode;

   typedef uint8_t _rsn_status_ack_type;
  _rsn_status_ack_type rsn_status_ack;

   typedef uint32_t _rsn_control_target_id_type;
  _rsn_control_target_id_type rsn_control_target_id;

   typedef uint8_t _dct_neutralize_result_type;
  _dct_neutralize_result_type dct_neutralize_result;

   typedef double _rsn_control_dlatitude_type;
  _rsn_control_dlatitude_type rsn_control_dlatitude;

   typedef double _rsn_control_dlongitude_type;
  _rsn_control_dlongitude_type rsn_control_dlongitude;

   typedef double _rsn_control_daltitude_type;
  _rsn_control_daltitude_type rsn_control_daltitude;

   typedef float _rsn_control_speedxy_type;
  _rsn_control_speedxy_type rsn_control_speedxy;

   typedef float _rsn_control_speedx_type;
  _rsn_control_speedx_type rsn_control_speedx;

   typedef float _rsn_control_speedy_type;
  _rsn_control_speedy_type rsn_control_speedy;

   typedef float _rsn_control_speedz_type;
  _rsn_control_speedz_type rsn_control_speedz;

   typedef uint8_t _dct_status_error_type;
  _dct_status_error_type dct_status_error;





  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> const> ConstPtr;

}; // struct payload_decepter_status_

typedef ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<std::allocator<void> > payload_decepter_status;

typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status > payload_decepter_statusPtr;
typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status const> payload_decepter_statusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.dct_status_mode == rhs.dct_status_mode &&
    lhs.rsn_status_ack == rhs.rsn_status_ack &&
    lhs.rsn_control_target_id == rhs.rsn_control_target_id &&
    lhs.dct_neutralize_result == rhs.dct_neutralize_result &&
    lhs.rsn_control_dlatitude == rhs.rsn_control_dlatitude &&
    lhs.rsn_control_dlongitude == rhs.rsn_control_dlongitude &&
    lhs.rsn_control_daltitude == rhs.rsn_control_daltitude &&
    lhs.rsn_control_speedxy == rhs.rsn_control_speedxy &&
    lhs.rsn_control_speedx == rhs.rsn_control_speedx &&
    lhs.rsn_control_speedy == rhs.rsn_control_speedy &&
    lhs.rsn_control_speedz == rhs.rsn_control_speedz &&
    lhs.dct_status_error == rhs.dct_status_error;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kari_dronecop_rd_payload_mgmt

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9697a428d5405d9322438a6dda4a1289";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9697a428d5405d93ULL;
  static const uint64_t static_value2 = 0x22438a6dda4a1289ULL;
};

template<class ContainerAllocator>
struct DataType< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kari_dronecop_rd_payload_mgmt/payload_decepter_status";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Payload TmTc in ROS env., Decepter, status\n"
"# Decepter mode status\n"
"# Decepter response acknowledgement\n"
"# Decepter neutralize target ID, response	\n"
"# Decepter neutralized result\n"
"# Decepter neutralize target position during control, latitude[deg]			\n"
"# Decepter neutralize target position during control, longitude[deg]\n"
"# Decepter neutralize target position during control, altitude[m]	\n"
"# Decepter neutralize target velocity during control, xy[m/s]\n"
"# Decepter neutralize target velocity during control, x[m/s]\n"
"# Decepter neutralize target velocity during control, y[m/s]\n"
"# Decepter neutralize target velocity during control, z[m/s]\n"
"# Decepter error status\n"
"Header header\n"
"uint8 dct_status_mode\n"
"uint8 rsn_status_ack\n"
"uint32 rsn_control_target_id\n"
"uint8 dct_neutralize_result\n"
"float64 rsn_control_dlatitude\n"
"float64 rsn_control_dlongitude\n"
"float64 rsn_control_daltitude\n"
"float32 rsn_control_speedxy\n"
"float32 rsn_control_speedx\n"
"float32 rsn_control_speedy\n"
"float32 rsn_control_speedz							\n"
"uint8 dct_status_error\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.dct_status_mode);
      stream.next(m.rsn_status_ack);
      stream.next(m.rsn_control_target_id);
      stream.next(m.dct_neutralize_result);
      stream.next(m.rsn_control_dlatitude);
      stream.next(m.rsn_control_dlongitude);
      stream.next(m.rsn_control_daltitude);
      stream.next(m.rsn_control_speedxy);
      stream.next(m.rsn_control_speedx);
      stream.next(m.rsn_control_speedy);
      stream.next(m.rsn_control_speedz);
      stream.next(m.dct_status_error);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct payload_decepter_status_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kari_dronecop_rd_payload_mgmt::payload_decepter_status_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "dct_status_mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.dct_status_mode);
    s << indent << "rsn_status_ack: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.rsn_status_ack);
    s << indent << "rsn_control_target_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.rsn_control_target_id);
    s << indent << "dct_neutralize_result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.dct_neutralize_result);
    s << indent << "rsn_control_dlatitude: ";
    Printer<double>::stream(s, indent + "  ", v.rsn_control_dlatitude);
    s << indent << "rsn_control_dlongitude: ";
    Printer<double>::stream(s, indent + "  ", v.rsn_control_dlongitude);
    s << indent << "rsn_control_daltitude: ";
    Printer<double>::stream(s, indent + "  ", v.rsn_control_daltitude);
    s << indent << "rsn_control_speedxy: ";
    Printer<float>::stream(s, indent + "  ", v.rsn_control_speedxy);
    s << indent << "rsn_control_speedx: ";
    Printer<float>::stream(s, indent + "  ", v.rsn_control_speedx);
    s << indent << "rsn_control_speedy: ";
    Printer<float>::stream(s, indent + "  ", v.rsn_control_speedy);
    s << indent << "rsn_control_speedz: ";
    Printer<float>::stream(s, indent + "  ", v.rsn_control_speedz);
    s << indent << "dct_status_error: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.dct_status_error);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_DECEPTER_STATUS_H
