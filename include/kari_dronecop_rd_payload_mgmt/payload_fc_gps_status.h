// Generated by gencpp from file kari_dronecop_rd_payload_mgmt/payload_fc_gps_status.msg
// DO NOT EDIT!


#ifndef KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_FC_GPS_STATUS_H
#define KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_FC_GPS_STATUS_H


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
struct payload_fc_gps_status_
{
  typedef payload_fc_gps_status_<ContainerAllocator> Type;

  payload_fc_gps_status_()
    : header()
    , fix_type(0)
    , lat(0)
    , lon(0)
    , alt(0)
    , eph(0)
    , epv(0)
    , vel(0)
    , cog(0)
    , satellites_visible(0)
    , alt_ellipsoid(0)
    , h_acc(0)
    , v_acc(0)
    , vel_acc(0)
    , hdg_acc(0)
    , yaw(0)  {
    }
  payload_fc_gps_status_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , fix_type(0)
    , lat(0)
    , lon(0)
    , alt(0)
    , eph(0)
    , epv(0)
    , vel(0)
    , cog(0)
    , satellites_visible(0)
    , alt_ellipsoid(0)
    , h_acc(0)
    , v_acc(0)
    , vel_acc(0)
    , hdg_acc(0)
    , yaw(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _fix_type_type;
  _fix_type_type fix_type;

   typedef int32_t _lat_type;
  _lat_type lat;

   typedef int32_t _lon_type;
  _lon_type lon;

   typedef int32_t _alt_type;
  _alt_type alt;

   typedef uint16_t _eph_type;
  _eph_type eph;

   typedef uint16_t _epv_type;
  _epv_type epv;

   typedef uint16_t _vel_type;
  _vel_type vel;

   typedef uint16_t _cog_type;
  _cog_type cog;

   typedef uint8_t _satellites_visible_type;
  _satellites_visible_type satellites_visible;

   typedef int32_t _alt_ellipsoid_type;
  _alt_ellipsoid_type alt_ellipsoid;

   typedef uint32_t _h_acc_type;
  _h_acc_type h_acc;

   typedef uint32_t _v_acc_type;
  _v_acc_type v_acc;

   typedef uint32_t _vel_acc_type;
  _vel_acc_type vel_acc;

   typedef uint32_t _hdg_acc_type;
  _hdg_acc_type hdg_acc;

   typedef uint16_t _yaw_type;
  _yaw_type yaw;





  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> const> ConstPtr;

}; // struct payload_fc_gps_status_

typedef ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<std::allocator<void> > payload_fc_gps_status;

typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status > payload_fc_gps_statusPtr;
typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status const> payload_fc_gps_statusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.fix_type == rhs.fix_type &&
    lhs.lat == rhs.lat &&
    lhs.lon == rhs.lon &&
    lhs.alt == rhs.alt &&
    lhs.eph == rhs.eph &&
    lhs.epv == rhs.epv &&
    lhs.vel == rhs.vel &&
    lhs.cog == rhs.cog &&
    lhs.satellites_visible == rhs.satellites_visible &&
    lhs.alt_ellipsoid == rhs.alt_ellipsoid &&
    lhs.h_acc == rhs.h_acc &&
    lhs.v_acc == rhs.v_acc &&
    lhs.vel_acc == rhs.vel_acc &&
    lhs.hdg_acc == rhs.hdg_acc &&
    lhs.yaw == rhs.yaw;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kari_dronecop_rd_payload_mgmt

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "032e33434d9fcfdca027c2cce6107932";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x032e33434d9fcfdcULL;
  static const uint64_t static_value2 = 0xa027c2cce6107932ULL;
};

template<class ContainerAllocator>
struct DataType< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kari_dronecop_rd_payload_mgmt/payload_fc_gps_status";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Payload TmTc in ROS env., FC-GPS status\n"
"Header header\n"
"uint8 fix_type\n"
"int32 lat\n"
"int32 lon\n"
"int32 alt\n"
"uint16 eph\n"
"uint16 epv\n"
"uint16 vel\n"
"uint16 cog\n"
"uint8 satellites_visible\n"
"int32 alt_ellipsoid\n"
"uint32 h_acc\n"
"uint32 v_acc\n"
"uint32 vel_acc\n"
"uint32 hdg_acc\n"
"uint16 yaw\n"
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

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.fix_type);
      stream.next(m.lat);
      stream.next(m.lon);
      stream.next(m.alt);
      stream.next(m.eph);
      stream.next(m.epv);
      stream.next(m.vel);
      stream.next(m.cog);
      stream.next(m.satellites_visible);
      stream.next(m.alt_ellipsoid);
      stream.next(m.h_acc);
      stream.next(m.v_acc);
      stream.next(m.vel_acc);
      stream.next(m.hdg_acc);
      stream.next(m.yaw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct payload_fc_gps_status_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kari_dronecop_rd_payload_mgmt::payload_fc_gps_status_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "fix_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.fix_type);
    s << indent << "lat: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lat);
    s << indent << "lon: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lon);
    s << indent << "alt: ";
    Printer<int32_t>::stream(s, indent + "  ", v.alt);
    s << indent << "eph: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.eph);
    s << indent << "epv: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.epv);
    s << indent << "vel: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.vel);
    s << indent << "cog: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.cog);
    s << indent << "satellites_visible: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.satellites_visible);
    s << indent << "alt_ellipsoid: ";
    Printer<int32_t>::stream(s, indent + "  ", v.alt_ellipsoid);
    s << indent << "h_acc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.h_acc);
    s << indent << "v_acc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.v_acc);
    s << indent << "vel_acc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.vel_acc);
    s << indent << "hdg_acc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.hdg_acc);
    s << indent << "yaw: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.yaw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_FC_GPS_STATUS_H
