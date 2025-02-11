// Generated by gencpp from file kari_dronecop_rd_payload_mgmt/payload_fc_states_wo_gps.msg
// DO NOT EDIT!


#ifndef KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_FC_STATES_WO_GPS_H
#define KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_FC_STATES_WO_GPS_H


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
struct payload_fc_states_wo_gps_
{
  typedef payload_fc_states_wo_gps_<ContainerAllocator> Type;

  payload_fc_states_wo_gps_()
    : header()
    , ub(0.0)
    , vb(0.0)
    , wb(0.0)
    , roll(0.0)
    , pitch(0.0)
    , yaw(0.0)
    , qx(0.0)
    , qy(0.0)
    , qz(0.0)
    , qw(0.0)
    , prate(0.0)
    , qrate(0.0)
    , rrate(0.0)
    , xned(0.0)
    , yned(0.0)
    , zned(0.0)
    , ax_c(0.0)
    , ay_c(0.0)
    , az_c(0.0)
    , vnx(0.0)
    , vny(0.0)
    , vnz(0.0)
    , comp_head(0.0)
    , rel_alt(0.0)  {
    }
  payload_fc_states_wo_gps_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , ub(0.0)
    , vb(0.0)
    , wb(0.0)
    , roll(0.0)
    , pitch(0.0)
    , yaw(0.0)
    , qx(0.0)
    , qy(0.0)
    , qz(0.0)
    , qw(0.0)
    , prate(0.0)
    , qrate(0.0)
    , rrate(0.0)
    , xned(0.0)
    , yned(0.0)
    , zned(0.0)
    , ax_c(0.0)
    , ay_c(0.0)
    , az_c(0.0)
    , vnx(0.0)
    , vny(0.0)
    , vnz(0.0)
    , comp_head(0.0)
    , rel_alt(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _ub_type;
  _ub_type ub;

   typedef float _vb_type;
  _vb_type vb;

   typedef float _wb_type;
  _wb_type wb;

   typedef float _roll_type;
  _roll_type roll;

   typedef float _pitch_type;
  _pitch_type pitch;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef float _qx_type;
  _qx_type qx;

   typedef float _qy_type;
  _qy_type qy;

   typedef float _qz_type;
  _qz_type qz;

   typedef float _qw_type;
  _qw_type qw;

   typedef float _prate_type;
  _prate_type prate;

   typedef float _qrate_type;
  _qrate_type qrate;

   typedef float _rrate_type;
  _rrate_type rrate;

   typedef float _xned_type;
  _xned_type xned;

   typedef float _yned_type;
  _yned_type yned;

   typedef float _zned_type;
  _zned_type zned;

   typedef float _ax_c_type;
  _ax_c_type ax_c;

   typedef float _ay_c_type;
  _ay_c_type ay_c;

   typedef float _az_c_type;
  _az_c_type az_c;

   typedef float _vnx_type;
  _vnx_type vnx;

   typedef float _vny_type;
  _vny_type vny;

   typedef float _vnz_type;
  _vnz_type vnz;

   typedef float _comp_head_type;
  _comp_head_type comp_head;

   typedef float _rel_alt_type;
  _rel_alt_type rel_alt;





  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> const> ConstPtr;

}; // struct payload_fc_states_wo_gps_

typedef ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<std::allocator<void> > payload_fc_states_wo_gps;

typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps > payload_fc_states_wo_gpsPtr;
typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps const> payload_fc_states_wo_gpsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.ub == rhs.ub &&
    lhs.vb == rhs.vb &&
    lhs.wb == rhs.wb &&
    lhs.roll == rhs.roll &&
    lhs.pitch == rhs.pitch &&
    lhs.yaw == rhs.yaw &&
    lhs.qx == rhs.qx &&
    lhs.qy == rhs.qy &&
    lhs.qz == rhs.qz &&
    lhs.qw == rhs.qw &&
    lhs.prate == rhs.prate &&
    lhs.qrate == rhs.qrate &&
    lhs.rrate == rhs.rrate &&
    lhs.xned == rhs.xned &&
    lhs.yned == rhs.yned &&
    lhs.zned == rhs.zned &&
    lhs.ax_c == rhs.ax_c &&
    lhs.ay_c == rhs.ay_c &&
    lhs.az_c == rhs.az_c &&
    lhs.vnx == rhs.vnx &&
    lhs.vny == rhs.vny &&
    lhs.vnz == rhs.vnz &&
    lhs.comp_head == rhs.comp_head &&
    lhs.rel_alt == rhs.rel_alt;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kari_dronecop_rd_payload_mgmt

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "186bad9f4145b9116e8b87c2767468ba";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x186bad9f4145b911ULL;
  static const uint64_t static_value2 = 0x6e8b87c2767468baULL;
};

template<class ContainerAllocator>
struct DataType< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kari_dronecop_rd_payload_mgmt/payload_fc_states_wo_gps";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Payload TmTc in ROS env., FC states without gps\n"
"Header header\n"
"float32 ub\n"
"float32 vb\n"
"float32 wb\n"
"float32 roll\n"
"float32 pitch\n"
"float32 yaw\n"
"float32 qx\n"
"float32 qy\n"
"float32 qz\n"
"float32 qw\n"
"float32 prate\n"
"float32 qrate\n"
"float32 rrate\n"
"float32 xned\n"
"float32 yned\n"
"float32 zned\n"
"float32 ax_c\n"
"float32 ay_c\n"
"float32 az_c\n"
"float32 vnx\n"
"float32 vny\n"
"float32 vnz\n"
"float32 comp_head\n"
"float32 rel_alt\n"
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

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.ub);
      stream.next(m.vb);
      stream.next(m.wb);
      stream.next(m.roll);
      stream.next(m.pitch);
      stream.next(m.yaw);
      stream.next(m.qx);
      stream.next(m.qy);
      stream.next(m.qz);
      stream.next(m.qw);
      stream.next(m.prate);
      stream.next(m.qrate);
      stream.next(m.rrate);
      stream.next(m.xned);
      stream.next(m.yned);
      stream.next(m.zned);
      stream.next(m.ax_c);
      stream.next(m.ay_c);
      stream.next(m.az_c);
      stream.next(m.vnx);
      stream.next(m.vny);
      stream.next(m.vnz);
      stream.next(m.comp_head);
      stream.next(m.rel_alt);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct payload_fc_states_wo_gps_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kari_dronecop_rd_payload_mgmt::payload_fc_states_wo_gps_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "ub: ";
    Printer<float>::stream(s, indent + "  ", v.ub);
    s << indent << "vb: ";
    Printer<float>::stream(s, indent + "  ", v.vb);
    s << indent << "wb: ";
    Printer<float>::stream(s, indent + "  ", v.wb);
    s << indent << "roll: ";
    Printer<float>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<float>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "qx: ";
    Printer<float>::stream(s, indent + "  ", v.qx);
    s << indent << "qy: ";
    Printer<float>::stream(s, indent + "  ", v.qy);
    s << indent << "qz: ";
    Printer<float>::stream(s, indent + "  ", v.qz);
    s << indent << "qw: ";
    Printer<float>::stream(s, indent + "  ", v.qw);
    s << indent << "prate: ";
    Printer<float>::stream(s, indent + "  ", v.prate);
    s << indent << "qrate: ";
    Printer<float>::stream(s, indent + "  ", v.qrate);
    s << indent << "rrate: ";
    Printer<float>::stream(s, indent + "  ", v.rrate);
    s << indent << "xned: ";
    Printer<float>::stream(s, indent + "  ", v.xned);
    s << indent << "yned: ";
    Printer<float>::stream(s, indent + "  ", v.yned);
    s << indent << "zned: ";
    Printer<float>::stream(s, indent + "  ", v.zned);
    s << indent << "ax_c: ";
    Printer<float>::stream(s, indent + "  ", v.ax_c);
    s << indent << "ay_c: ";
    Printer<float>::stream(s, indent + "  ", v.ay_c);
    s << indent << "az_c: ";
    Printer<float>::stream(s, indent + "  ", v.az_c);
    s << indent << "vnx: ";
    Printer<float>::stream(s, indent + "  ", v.vnx);
    s << indent << "vny: ";
    Printer<float>::stream(s, indent + "  ", v.vny);
    s << indent << "vnz: ";
    Printer<float>::stream(s, indent + "  ", v.vnz);
    s << indent << "comp_head: ";
    Printer<float>::stream(s, indent + "  ", v.comp_head);
    s << indent << "rel_alt: ";
    Printer<float>::stream(s, indent + "  ", v.rel_alt);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_FC_STATES_WO_GPS_H
