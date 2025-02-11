// Generated by gencpp from file kari_dronecop_rd_payload_mgmt/ViewLinkJoyCtrlCmd.msg
// DO NOT EDIT!


#ifndef KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_VIEWLINKJOYCTRLCMD_H
#define KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_VIEWLINKJOYCTRLCMD_H


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
struct ViewLinkJoyCtrlCmd_
{
  typedef ViewLinkJoyCtrlCmd_<ContainerAllocator> Type;

  ViewLinkJoyCtrlCmd_()
    : header()
    , pan_rate_axis(0.0)
    , tilt_rate_axis(0.0)
    , pan_ang_axis(0.0)
    , tilt_ang_axis(0.0)
    , zoom_axis(0.0)
    , auto_mode(0.0)
    , manual_mode(0.0)
    , shoot(0.0)
    , homing(0.0)
    , eo_img_submode(0.0)
    , ir_img_submode(0.0)
    , all_img_submode(0.0)
    , stab_submode(0.0)
    , ang_ctrl_submode(0.0)
    , angrate_ctrl_submode(0.0)
    , netgun_mode(0.0)  {
    }
  ViewLinkJoyCtrlCmd_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pan_rate_axis(0.0)
    , tilt_rate_axis(0.0)
    , pan_ang_axis(0.0)
    , tilt_ang_axis(0.0)
    , zoom_axis(0.0)
    , auto_mode(0.0)
    , manual_mode(0.0)
    , shoot(0.0)
    , homing(0.0)
    , eo_img_submode(0.0)
    , ir_img_submode(0.0)
    , all_img_submode(0.0)
    , stab_submode(0.0)
    , ang_ctrl_submode(0.0)
    , angrate_ctrl_submode(0.0)
    , netgun_mode(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _pan_rate_axis_type;
  _pan_rate_axis_type pan_rate_axis;

   typedef double _tilt_rate_axis_type;
  _tilt_rate_axis_type tilt_rate_axis;

   typedef double _pan_ang_axis_type;
  _pan_ang_axis_type pan_ang_axis;

   typedef double _tilt_ang_axis_type;
  _tilt_ang_axis_type tilt_ang_axis;

   typedef double _zoom_axis_type;
  _zoom_axis_type zoom_axis;

   typedef double _auto_mode_type;
  _auto_mode_type auto_mode;

   typedef double _manual_mode_type;
  _manual_mode_type manual_mode;

   typedef double _shoot_type;
  _shoot_type shoot;

   typedef double _homing_type;
  _homing_type homing;

   typedef double _eo_img_submode_type;
  _eo_img_submode_type eo_img_submode;

   typedef double _ir_img_submode_type;
  _ir_img_submode_type ir_img_submode;

   typedef double _all_img_submode_type;
  _all_img_submode_type all_img_submode;

   typedef double _stab_submode_type;
  _stab_submode_type stab_submode;

   typedef double _ang_ctrl_submode_type;
  _ang_ctrl_submode_type ang_ctrl_submode;

   typedef double _angrate_ctrl_submode_type;
  _angrate_ctrl_submode_type angrate_ctrl_submode;

   typedef double _netgun_mode_type;
  _netgun_mode_type netgun_mode;





  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> const> ConstPtr;

}; // struct ViewLinkJoyCtrlCmd_

typedef ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<std::allocator<void> > ViewLinkJoyCtrlCmd;

typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd > ViewLinkJoyCtrlCmdPtr;
typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd const> ViewLinkJoyCtrlCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.pan_rate_axis == rhs.pan_rate_axis &&
    lhs.tilt_rate_axis == rhs.tilt_rate_axis &&
    lhs.pan_ang_axis == rhs.pan_ang_axis &&
    lhs.tilt_ang_axis == rhs.tilt_ang_axis &&
    lhs.zoom_axis == rhs.zoom_axis &&
    lhs.auto_mode == rhs.auto_mode &&
    lhs.manual_mode == rhs.manual_mode &&
    lhs.shoot == rhs.shoot &&
    lhs.homing == rhs.homing &&
    lhs.eo_img_submode == rhs.eo_img_submode &&
    lhs.ir_img_submode == rhs.ir_img_submode &&
    lhs.all_img_submode == rhs.all_img_submode &&
    lhs.stab_submode == rhs.stab_submode &&
    lhs.ang_ctrl_submode == rhs.ang_ctrl_submode &&
    lhs.angrate_ctrl_submode == rhs.angrate_ctrl_submode &&
    lhs.netgun_mode == rhs.netgun_mode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kari_dronecop_rd_payload_mgmt

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0d734de8d6482702dd0b2439ce358060";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0d734de8d6482702ULL;
  static const uint64_t static_value2 = 0xdd0b2439ce358060ULL;
};

template<class ContainerAllocator>
struct DataType< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kari_dronecop_rd_payload_mgmt/ViewLinkJoyCtrlCmd";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float64 pan_rate_axis\n"
"float64 tilt_rate_axis\n"
"float64 pan_ang_axis\n"
"float64 tilt_ang_axis\n"
"float64 zoom_axis\n"
"float64 auto_mode\n"
"float64 manual_mode\n"
"float64 shoot\n"
"float64 homing\n"
"float64 eo_img_submode\n"
"float64 ir_img_submode\n"
"float64 all_img_submode\n"
"float64 stab_submode\n"
"float64 ang_ctrl_submode\n"
"float64 angrate_ctrl_submode\n"
"float64 netgun_mode\n"
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

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.pan_rate_axis);
      stream.next(m.tilt_rate_axis);
      stream.next(m.pan_ang_axis);
      stream.next(m.tilt_ang_axis);
      stream.next(m.zoom_axis);
      stream.next(m.auto_mode);
      stream.next(m.manual_mode);
      stream.next(m.shoot);
      stream.next(m.homing);
      stream.next(m.eo_img_submode);
      stream.next(m.ir_img_submode);
      stream.next(m.all_img_submode);
      stream.next(m.stab_submode);
      stream.next(m.ang_ctrl_submode);
      stream.next(m.angrate_ctrl_submode);
      stream.next(m.netgun_mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ViewLinkJoyCtrlCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kari_dronecop_rd_payload_mgmt::ViewLinkJoyCtrlCmd_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pan_rate_axis: ";
    Printer<double>::stream(s, indent + "  ", v.pan_rate_axis);
    s << indent << "tilt_rate_axis: ";
    Printer<double>::stream(s, indent + "  ", v.tilt_rate_axis);
    s << indent << "pan_ang_axis: ";
    Printer<double>::stream(s, indent + "  ", v.pan_ang_axis);
    s << indent << "tilt_ang_axis: ";
    Printer<double>::stream(s, indent + "  ", v.tilt_ang_axis);
    s << indent << "zoom_axis: ";
    Printer<double>::stream(s, indent + "  ", v.zoom_axis);
    s << indent << "auto_mode: ";
    Printer<double>::stream(s, indent + "  ", v.auto_mode);
    s << indent << "manual_mode: ";
    Printer<double>::stream(s, indent + "  ", v.manual_mode);
    s << indent << "shoot: ";
    Printer<double>::stream(s, indent + "  ", v.shoot);
    s << indent << "homing: ";
    Printer<double>::stream(s, indent + "  ", v.homing);
    s << indent << "eo_img_submode: ";
    Printer<double>::stream(s, indent + "  ", v.eo_img_submode);
    s << indent << "ir_img_submode: ";
    Printer<double>::stream(s, indent + "  ", v.ir_img_submode);
    s << indent << "all_img_submode: ";
    Printer<double>::stream(s, indent + "  ", v.all_img_submode);
    s << indent << "stab_submode: ";
    Printer<double>::stream(s, indent + "  ", v.stab_submode);
    s << indent << "ang_ctrl_submode: ";
    Printer<double>::stream(s, indent + "  ", v.ang_ctrl_submode);
    s << indent << "angrate_ctrl_submode: ";
    Printer<double>::stream(s, indent + "  ", v.angrate_ctrl_submode);
    s << indent << "netgun_mode: ";
    Printer<double>::stream(s, indent + "  ", v.netgun_mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_VIEWLINKJOYCTRLCMD_H
