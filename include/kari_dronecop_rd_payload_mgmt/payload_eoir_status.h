// Generated by gencpp from file kari_dronecop_rd_payload_mgmt/payload_eoir_status.msg
// DO NOT EDIT!


#ifndef KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_EOIR_STATUS_H
#define KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_EOIR_STATUS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>

namespace kari_dronecop_rd_payload_mgmt
{
template <class ContainerAllocator>
struct payload_eoir_status_
{
  typedef payload_eoir_status_<ContainerAllocator> Type;

  payload_eoir_status_()
    : header()
    , img_ros_src()
    , pan_ang(0.0)
    , tilt_ang(0.0)
    , zoom_status(0)
    , img_mode(0)
    , capture_state(0)
    , gimbal_ctrl_mode(0)  {
    }
  payload_eoir_status_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , img_ros_src(_alloc)
    , pan_ang(0.0)
    , tilt_ang(0.0)
    , zoom_status(0)
    , img_mode(0)
    , capture_state(0)
    , gimbal_ctrl_mode(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _img_ros_src_type;
  _img_ros_src_type img_ros_src;

   typedef double _pan_ang_type;
  _pan_ang_type pan_ang;

   typedef double _tilt_ang_type;
  _tilt_ang_type tilt_ang;

   typedef int8_t _zoom_status_type;
  _zoom_status_type zoom_status;

   typedef int8_t _img_mode_type;
  _img_mode_type img_mode;

   typedef int8_t _capture_state_type;
  _capture_state_type capture_state;

   typedef int64_t _gimbal_ctrl_mode_type;
  _gimbal_ctrl_mode_type gimbal_ctrl_mode;





  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> const> ConstPtr;

}; // struct payload_eoir_status_

typedef ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<std::allocator<void> > payload_eoir_status;

typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status > payload_eoir_statusPtr;
typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status const> payload_eoir_statusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.img_ros_src == rhs.img_ros_src &&
    lhs.pan_ang == rhs.pan_ang &&
    lhs.tilt_ang == rhs.tilt_ang &&
    lhs.zoom_status == rhs.zoom_status &&
    lhs.img_mode == rhs.img_mode &&
    lhs.capture_state == rhs.capture_state &&
    lhs.gimbal_ctrl_mode == rhs.gimbal_ctrl_mode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kari_dronecop_rd_payload_mgmt

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b45041690354eacf132ecc3280c40285";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb45041690354eacfULL;
  static const uint64_t static_value2 = 0x132ecc3280c40285ULL;
};

template<class ContainerAllocator>
struct DataType< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kari_dronecop_rd_payload_mgmt/payload_eoir_status";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Payload TmTc in ROS env., EO/IR, status, with image\n"
"Header header\n"
"sensor_msgs/Image img_ros_src\n"
"float64 pan_ang\n"
"float64 tilt_ang\n"
"int8 zoom_status\n"
"int8 img_mode\n"
"int8 capture_state\n"
"int64 gimbal_ctrl_mode\n"
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
"\n"
"================================================================================\n"
"MSG: sensor_msgs/Image\n"
"# This message contains an uncompressed image\n"
"# (0, 0) is at top-left corner of image\n"
"#\n"
"\n"
"Header header        # Header timestamp should be acquisition time of image\n"
"                     # Header frame_id should be optical frame of camera\n"
"                     # origin of frame should be optical center of camera\n"
"                     # +x should point to the right in the image\n"
"                     # +y should point down in the image\n"
"                     # +z should point into to plane of the image\n"
"                     # If the frame_id here and the frame_id of the CameraInfo\n"
"                     # message associated with the image conflict\n"
"                     # the behavior is undefined\n"
"\n"
"uint32 height         # image height, that is, number of rows\n"
"uint32 width          # image width, that is, number of columns\n"
"\n"
"# The legal values for encoding are in file src/image_encodings.cpp\n"
"# If you want to standardize a new string format, join\n"
"# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n"
"\n"
"string encoding       # Encoding of pixels -- channel meaning, ordering, size\n"
"                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n"
"\n"
"uint8 is_bigendian    # is this data bigendian?\n"
"uint32 step           # Full row length in bytes\n"
"uint8[] data          # actual matrix data, size is (step * rows)\n"
;
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.img_ros_src);
      stream.next(m.pan_ang);
      stream.next(m.tilt_ang);
      stream.next(m.zoom_status);
      stream.next(m.img_mode);
      stream.next(m.capture_state);
      stream.next(m.gimbal_ctrl_mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct payload_eoir_status_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kari_dronecop_rd_payload_mgmt::payload_eoir_status_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "img_ros_src: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.img_ros_src);
    s << indent << "pan_ang: ";
    Printer<double>::stream(s, indent + "  ", v.pan_ang);
    s << indent << "tilt_ang: ";
    Printer<double>::stream(s, indent + "  ", v.tilt_ang);
    s << indent << "zoom_status: ";
    Printer<int8_t>::stream(s, indent + "  ", v.zoom_status);
    s << indent << "img_mode: ";
    Printer<int8_t>::stream(s, indent + "  ", v.img_mode);
    s << indent << "capture_state: ";
    Printer<int8_t>::stream(s, indent + "  ", v.capture_state);
    s << indent << "gimbal_ctrl_mode: ";
    Printer<int64_t>::stream(s, indent + "  ", v.gimbal_ctrl_mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_EOIR_STATUS_H
