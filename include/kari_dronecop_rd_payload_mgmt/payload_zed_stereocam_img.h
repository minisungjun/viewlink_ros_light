// Generated by gencpp from file kari_dronecop_rd_payload_mgmt/payload_zed_stereocam_img.msg
// DO NOT EDIT!


#ifndef KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_ZED_STEREOCAM_IMG_H
#define KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_ZED_STEREOCAM_IMG_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>

namespace kari_dronecop_rd_payload_mgmt
{
template <class ContainerAllocator>
struct payload_zed_stereocam_img_
{
  typedef payload_zed_stereocam_img_<ContainerAllocator> Type;

  payload_zed_stereocam_img_()
    : header()
    , img_ros_calib_rgb()
    , img_ros_depth()  {
    }
  payload_zed_stereocam_img_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , img_ros_calib_rgb(_alloc)
    , img_ros_depth(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _img_ros_calib_rgb_type;
  _img_ros_calib_rgb_type img_ros_calib_rgb;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _img_ros_depth_type;
  _img_ros_depth_type img_ros_depth;





  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> const> ConstPtr;

}; // struct payload_zed_stereocam_img_

typedef ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<std::allocator<void> > payload_zed_stereocam_img;

typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img > payload_zed_stereocam_imgPtr;
typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img const> payload_zed_stereocam_imgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.img_ros_calib_rgb == rhs.img_ros_calib_rgb &&
    lhs.img_ros_depth == rhs.img_ros_depth;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kari_dronecop_rd_payload_mgmt

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> >
{
  static const char* value()
  {
    return "00dc5372c0dfe434d502b124709405ab";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x00dc5372c0dfe434ULL;
  static const uint64_t static_value2 = 0xd502b124709405abULL;
};

template<class ContainerAllocator>
struct DataType< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kari_dronecop_rd_payload_mgmt/payload_zed_stereocam_img";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Payload TmTc in ROS env., Zed Stereo Camera, image status\n"
"Header header\n"
"sensor_msgs/Image img_ros_calib_rgb\n"
"sensor_msgs/Image img_ros_depth\n"
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

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.img_ros_calib_rgb);
      stream.next(m.img_ros_depth);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct payload_zed_stereocam_img_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kari_dronecop_rd_payload_mgmt::payload_zed_stereocam_img_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "img_ros_calib_rgb: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.img_ros_calib_rgb);
    s << indent << "img_ros_depth: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.img_ros_depth);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_ZED_STEREOCAM_IMG_H
