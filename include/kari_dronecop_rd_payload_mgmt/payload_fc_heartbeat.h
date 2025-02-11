// Generated by gencpp from file kari_dronecop_rd_payload_mgmt/payload_fc_heartbeat.msg
// DO NOT EDIT!


#ifndef KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_FC_HEARTBEAT_H
#define KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_FC_HEARTBEAT_H


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
struct payload_fc_heartbeat_
{
  typedef payload_fc_heartbeat_<ContainerAllocator> Type;

  payload_fc_heartbeat_()
    : header()
    , connected(false)
    , armed(false)
    , guided(false)
    , fc_type()
    , base_mode()
    , system_status(0)
    , mavlink_version(0)
    , remain_percentage(0.0)
    , voltage(0.0)
    , current(0.0)  {
    }
  payload_fc_heartbeat_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , connected(false)
    , armed(false)
    , guided(false)
    , fc_type(_alloc)
    , base_mode(_alloc)
    , system_status(0)
    , mavlink_version(0)
    , remain_percentage(0.0)
    , voltage(0.0)
    , current(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _connected_type;
  _connected_type connected;

   typedef uint8_t _armed_type;
  _armed_type armed;

   typedef uint8_t _guided_type;
  _guided_type guided;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _fc_type_type;
  _fc_type_type fc_type;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _base_mode_type;
  _base_mode_type base_mode;

   typedef int8_t _system_status_type;
  _system_status_type system_status;

   typedef int8_t _mavlink_version_type;
  _mavlink_version_type mavlink_version;

   typedef float _remain_percentage_type;
  _remain_percentage_type remain_percentage;

   typedef float _voltage_type;
  _voltage_type voltage;

   typedef float _current_type;
  _current_type current;





  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> const> ConstPtr;

}; // struct payload_fc_heartbeat_

typedef ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<std::allocator<void> > payload_fc_heartbeat;

typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat > payload_fc_heartbeatPtr;
typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat const> payload_fc_heartbeatConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.connected == rhs.connected &&
    lhs.armed == rhs.armed &&
    lhs.guided == rhs.guided &&
    lhs.fc_type == rhs.fc_type &&
    lhs.base_mode == rhs.base_mode &&
    lhs.system_status == rhs.system_status &&
    lhs.mavlink_version == rhs.mavlink_version &&
    lhs.remain_percentage == rhs.remain_percentage &&
    lhs.voltage == rhs.voltage &&
    lhs.current == rhs.current;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator1> & lhs, const ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kari_dronecop_rd_payload_mgmt

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e06b14bc9d42d2a77043df9fd5dd6320";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe06b14bc9d42d2a7ULL;
  static const uint64_t static_value2 = 0x7043df9fd5dd6320ULL;
};

template<class ContainerAllocator>
struct DataType< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kari_dronecop_rd_payload_mgmt/payload_fc_heartbeat";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Payload TmTc in ROS env., flight control heartbeat and baterry status\n"
"Header header\n"
"bool connected\n"
"bool armed\n"
"bool guided\n"
"string fc_type\n"
"string base_mode\n"
"int8 system_status\n"
"int8 mavlink_version\n"
"float32 remain_percentage\n"
"float32 voltage\n"
"float32 current\n"
"\n"
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

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.connected);
      stream.next(m.armed);
      stream.next(m.guided);
      stream.next(m.fc_type);
      stream.next(m.base_mode);
      stream.next(m.system_status);
      stream.next(m.mavlink_version);
      stream.next(m.remain_percentage);
      stream.next(m.voltage);
      stream.next(m.current);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct payload_fc_heartbeat_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kari_dronecop_rd_payload_mgmt::payload_fc_heartbeat_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "connected: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.connected);
    s << indent << "armed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.armed);
    s << indent << "guided: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.guided);
    s << indent << "fc_type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.fc_type);
    s << indent << "base_mode: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.base_mode);
    s << indent << "system_status: ";
    Printer<int8_t>::stream(s, indent + "  ", v.system_status);
    s << indent << "mavlink_version: ";
    Printer<int8_t>::stream(s, indent + "  ", v.mavlink_version);
    s << indent << "remain_percentage: ";
    Printer<float>::stream(s, indent + "  ", v.remain_percentage);
    s << indent << "voltage: ";
    Printer<float>::stream(s, indent + "  ", v.voltage);
    s << indent << "current: ";
    Printer<float>::stream(s, indent + "  ", v.current);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_FC_HEARTBEAT_H
