// Generated by gencpp from file kari_dronecop_rd_payload_mgmt/payload_mgmt_define.msg
// DO NOT EDIT!


#ifndef KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_MGMT_DEFINE_H
#define KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_MGMT_DEFINE_H


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
struct payload_mgmt_define_
{
  typedef payload_mgmt_define_<ContainerAllocator> Type;

  payload_mgmt_define_()
    {
    }
  payload_mgmt_define_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(RD1_SYS_ID)
  #undef RD1_SYS_ID
#endif
#if defined(_WIN32) && defined(RD2_SYS_ID)
  #undef RD2_SYS_ID
#endif
#if defined(_WIN32) && defined(RD_COMP_ID_ADT)
  #undef RD_COMP_ID_ADT
#endif
#if defined(_WIN32) && defined(RD_COMP_ID_GDT)
  #undef RD_COMP_ID_GDT
#endif
#if defined(_WIN32) && defined(RD_COMP_ID_MC)
  #undef RD_COMP_ID_MC
#endif
#if defined(_WIN32) && defined(RD_COMP_ID_FC)
  #undef RD_COMP_ID_FC
#endif
#if defined(_WIN32) && defined(RD_COMP_ID_EOIR)
  #undef RD_COMP_ID_EOIR
#endif
#if defined(_WIN32) && defined(RD_COMP_ID_STEREO_CAMERA)
  #undef RD_COMP_ID_STEREO_CAMERA
#endif
#if defined(_WIN32) && defined(RD_COMP_ID_VU8_LIDAR)
  #undef RD_COMP_ID_VU8_LIDAR
#endif
#if defined(_WIN32) && defined(RD_COMP_ID_EXTERNAL_SENSOR_SYS)
  #undef RD_COMP_ID_EXTERNAL_SENSOR_SYS
#endif
#if defined(_WIN32) && defined(RD_COMP_ID_GNSS_SPOOFER)
  #undef RD_COMP_ID_GNSS_SPOOFER
#endif
#if defined(_WIN32) && defined(RD_COMP_ID_DECEPTER)
  #undef RD_COMP_ID_DECEPTER
#endif
#if defined(_WIN32) && defined(RD_COMP_ID_JAMMER)
  #undef RD_COMP_ID_JAMMER
#endif
#if defined(_WIN32) && defined(RD_COMP_ID_NETGUN)
  #undef RD_COMP_ID_NETGUN
#endif
#if defined(_WIN32) && defined(START_DCT_NEUTRALIZE)
  #undef START_DCT_NEUTRALIZE
#endif
#if defined(_WIN32) && defined(STOP_DCT_NEUTRALIZE)
  #undef STOP_DCT_NEUTRALIZE
#endif
#if defined(_WIN32) && defined(A001)
  #undef A001
#endif
#if defined(_WIN32) && defined(A002)
  #undef A002
#endif
#if defined(_WIN32) && defined(A003)
  #undef A003
#endif
#if defined(_WIN32) && defined(A004)
  #undef A004
#endif
#if defined(_WIN32) && defined(A005)
  #undef A005
#endif
#if defined(_WIN32) && defined(A006)
  #undef A006
#endif
#if defined(_WIN32) && defined(A007)
  #undef A007
#endif
#if defined(_WIN32) && defined(A008)
  #undef A008
#endif
#if defined(_WIN32) && defined(A009)
  #undef A009
#endif
#if defined(_WIN32) && defined(A010)
  #undef A010
#endif
#if defined(_WIN32) && defined(C001)
  #undef C001
#endif
#if defined(_WIN32) && defined(C002)
  #undef C002
#endif
#if defined(_WIN32) && defined(C003)
  #undef C003
#endif
#if defined(_WIN32) && defined(C004)
  #undef C004
#endif
#if defined(_WIN32) && defined(C005)
  #undef C005
#endif
#if defined(_WIN32) && defined(C006)
  #undef C006
#endif
#if defined(_WIN32) && defined(C007)
  #undef C007
#endif
#if defined(_WIN32) && defined(C008)
  #undef C008
#endif
#if defined(_WIN32) && defined(C009)
  #undef C009
#endif
#if defined(_WIN32) && defined(C010)
  #undef C010
#endif
#if defined(_WIN32) && defined(UNKNOWN_RSN_NEUTRALIZE_MANUFACTURER)
  #undef UNKNOWN_RSN_NEUTRALIZE_MANUFACTURER
#endif
#if defined(_WIN32) && defined(M001)
  #undef M001
#endif
#if defined(_WIN32) && defined(M002)
  #undef M002
#endif
#if defined(_WIN32) && defined(M003)
  #undef M003
#endif
#if defined(_WIN32) && defined(M004)
  #undef M004
#endif
#if defined(_WIN32) && defined(M005)
  #undef M005
#endif
#if defined(_WIN32) && defined(M006)
  #undef M006
#endif
#if defined(_WIN32) && defined(M007)
  #undef M007
#endif
#if defined(_WIN32) && defined(M008)
  #undef M008
#endif
#if defined(_WIN32) && defined(M009)
  #undef M009
#endif
#if defined(_WIN32) && defined(M010)
  #undef M010
#endif
#if defined(_WIN32) && defined(UNKNOWN_RSN_NEUTRALIZE_MODEL)
  #undef UNKNOWN_RSN_NEUTRALIZE_MODEL
#endif
#if defined(_WIN32) && defined(READY_DCT_STATUS_MODE)
  #undef READY_DCT_STATUS_MODE
#endif
#if defined(_WIN32) && defined(NEUTRALIZING)
  #undef NEUTRALIZING
#endif
#if defined(_WIN32) && defined(NEUTRALIZED)
  #undef NEUTRALIZED
#endif
#if defined(_WIN32) && defined(CONTROLLING)
  #undef CONTROLLING
#endif
#if defined(_WIN32) && defined(CONTROLLING_D)
  #undef CONTROLLING_D
#endif
#if defined(_WIN32) && defined(CONTROLLING_I)
  #undef CONTROLLING_I
#endif
#if defined(_WIN32) && defined(CONTROLLING_L)
  #undef CONTROLLING_L
#endif
#if defined(_WIN32) && defined(STOPPING_DCT_STATUS_MODE)
  #undef STOPPING_DCT_STATUS_MODE
#endif
#if defined(_WIN32) && defined(START_RSN_STATUS_ACK)
  #undef START_RSN_STATUS_ACK
#endif
#if defined(_WIN32) && defined(STOP_RSN_STATUS_ACK)
  #undef STOP_RSN_STATUS_ACK
#endif
#if defined(_WIN32) && defined(INIT_RSN_STATUS_ACK)
  #undef INIT_RSN_STATUS_ACK
#endif
#if defined(_WIN32) && defined(SUCESS_DCT_NEUTRALIZE_RESULT)
  #undef SUCESS_DCT_NEUTRALIZE_RESULT
#endif
#if defined(_WIN32) && defined(FAILURE_DCT_NEUTRALIZE_RESULT)
  #undef FAILURE_DCT_NEUTRALIZE_RESULT
#endif
#if defined(_WIN32) && defined(UNKNOWN_DCT_NEUTRALIZE_RESULT)
  #undef UNKNOWN_DCT_NEUTRALIZE_RESULT
#endif
#if defined(_WIN32) && defined(ERROR_TEMP)
  #undef ERROR_TEMP
#endif
#if defined(_WIN32) && defined(ERROR_RF)
  #undef ERROR_RF
#endif
#if defined(_WIN32) && defined(ERROR_NOTFOUND)
  #undef ERROR_NOTFOUND
#endif
#if defined(_WIN32) && defined(ERROR_UNKNOWN_DCT_STATUS_ERROR)
  #undef ERROR_UNKNOWN_DCT_STATUS_ERROR
#endif

  enum {
    RD1_SYS_ID = 1u,
    RD2_SYS_ID = 2u,
    RD_COMP_ID_ADT = 1u,
    RD_COMP_ID_GDT = 2u,
    RD_COMP_ID_MC = 3u,
    RD_COMP_ID_FC = 4u,
    RD_COMP_ID_EOIR = 5u,
    RD_COMP_ID_STEREO_CAMERA = 6u,
    RD_COMP_ID_VU8_LIDAR = 7u,
    RD_COMP_ID_EXTERNAL_SENSOR_SYS = 8u,
    RD_COMP_ID_GNSS_SPOOFER = 11u,
    RD_COMP_ID_DECEPTER = 12u,
    RD_COMP_ID_JAMMER = 21u,
    RD_COMP_ID_NETGUN = 22u,
    START_DCT_NEUTRALIZE = 170u,
    STOP_DCT_NEUTRALIZE = 85u,
    A001 = 100001u,
    A002 = 100002u,
    A003 = 100003u,
    A004 = 100004u,
    A005 = 100005u,
    A006 = 100006u,
    A007 = 100007u,
    A008 = 100008u,
    A009 = 100009u,
    A010 = 100010u,
    C001 = 900001u,
    C002 = 900002u,
    C003 = 900003u,
    C004 = 900004u,
    C005 = 900005u,
    C006 = 900006u,
    C007 = 900007u,
    C008 = 900008u,
    C009 = 900009u,
    C010 = 900010u,
    UNKNOWN_RSN_NEUTRALIZE_MANUFACTURER = 0u,
    M001 = 10001u,
    M002 = 10002u,
    M003 = 10003u,
    M004 = 10004u,
    M005 = 10005u,
    M006 = 10006u,
    M007 = 10007u,
    M008 = 10008u,
    M009 = 10009u,
    M010 = 10010u,
    UNKNOWN_RSN_NEUTRALIZE_MODEL = 0u,
    READY_DCT_STATUS_MODE = 0u,
    NEUTRALIZING = 16u,
    NEUTRALIZED = 32u,
    CONTROLLING = 5u,
    CONTROLLING_D = 6u,
    CONTROLLING_I = 7u,
    CONTROLLING_L = 8u,
    STOPPING_DCT_STATUS_MODE = 255u,
    START_RSN_STATUS_ACK = 170u,
    STOP_RSN_STATUS_ACK = 85u,
    INIT_RSN_STATUS_ACK = 0u,
    SUCESS_DCT_NEUTRALIZE_RESULT = 33u,
    FAILURE_DCT_NEUTRALIZE_RESULT = 32u,
    UNKNOWN_DCT_NEUTRALIZE_RESULT = 0u,
    ERROR_TEMP = 16u,
    ERROR_RF = 32u,
    ERROR_NOTFOUND = 1u,
    ERROR_UNKNOWN_DCT_STATUS_ERROR = 8u,
  };


  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> const> ConstPtr;

}; // struct payload_mgmt_define_

typedef ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<std::allocator<void> > payload_mgmt_define;

typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define > payload_mgmt_definePtr;
typedef boost::shared_ptr< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define const> payload_mgmt_defineConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace kari_dronecop_rd_payload_mgmt

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1d7baa7c6aeb250e48ac57ab17360e76";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1d7baa7c6aeb250eULL;
  static const uint64_t static_value2 = 0x48ac57ab17360e76ULL;
};

template<class ContainerAllocator>
struct DataType< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kari_dronecop_rd_payload_mgmt/payload_mgmt_define";
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Definition in ROS env.\n"
"\n"
"# ============================================================================== #\n"
"# RD_SYS_ID\n"
"uint8 RD1_SYS_ID = 1\n"
"uint8 RD2_SYS_ID = 2\n"
"# ============================================================================== #\n"
"\n"
"# ============================================================================== #\n"
"# RD_COMPONENT_ID, x: for common, 1x: for RD#1, 2x: for RD#2\n"
"uint8 RD_COMP_ID_ADT = 1\n"
"uint8 RD_COMP_ID_GDT = 2\n"
"uint8 RD_COMP_ID_MC = 3\n"
"uint8 RD_COMP_ID_FC = 4\n"
"uint8 RD_COMP_ID_EOIR = 5\n"
"uint8 RD_COMP_ID_STEREO_CAMERA = 6\n"
"uint8 RD_COMP_ID_VU8_LIDAR = 7\n"
"uint8 RD_COMP_ID_EXTERNAL_SENSOR_SYS = 8\n"
"uint8 RD_COMP_ID_GNSS_SPOOFER = 11\n"
"uint8 RD_COMP_ID_DECEPTER = 12\n"
"uint8 RD_COMP_ID_JAMMER = 21\n"
"uint8 RD_COMP_ID_NETGUN = 22\n"
"# ============================================================================== #\n"
"\n"
"# ============================================================================== #\n"
"# Decepter, Neutralize mission start/stop status\n"
"uint8 START_DCT_NEUTRALIZE = 170\n"
"uint8 STOP_DCT_NEUTRALIZE = 85\n"
"# ============================================================================== #\n"
"\n"
"# ============================================================================== #\n"
"# Decepter, Neutralize target illegal drone, for manufacturer\n"
"uint32 A001 = 100001\n"
"uint32 A002 = 100002\n"
"uint32 A003 = 100003\n"
"uint32 A004 = 100004\n"
"uint32 A005 = 100005\n"
"uint32 A006 = 100006\n"
"uint32 A007 = 100007\n"
"uint32 A008 = 100008\n"
"uint32 A009 = 100009\n"
"uint32 A010 = 100010\n"
"uint32 C001 = 900001\n"
"uint32 C002 = 900002\n"
"uint32 C003 = 900003\n"
"uint32 C004 = 900004\n"
"uint32 C005 = 900005\n"
"uint32 C006 = 900006\n"
"uint32 C007 = 900007\n"
"uint32 C008 = 900008\n"
"uint32 C009 = 900009\n"
"uint32 C010 = 900010\n"
"uint32 UNKNOWN_RSN_NEUTRALIZE_MANUFACTURER = 0\n"
"# ============================================================================== #\n"
"\n"
"# ============================================================================== #\n"
"# Decepter, Neutralize target illegal drone, for model\n"
"uint16 M001 = 10001\n"
"uint16 M002 = 10002\n"
"uint16 M003 = 10003\n"
"uint16 M004 = 10004\n"
"uint16 M005 = 10005\n"
"uint16 M006 = 10006\n"
"uint16 M007 = 10007\n"
"uint16 M008 = 10008\n"
"uint16 M009 = 10009\n"
"uint16 M010 = 10010\n"
"uint16 UNKNOWN_RSN_NEUTRALIZE_MODEL = 0\n"
"# ============================================================================== #\n"
"\n"
"# ============================================================================== #\n"
"# Decepter, Neutralize mission mode\n"
"uint8 READY_DCT_STATUS_MODE = 0\n"
"uint8 NEUTRALIZING = 16\n"
"uint8 NEUTRALIZED = 32\n"
"uint8 CONTROLLING = 5\n"
"uint8 CONTROLLING_D = 6\n"
"uint8 CONTROLLING_I = 7\n"
"uint8 CONTROLLING_L = 8\n"
"uint8 STOPPING_DCT_STATUS_MODE = 255\n"
"# ============================================================================== #\n"
"\n"
"# ============================================================================== #\n"
"# Decepter, Neutralize response\n"
"uint8 START_RSN_STATUS_ACK = 170\n"
"uint8 STOP_RSN_STATUS_ACK = 85\n"
"uint8 INIT_RSN_STATUS_ACK = 0\n"
"# ============================================================================== #\n"
"\n"
"# ============================================================================== #\n"
"# Decepter, Neutralize result\n"
"uint8 SUCESS_DCT_NEUTRALIZE_RESULT = 33\n"
"uint8 FAILURE_DCT_NEUTRALIZE_RESULT = 32\n"
"uint8 UNKNOWN_DCT_NEUTRALIZE_RESULT = 0\n"
"# ============================================================================== #\n"
"\n"
"# ============================================================================== #\n"
"# Decepter, Neutralized error status\n"
"uint8 ERROR_TEMP = 16\n"
"uint8 ERROR_RF = 32\n"
"uint8 ERROR_NOTFOUND = 1\n"
"uint8 ERROR_UNKNOWN_DCT_STATUS_ERROR = 8\n"
"# ============================================================================== #\n"
;
  }

  static const char* value(const ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct payload_mgmt_define_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::kari_dronecop_rd_payload_mgmt::payload_mgmt_define_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // KARI_DRONECOP_RD_PAYLOAD_MGMT_MESSAGE_PAYLOAD_MGMT_DEFINE_H
