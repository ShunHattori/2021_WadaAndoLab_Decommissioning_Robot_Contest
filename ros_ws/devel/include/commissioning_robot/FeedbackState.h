// Generated by gencpp from file commissioning_robot/FeedbackState.msg
// DO NOT EDIT!


#ifndef COMMISSIONING_ROBOT_MESSAGE_FEEDBACKSTATE_H
#define COMMISSIONING_ROBOT_MESSAGE_FEEDBACKSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace commissioning_robot
{
template <class ContainerAllocator>
struct FeedbackState_
{
  typedef FeedbackState_<ContainerAllocator> Type;

  FeedbackState_()
    : is_ended()
    , current()
    , reference_feedbackside()
    , mode_feedbackside()  {
    }
  FeedbackState_(const ContainerAllocator& _alloc)
    : is_ended(_alloc)
    , current(_alloc)
    , reference_feedbackside(_alloc)
    , mode_feedbackside(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _is_ended_type;
  _is_ended_type is_ended;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _current_type;
  _current_type current;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _reference_feedbackside_type;
  _reference_feedbackside_type reference_feedbackside;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _mode_feedbackside_type;
  _mode_feedbackside_type mode_feedbackside;





  typedef boost::shared_ptr< ::commissioning_robot::FeedbackState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::commissioning_robot::FeedbackState_<ContainerAllocator> const> ConstPtr;

}; // struct FeedbackState_

typedef ::commissioning_robot::FeedbackState_<std::allocator<void> > FeedbackState;

typedef boost::shared_ptr< ::commissioning_robot::FeedbackState > FeedbackStatePtr;
typedef boost::shared_ptr< ::commissioning_robot::FeedbackState const> FeedbackStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::commissioning_robot::FeedbackState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::commissioning_robot::FeedbackState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::commissioning_robot::FeedbackState_<ContainerAllocator1> & lhs, const ::commissioning_robot::FeedbackState_<ContainerAllocator2> & rhs)
{
  return lhs.is_ended == rhs.is_ended &&
    lhs.current == rhs.current &&
    lhs.reference_feedbackside == rhs.reference_feedbackside &&
    lhs.mode_feedbackside == rhs.mode_feedbackside;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::commissioning_robot::FeedbackState_<ContainerAllocator1> & lhs, const ::commissioning_robot::FeedbackState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace commissioning_robot

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::commissioning_robot::FeedbackState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::commissioning_robot::FeedbackState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::commissioning_robot::FeedbackState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::commissioning_robot::FeedbackState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::commissioning_robot::FeedbackState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::commissioning_robot::FeedbackState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::commissioning_robot::FeedbackState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "73c7091a2c6badbe1419f63c8f01c277";
  }

  static const char* value(const ::commissioning_robot::FeedbackState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x73c7091a2c6badbeULL;
  static const uint64_t static_value2 = 0x1419f63c8f01c277ULL;
};

template<class ContainerAllocator>
struct DataType< ::commissioning_robot::FeedbackState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "commissioning_robot/FeedbackState";
  }

  static const char* value(const ::commissioning_robot::FeedbackState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::commissioning_robot::FeedbackState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8[] is_ended\n"
"float64[] current\n"
"float64[] reference_feedbackside\n"
"uint8[] mode_feedbackside\n"
;
  }

  static const char* value(const ::commissioning_robot::FeedbackState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::commissioning_robot::FeedbackState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.is_ended);
      stream.next(m.current);
      stream.next(m.reference_feedbackside);
      stream.next(m.mode_feedbackside);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FeedbackState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::commissioning_robot::FeedbackState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::commissioning_robot::FeedbackState_<ContainerAllocator>& v)
  {
    s << indent << "is_ended[]" << std::endl;
    for (size_t i = 0; i < v.is_ended.size(); ++i)
    {
      s << indent << "  is_ended[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.is_ended[i]);
    }
    s << indent << "current[]" << std::endl;
    for (size_t i = 0; i < v.current.size(); ++i)
    {
      s << indent << "  current[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.current[i]);
    }
    s << indent << "reference_feedbackside[]" << std::endl;
    for (size_t i = 0; i < v.reference_feedbackside.size(); ++i)
    {
      s << indent << "  reference_feedbackside[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.reference_feedbackside[i]);
    }
    s << indent << "mode_feedbackside[]" << std::endl;
    for (size_t i = 0; i < v.mode_feedbackside.size(); ++i)
    {
      s << indent << "  mode_feedbackside[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.mode_feedbackside[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // COMMISSIONING_ROBOT_MESSAGE_FEEDBACKSTATE_H
