// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/PrintJob.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__PRINT_JOB__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__PRINT_JOB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/print_job__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_PrintJob_Request_document_name
{
public:
  Init_PrintJob_Request_document_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::PrintJob_Request document_name(::custom_interfaces::srv::PrintJob_Request::_document_name_type arg)
  {
    msg_.document_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::PrintJob_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::PrintJob_Request>()
{
  return custom_interfaces::srv::builder::Init_PrintJob_Request_document_name();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_PrintJob_Response_accepted
{
public:
  Init_PrintJob_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::PrintJob_Response accepted(::custom_interfaces::srv::PrintJob_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::PrintJob_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::PrintJob_Response>()
{
  return custom_interfaces::srv::builder::Init_PrintJob_Response_accepted();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__PRINT_JOB__BUILDER_HPP_
