// Two different kinds of concatenation are needed for these namespaces
#define INTROSPECTION_CPP_TYPE(A) rosidl_typesupport_introspection_cpp::A

#define INTROSPECTION_C_TYPE(A) rosidl_typesupport_introspection_c__ ## A

#define CREATE_TYPENAME_PREFIX(INTROSPECTION_TYPE) \
  auto members = static_cast<const INTROSPECTION_TYPE(MessageMembers) *>(untyped_members); \
  if (!members) { \
    return ""; \
  } \
  package_name = members->package_name_; \
  message_name = members->message_name_;


#define CREATE_TYPE_CODE(INTROSPECTION_TYPE) \
  auto members = static_cast<const INTROSPECTION_TYPE(MessageMembers) *>(untyped_members); \
  if (!members) { \
    RMW_SET_ERROR_MSG("members handle is null"); \
    return NULL; \
  } \
  for (uint32_t i = 0; i < members->member_count_; ++i) { \
    const INTROSPECTION_TYPE(MessageMember) * member = members->members_ + i; \
    const DDS_TypeCode * member_type_code = nullptr; \
    DDS_TypeCode * member_type_code_non_const = nullptr; \
    switch (member->type_id_) { \
      case INTROSPECTION_TYPE(ROS_TYPE_BOOL): \
        member_type_code = factory->get_primitive_tc(DDS_TK_BOOLEAN); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_BYTE): \
        member_type_code = factory->get_primitive_tc(DDS_TK_OCTET); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_CHAR): \
        member_type_code = factory->get_primitive_tc(DDS_TK_CHAR); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_FLOAT32): \
        member_type_code = factory->get_primitive_tc(DDS_TK_FLOAT); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_FLOAT64): \
        member_type_code = factory->get_primitive_tc(DDS_TK_DOUBLE); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_INT8): \
        member_type_code = factory->get_primitive_tc(DDS_TK_OCTET); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_UINT8): \
        member_type_code = factory->get_primitive_tc(DDS_TK_OCTET); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_INT16): \
        member_type_code = factory->get_primitive_tc(DDS_TK_SHORT); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_UINT16): \
        member_type_code = factory->get_primitive_tc(DDS_TK_USHORT); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_INT32): \
        member_type_code = factory->get_primitive_tc(DDS_TK_LONG); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_UINT32): \
        member_type_code = factory->get_primitive_tc(DDS_TK_ULONG); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_INT64): \
        member_type_code = factory->get_primitive_tc(DDS_TK_LONGLONG); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_UINT64): \
        member_type_code = factory->get_primitive_tc(DDS_TK_ULONGLONG); \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_STRING): \
        { \
          DDS_UnsignedLong max_string_size; \
          if (member->string_upper_bound_) { \
            if (member->string_upper_bound_ > (std::numeric_limits<DDS_UnsignedLong>::max)()) { \
              RMW_SET_ERROR_MSG( \
                "failed to create string typecode since the upper bound exceeds the DDS type"); \
              goto fail; \
            } \
            max_string_size = static_cast<DDS_UnsignedLong>(member->string_upper_bound_); \
          } else { \
            max_string_size = RTI_INT32_MAX; \
          } \
          member_type_code_non_const = factory->create_string_tc(max_string_size, ex); \
          member_type_code = member_type_code_non_const; \
        } \
        if (!member_type_code || ex != DDS_NO_EXCEPTION_CODE) { \
          RMW_SET_ERROR_MSG("failed to create string typecode"); \
          goto fail; \
        } \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_MESSAGE): \
        { \
          if (!member->members_) { \
            RMW_SET_ERROR_MSG("members handle is null"); \
            return NULL; \
          } \
          auto sub_members = \
            static_cast<const INTROSPECTION_TYPE(MessageMembers) *>( \
            member->members_->data); \
          if (!sub_members) { \
            RMW_SET_ERROR_MSG("sub members handle is null"); \
            return NULL; \
          } \
          std::string field_type_name = _create_type_name(sub_members, "msg", typesupport); \
          member_type_code = create_type_code(field_type_name, sub_members, typesupport); \
          if (!member_type_code) { \
            goto fail; \
          } \
        } \
        break; \
      default: \
        RMW_SET_ERROR_MSG( \
          (std::string("unknown type id ") + std::to_string(member->type_id_)).c_str()); \
        goto fail; \
    } \
    if (!member_type_code) { \
      RMW_SET_ERROR_MSG("failed to create typecode"); \
      goto fail; \
    } \
    if (member->is_array_) { \
      if (member->array_size_) { \
        if (member->array_size_ > (std::numeric_limits<DDS_UnsignedLong>::max)()) { \
          RMW_SET_ERROR_MSG( \
            "failed to create array typecode since the size exceeds the DDS type"); \
          goto fail; \
        } \
        DDS_UnsignedLong array_size = static_cast<DDS_UnsignedLong>(member->array_size_); \
        if (!member->is_upper_bound_) { \
          member_type_code_non_const = factory->create_array_tc(array_size, member_type_code, ex); \
          member_type_code = member_type_code_non_const; \
          if (!member_type_code || ex != DDS_NO_EXCEPTION_CODE) { \
            RMW_SET_ERROR_MSG("failed to create array typecode"); \
            goto fail; \
          } \
        } else { \
          member_type_code_non_const = \
            factory->create_sequence_tc(array_size, member_type_code, ex); \
          member_type_code = member_type_code_non_const; \
          if (!member_type_code || ex != DDS_NO_EXCEPTION_CODE) { \
            RMW_SET_ERROR_MSG("failed to create sequence typecode"); \
            goto fail; \
          } \
        } \
      } else { \
        member_type_code_non_const = factory->create_sequence_tc( \
          RTI_INT32_MAX, member_type_code, ex); \
        member_type_code = member_type_code_non_const; \
        if (!member_type_code || ex != DDS_NO_EXCEPTION_CODE) { \
          RMW_SET_ERROR_MSG("failed to create sequence typecode"); \
          goto fail; \
        } \
      } \
    } \
    auto zero_based_index = type_code->add_member( \
      (std::string(member->name_) + "_").c_str(), \
      i, \
      member_type_code, \
      DDS_TYPECODE_NONKEY_REQUIRED_MEMBER, ex); \
    if (ex != DDS_NO_EXCEPTION_CODE) { \
      RMW_SET_ERROR_MSG("failed to add member"); \
      goto fail; \
    } \
    if (zero_based_index != i) { \
      RMW_SET_ERROR_MSG("unexpected member index"); \
      return NULL; \
    } \
  } \
  if (members->member_count_ == 0) { \
    const DDS_TypeCode * member_type_code; \
    member_type_code = factory->get_primitive_tc(DDS_TK_BOOLEAN); \
    if (!member_type_code) { \
      RMW_SET_ERROR_MSG("failed to get primitive typecode"); \
      goto fail; \
    } \
    type_code->add_member("_dummy", DDS_TYPECODE_MEMBER_ID_INVALID, \
      member_type_code, \
      DDS_TYPECODE_NONKEY_REQUIRED_MEMBER, ex); \
    if (ex != DDS_NO_EXCEPTION_CODE) { \
      RMW_SET_ERROR_MSG("failed to add member"); \
      goto fail; \
    } \
  }

#define PUBLISH(INTROSPECTION_TYPE) \
  auto members = static_cast<const INTROSPECTION_TYPE(MessageMembers) *>(untyped_members); \
  if (!members) { \
    RMW_SET_ERROR_MSG("Wrong MessageMember type received") \
    return false; \
  } \
  for (uint32_t i = 0; i < members->member_count_; ++i) { \
    const INTROSPECTION_TYPE(MessageMember) * member = members->members_ + i; \
    switch (member->type_id_) { \
      case INTROSPECTION_TYPE(ROS_TYPE_BOOL): \
        SET_VALUE_WITH_BOOL_TYPE(bool, DDS_Boolean, set_boolean, set_boolean_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_BYTE): \
        SET_VALUE(uint8_t, set_octet, set_octet_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_CHAR): \
        SET_VALUE(char, set_char, set_char_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_FLOAT32): \
        SET_VALUE(float, set_float, set_float_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_FLOAT64): \
        SET_VALUE(double, set_double, set_double_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_INT8): \
        SET_VALUE_WITH_DIFFERENT_TYPES(int8_t, DDS_Octet, set_octet, set_octet_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_UINT8): \
        SET_VALUE(uint8_t, set_octet, set_octet_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_INT16): \
        SET_VALUE(int16_t, set_short, set_short_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_UINT16): \
        SET_VALUE(uint16_t, set_ushort, set_ushort_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_INT32): \
        SET_VALUE(int32_t, set_long, set_long_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_UINT32): \
        SET_VALUE(uint32_t, set_ulong, set_ulong_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_INT64): \
        SET_VALUE_WITH_DIFFERENT_TYPES(int64_t, DDS_LongLong, set_longlong, set_longlong_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_UINT64): \
        SET_VALUE_WITH_DIFFERENT_TYPES( \
          uint64_t, DDS_UnsignedLongLong, set_ulonglong, set_ulonglong_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_STRING): \
        if (using_introspection_c_typesupport(typesupport)) { \
          SET_C_STRING_VALUE(rosidl_generator_c__String, set_string) \
        } else if (using_introspection_cpp_typesupport(typesupport)) { \
          SET_CPP_STRING_VALUE(std::string, set_string) \
        } \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_MESSAGE): \
        { \
          if (member->is_array_) { \
            const void * untyped_member = static_cast<const char *>( \
              ros_message) + member->offset_; \
            if (!member->size_function) { \
              RMW_SET_ERROR_MSG("size function handle is null"); \
              return false; \
            } \
            if (!member->get_const_function) { \
              RMW_SET_ERROR_MSG("get const function handle is null"); \
              return false; \
            } \
            DDS_DynamicData array_data(NULL, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT); \
            DDS_ReturnCode_t status = dynamic_data->bind_complex_member( \
              array_data, \
              NULL, \
              i + 1); \
            if (status != DDS_RETCODE_OK) { \
              RMW_SET_ERROR_MSG("failed to bind complex member"); \
              return false; \
            } \
            size_t array_size = member->size_function(untyped_member); \
            for (size_t j = 0; j < array_size; ++j) { \
              const void * ros_message; \
              { \
                const void * sub_ros_message = member->get_const_function(untyped_member, j); \
                ros_message = static_cast<const char *>(sub_ros_message) - member->offset_; \
              } \
              DDS_DynamicData * array_data_ptr = &array_data; \
              SET_SUBMESSAGE_VALUE(array_data_ptr, j, INTROSPECTION_TYPE) \
            } \
            status = dynamic_data->unbind_complex_member(array_data); \
            if (status != DDS_RETCODE_OK) { \
              RMW_SET_ERROR_MSG("failed to unbind complex member"); \
              return false; \
            } \
          } else { \
            SET_SUBMESSAGE_VALUE(dynamic_data, i, INTROSPECTION_TYPE) \
          } \
        } \
        break; \
      default: \
        RMW_SET_ERROR_MSG( \
          (std::string("unknown type id ") + std::to_string(member->type_id_)).c_str()); \
        return false; \
    } \
  }

#define TAKE(INTROSPECTION_TYPE) \
  auto members = static_cast<const INTROSPECTION_TYPE(MessageMembers) *>(untyped_members); \
  if (!members) { \
    RMW_SET_ERROR_MSG("Wrong MessageMember type received") \
    return false; \
  } \
  for (uint32_t i = 0; i < members->member_count_; ++i) { \
    const INTROSPECTION_TYPE(MessageMember) * member = members->members_ + i; \
    switch (member->type_id_) { \
      case INTROSPECTION_TYPE(ROS_TYPE_BOOL): \
        GET_VALUE_WITH_BOOL_TYPE(bool, DDS_Boolean, get_boolean, get_boolean_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_BYTE): \
        GET_VALUE(uint8_t, get_octet, get_octet_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_CHAR): \
        GET_VALUE(char, get_char, get_char_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_FLOAT32): \
        GET_VALUE(float, get_float, get_float_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_FLOAT64): \
        GET_VALUE(double, get_double, get_double_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_INT8): \
        GET_VALUE_WITH_DIFFERENT_TYPES(int8_t, DDS_Octet, get_octet, get_octet_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_UINT8): \
        GET_VALUE(uint8_t, get_octet, get_octet_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_INT16): \
        GET_VALUE(int16_t, get_short, get_short_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_UINT16): \
        GET_VALUE(uint16_t, get_ushort, get_ushort_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_INT32): \
        GET_VALUE(int32_t, get_long, get_long_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_UINT32): \
        GET_VALUE(uint32_t, get_ulong, get_ulong_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_INT64): \
        GET_VALUE_WITH_DIFFERENT_TYPES(int64_t, DDS_LongLong, get_longlong, get_longlong_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_UINT64): \
        GET_VALUE_WITH_DIFFERENT_TYPES( \
          uint64_t, DDS_UnsignedLongLong, get_ulonglong, get_ulonglong_array) \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_STRING): \
        if (using_introspection_c_typesupport(typesupport)) { \
          GET_C_STRING_VALUE(rosidl_generator_c__String, get_string) \
        } else if (using_introspection_cpp_typesupport(typesupport)) { \
          GET_CPP_STRING_VALUE(std::string, get_string) \
        } \
        break; \
      case INTROSPECTION_TYPE(ROS_TYPE_MESSAGE): \
        { \
          if (member->is_array_) { \
            void * untyped_member = static_cast<char *>(ros_message) + member->offset_; \
            if (!member->array_size_ || member->is_upper_bound_) { \
              if (!member->resize_function) { \
                RMW_SET_ERROR_MSG("resize function handle is null"); \
                return false; \
              } \
            } \
            if (!member->get_function) { \
              RMW_SET_ERROR_MSG("get function handle is null"); \
              return false; \
            } \
            ARRAY_SIZE() \
            DDS_DynamicData array_data(NULL, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT); \
            DDS_ReturnCode_t status = dynamic_data->bind_complex_member( \
              array_data, \
              NULL, \
              i + 1); \
            if (status != DDS_RETCODE_OK) { \
              RMW_SET_ERROR_MSG("failed to bind complex member"); \
              return false; \
            } \
            if (!member->array_size_ || member->is_upper_bound_) { \
              member->resize_function(untyped_member, array_size); \
            } \
            for (size_t j = 0; j < array_size; ++j) { \
              void * ros_message; \
              { \
                void * sub_ros_message = member->get_function(untyped_member, j); \
                ros_message = static_cast<char *>(sub_ros_message) - member->offset_; \
              } \
              DDS_DynamicData * array_data_ptr = &array_data; \
              GET_SUBMESSAGE_VALUE(array_data_ptr, j, INTROSPECTION_TYPE) \
            } \
            status = dynamic_data->unbind_complex_member(array_data); \
            if (status != DDS_RETCODE_OK) { \
              RMW_SET_ERROR_MSG("failed to unbind complex member"); \
              return false; \
            } \
          } else { \
            GET_SUBMESSAGE_VALUE(dynamic_data, i, INTROSPECTION_TYPE) \
          } \
        } \
        break; \
      default: \
        RMW_SET_ERROR_MSG( \
          (std::string("unknown type id ") + std::to_string(member->type_id_)).c_str()); \
        return false; \
    } \
  }

#define CREATE_CLIENT(INTROSPECTION_TYPE) \
auto service_members = static_cast<const INTROSPECTION_TYPE(ServiceMembers) *>( \
  type_support->data); \
if (!service_members) { \
  RMW_SET_ERROR_MSG("service members handle is null"); \
  return NULL; \
} \
auto request_members = service_members->request_members_; \
auto response_members = service_members->response_members_; \
request_type_name = _create_type_name( \
  request_members, "srv", type_support->typesupport_identifier); \
response_type_name = _create_type_name( \
  response_members, "srv", type_support->typesupport_identifier); \
request_type_code = create_type_code( \
  request_type_name, request_members, type_support->typesupport_identifier); \
\
if (!request_type_code) { \
  /* error string was set within the function */ \
  goto fail; \
} \
\
/* Allocate memory for the DDS::DynamicDataTypeSupport object. */ \
buf = rmw_allocate(sizeof(DDS::DynamicDataTypeSupport)); \
if (!buf) { \
  RMW_SET_ERROR_MSG("failed to allocate memory"); \
  goto fail; \
} \
/* Use a placement new to construct the DDS::DynamicDataTypeSupport in preallocated buffer. */ \
RMW_TRY_PLACEMENT_NEW( \
  request_type_support, buf, \
  goto fail, \
  DDS::DynamicDataTypeSupport, request_type_code, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT) \
buf = nullptr;  /* Only free the casted pointer; don't need the buf pointer anymore. */ \
\
response_type_code = create_type_code( \
  response_type_name, \
  response_members, \
  type_support->typesupport_identifier); \
if (!request_type_support->is_valid()) { \
  RMW_SET_ERROR_MSG("failed to construct dynamic data type support for request"); \
  goto fail; \
} \
if (!request_type_code) { \
  /* error string was set within the function */ \
  goto fail; \
} \
/* Allocate memory for the DDS::DynamicDataTypeSupport object. */ \
buf = rmw_allocate(sizeof(DDS::DynamicDataTypeSupport)); \
if (!buf) { \
  RMW_SET_ERROR_MSG("failed to allocate memory"); \
  goto fail; \
} \
/* Use a placement new to construct the DDS::DynamicDataTypeSupport in preallocated buffer. */ \
RMW_TRY_PLACEMENT_NEW( \
  response_type_support, buf, \
  goto fail, \
  DDS::DynamicDataTypeSupport, response_type_code, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT) \
buf = nullptr;  /* Only free the casted pointer; don't need the buf pointer anymore. */ \
\
/* create requester */ \
{ \
  if (!get_datareader_qos(participant, * qos_profile, datareader_qos)) { \
    /* error string was set within the function */ \
    goto fail; \
  } \
  if (!get_datawriter_qos(participant, * qos_profile, datawriter_qos)) { \
    /* error string was set within the function */ \
    goto fail; \
  } \
\
  connext::RequesterParams requester_params(participant); \
  requester_params.service_name(service_name); \
  requester_params.request_type_support(request_type_support); \
  requester_params.reply_type_support(response_type_support); \
  requester_params.datareader_qos(datareader_qos); \
  requester_params.datawriter_qos(datawriter_qos); \
\
  /* Allocate memory for the Requester object. */ \
  using Requester = connext::Requester<DDS_DynamicData, DDS_DynamicData>; \
  buf = rmw_allocate(sizeof(connext::Requester<DDS_DynamicData, DDS_DynamicData>)); \
  if (!buf) { \
    RMW_SET_ERROR_MSG("failed to allocate memory"); \
    goto fail; \
  } \
  /* Use a placement new to construct the Requester in the preallocated buffer. */ \
  RMW_TRY_PLACEMENT_NEW( \
    requester, buf, \
    goto fail, \
    Requester, requester_params) \
  buf = nullptr;  /* Only free the casted pointer; don't need the buf pointer anymore. */ \
} \
if (!response_type_support->is_valid()) { \
  RMW_SET_ERROR_MSG("failed to construct dynamic data type support for response"); \
  goto fail; \
} \
\
response_datareader = requester->get_reply_datareader(); \
if (!response_datareader) { \
  RMW_SET_ERROR_MSG("failed to get response datareader"); \
  goto fail; \
} \
\
read_condition = response_datareader->create_readcondition( \
  DDS_ANY_SAMPLE_STATE, DDS_ANY_VIEW_STATE, DDS_ANY_INSTANCE_STATE); \
if (!read_condition) { \
  RMW_SET_ERROR_MSG("failed to create read condition"); \
  goto fail; \
} \
\
/* Allocate memory for the ConnextDynamicClientInfo object. */ \
buf = rmw_allocate(sizeof(ConnextDynamicClientInfo)); \
if (!buf) { \
  RMW_SET_ERROR_MSG("failed to allocate memory"); \
  goto fail; \
} \
RMW_TRY_PLACEMENT_NEW(client_info, buf, goto fail, ConnextDynamicClientInfo) \
client_info->untyped_request_members_ = request_members; \
client_info->untyped_response_members_ = response_members;

#define CREATE_SERVICE(INTROSPECTION_TYPE) \
  auto service_members = static_cast<const INTROSPECTION_TYPE(ServiceMembers) *>( \
    type_support->data); \
  if (!service_members) { \
    RMW_SET_ERROR_MSG("service members handle is null"); \
    return NULL; \
  } \
 \
  const INTROSPECTION_TYPE(MessageMembers) * request_members = \
    service_members->request_members_; \
  if (!request_members) { \
    RMW_SET_ERROR_MSG("request members handle is null"); \
    return NULL; \
  } \
  const INTROSPECTION_TYPE(MessageMembers) * response_members = \
    service_members->response_members_; \
  if (!response_members) { \
    RMW_SET_ERROR_MSG("response members handle is null"); \
    return NULL; \
  } \
 \
  std::string request_type_name = _create_type_name( \
    request_members, \
    "srv", \
    type_support->typesupport_identifier); \
  std::string response_type_name = _create_type_name( \
    response_members, \
    "srv", \
    type_support->typesupport_identifier); \
 \
  DDS_DomainParticipantQos participant_qos; \
  DDS_ReturnCode_t status = participant->get_qos(participant_qos); \
  if (status != DDS_RETCODE_OK) { \
    RMW_SET_ERROR_MSG("failed to get participant qos"); \
    return NULL; \
  } \
  /* Past this point, a failure results in unrolling code in the goto fail block. */ \
  /* Begin initializing elements */ \
  service = rmw_service_allocate(); \
  if (!service) { \
    RMW_SET_ERROR_MSG("failed to allocate memory"); \
    goto fail; \
  } \
 \
  request_type_code = create_type_code( \
    request_type_name, \
    request_members, \
    type_support->typesupport_identifier); \
  if (!request_type_code) { \
    /* error string was set within the function */ \
    goto fail; \
  } \
  /* Allocate memory for the DDS::DynamicDataTypeSupport object. */ \
  buf = rmw_allocate(sizeof(DDS::DynamicDataTypeSupport)); \
  if (!buf) { \
    RMW_SET_ERROR_MSG("failed to allocate memory"); \
    goto fail; \
  } \
  /* Use a placement new to construct the DDS::DynamicDataTypeSupport in preallocated buffer. */ \
  RMW_TRY_PLACEMENT_NEW( \
    request_type_support, buf, \
    goto fail, \
    DDS::DynamicDataTypeSupport, request_type_code, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT) \
  buf = nullptr;  /* Only free the casted pointer; don't need the buf anymore. */ \
 \
  response_type_code = create_type_code( \
    response_type_name, \
    response_members, \
    type_support->typesupport_identifier); \
  if (!response_type_code) { \
    /* error string was set within the function */ \
    goto fail; \
  } \
  /* Allocate memory for the DDS::DynamicDataTypeSupport object. */ \
  buf = rmw_allocate(sizeof(DDS::DynamicDataTypeSupport)); \
  if (!buf) { \
    RMW_SET_ERROR_MSG("failed to allocate memory"); \
    goto fail; \
  } \
  /* Use a placement new to construct the DDS::DynamicDataTypeSupport in preallocated buffer. */ \
  RMW_TRY_PLACEMENT_NEW( \
    response_type_support, buf, \
    goto fail, \
    DDS::DynamicDataTypeSupport, response_type_code, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT) \
  buf = nullptr;  /* Only free the casted pointer; don't need the buf anymore. */ \
 \
  { \
    if (!get_datareader_qos(participant, * qos_profile, datareader_qos)) { \
      /* error string was set within the function */ \
      goto fail; \
    } \
    if (!get_datawriter_qos(participant, * qos_profile, datawriter_qos)) { \
      /* error string was set within the function */ \
      goto fail; \
    } \
 \
    /* create requester */ \
    connext::ReplierParams<DDS_DynamicData, DDS_DynamicData> replier_params(participant); \
    replier_params.service_name(service_name); \
    replier_params.request_type_support(request_type_support); \
    replier_params.reply_type_support(response_type_support); \
    replier_params.datareader_qos(datareader_qos); \
    replier_params.datawriter_qos(datawriter_qos); \
 \
    /* Allocate memory for the Replier object. */ \
    using Replier = connext::Replier<DDS_DynamicData, DDS_DynamicData>; \
    buf = rmw_allocate(sizeof(Replier)); \
    if (!buf) { \
      RMW_SET_ERROR_MSG("failed to allocate memory"); \
      goto fail; \
    } \
    /* Use a placement new to construct the Replier in the preallocated buffer. */ \
    RMW_TRY_PLACEMENT_NEW(replier, buf, goto fail, Replier, replier_params) \
    buf = nullptr;  /* Only free casted pointer; don't need the buf pointer anymore. */ \
  } \
 \
  request_datareader = replier->get_request_datareader(); \
  if (!request_datareader) { \
    RMW_SET_ERROR_MSG("failed to get request datareader"); \
    goto fail; \
  } \
 \
  read_condition = request_datareader->create_readcondition( \
    DDS_ANY_SAMPLE_STATE, DDS_ANY_VIEW_STATE, DDS_ANY_INSTANCE_STATE); \
  if (!read_condition) { \
    RMW_SET_ERROR_MSG("failed to create read condition"); \
    goto fail; \
  } \
 \
  /* Allocate memory for the ConnextDynamicServiceInfo object. */ \
  buf = rmw_allocate(sizeof(ConnextDynamicServiceInfo)); \
  if (!buf) { \
    RMW_SET_ERROR_MSG("failed to allocate memory"); \
    goto fail; \
  } \
  /* Use a placement new to construct the ConnextDynamicServiceInfo in the preallocated buffer.*/ \
  RMW_TRY_PLACEMENT_NEW(server_info, buf, goto fail, ConnextDynamicServiceInfo) \
  buf = nullptr;  /* Only free the casted pointer; don't need the buf pointer anymore. */ \
  server_info->replier_ = replier; \
  server_info->request_datareader_ = request_datareader; \
  server_info->read_condition_ = read_condition; \
  server_info->response_type_support_ = response_type_support; \
  server_info->untyped_request_members_ = request_members; \
  server_info->untyped_response_members_ = response_members; \
  server_info->typesupport_identifier = type_support->typesupport_identifier; \
  service->implementation_identifier = rti_connext_dynamic_identifier; \
  service->data = server_info;

