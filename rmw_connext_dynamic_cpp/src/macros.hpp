// Two different kinds of concatenation are needed for these namespaces
#define INTROSPECTION_CPP_TYPE(A) rosidl_typesupport_introspection_cpp::A

#define INTROSPECTION_C_TYPE(A) rosidl_typesupport_introspection_c__ ## A

#define ASSIGN_INTROSPECTION_FIELDS(INTROSPECTION_TYPE) \
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
          auto sub_members = static_cast<const MembersType *>(member->members_->data); \
          if (!sub_members) { \
            RMW_SET_ERROR_MSG("sub members handle is null"); \
            return NULL; \
          } \
          std::string field_type_name = _create_type_name(sub_members, "msg"); \
          member_type_code = create_type_code(field_type_name, sub_members); \
          if (!member_type_code) { \
            /* error string was set within the function */ \
            goto fail; \
          } \
        } \
        break; \
      default: \
        RMW_SET_ERROR_MSG( \
          (std::string("unknown type id ") + std::to_string(member->type_id_)).c_str()); \
        goto fail; \
    } \


