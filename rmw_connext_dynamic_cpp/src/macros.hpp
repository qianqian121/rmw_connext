// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MACROS_HPP_
#define MACROS_HPP_

#include <limits>
#include <string>

// Two different kinds of concatenation are needed for these namespaces
#define INTROSPECTION_CPP_TYPE(A) rosidl_typesupport_introspection_cpp::A

#define INTROSPECTION_C_TYPE(A) rosidl_typesupport_introspection_c__ ## A

#define C_STRING_ASSIGN(A, B) rosidl_generator_c__String__assignn(&A, B, size - 1);

#define CPP_STRING_ASSIGN(A, B) A = B;

#define ASSIGN_TYPE_CODE_INTROSPECTION_FIELDS(INTROSPECTION_TYPE) \
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
        std::string field_type_name = _create_type_name<MembersType>(sub_members, "msg"); \
        member_type_code = create_type_code<MembersType>(field_type_name, sub_members, \
            introspection_identifier); \
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
  }


#define SET_VALUE_INTROSPECTION_FIELDS(INTROSPECTION_TYPE) \
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
      /* TODO */ \
      if (using_introspection_c_typesupport(typesupport)) { \
        SET_STRING_VALUE(rosidl_generator_c__String, set_string, data) \
      } else if (using_introspection_cpp_typesupport(typesupport)) { \
        SET_STRING_VALUE(std::string, set_string, c_str()) \
      } \
      break; \
    case INTROSPECTION_TYPE(ROS_TYPE_MESSAGE): \
      { \
        if (member->is_array_) { \
          const void * untyped_member = static_cast<const char *>(ros_message) + member->offset_; \
          if (!member->size_function) { \
            RMW_SET_ERROR_MSG("size function handle is null"); \
            return false; \
          } \
          if (!member->get_const_function) { \
            RMW_SET_ERROR_MSG("get const function handle is null"); \
            return false; \
          } \
 \
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
              /* offset message pointer since the macro adds the member offset to it */ \
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
  }


#define GET_VALUE_INTROSPECTION_FIELDS(INTROSPECTION_TYPE) \
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
        GET_STRING_VALUE(rosidl_generator_c__String, get_string, C_STRING_ASSIGN) \
      } else if (using_introspection_cpp_typesupport(typesupport)) { \
        GET_STRING_VALUE(std::string, get_string, CPP_STRING_ASSIGN) \
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
 \
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
              /* offset message pointer since the macro adds the member offset to it */ \
              ros_message = static_cast<char *>(sub_ros_message) - member->offset_; \
            } \
            DDS_DynamicData * array_data_ptr = &array_data; \
            /* TODO(dirk-thomas) if the macro return unbind is not called */ \
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
  }

#endif  // MACROS_HPP_
