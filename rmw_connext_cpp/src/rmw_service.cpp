// Copyright 2014-2017 Open Source Robotics Foundation, Inc.
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

#include <string>

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

#include "rmw_connext_shared_cpp/qos.hpp"
#include "rmw_connext_shared_cpp/types.hpp"

#include "identifier.hpp"
#include "process_topic_and_service_names.hpp"
#include "type_support_common.hpp"
#include "types/connext_static_service_info.hpp"

// Uncomment this to get extra console output about discovery.
// This affects code in this file, but there is a similar variable in:
//   rmw_connext_shared_cpp/shared_functions.cpp
// #define DISCOVERY_DEBUG_LOGGING 1

extern "C"
{
rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return NULL;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, rti_connext_identifier,
    return NULL)

  RMW_CONNEXT_EXTRACT_SERVICE_TYPESUPPORT(type_supports, type_support)

  if (!qos_profile) {
    RMW_SET_ERROR_MSG("qos_profile is null");
    return nullptr;
  }

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return NULL;
  }
  auto participant = static_cast<DDSDomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return NULL;
  }

  const service_type_support_callbacks_t * callbacks =
    static_cast<const service_type_support_callbacks_t *>(type_support->data);
  if (!callbacks) {
    RMW_SET_ERROR_MSG("callbacks handle is null");
    return NULL;
  }

  // Past this point, a failure results in unrolling code in the goto fail block.
  DDS_DataReaderQos datareader_qos;
  DDS_DataWriterQos datawriter_qos;
  DDS_SubscriberQos subscriber_qos;
  DDS_PublisherQos publisher_qos;
  DDS_ReturnCode_t status;
  DDS::Publisher * dds_publisher = nullptr;
  DDS::Subscriber * dds_subscriber = nullptr;
  DDS::DataReader * request_datareader = nullptr;
  DDS::DataWriter * response_datawriter = nullptr;
  DDS::ReadCondition * read_condition = nullptr;
  void * replier = nullptr;
  void * buf = nullptr;
  ConnextStaticServiceInfo * service_info = nullptr;
  rmw_service_t * service = nullptr;
  std::string mangled_name = "";

  // memory allocations for namespacing
  char * request_partition_str = nullptr;
  char * response_partition_str = nullptr;
  char * service_str = nullptr;

  // Begin initializing elements.
  service = rmw_service_allocate();
  if (!service) {
    RMW_SET_ERROR_MSG("service handle is null");
    goto fail;
  }

  if (!get_datareader_qos(participant, *qos_profile, datareader_qos)) {
    // error string was set within the function
    goto fail;
  }

  if (!get_datawriter_qos(participant, *qos_profile, datawriter_qos)) {
    // error string was set within the function
    goto fail;
  }

  if (!_process_service_name(
      service_name,
      qos_profile->avoid_ros_namespace_conventions,
      &service_str,
      &request_partition_str,
      &response_partition_str))
  {
    goto fail;
  }

  replier = callbacks->create_replier(
    participant, service_str, &datareader_qos, &datawriter_qos,
    reinterpret_cast<void **>(&request_datareader),
    reinterpret_cast<void **>(&response_datawriter),
    &rmw_allocate);
  if (!replier) {
    RMW_SET_ERROR_MSG("failed to create replier");
    goto fail;
  }
  if (!request_datareader) {
    RMW_SET_ERROR_MSG("data reader handle is null");
    goto fail;
  }
  if (!response_datawriter) {
    RMW_SET_ERROR_MSG("data writer handle is null");
    goto fail;
  }

  read_condition = request_datareader->create_readcondition(
    DDS_ANY_SAMPLE_STATE, DDS_ANY_VIEW_STATE, DDS_ANY_INSTANCE_STATE);
  if (!read_condition) {
    RMW_SET_ERROR_MSG("failed to create read condition");
    goto fail;
  }

  dds_subscriber = request_datareader->get_subscriber();
  status = participant->get_default_subscriber_qos(subscriber_qos);
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get default subscriber qos");
    goto fail;
  }

  dds_publisher = response_datawriter->get_publisher();
  status = participant->get_default_publisher_qos(publisher_qos);
  if (status != DDS_RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get default subscriber qos");
    goto fail;
  }

  // we have to set the partition array to length 1
  // and then set the partition_str in it
  if (request_partition_str && strlen(request_partition_str) != 0) {
    subscriber_qos.partition.name.ensure_length(1, 1);
    subscriber_qos.partition.name[0] = request_partition_str;
  }
  // update attached subscriber
  dds_subscriber->set_qos(subscriber_qos);

  if (response_partition_str && strlen(response_partition_str) != 0) {
    publisher_qos.partition.name.ensure_length(1, 1);
    publisher_qos.partition.name[0] = response_partition_str;
  }
  // update attached publisher
  dds_publisher->set_qos(publisher_qos);

  buf = rmw_allocate(sizeof(ConnextStaticServiceInfo));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  // Use a placement new to construct the ConnextStaticServiceInfo in the preallocated buffer.
  RMW_TRY_PLACEMENT_NEW(service_info, buf, goto fail, ConnextStaticServiceInfo, )
  buf = nullptr;  // Only free the service_info pointer; don't need the buf pointer anymore.
  service_info->replier_ = replier;
  service_info->callbacks_ = callbacks;
  service_info->request_datareader_ = request_datareader;
  service_info->read_condition_ = read_condition;

  service->implementation_identifier = rti_connext_identifier;
  service->data = service_info;
  service->service_name = reinterpret_cast<const char *>(rmw_allocate(strlen(service_name) + 1));
  if (!service->service_name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node name");
    goto fail;
  }
  memcpy(const_cast<char *>(service->service_name), service_name, strlen(service_name) + 1);

  mangled_name =
    std::string(subscriber_qos.partition.name[0]) +
    "/" +
    request_datareader->get_topicdescription()->get_name();
  node_info->subscriber_listener->add_information(
    request_datareader->get_instance_handle(),
    mangled_name.c_str(),
    request_datareader->get_topicdescription()->get_type_name(),
    EntityType::Subscriber);
  node_info->subscriber_listener->trigger_graph_guard_condition();

  mangled_name =
    std::string(publisher_qos.partition.name[0]) +
    "/" +
    response_datawriter->get_topic()->get_name();
  node_info->publisher_listener->add_information(
    response_datawriter->get_instance_handle(),
    mangled_name.c_str(),
    response_datawriter->get_topic()->get_type_name(),
    EntityType::Publisher);
  node_info->publisher_listener->trigger_graph_guard_condition();

// TODO(karsten1987): replace this block with logging macros
#ifdef DISCOVERY_DEBUG_LOGGING
  fprintf(stderr, "******* Creating Service Details: ********\n");
  fprintf(stderr, "Req DataReader Subscriber partition %s\n", subscriber_qos.partition.name[0]);
  fprintf(stderr, "Subscriber topic %s\n", request_datareader->get_topicdescription()->get_name());
  fprintf(stderr, "Subscriber address %p\n", static_cast<void *>(dds_subscriber));
  fprintf(stderr, "Resp DataWriter Publisher partition %s\n", publisher_qos.partition.name[0]);
  fprintf(stderr, "Publisher topic %s\n", response_datawriter->get_topic()->get_name());
  fprintf(stderr, "Publisher address %p\n", static_cast<void *>(dds_publisher));
  fprintf(stderr, "******\n");
#endif

  return service;
fail:
  if (service) {
    rmw_service_free(service);
  }
  if (request_datareader) {
    if (read_condition) {
      if (request_datareader->delete_readcondition(read_condition) != DDS_RETCODE_OK) {
        std::stringstream ss;
        ss << "leaking readcondition while handling failure at " <<
          __FILE__ << ":" << __LINE__ << '\n';
        (std::cerr << ss.str()).flush();
      }
    }
    if (participant->delete_datareader(request_datareader) != RMW_RET_OK) {
      std::stringstream ss;
      ss << "leaking datareader while handling failure at " <<
        __FILE__ << ":" << __LINE__ << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (service_info) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      service_info->~ConnextStaticServiceInfo(), ConnextStaticServiceInfo)
    rmw_free(service_info);
  }
  if (buf) {
    rmw_free(buf);
  }

  return NULL;
}

rmw_ret_t
rmw_destroy_service(rmw_node_t * node, rmw_service_t * service)
{
  if (!node) {
    RMW_SET_ERROR_MSG("node handle is null");
    return RMW_RET_ERROR;
  }
  if (!service) {
    RMW_SET_ERROR_MSG("service handle is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service handle,
    service->implementation_identifier, rti_connext_identifier,
    return RMW_RET_ERROR)

  auto result = RMW_RET_OK;
  ConnextStaticServiceInfo * service_info = static_cast<ConnextStaticServiceInfo *>(service->data);

  auto node_info = static_cast<ConnextNodeInfo *>(node->data);

  if (service_info) {
    auto request_datareader = service_info->request_datareader_;

    node_info->subscriber_listener->remove_information(
      service_info->request_datareader_->get_instance_handle(),
      EntityType::Subscriber);
    node_info->subscriber_listener->trigger_graph_guard_condition();

    DDS::DataWriter * reply_datawriter = static_cast<DDS::DataWriter *>(
      service_info->callbacks_->get_reply_datawriter(service_info->replier_));
    node_info->publisher_listener->remove_information(
      reply_datawriter->get_instance_handle(),
      EntityType::Publisher);
    node_info->publisher_listener->trigger_graph_guard_condition();

    if (request_datareader) {
      auto read_condition = service_info->read_condition_;
      if (read_condition) {
        if (request_datareader->delete_readcondition(read_condition) != DDS_RETCODE_OK) {
          RMW_SET_ERROR_MSG("failed to delete readcondition");
          result = RMW_RET_ERROR;
        }
        service_info->read_condition_ = nullptr;
      }
    } else if (service_info->read_condition_) {
      RMW_SET_ERROR_MSG("cannot delete readcondition because the datareader is null");
      result = RMW_RET_ERROR;
    }
    const service_type_support_callbacks_t * callbacks = service_info->callbacks_;
    if (callbacks) {
      if (service_info->replier_) {
        callbacks->destroy_replier(service_info->replier_, &rmw_free);
      }
    }

    RMW_TRY_DESTRUCTOR(
      service_info->~ConnextStaticServiceInfo(),
      ConnextStaticServiceInfo, result = RMW_RET_ERROR)
    rmw_free(service_info);
    service->data = nullptr;
    if (service->service_name) {
      rmw_free(const_cast<char *>(service->service_name));
    }
  }
  rmw_service_free(service);

  return result;
}
}  // extern "C"
