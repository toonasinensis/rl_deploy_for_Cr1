#ifndef DDS_MANAGER_H_
#define DDS_MANAGER_H_

#include <fastrtps/Domain.h>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>

#include <set>

#include "dridl/builtin_interfaces/msg/Time.h"
#include "drdds/base/pthread_pool.h"
#include "drdds/base/dds_common.h"

class DrDDSManager
{
  public:
  static void Init(std::vector<int> domain_id, std::string module_id, std::string node_name,
                   bool enable_config = false, bool enable_pool = false,
                   bool enable_discovery = false);
  static void SetDiscovery(std::string ip, uint16_t port);
  static std::set<std::string> GetParticipants();
  static builtin_interfaces::msg::Time Now(bool is_steady = true);
  static void Delete();
  static bool Ok();

  protected:
  static eprosima::fastdds::dds::DomainParticipant *mMultiParticipant_;
  static eprosima::fastdds::dds::DomainParticipant *mLocalParticipant_;
  static DrDDS::PthreadPool *pool_;
  static bool enable_config_;
  static bool enable_pool_;
  static bool enable_discovery_;
  static std::string name_;

  private:
  static bool InitParticipant();
  static std::vector<int> domain_id_;
  static std::vector<uint8_t> ip_;
  static uint16_t port_;
  static class PartListener : public eprosima::fastdds::dds::DomainParticipantListener
  {
public:
    PartListener() {}
    ~PartListener() override {}
    void on_participant_discovery(
        eprosima::fastdds::dds::DomainParticipant *participant,
        eprosima::fastrtps::rtps::ParticipantDiscoveryInfo &&info) override;
    std::set<std::string> participant_list_;
  } listener_;
};

#endif
