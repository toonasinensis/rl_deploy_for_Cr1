#ifndef DRDDS_CORE_H_
#define DRDDS_CORE_H_

#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <unordered_map>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/WriterQos.hpp>
#include <fastdds/dds/publisher/PublisherListener.hpp>

#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SubscriberListener.hpp>
#include <fastdds/rtps/common/WriteParams.h>
#include <pthread.h>

#include <string>
#include <functional>
#include <chrono>
#include <deque>
#include <condition_variable>

#include "drdds/core/drdds_manager.h"
#include "drdds/base/dds_common.h"

template <typename PubSubType>
class DrDDSMessage : public DrDDSManager
{
  public:
  DrDDSMessage(std::string name, int domain_id, std::string topic_prefix);
  ~DrDDSMessage();
  std::string GetMessageName(std::string type_name);
  std::string name_;
  eprosima::fastdds::dds::Topic *topic_{nullptr};
  eprosima::fastdds::dds::TypeSupport type_{nullptr};

  private:
  eprosima::fastdds::dds::DomainParticipant *participant_;
  static std::unordered_map<std::string, eprosima::fastdds::dds::Topic *> topic_table;
  static std::unordered_map<std::string, int> topic_count_table;
  static std::unordered_map<std::string, int> type_name_count_table;
};

template <typename PubSubType>
class DrDDSPublisher : public DrDDSManager
{
  using Type = typename PubSubType::type;
  using CallBack = std::function<void(const Type *)>;

  public:
  DrDDSPublisher(std::string topic_name, int domain_id, std::string topic_prefix);
  ~DrDDSPublisher();
  bool Write(const Type *data);
  bool Write();
  void *CreateSample();
  void DeleteSample(void *data);
  float GetRate();
  int GetMatchedCount();

  Type *data_;

  private:
  eprosima::fastdds::dds::DomainParticipant *participant_;
  eprosima::fastdds::dds::Publisher *publisher_{nullptr};
  eprosima::fastdds::dds::DataWriter *writer_;
  class PubListener : public eprosima::fastdds::dds::DataWriterListener
  {
public:
    PubListener() : matched_(0) {}
    ~PubListener() override {}
    void on_publication_matched(
        eprosima::fastdds::dds::DataWriter *writer,
        const eprosima::fastdds::dds::PublicationMatchedStatus &info) override;

    std::atomic_int matched_{0};
    std::deque<std::chrono::steady_clock::time_point> time_stamps_;
    float rate_{1.0f};
  } listener_;

  DrDDSMessage<PubSubType> *message_{nullptr};
};

template <typename PubSubType>
class DrDDSSubscriber : public DrDDSManager
{
  using Type = typename PubSubType::type;
  using CallBack = std::function<void(const Type *)>;

  public:
  DrDDSSubscriber(CallBack handler, std::string topic_name, int domain_id,
                  std::string topic_prefix);
  ~DrDDSSubscriber();
  void BindCallBack(CallBack handler);
  float GetRate();
  int GetMatchedCount();
  bool IsUpdate(uint16_t msec);

  private:
  eprosima::fastdds::dds::DomainParticipant *participant_;
  eprosima::fastdds::dds::Subscriber *subscriber_{nullptr};
  eprosima::fastdds::dds::DataReader *reader_{nullptr};
  class SubListener : public eprosima::fastdds::dds::DataReaderListener
  {
public:
    SubListener() {}
    ~SubListener() override {}
    void on_data_available(eprosima::fastdds::dds::DataReader *reader) override;
    void on_subscription_matched(
        eprosima::fastdds::dds::DataReader *reader,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus &info) override;
    void Doing(void *data);
    CallBack handler_;
    eprosima::fastdds::dds::TypeSupport type_;
    bool is_shutdown_;
    int doing_num_;
    pthread_mutex_t busy_mutex_;

    std::atomic_int matched_{0};
    std::deque<std::chrono::steady_clock::time_point> time_stamps_;
    float rate_{1.0f};
  } listener_;

  DrDDSMessage<PubSubType> *message_{nullptr};
};

template <typename PubSubType>
class DrDDSChannel
{
  using Type = typename PubSubType::type;
  using CallBack = std::function<void(const Type *)>;

  public:
  DrDDSChannel(std::string topic_name = "default", int domain_id = 0);
  DrDDSChannel(const CallBack &handler, std::string topic_name = "default", int domain_id = 0);
  ~DrDDSChannel();
  bool Write(const Type *data);
  bool Write();
  float GetRate();
  int GetMatchedCount();
  bool IsUpdate(uint16_t msec = 1000);

  Type *data_{nullptr};

  private:
  DrDDSPublisher<PubSubType> *publisher_{nullptr};
  DrDDSSubscriber<PubSubType> *subscriber_{nullptr};
};

template <typename RequestType, typename ResponseType>
class DrDDSServerChannel
{
  using ReqType = typename RequestType::type;
  using ResType = typename ResponseType::type;
  using CallBack = std::function<void(const ReqType *, ResType *)>;
  class Listener : public eprosima::fastdds::dds::DataReaderListener
  {
public:
    Listener(eprosima::fastdds::dds::DataWriter *writer) : writer_(writer) {};
    void on_data_available(eprosima::fastdds::dds::DataReader *reader) override;
    void on_subscription_matched(
        eprosima::fastdds::dds::DataReader *reader,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus &info) override;

    CallBack handler_;

private:
    eprosima::fastdds::dds::DataWriter *writer_ = nullptr;
  };

  public:
  DrDDSServerChannel(const CallBack &handler, std::string serivce_name);
  ~DrDDSServerChannel();
  bool Init(const CallBack &handler);

  private:
  eprosima::fastdds::dds::DomainParticipant *participant_ = nullptr;
  eprosima::fastdds::dds::Publisher *publisher_ = nullptr;
  eprosima::fastdds::dds::Subscriber *subscriber_ = nullptr;
  eprosima::fastdds::dds::Topic *request_topic_ = nullptr;
  eprosima::fastdds::dds::Topic *reply_topic_ = nullptr;
  eprosima::fastdds::dds::DataWriter *reply_writer_ = nullptr;
  eprosima::fastdds::dds::DataReader *request_reader_ = nullptr;
  eprosima::fastdds::dds::TypeSupport request_type_;
  eprosima::fastdds::dds::TypeSupport reply_type_;
  Listener listener_ = {nullptr};
  std::string mRequestTopic_;
  std::string mReplyTopic_;
  DrDDSSubscriber<RequestType> *request_subscriber_{nullptr};
  DrDDSPublisher<ResponseType> *response_publisher_{nullptr};
};

template <typename RequestType, typename ResponseType>
class DrDDSClientChannel
{
  using ReqType = typename RequestType::type;
  using ResType = typename ResponseType::type;
  using CallBack = std::function<void(const ReqType *, ResType *)>;
  class Listener : public eprosima::fastdds::dds::DataReaderListener
  {
public:
    void on_data_available(eprosima::fastdds::dds::DataReader *reader) override;
    eprosima::fastrtps::rtps::WriteParams write_params;
    std::mutex reception_mutex;
    std::condition_variable reception_cv;
    bool received_reply = false;
    ResType mResdata_;
  } listener_;

  class PubListener : public eprosima::fastdds::dds::DataWriterListener
  {
public:
    void on_publication_matched(
        eprosima::fastdds::dds::DataWriter *writer,
        const eprosima::fastdds::dds::PublicationMatchedStatus &info) override;
    int mMatchCount_ = {0};
  };

  public:
  DrDDSClientChannel(std::string serivce_name);
  ~DrDDSClientChannel();
  bool Call(ReqType req, ResType &res);
  bool Init();
  PubListener mWriteListener_;

  private:
  eprosima::fastdds::dds::DomainParticipant *participant_ = nullptr;
  eprosima::fastdds::dds::Publisher *publisher_ = nullptr;
  eprosima::fastdds::dds::Subscriber *subscriber_ = nullptr;
  eprosima::fastdds::dds::Topic *request_topic_ = nullptr;
  eprosima::fastdds::dds::Topic *reply_topic_ = nullptr;
  eprosima::fastdds::dds::DataWriter *request_writer_ = nullptr;
  eprosima::fastdds::dds::DataReader *reply_reader_ = nullptr;
  eprosima::fastdds::dds::TypeSupport request_type_;
  eprosima::fastdds::dds::TypeSupport reply_type_;
  std::string mRequestTopic_;
  std::string mReplyTopic_;
  DrDDSPublisher<RequestType> *request_publisher_{nullptr};
  DrDDSSubscriber<ResponseType> *response_subscriber_{nullptr};
};

#include "drdds/core/common_type.h"

#endif
