#include <iostream>
#include <drdds/core/drdds_core.h>
#include <chrono>
#include <ctime>

class DrDDSCtl
{
  private:
  std::shared_ptr<ChannelJointsData> JDSub_;
  std::shared_ptr<ChannelImuData> ImuDataSub_;
  std::shared_ptr<ChannelLidarPointCloud> LidarSub_;
  static std::queue<std::chrono::system_clock::time_point> TimeQue_;
  static double freq_;

  static void handleJointsData(const drdds::msg::JointsData *msg);
  static void handleImuData(const drdds::msg::ImuData *msg);
  static void handleLidarData(const sensor_msgs::msg::PointCloud2 *msg);
  static void handleMetaType(const drdds::msg::MetaType &msg);
  static void handleStdMsgsHead(const std_msgs::msg::Header &msg);
  static void GetFreq();

  public:
  DrDDSCtl(/* args */);
  ~DrDDSCtl();

  void echo(std::string topic);
  static std::string cmd_;
  static int wsize_;
};
