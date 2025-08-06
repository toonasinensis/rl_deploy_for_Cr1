#ifndef COMMON_TYPE_H_
#define COMMON_TYPE_H_

#include "dridl/gen_types.h"
#include "drdds/core/drdds_core.h"

// Node
template class DrDDSChannel<drdds::msg::AppStatusPubSubType>;
template class DrDDSPublisher<drdds::msg::AppStatusPubSubType>;
template class DrDDSSubscriber<drdds::msg::AppStatusPubSubType>;
template class DrDDSMessage<drdds::msg::AppStatusPubSubType>;
using ChannelAppStatus = DrDDSChannel<drdds::msg::AppStatusPubSubType>;

template class DrDDSChannel<drdds::msg::SetNodePubSubType>;
template class DrDDSPublisher<drdds::msg::SetNodePubSubType>;
template class DrDDSSubscriber<drdds::msg::SetNodePubSubType>;
template class DrDDSMessage<drdds::msg::SetNodePubSubType>;
using ChannelSetNode = DrDDSChannel<drdds::msg::SetNodePubSubType>;

template class DrDDSChannel<drdds::msg::NodeStatusPubSubType>;
template class DrDDSPublisher<drdds::msg::NodeStatusPubSubType>;
template class DrDDSSubscriber<drdds::msg::NodeStatusPubSubType>;
template class DrDDSMessage<drdds::msg::NodeStatusPubSubType>;
using ChannelNodeStatus = DrDDSChannel<drdds::msg::NodeStatusPubSubType>;

// Drivers
template class DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>;
template class DrDDSPublisher<sensor_msgs::msg::PointCloud2PubSubType>;
template class DrDDSSubscriber<sensor_msgs::msg::PointCloud2PubSubType>;
template class DrDDSMessage<sensor_msgs::msg::PointCloud2PubSubType>;
using ChannelPointCloud2 = DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>;
using ChannelLidarPointCloud = DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>;

template class DrDDSChannel<sensor_msgs::msg::ImuPubSubType>;
template class DrDDSPublisher<sensor_msgs::msg::ImuPubSubType>;
template class DrDDSSubscriber<sensor_msgs::msg::ImuPubSubType>;
template class DrDDSMessage<sensor_msgs::msg::ImuPubSubType>;
using ChannelImu = DrDDSChannel<sensor_msgs::msg::ImuPubSubType>;

// Motion
template class DrDDSChannel<drdds::msg::ExeHealthPubSubType>;
template class DrDDSPublisher<drdds::msg::ExeHealthPubSubType>;
template class DrDDSSubscriber<drdds::msg::ExeHealthPubSubType>;
template class DrDDSMessage<drdds::msg::ExeHealthPubSubType>;
using ChannelExeHealth = DrDDSChannel<drdds::msg::ExeHealthPubSubType>;

template class DrDDSChannel<drdds::msg::ExeServicePubSubType>;
template class DrDDSPublisher<drdds::msg::ExeServicePubSubType>;
template class DrDDSSubscriber<drdds::msg::ExeServicePubSubType>;
template class DrDDSMessage<drdds::msg::ExeServicePubSubType>;
using ChannelExeService = DrDDSChannel<drdds::msg::ExeServicePubSubType>;

template class DrDDSChannel<drdds::msg::GaitPubSubType>;
template class DrDDSPublisher<drdds::msg::GaitPubSubType>;
template class DrDDSSubscriber<drdds::msg::GaitPubSubType>;
template class DrDDSMessage<drdds::msg::GaitPubSubType>;
using ChannelGait = DrDDSChannel<drdds::msg::GaitPubSubType>;

template class DrDDSChannel<drdds::msg::ImuDataPubSubType>;
template class DrDDSPublisher<drdds::msg::ImuDataPubSubType>;
template class DrDDSSubscriber<drdds::msg::ImuDataPubSubType>;
template class DrDDSMessage<drdds::msg::ImuDataPubSubType>;
using ChannelImuData = DrDDSChannel<drdds::msg::ImuDataPubSubType>;

template class DrDDSChannel<drdds::msg::JointsDataPubSubType>;
template class DrDDSPublisher<drdds::msg::JointsDataPubSubType>;
template class DrDDSSubscriber<drdds::msg::JointsDataPubSubType>;
template class DrDDSMessage<drdds::msg::JointsDataPubSubType>;
using ChannelJointsData = DrDDSChannel<drdds::msg::JointsDataPubSubType>;

template class DrDDSChannel<drdds::msg::MotionActionPubSubType>;
template class DrDDSPublisher<drdds::msg::MotionActionPubSubType>;
template class DrDDSSubscriber<drdds::msg::MotionActionPubSubType>;
template class DrDDSMessage<drdds::msg::MotionActionPubSubType>;
using ChannelMotionAction = DrDDSChannel<drdds::msg::MotionActionPubSubType>;

template class DrDDSChannel<drdds::msg::MotionStatePubSubType>;
template class DrDDSPublisher<drdds::msg::MotionStatePubSubType>;
template class DrDDSSubscriber<drdds::msg::MotionStatePubSubType>;
template class DrDDSMessage<drdds::msg::MotionStatePubSubType>;
using ChannelMotionState = DrDDSChannel<drdds::msg::MotionStatePubSubType>;

template class DrDDSChannel<drdds::msg::MotionBasePubSubType>;
template class DrDDSPublisher<drdds::msg::MotionBasePubSubType>;
template class DrDDSSubscriber<drdds::msg::MotionBasePubSubType>;
template class DrDDSMessage<drdds::msg::MotionBasePubSubType>;
using ChannelMotionBaseData = DrDDSChannel<drdds::msg::MotionBasePubSubType>;

template class DrDDSChannel<drdds::msg::MotionParamSetPubSubType>;
template class DrDDSPublisher<drdds::msg::MotionParamSetPubSubType>;
template class DrDDSSubscriber<drdds::msg::MotionParamSetPubSubType>;
template class DrDDSMessage<drdds::msg::MotionParamSetPubSubType>;
using ChannelMotionParamSet = DrDDSChannel<drdds::msg::MotionParamSetPubSubType>;

template class DrDDSChannel<drdds::msg::MotionStatusPubSubType>;
template class DrDDSPublisher<drdds::msg::MotionStatusPubSubType>;
template class DrDDSSubscriber<drdds::msg::MotionStatusPubSubType>;
template class DrDDSMessage<drdds::msg::MotionStatusPubSubType>;
using ChannelMotionStatus = DrDDSChannel<drdds::msg::MotionStatusPubSubType>;

template class DrDDSChannel<drdds::msg::MotionToPubSubType>;
template class DrDDSPublisher<drdds::msg::MotionToPubSubType>;
template class DrDDSSubscriber<drdds::msg::MotionToPubSubType>;
template class DrDDSMessage<drdds::msg::MotionToPubSubType>;
using ChannelMotionTo = DrDDSChannel<drdds::msg::MotionToPubSubType>;

template class DrDDSChannel<drdds::msg::MotionPosePubSubType>;
template class DrDDSPublisher<drdds::msg::MotionPosePubSubType>;
template class DrDDSSubscriber<drdds::msg::MotionPosePubSubType>;
template class DrDDSMessage<drdds::msg::MotionPosePubSubType>;
using ChannelMotionPose = DrDDSChannel<drdds::msg::MotionPosePubSubType>;

template class DrDDSChannel<drdds::msg::SteerPubSubType>;
template class DrDDSPublisher<drdds::msg::SteerPubSubType>;
template class DrDDSSubscriber<drdds::msg::SteerPubSubType>;
template class DrDDSMessage<drdds::msg::SteerPubSubType>;
using ChannelSteer = DrDDSChannel<drdds::msg::SteerPubSubType>;

template class DrDDSChannel<drdds::msg::FaultStatusPubSubType>;
template class DrDDSPublisher<drdds::msg::FaultStatusPubSubType>;
template class DrDDSSubscriber<drdds::msg::FaultStatusPubSubType>;
template class DrDDSMessage<drdds::msg::FaultStatusPubSubType>;
using ChannelFaultStatus = DrDDSChannel<drdds::msg::FaultStatusPubSubType>;

// Location
using ChannelGlobalMap = DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>;

template class DrDDSChannel<nav_msgs::msg::OdometryPubSubType>;
template class DrDDSPublisher<nav_msgs::msg::OdometryPubSubType>;
template class DrDDSSubscriber<nav_msgs::msg::OdometryPubSubType>;
template class DrDDSMessage<nav_msgs::msg::OdometryPubSubType>;
using ChannelOdometry = DrDDSChannel<nav_msgs::msg::OdometryPubSubType>;

template class DrDDSChannel<geometry_msgs::msg::PoseWithCovarianceStampedPubSubType>;
template class DrDDSPublisher<geometry_msgs::msg::PoseWithCovarianceStampedPubSubType>;
template class DrDDSSubscriber<geometry_msgs::msg::PoseWithCovarianceStampedPubSubType>;
template class DrDDSMessage<geometry_msgs::msg::PoseWithCovarianceStampedPubSubType>;
using ChannelInitialPose = DrDDSChannel<geometry_msgs::msg::PoseWithCovarianceStampedPubSubType>;

template class DrDDSChannel<drdds::msg::LocationStatusPubSubType>;
template class DrDDSPublisher<drdds::msg::LocationStatusPubSubType>;
template class DrDDSSubscriber<drdds::msg::LocationStatusPubSubType>;
template class DrDDSMessage<drdds::msg::LocationStatusPubSubType>;
using ChannelLocationStatus = DrDDSChannel<drdds::msg::LocationStatusPubSubType>;

template class DrDDSChannel<nav_msgs::msg::OccupancyGridPubSubType>;
template class DrDDSPublisher<nav_msgs::msg::OccupancyGridPubSubType>;
template class DrDDSSubscriber<nav_msgs::msg::OccupancyGridPubSubType>;
template class DrDDSMessage<nav_msgs::msg::OccupancyGridPubSubType>;
using ChannelMap = DrDDSChannel<nav_msgs::msg::OccupancyGridPubSubType>;

// Plan
template class DrDDSChannel<geometry_msgs::msg::PoseStampedPubSubType>;
template class DrDDSPublisher<geometry_msgs::msg::PoseStampedPubSubType>;
template class DrDDSSubscriber<geometry_msgs::msg::PoseStampedPubSubType>;
template class DrDDSMessage<geometry_msgs::msg::PoseStampedPubSubType>;
using ChannelGoal = DrDDSChannel<geometry_msgs::msg::PoseStampedPubSubType>;

template class DrDDSChannel<nav_msgs::msg::PathPubSubType>;
template class DrDDSPublisher<nav_msgs::msg::PathPubSubType>;
template class DrDDSSubscriber<nav_msgs::msg::PathPubSubType>;
template class DrDDSMessage<nav_msgs::msg::PathPubSubType>;
using ChannelPath = DrDDSChannel<nav_msgs::msg::PathPubSubType>;

template class DrDDSChannel<drdds::msg::NavCmdPubSubType>;
template class DrDDSPublisher<drdds::msg::NavCmdPubSubType>;
template class DrDDSSubscriber<drdds::msg::NavCmdPubSubType>;
template class DrDDSMessage<drdds::msg::NavCmdPubSubType>;
using ChannelNavCmd = DrDDSChannel<drdds::msg::NavCmdPubSubType>;

template class DrDDSChannel<drdds::msg::CancelNavTaskPubSubType>;
template class DrDDSPublisher<drdds::msg::CancelNavTaskPubSubType>;
template class DrDDSSubscriber<drdds::msg::CancelNavTaskPubSubType>;
template class DrDDSMessage<drdds::msg::CancelNavTaskPubSubType>;
using ChannelCancelNavTask = DrDDSChannel<drdds::msg::CancelNavTaskPubSubType>;

template class DrDDSChannel<drdds::msg::PlannerStatusPubSubType>;
template class DrDDSPublisher<drdds::msg::PlannerStatusPubSubType>;
template class DrDDSSubscriber<drdds::msg::PlannerStatusPubSubType>;
template class DrDDSMessage<drdds::msg::PlannerStatusPubSubType>;
using ChannelPlannerStatus = DrDDSChannel<drdds::msg::PlannerStatusPubSubType>;

template class DrDDSChannel<drdds::msg::AntiCollisionStatusPubSubType>;
template class DrDDSPublisher<drdds::msg::AntiCollisionStatusPubSubType>;
template class DrDDSSubscriber<drdds::msg::AntiCollisionStatusPubSubType>;
template class DrDDSMessage<drdds::msg::AntiCollisionStatusPubSubType>;
using ChannelAntiCollisionStatus = DrDDSChannel<drdds::msg::AntiCollisionStatusPubSubType>;

template class DrDDSChannel<drdds::msg::SetAntiCollisionPubSubType>;
template class DrDDSPublisher<drdds::msg::SetAntiCollisionPubSubType>;
template class DrDDSSubscriber<drdds::msg::SetAntiCollisionPubSubType>;
template class DrDDSMessage<drdds::msg::SetAntiCollisionPubSubType>;
using ChannelSetAntiCollision = DrDDSChannel<drdds::msg::SetAntiCollisionPubSubType>;

template class DrDDSChannel<geometry_msgs::msg::TwistPubSubType>;
template class DrDDSPublisher<geometry_msgs::msg::TwistPubSubType>;
template class DrDDSSubscriber<geometry_msgs::msg::TwistPubSubType>;
template class DrDDSMessage<geometry_msgs::msg::TwistPubSubType>;
using ChannelHandlerVel = DrDDSChannel<geometry_msgs::msg::TwistPubSubType>;

// VMap
using ChannelObstaclePointCloud = DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>;
using ChannelQuadrangle = DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>;

template class DrDDSChannel<drdds::msg::SetVMapTerrainModePubSubType>;
template class DrDDSPublisher<drdds::msg::SetVMapTerrainModePubSubType>;
template class DrDDSSubscriber<drdds::msg::SetVMapTerrainModePubSubType>;
template class DrDDSMessage<drdds::msg::SetVMapTerrainModePubSubType>;
using ChannelSetVMapTerrainMode = DrDDSChannel<drdds::msg::SetVMapTerrainModePubSubType>;

template class DrDDSChannel<drdds::msg::SetVMapObsHeightPubSubType>;
template class DrDDSPublisher<drdds::msg::SetVMapObsHeightPubSubType>;
template class DrDDSSubscriber<drdds::msg::SetVMapObsHeightPubSubType>;
template class DrDDSMessage<drdds::msg::SetVMapObsHeightPubSubType>;
using ChannelSetVMapObsHeight = DrDDSChannel<drdds::msg::SetVMapObsHeightPubSubType>;

template class DrDDSChannel<drdds::msg::VMapStatusPubSubType>;
template class DrDDSPublisher<drdds::msg::VMapStatusPubSubType>;
template class DrDDSSubscriber<drdds::msg::VMapStatusPubSubType>;
template class DrDDSMessage<drdds::msg::VMapStatusPubSubType>;
using ChannelVMapStatus = DrDDSChannel<drdds::msg::VMapStatusPubSubType>;

// Charger
template class DrDDSChannel<drdds::msg::SetChargeManagerPubSubType>;
template class DrDDSPublisher<drdds::msg::SetChargeManagerPubSubType>;
template class DrDDSSubscriber<drdds::msg::SetChargeManagerPubSubType>;
template class DrDDSMessage<drdds::msg::SetChargeManagerPubSubType>;
using ChannelSetCharger = DrDDSChannel<drdds::msg::SetChargeManagerPubSubType>;

template class DrDDSChannel<drdds::msg::ChargeManagerStatusPubSubType>;
template class DrDDSPublisher<drdds::msg::ChargeManagerStatusPubSubType>;
template class DrDDSSubscriber<drdds::msg::ChargeManagerStatusPubSubType>;
template class DrDDSMessage<drdds::msg::ChargeManagerStatusPubSubType>;
using ChannelChargerStatus = DrDDSChannel<drdds::msg::ChargeManagerStatusPubSubType>;

// test
template class DrDDSServerChannel<std_srvs::srv::SetBool_RequestPubSubType,
                                  std_srvs::srv::SetBool_ResponsePubSubType>;
using ChannelServerTest = DrDDSServerChannel<std_srvs::srv::SetBool_RequestPubSubType,
                                             std_srvs::srv::SetBool_ResponsePubSubType>;
template class DrDDSClientChannel<std_srvs::srv::SetBool_RequestPubSubType,
                                  std_srvs::srv::SetBool_ResponsePubSubType>;
using ChannelClientTest = DrDDSClientChannel<std_srvs::srv::SetBool_RequestPubSubType,
                                             std_srvs::srv::SetBool_ResponsePubSubType>;

// set_param
template class DrDDSServerChannel<drdds::srv::SetParam_RequestPubSubType,
                                  drdds::srv::SetParam_ResponsePubSubType>;
using ChannelServerSetParam = DrDDSServerChannel<drdds::srv::SetParam_RequestPubSubType,
                                                 drdds::srv::SetParam_ResponsePubSubType>;
template class DrDDSClientChannel<drdds::srv::SetParam_RequestPubSubType,
                                  drdds::srv::SetParam_ResponsePubSubType>;
using ChannelClientSetParam = DrDDSClientChannel<drdds::srv::SetParam_RequestPubSubType,
                                                 drdds::srv::SetParam_ResponsePubSubType>;

template class DrDDSServerChannel<drdds::srv::UpdateMap_RequestPubSubType,
                                  drdds::srv::UpdateMap_ResponsePubSubType>;
using ChannelServerUpdateMap = DrDDSServerChannel<drdds::srv::UpdateMap_RequestPubSubType,
                                                  drdds::srv::UpdateMap_ResponsePubSubType>;
template class DrDDSClientChannel<drdds::srv::UpdateMap_RequestPubSubType,
                                  drdds::srv::UpdateMap_ResponsePubSubType>;
using ChannelClientUpdateMap = DrDDSClientChannel<drdds::srv::UpdateMap_RequestPubSubType,
                                                  drdds::srv::UpdateMap_ResponsePubSubType>;

template class DrDDSServerChannel<drdds::srv::LoadMap_RequestPubSubType,
                                  drdds::srv::LoadMap_ResponsePubSubType>;
using ChannelServerLoadMap = DrDDSServerChannel<drdds::srv::LoadMap_RequestPubSubType,
                                                drdds::srv::LoadMap_ResponsePubSubType>;
template class DrDDSClientChannel<drdds::srv::LoadMap_RequestPubSubType,
                                  drdds::srv::LoadMap_ResponsePubSubType>;
using ChannelClientLoadMap = DrDDSClientChannel<drdds::srv::LoadMap_RequestPubSubType,
                                                drdds::srv::LoadMap_ResponsePubSubType>;

#endif
