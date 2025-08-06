#ifndef DDS_COMMON_H_
#define DDS_COMMON_H_

namespace DrDDS {
namespace Common {
const int min_pthread_num = 5;
const int max_pthread_num = 25;
const int timestamp_frames_max = 5;
}; // namespace Common

namespace DomainID {
const int LOCALHOST = 0;
const int MULTIHOST = 10;
const int CLUSTERHOST = 100;
} // namespace DomainID
} // namespace DrDDS

#endif
