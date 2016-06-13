#ifndef EUROC_STAGE2_COMMON_RESULTS_FILE_WRITER_H
#define EUROC_STAGE2_COMMON_RESULTS_FILE_WRITER_H

#include <fstream>
#include <memory>
#include <boost/filesystem.hpp>

#include <ros/ros.h>

namespace euroc_stage2 {

class ResultsFileWriter {
 public:
  ResultsFileWriter(const std::string& results_folder,
                    const std::string& team_name, const std::string& task_name);
  ~ResultsFileWriter();
  template <typename T>
  ResultsFileWriter& operator<<(const T& data) {
    if (file_->is_open()) {
      (*file_) << data << std::flush;
      return *this;
    } else {
      ROS_ERROR("No open file, cannot write data");
      return *this;
    }
  }

 private:
  std::shared_ptr<std::ofstream> file_;
};
}

#endif
