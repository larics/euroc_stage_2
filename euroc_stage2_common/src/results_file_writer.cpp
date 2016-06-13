#include <euroc_stage2/results_file_writer.h>

namespace euroc_stage2 {

ResultsFileWriter::ResultsFileWriter(const std::string& results_folder_string,
                                     const std::string& team_name,
                                     const std::string& task_name)
    : file_(new std::ofstream) {
  try {
    boost::filesystem::path path =
        boost::filesystem::complete(results_folder_string);

    boost::filesystem::create_directory(path);
    path /= team_name;
    boost::filesystem::create_directory(path);
    path /= task_name;
    boost::filesystem::create_directory(path);

    // use time as filename
    path /= std::to_string(ros::WallTime::now().toSec()) + ".txt";
    ROS_INFO("Creating results file at %s", path.string().c_str());

    file_->open(path.string(), std::ofstream::out | std::ofstream::app);
  } catch (const boost::filesystem::filesystem_error& e) {
    ROS_ERROR(e.what());
  }
}

ResultsFileWriter::~ResultsFileWriter() {
  if (file_->is_open()) file_->close();
}

}  // namespace euroc_stage2
