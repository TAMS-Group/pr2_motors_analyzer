#ifndef PR2_MOTORS_ANALYZER_H
#define PR2_MOTORS_ANALYZER_H

#include <ros/ros.h>
#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/status_item.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <map>

namespace pr2_motors_analyzer {

class PR2MotorsAnalyzer : public diagnostic_aggregator::Analyzer
{
public:
  PR2MotorsAnalyzer();

  ~PR2MotorsAnalyzer();

  bool init(const std::string base_name, const ros::NodeHandle &n);

  bool match(const std::string name);

  bool analyze(const boost::shared_ptr<diagnostic_aggregator::StatusItem> item);

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

  std::string getPath() const { return path_; }

  std::string getName() const { return nice_name_; }

private:

  // Store status item for EtherCAT master
  boost::shared_ptr<diagnostic_aggregator::StatusItem> eth_master_item_;

  std::map< std::string, boost::shared_ptr<diagnostic_aggregator::StatusItem> > eth_dev_items_;

  std::string path_, nice_name_, power_board_name_;

  bool runstop_hit_;
};

}
#endif //PR2_MOTORS_ANALYZER_H
