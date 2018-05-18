#include <pr2_motors_analyzer/pr2_motors_analyzer.h>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

using namespace pr2_motors_analyzer;

using diagnostic_aggregator::StatusItem;

PLUGINLIB_REGISTER_CLASS(PR2MotorsAnalyzer,
                         pr2_motors_analyzer::PR2MotorsAnalyzer,
                         diagnostic_aggregator::Analyzer)

PR2MotorsAnalyzer::PR2MotorsAnalyzer() :
  path_(""), nice_name_("Motors"), power_board_name_(""),
  runstop_hit_(false)
{ }

PR2MotorsAnalyzer::~PR2MotorsAnalyzer() { }

bool PR2MotorsAnalyzer::init(const std::string base_name, const ros::NodeHandle &n)
{
  // path_ = BASE_NAME/Motors
  path_ = base_name + nice_name_;

  if (!n.getParam("power_board_name", power_board_name_))
  {
     ROS_ERROR("No power board name was specified in PR2MotorsAnalyzer! Power board must be \"Power board 10XX\". Namespace: %s", n.getNamespace().c_str());
     return false;
  }

  // Make a "missing" item for the EtherCAT Master
  boost::shared_ptr<StatusItem> item(new StatusItem("EtherCAT Master"));
  eth_master_item_ = item;

  return true;
}

bool PR2MotorsAnalyzer::match(const std::string name)
{
  return name == "EtherCAT Master"
      || name.find("EtherCAT Device") == 0
      || name == power_board_name_;
}

bool PR2MotorsAnalyzer::analyze(const boost::shared_ptr<StatusItem> item)
{
  if (item->getName() == power_board_name_)
  {
    runstop_hit_ = item->getValue("RunStop Button Status") != "True" || item->getValue("RunStop Wireless Status") != "True";
    return false; // Won't report this item
  }

  if (item->getName() == "EtherCAT Master")
  {
    eth_master_item_ = item;
    return true;
  }

  eth_dev_items_[item->getName()] = item;
  return true;
}

namespace {
void update_top_level_stat(diagnostic_msgs::DiagnosticStatus& tls, diagnostic_msgs::DiagnosticStatus child)
{
  tls.level = std::max(tls.level, child.level);

  diagnostic_msgs::KeyValue kv;
  kv.key = child.name;
  kv.value = diagnostic_aggregator::valToMsg(child.level);
  tls.values.push_back(kv);
}
}

std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > PR2MotorsAnalyzer::report()
{
  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > stats;

  // There has to be a top level status for path_ to see the stats in the tree of the robot_monitor
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> top_level_status(new diagnostic_msgs::DiagnosticStatus);
  top_level_status->name = path_;
  top_level_status->level = diagnostic_msgs::DiagnosticStatus::OK;

  // report EtherCAT Master
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> s = eth_master_item_->toStatusMsg(path_);
  if(runstop_hit_ && (s->message == "Motors halted (device error)" || s->message == "Motors halted soon after reset (device error)"))
    s->level = diagnostic_msgs::DiagnosticStatus::OK;
  update_top_level_stat(*top_level_status, *s);

  stats.push_back(s);

  // report EtherCAT devices
  for(std::map<std::string, boost::shared_ptr<diagnostic_aggregator::StatusItem> >::const_iterator it = eth_dev_items_.begin();
      it != eth_dev_items_.end();
      ++it)
  {
    s = it->second->toStatusMsg(path_);
    if( runstop_hit_ && s->message == "Safety Lockout: UNDERVOLTAGE")
      s->level = diagnostic_msgs::DiagnosticStatus::OK;
    update_top_level_stat(*top_level_status, *s);
    stats.push_back(s);
  }

  // report runstop pressed if there is nothing else to report
  if(runstop_hit_ && top_level_status->level == diagnostic_msgs::DiagnosticStatus::OK)
  {
    top_level_status->level = diagnostic_msgs::DiagnosticStatus::WARN;
    top_level_status->message = "Emergency stop is pressed";
  }
  else
    top_level_status->message = diagnostic_aggregator::valToMsg(top_level_status->level);

  stats.push_back(top_level_status);

  return stats;
}
