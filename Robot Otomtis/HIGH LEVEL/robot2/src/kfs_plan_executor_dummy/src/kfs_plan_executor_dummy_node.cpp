#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <eira_msgs/msg/forest_plan_step.hpp>
#include <eira_msgs/msg/gui_data.hpp>
#include <eira_msgs/msg/kfs_retrieval_plan.hpp>

class KfsPlanExecutorDummyNode : public rclcpp::Node {
 public:
  KfsPlanExecutorDummyNode()
      : rclcpp::Node("kfs_plan_executor_dummy") {
    plan_sub_ = create_subscription<eira_msgs::msg::KfsRetrievalPlan>(
        "/intelligent/kfs_retrieval_plan",
        rclcpp::QoS(10),
        [this](const eira_msgs::msg::KfsRetrievalPlan::SharedPtr msg) {
          onPlan(msg);
        });

    gui_sub_ = create_subscription<eira_msgs::msg::GuiData>(
        "/gui_data",
        rclcpp::QoS(10),
        [this](const eira_msgs::msg::GuiData::SharedPtr msg) {
          onGuiData(msg);
        });

    RCLCPP_INFO(
        get_logger(),
        "[KFS Dummy] Ready | waiting for /intelligent/kfs_retrieval_plan and /gui_data.start");
  }

 private:
  rclcpp::Subscription<eira_msgs::msg::KfsRetrievalPlan>::SharedPtr plan_sub_;
  rclcpp::Subscription<eira_msgs::msg::GuiData>::SharedPtr gui_sub_;

  eira_msgs::msg::KfsRetrievalPlan::SharedPtr current_plan_;
  std::vector<int> picked_kfs_;

  bool has_plan_ = false;
  bool start_latched_ = false;
  bool warned_no_plan_ = false;
  int execution_count_ = 0;

  static std::string actionNameFromId(const int32_t action) {
    switch (action) {
      case 1:
        return "APPROACH";
      case 2:
        return "ENTER";
      case 3:
        return "MOVE";
      case 4:
        return "ROTATE";
      case 5:
        return "PICK_UP";
      case 6:
        return "VALIDATE";
      case 7:
        return "EXIT";
      default:
        return "UNKNOWN";
    }
  }

  static std::string normalizedActionName(
      const eira_msgs::msg::ForestPlanStep& step) {
    if (!step.action_name.empty()) {
      return step.action_name;
    }

    return actionNameFromId(step.action);
  }

  void onPlan(const eira_msgs::msg::KfsRetrievalPlan::SharedPtr msg) {
    current_plan_ = msg;
    has_plan_ = true;
    warned_no_plan_ = false;

    RCLCPP_INFO(
        get_logger(),
        "[KFS Dummy] Plan received | steps=%zu | total_cost=%.3f",
        msg->steps.size(),
        msg->total_cost);
  }

  void onGuiData(const eira_msgs::msg::GuiData::SharedPtr msg) {
    if (!msg->start) {
      start_latched_ = false;
      warned_no_plan_ = false;
      return;
    }

    if (start_latched_) {
      return;
    }

    if (!has_plan_ || !current_plan_) {
      if (!warned_no_plan_) {
        RCLCPP_WARN(
            get_logger(),
            "[KFS Dummy] start=true received, but no plan has been received yet");
        warned_no_plan_ = true;
      }
      return;
    }

    start_latched_ = true;
    warned_no_plan_ = false;
    executeCurrentPlan();
  }

  void executeCurrentPlan() {
    ++execution_count_;
    picked_kfs_.clear();

    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "[KFS Dummy] ===== EXECUTION START #%d =====", execution_count_);
    RCLCPP_INFO(get_logger(), "[KFS Dummy] Total steps: %zu", current_plan_->steps.size());
    RCLCPP_INFO(get_logger(), "[KFS Dummy] Step count : %d", current_plan_->step_count);
    RCLCPP_INFO(get_logger(), "[KFS Dummy] Total cost : %.3f", current_plan_->total_cost);

    for (std::size_t i = 0; i < current_plan_->steps.size(); ++i) {
      executeStep(current_plan_->steps[i], static_cast<int>(i + 1));
    }

    RCLCPP_INFO(get_logger(), "[KFS Dummy] ===== EXECUTION DONE #%d =====", execution_count_);
    RCLCPP_INFO(get_logger(), "%s", pickedSummary().c_str());
    RCLCPP_INFO(get_logger(), "");
  }

  std::string pickedSummary() const {
    std::ostringstream oss;
    oss << "[KFS Dummy] Picked KFS: [";

    for (std::size_t i = 0; i < picked_kfs_.size(); ++i) {
      if (i > 0) {
        oss << ", ";
      }
      oss << picked_kfs_[i];
    }

    oss << "]";
    return oss.str();
  }

  void executeStep(
      const eira_msgs::msg::ForestPlanStep& step,
      const int display_index) {
    const std::string action_name = normalizedActionName(step);

    RCLCPP_INFO(
        get_logger(),
        "[Step %d] %s | msg_step=%d | node=%d | target=%d | cost=%.3f",
        display_index,
        action_name.c_str(),
        step.step,
        step.node,
        step.target,
        step.cost);

    if (action_name == "APPROACH" || step.action == 1) {
      executeApproach(step);
      return;
    }

    if (action_name == "ENTER" || step.action == 2) {
      executeEnter(step);
      return;
    }

    if (action_name == "MOVE" || step.action == 3) {
      executeMove(step);
      return;
    }

    if (action_name == "ROTATE" || step.action == 4) {
      executeRotate(step);
      return;
    }

    if (action_name == "PICK_UP" || step.action == 5) {
      executePickup(step);
      return;
    }

    if (action_name == "EXIT" || step.action == 7) {
      executeExit(step);
      return;
    }

    if (action_name == "VALIDATE" || step.action == 6) {
      RCLCPP_WARN(
          get_logger(),
          "  -> VALIDATE step is legacy and is skipped. Validate should be included in MOVE/ENTER cost.");
      return;
    }

    RCLCPP_WARN(
        get_logger(),
        "  -> Unknown action. action_id=%d, action_name='%s'. Step skipped.",
        step.action,
        action_name.c_str());
  }

  void executeApproach(const eira_msgs::msg::ForestPlanStep& step) {
    RCLCPP_INFO(
        get_logger(),
        "  -> APPROACH: direction=%s, delta=%d, target=%d, heading=%d deg",
        step.direction.c_str(),
        step.delta,
        step.target,
        step.heading);
  }

  void executeEnter(const eira_msgs::msg::ForestPlanStep& step) {
    RCLCPP_INFO(
        get_logger(),
        "  -> ENTER: target=%d, elev=%s, delta=%d mm, heading=%d deg",
        step.target,
        step.elev.c_str(),
        step.delta,
        step.heading);
  }

  void executeMove(const eira_msgs::msg::ForestPlanStep& step) {
    RCLCPP_INFO(
        get_logger(),
        "  -> MOVE: %d -> %d, elev=%s, delta=%d mm, heading=%d deg",
        step.node,
        step.target,
        step.elev.c_str(),
        step.delta,
        step.heading);
  }

  void executeRotate(const eira_msgs::msg::ForestPlanStep& step) {
    RCLCPP_INFO(
        get_logger(),
        "  -> ROTATE: heading=%d deg",
        step.heading);
  }

  void executePickup(const eira_msgs::msg::ForestPlanStep& step) {
    picked_kfs_.push_back(step.target);

    RCLCPP_INFO(
        get_logger(),
        "  -> PICK_UP: KFS %d from node %d, direction=%s, elev=%s, delta=%d mm, heading=%d deg",
        step.target,
        step.node,
        step.direction.c_str(),
        step.elev.c_str(),
        step.delta,
        step.heading);

    RCLCPP_INFO(get_logger(), "     [SUCCESS] KFS %d picked", step.target);
  }

  void executeExit(const eira_msgs::msg::ForestPlanStep& step) {
    RCLCPP_INFO(
        get_logger(),
        "  -> EXIT: %d -> %d, elev=%s, delta=%d mm, heading=%d deg",
        step.node,
        step.target,
        step.elev.c_str(),
        step.delta,
        step.heading);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KfsPlanExecutorDummyNode>());
  rclcpp::shutdown();
  return 0;
}
