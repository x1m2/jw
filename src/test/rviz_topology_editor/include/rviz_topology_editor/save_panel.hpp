#pragma once

#include <memory>
#include <unordered_map>

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>

#include <rclcpp/client.hpp>
#include <rclcpp/subscription.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_topology_editor/msg/topology_edge_policy_array.hpp>
#include <rviz_topology_editor/srv/set_topology_edge_policy.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rviz_topology_editor {

class SavePanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit SavePanel(QWidget *parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void requestSave();
  void requestUndo();
  void requestClear();
  void requestApplyEdgePolicy();
  void selectedEdgeChanged(int index);

private:
  void
  sendRequest(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
              const QString &pending_text);
  void setButtonsEnabled(bool enabled);
  void updateEdgePolicies(
      const msg::TopologyEdgePolicyArray &message);

  QComboBox *edge_selector_{nullptr};
  QCheckBox *obstacle_stop_enabled_{nullptr};
  QCheckBox *obstacle_avoidance_enabled_{nullptr};
  QPushButton *apply_edge_policy_button_{nullptr};
  QPushButton *save_button_{nullptr};
  QPushButton *undo_button_{nullptr};
  QPushButton *clear_button_{nullptr};
  QLabel *status_label_{nullptr};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_client_;
  rclcpp::Client<srv::SetTopologyEdgePolicy>::SharedPtr
      set_edge_policy_client_;
  rclcpp::Subscription<msg::TopologyEdgePolicyArray>::SharedPtr
      edge_policy_sub_;
  std::unordered_map<int, msg::TopologyEdgePolicy> edge_policies_;
};

} // namespace rviz_topology_editor
