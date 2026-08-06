#include "rviz_topology_editor/save_panel.hpp"

#include <future>
#include <memory>
#include <string>

#include <QMessageBox>
#include <QMetaObject>
#include <QPointer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace rviz_topology_editor {

SavePanel::SavePanel(QWidget *parent) : rviz_common::Panel(parent) {
  auto *layout = new QVBoxLayout(this);
  auto *edge_label = new QLabel(tr("边属性设置"), this);
  edge_selector_ = new QComboBox(this);
  edge_selector_->setPlaceholderText(tr("暂无可编辑边"));
  obstacle_stop_enabled_ = new QCheckBox(tr("启用停障"), this);
  obstacle_stop_enabled_->setChecked(true);
  obstacle_avoidance_enabled_ = new QCheckBox(tr("启用绕障"), this);
  obstacle_avoidance_enabled_->setChecked(false);
  apply_edge_policy_button_ = new QPushButton(tr("应用到当前边"), this);
  apply_edge_policy_button_->setEnabled(false);
  save_button_ = new QPushButton(tr("保存并生成"), this);
  undo_button_ = new QPushButton(tr("撤销最后一点"), this);
  clear_button_ = new QPushButton(tr("清空所有点"), this);
  status_label_ = new QLabel(tr("打点完成后点击保存"), this);
  status_label_->setWordWrap(true);

  layout->addWidget(edge_label);
  layout->addWidget(edge_selector_);
  layout->addWidget(obstacle_stop_enabled_);
  layout->addWidget(obstacle_avoidance_enabled_);
  layout->addWidget(apply_edge_policy_button_);
  layout->addWidget(save_button_);
  layout->addWidget(undo_button_);
  layout->addWidget(clear_button_);
  layout->addWidget(status_label_);
  layout->addStretch();

  connect(apply_edge_policy_button_, &QPushButton::clicked, this,
          &SavePanel::requestApplyEdgePolicy);
  connect(edge_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &SavePanel::selectedEdgeChanged);
  connect(save_button_, &QPushButton::clicked, this, &SavePanel::requestSave);
  connect(undo_button_, &QPushButton::clicked, this, &SavePanel::requestUndo);
  connect(clear_button_, &QPushButton::clicked, this, &SavePanel::requestClear);
}

void SavePanel::onInitialize() {
  auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    save_button_->setEnabled(false);
    status_label_->setText(tr("无法获取RViz ROS节点"));
    return;
  }

  auto node = node_abstraction->get_raw_node();
  save_client_ =
      node->create_client<std_srvs::srv::Trigger>("/rviz_topology_editor/save");
  undo_client_ = node->create_client<std_srvs::srv::Trigger>(
      "/rviz_topology_editor/undo_last");
  clear_client_ = node->create_client<std_srvs::srv::Trigger>(
      "/rviz_topology_editor/clear");
  set_edge_policy_client_ = node->create_client<srv::SetTopologyEdgePolicy>(
      "/rviz_topology_editor/set_edge_policy");
  edge_policy_sub_ = node->create_subscription<msg::TopologyEdgePolicyArray>(
      "/rviz_topology_editor/edge_policies",
      rclcpp::QoS(1).transient_local().reliable(),
      [this](const msg::TopologyEdgePolicyArray::SharedPtr message) {
        QPointer<SavePanel> panel(this);
        const auto policies = *message;
        QMetaObject::invokeMethod(
            this,
            [panel, policies]() {
              if (panel) {
                panel->updateEdgePolicies(policies);
              }
            },
            Qt::QueuedConnection);
      });
}

void SavePanel::updateEdgePolicies(
    const msg::TopologyEdgePolicyArray &message) {
  const int previous_edge_id =
      edge_selector_->currentIndex() >= 0
          ? edge_selector_->currentData().toInt()
          : -1;
  edge_selector_->blockSignals(true);
  edge_selector_->clear();
  edge_policies_.clear();

  int selected_index = -1;
  for (const auto &edge : message.edges) {
    edge_policies_[edge.edge_id] = edge;
    edge_selector_->addItem(
        tr("边%1：节点%2 ↔ 节点%3")
            .arg(edge.edge_id)
            .arg(edge.from_id)
            .arg(edge.to_id),
        edge.edge_id);
    if (edge.edge_id == previous_edge_id) {
      selected_index = edge_selector_->count() - 1;
    }
  }
  if (selected_index < 0 && edge_selector_->count() > 0) {
    selected_index = 0;
  }
  edge_selector_->setCurrentIndex(selected_index);
  edge_selector_->blockSignals(false);
  selectedEdgeChanged(selected_index);
  apply_edge_policy_button_->setEnabled(selected_index >= 0);
}

void SavePanel::selectedEdgeChanged(int index) {
  if (index < 0) {
    obstacle_stop_enabled_->setChecked(true);
    obstacle_avoidance_enabled_->setChecked(false);
    return;
  }
  const int edge_id = edge_selector_->itemData(index).toInt();
  const auto edge = edge_policies_.find(edge_id);
  if (edge == edge_policies_.end()) {
    return;
  }
  obstacle_stop_enabled_->setChecked(edge->second.obstacle_stop_enabled);
  obstacle_avoidance_enabled_->setChecked(
      edge->second.obstacle_avoidance_enabled);
}

void SavePanel::requestApplyEdgePolicy() {
  if (edge_selector_->currentIndex() < 0 || !set_edge_policy_client_ ||
      !set_edge_policy_client_->service_is_ready()) {
    status_label_->setText(tr("边属性服务未就绪或没有可编辑边"));
    return;
  }

  setButtonsEnabled(false);
  status_label_->setText(tr("正在应用边属性..."));
  auto request = std::make_shared<srv::SetTopologyEdgePolicy::Request>();
  request->edge_id = edge_selector_->currentData().toInt();
  request->obstacle_stop_enabled = obstacle_stop_enabled_->isChecked();
  request->obstacle_avoidance_enabled =
      obstacle_avoidance_enabled_->isChecked();

  QPointer<SavePanel> panel(this);
  set_edge_policy_client_->async_send_request(
      request,
      [panel](rclcpp::Client<srv::SetTopologyEdgePolicy>::SharedFuture future) {
        const auto response = future.get();
        if (!panel) {
          return;
        }
        const QString message = QString::fromStdString(response->message);
        const bool success = response->success;
        QMetaObject::invokeMethod(
            panel,
            [panel, message, success]() {
              if (!panel) {
                return;
              }
              panel->setButtonsEnabled(true);
              panel->status_label_->setText(
                  success ? QObject::tr("边属性已应用：%1").arg(message)
                          : QObject::tr("边属性应用失败：%1").arg(message));
            },
            Qt::QueuedConnection);
      });
}

void SavePanel::requestSave() {
  sendRequest(save_client_, tr("正在保存、连边并生成任务文件..."));
}

void SavePanel::requestUndo() {
  sendRequest(undo_client_, tr("正在撤销最后一点..."));
}

void SavePanel::requestClear() {
  const auto answer = QMessageBox::question(
      this, tr("确认清空"), tr("确定清空当前会话中的所有打点吗？"),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (answer != QMessageBox::Yes) {
    return;
  }
  sendRequest(clear_client_, tr("正在清空所有点..."));
}

void SavePanel::sendRequest(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
    const QString &pending_text) {
  if (!client || !client->service_is_ready()) {
    status_label_->setText(tr("操作服务未就绪，请确认编辑节点已启动"));
    return;
  }

  setButtonsEnabled(false);
  status_label_->setText(pending_text);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  QPointer<SavePanel> panel(this);
  client->async_send_request(
      request,
      [panel](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        if (!panel) {
          return;
        }

        const QString message = QString::fromStdString(response->message);
        const bool success = response->success;
        QMetaObject::invokeMethod(
            panel,
            [panel, message, success]() {
              if (!panel) {
                return;
              }
              panel->setButtonsEnabled(true);
              panel->status_label_->setText(
                  success ? QObject::tr("操作成功：%1").arg(message)
                          : QObject::tr("操作失败：%1").arg(message));
            },
            Qt::QueuedConnection);
      });
}

void SavePanel::setButtonsEnabled(bool enabled) {
  apply_edge_policy_button_->setEnabled(enabled &&
                                        edge_selector_->currentIndex() >= 0);
  save_button_->setEnabled(enabled);
  undo_button_->setEnabled(enabled);
  clear_button_->setEnabled(enabled);
}

} // namespace rviz_topology_editor

PLUGINLIB_EXPORT_CLASS(rviz_topology_editor::SavePanel, rviz_common::Panel)
