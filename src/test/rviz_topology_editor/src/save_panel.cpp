#include "rviz_topology_editor/save_panel.hpp"

#include <future>
#include <memory>
#include <string>

#include <QMessageBox>
#include <QMetaObject>
#include <QPointer>
#include <QInputDialog>
#include <QLineEdit>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace rviz_topology_editor {

SavePanel::SavePanel(QWidget *parent) : rviz_common::Panel(parent) {
  auto *layout = new QVBoxLayout(this);

  auto *node_label = new QLabel(tr("节点属性设置"), this);
  node_selector_ = new QComboBox(this);
  node_selector_->setPlaceholderText(tr("暂无可编辑节点"));
  stop_duration_sec_ = new QDoubleSpinBox(this);
  stop_duration_sec_->setRange(0.0, 86400.0);
  stop_duration_sec_->setDecimals(2);
  stop_duration_sec_->setSingleStep(1.0);
  stop_duration_sec_->setSuffix(tr(" 秒"));
  stop_duration_sec_->setValue(0.0);
  apply_node_policy_button_ = new QPushButton(tr("应用到当前节点"), this);
  apply_node_policy_button_->setEnabled(false);

  auto *edge_label = new QLabel(tr("边属性设置"), this);
  edge_selector_ = new QComboBox(this);
  edge_selector_->setPlaceholderText(tr("暂无可编辑边"));
  edge_type_ = new QComboBox(this);
  edge_type_->addItem(tr("直线"), QStringLiteral("straight"));
  edge_type_->addItem(tr("地图连接"), QStringLiteral("map_connection"));
  linear_velocity_ = new QDoubleSpinBox(this);
  linear_velocity_->setRange(0.0, 100.0);
  linear_velocity_->setDecimals(3);
  linear_velocity_->setSingleStep(0.05);
  linear_velocity_->setSuffix(tr(" m/s"));
  linear_velocity_->setValue(0.5);
  angular_velocity_ = new QDoubleSpinBox(this);
  angular_velocity_->setRange(0.0, 100.0);
  angular_velocity_->setDecimals(3);
  angular_velocity_->setSingleStep(0.05);
  angular_velocity_->setSuffix(tr(" rad/s"));
  angular_velocity_->setValue(0.5);
  motion_direction_ = new QComboBox(this);
  motion_direction_->addItem(tr("前进（1）"), 1);
  motion_direction_->addItem(tr("倒退（-1）"), -1);
  obstacle_stop_enabled_ = new QCheckBox(tr("启用停障"), this);
  obstacle_stop_enabled_->setChecked(true);
  obstacle_avoidance_enabled_ = new QCheckBox(tr("启用绕障"), this);
  obstacle_avoidance_enabled_->setChecked(false);
  apply_edge_policy_button_ = new QPushButton(tr("应用到当前边"), this);
  apply_edge_policy_button_->setEnabled(false);
  load_button_ = new QPushButton(tr("加载 task_id 中的点"), this);
  save_button_ = new QPushButton(tr("保存并生成"), this);
  undo_button_ = new QPushButton(tr("撤销最后一点"), this);
  clear_button_ = new QPushButton(tr("清空所有点"), this);
  status_label_ =
      new QLabel(tr("启动时为空；可打新点或按 task_id 加载"), this);
  status_label_->setWordWrap(true);

  layout->addWidget(node_label);
  layout->addWidget(node_selector_);
  layout->addWidget(new QLabel(tr("到点停留时间"), this));
  layout->addWidget(stop_duration_sec_);
  layout->addWidget(apply_node_policy_button_);
  layout->addSpacing(12);
  layout->addWidget(edge_label);
  layout->addWidget(edge_selector_);
  layout->addWidget(new QLabel(tr("边类型"), this));
  layout->addWidget(edge_type_);
  layout->addWidget(new QLabel(tr("线速度上限"), this));
  layout->addWidget(linear_velocity_);
  layout->addWidget(new QLabel(tr("角速度上限"), this));
  layout->addWidget(angular_velocity_);
  layout->addWidget(new QLabel(tr("运动方向"), this));
  layout->addWidget(motion_direction_);
  layout->addWidget(obstacle_stop_enabled_);
  layout->addWidget(obstacle_avoidance_enabled_);
  layout->addWidget(apply_edge_policy_button_);
  layout->addWidget(load_button_);
  layout->addWidget(save_button_);
  layout->addWidget(undo_button_);
  layout->addWidget(clear_button_);
  layout->addWidget(status_label_);
  layout->addStretch();

  connect(apply_node_policy_button_, &QPushButton::clicked, this,
          &SavePanel::requestApplyNodePolicy);
  connect(node_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &SavePanel::selectedNodeChanged);
  connect(apply_edge_policy_button_, &QPushButton::clicked, this,
          &SavePanel::requestApplyEdgePolicy);
  connect(edge_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &SavePanel::selectedEdgeChanged);
  connect(load_button_, &QPushButton::clicked, this, &SavePanel::requestLoad);
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
  load_client_ = node->create_client<srv::LoadTopology>(
      "/rviz_topology_editor/load");
  save_client_ = node->create_client<srv::SaveTopology>(
      "/rviz_topology_editor/save");
  undo_client_ = node->create_client<std_srvs::srv::Trigger>(
      "/rviz_topology_editor/undo_last");
  clear_client_ = node->create_client<std_srvs::srv::Trigger>(
      "/rviz_topology_editor/clear");
  set_node_policy_client_ = node->create_client<srv::SetTopologyNodePolicy>(
      "/rviz_topology_editor/set_node_policy");
  set_edge_policy_client_ = node->create_client<srv::SetTopologyEdgePolicy>(
      "/rviz_topology_editor/set_edge_policy");
  node_policy_sub_ = node->create_subscription<msg::TopologyNodePolicyArray>(
      "/rviz_topology_editor/node_policies",
      rclcpp::QoS(1).transient_local().reliable(),
      [this](const msg::TopologyNodePolicyArray::SharedPtr message) {
        QPointer<SavePanel> panel(this);
        const auto policies = *message;
        QMetaObject::invokeMethod(
            this,
            [panel, policies]() {
              if (panel)
                panel->updateNodePolicies(policies);
            },
            Qt::QueuedConnection);
      });
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

void SavePanel::updateNodePolicies(
    const msg::TopologyNodePolicyArray &message) {
  const int previous_node_id = node_selector_->currentIndex() >= 0
                                   ? node_selector_->currentData().toInt()
                                   : -1;
  node_selector_->blockSignals(true);
  node_selector_->clear();
  node_policies_.clear();

  int selected_index = -1;
  for (const auto &node : message.nodes) {
    node_policies_[node.node_id] = node;
    node_selector_->addItem(tr("节点%1").arg(node.node_id), node.node_id);
    if (node.node_id == previous_node_id) {
      selected_index = node_selector_->count() - 1;
    }
  }
  if (selected_index < 0 && node_selector_->count() > 0)
    selected_index = 0;
  node_selector_->setCurrentIndex(selected_index);
  node_selector_->blockSignals(false);
  selectedNodeChanged(selected_index);
  apply_node_policy_button_->setEnabled(selected_index >= 0);
}

void SavePanel::updateEdgePolicies(
    const msg::TopologyEdgePolicyArray &message) {
  const int previous_edge_id = edge_selector_->currentIndex() >= 0
                                   ? edge_selector_->currentData().toInt()
                                   : -1;
  edge_selector_->blockSignals(true);
  edge_selector_->clear();
  edge_policies_.clear();

  int selected_index = -1;
  for (const auto &edge : message.edges) {
    edge_policies_[edge.edge_id] = edge;
    edge_selector_->addItem(tr("边%1：节点%2 ↔ 节点%3")
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
    edge_type_->setCurrentIndex(0);
    linear_velocity_->setValue(0.5);
    angular_velocity_->setValue(0.5);
    motion_direction_->setCurrentIndex(0);
    obstacle_stop_enabled_->setChecked(true);
    obstacle_avoidance_enabled_->setChecked(false);
    return;
  }
  const int edge_id = edge_selector_->itemData(index).toInt();
  const auto edge = edge_policies_.find(edge_id);
  if (edge == edge_policies_.end()) {
    return;
  }
  const int edge_type_index =
      edge_type_->findData(QString::fromStdString(edge->second.edge_type));
  edge_type_->setCurrentIndex(edge_type_index >= 0 ? edge_type_index : 0);
  linear_velocity_->setValue(edge->second.linear_velocity);
  angular_velocity_->setValue(edge->second.angular_velocity);
  const int direction_index = motion_direction_->findData(
      static_cast<int>(edge->second.motion_direction));
  motion_direction_->setCurrentIndex(direction_index >= 0 ? direction_index
                                                          : 0);
  obstacle_stop_enabled_->setChecked(edge->second.obstacle_stop_enabled);
  obstacle_avoidance_enabled_->setChecked(
      edge->second.obstacle_avoidance_enabled);
}

void SavePanel::selectedNodeChanged(int index) {
  if (index < 0) {
    stop_duration_sec_->setValue(0.0);
    return;
  }
  const int node_id = node_selector_->itemData(index).toInt();
  const auto node = node_policies_.find(node_id);
  if (node != node_policies_.end()) {
    stop_duration_sec_->setValue(node->second.stop_duration_sec);
  }
}

void SavePanel::requestApplyNodePolicy() {
  if (node_selector_->currentIndex() < 0 || !set_node_policy_client_ ||
      !set_node_policy_client_->service_is_ready()) {
    status_label_->setText(tr("节点属性服务未就绪或没有可编辑节点"));
    return;
  }

  setButtonsEnabled(false);
  status_label_->setText(tr("正在应用节点属性..."));
  auto request = std::make_shared<srv::SetTopologyNodePolicy::Request>();
  request->node_id = node_selector_->currentData().toInt();
  request->stop_duration_sec = stop_duration_sec_->value();

  QPointer<SavePanel> panel(this);
  set_node_policy_client_->async_send_request(
      request,
      [panel](rclcpp::Client<srv::SetTopologyNodePolicy>::SharedFuture future) {
        const auto response = future.get();
        if (!panel)
          return;
        const QString message = QString::fromStdString(response->message);
        const bool success = response->success;
        QMetaObject::invokeMethod(
            panel,
            [panel, message, success]() {
              if (!panel)
                return;
              panel->setButtonsEnabled(true);
              panel->status_label_->setText(
                  success ? QObject::tr("节点属性已应用：%1").arg(message)
                          : QObject::tr("节点属性应用失败：%1").arg(message));
            },
            Qt::QueuedConnection);
      });
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
  request->edge_type = edge_type_->currentData().toString().toStdString();
  request->linear_velocity = linear_velocity_->value();
  request->angular_velocity = angular_velocity_->value();
  request->motion_direction =
      static_cast<int8_t>(motion_direction_->currentData().toInt());
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

void SavePanel::requestLoad() {
  if (!node_policies_.empty()) {
    const auto answer = QMessageBox::question(
        this, tr("确认加载任务"),
        tr("加载 task_id 会替换当前内存中的所有打点，是否继续？"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes) {
      return;
    }
  }
  bool accepted = false;
  const QString task_id = QInputDialog::getText(
      this, tr("加载巡检任务"), tr("请输入 task_id："), QLineEdit::Normal,
      QStringLiteral("task_0"), &accepted);
  if (!accepted) {
    return;
  }
  const QString normalized_task_id = task_id.trimmed();
  if (normalized_task_id.isEmpty()) {
    status_label_->setText(tr("task_id 不能为空"));
    return;
  }
  for (const QChar character : normalized_task_id) {
    if (!character.isLetterOrNumber() && character != QLatin1Char('_') &&
        character != QLatin1Char('-')) {
      status_label_->setText(
          tr("task_id 只能包含字母、数字、下划线和连字符"));
      return;
    }
  }
  if (!load_client_ || !load_client_->service_is_ready()) {
    status_label_->setText(tr("加载服务未就绪，请确认编辑节点已启动"));
    return;
  }

  setButtonsEnabled(false);
  status_label_->setText(
      tr("正在加载任务 %1...").arg(normalized_task_id));
  auto request = std::make_shared<srv::LoadTopology::Request>();
  request->task_id = normalized_task_id.toStdString();
  QPointer<SavePanel> panel(this);
  load_client_->async_send_request(
      request,
      [panel](rclcpp::Client<srv::LoadTopology>::SharedFuture future) {
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
                  success ? QObject::tr("加载成功：%1").arg(message)
                          : QObject::tr("加载失败：%1").arg(message));
            },
            Qt::QueuedConnection);
      });
}

void SavePanel::requestSave() {
  bool accepted = false;
  const QString task_id = QInputDialog::getText(
      this, tr("保存巡检任务"), tr("请输入 task_id："), QLineEdit::Normal,
      QStringLiteral("task_0"), &accepted);
  if (!accepted) {
    return;
  }
  const QString normalized_task_id = task_id.trimmed();
  if (normalized_task_id.isEmpty()) {
    status_label_->setText(tr("task_id 不能为空"));
    return;
  }
  for (const QChar character : normalized_task_id) {
    if (!character.isLetterOrNumber() && character != QLatin1Char('_') &&
        character != QLatin1Char('-')) {
      status_label_->setText(
          tr("task_id 只能包含字母、数字、下划线和连字符"));
      return;
    }
  }
  if (!save_client_ || !save_client_->service_is_ready()) {
    status_label_->setText(tr("保存服务未就绪，请确认编辑节点已启动"));
    return;
  }

  setButtonsEnabled(false);
  status_label_->setText(
      tr("正在保存任务 %1...").arg(normalized_task_id));
  auto request = std::make_shared<srv::SaveTopology::Request>();
  request->task_id = normalized_task_id.toStdString();
  QPointer<SavePanel> panel(this);
  save_client_->async_send_request(
      request,
      [panel](rclcpp::Client<srv::SaveTopology>::SharedFuture future) {
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
                  success ? QObject::tr("保存成功：%1").arg(message)
                          : QObject::tr("保存失败：%1").arg(message));
            },
            Qt::QueuedConnection);
      });
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
  apply_node_policy_button_->setEnabled(enabled &&
                                        node_selector_->currentIndex() >= 0);
  apply_edge_policy_button_->setEnabled(enabled &&
                                        edge_selector_->currentIndex() >= 0);
  load_button_->setEnabled(enabled);
  save_button_->setEnabled(enabled);
  undo_button_->setEnabled(enabled);
  clear_button_->setEnabled(enabled);
}

} // namespace rviz_topology_editor

PLUGINLIB_EXPORT_CLASS(rviz_topology_editor::SavePanel, rviz_common::Panel)
