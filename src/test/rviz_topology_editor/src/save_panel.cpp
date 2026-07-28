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
  save_button_ = new QPushButton(tr("保存并生成"), this);
  undo_button_ = new QPushButton(tr("撤销最后一点"), this);
  clear_button_ = new QPushButton(tr("清空所有点"), this);
  status_label_ = new QLabel(tr("打点完成后点击保存"), this);
  status_label_->setWordWrap(true);

  layout->addWidget(save_button_);
  layout->addWidget(undo_button_);
  layout->addWidget(clear_button_);
  layout->addWidget(status_label_);
  layout->addStretch();

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
  save_button_->setEnabled(enabled);
  undo_button_->setEnabled(enabled);
  clear_button_->setEnabled(enabled);
}

} // namespace rviz_topology_editor

PLUGINLIB_EXPORT_CLASS(rviz_topology_editor::SavePanel, rviz_common::Panel)
