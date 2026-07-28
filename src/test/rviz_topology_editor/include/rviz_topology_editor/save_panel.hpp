#pragma once

#include <memory>

#include <QLabel>
#include <QPushButton>

#include <rclcpp/client.hpp>
#include <rviz_common/panel.hpp>
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

private:
  void
  sendRequest(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
              const QString &pending_text);
  void setButtonsEnabled(bool enabled);

  QPushButton *save_button_{nullptr};
  QPushButton *undo_button_{nullptr};
  QPushButton *clear_button_{nullptr};
  QLabel *status_label_{nullptr};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_client_;
};

} // namespace rviz_topology_editor
