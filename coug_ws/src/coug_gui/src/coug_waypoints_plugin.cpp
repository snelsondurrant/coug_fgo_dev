// Modified from the plan_route plugin - Nelson Durrant, Nov 2025
// https://github.com/swri-robotics/mapviz/blob/ros2-devel/coug_gui/src/plan_route_plugin.cpp

#include <coug_gui/coug_waypoints_plugin.h>

// QT libraries
#include <QDateTime>
#include <QDialog>
#include <QGLWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPalette>
#include <QStaticText>

// ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <swri_transform_util/frames.h>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ standard libraries
#include <chrono>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(coug_gui::CougWaypointsPlugin, mapviz::MapvizPlugin)

using namespace std::chrono_literals;

namespace stu = swri_transform_util;

namespace coug_gui
{
  CougWaypointsPlugin::CougWaypointsPlugin()
  : MapvizPlugin()
  , ui_()
  , config_widget_(new QWidget())
  , map_canvas_(nullptr)
  , selected_point_(-1)
  , dragged_point_(-1)
  , is_mouse_down_(false)
  , mouse_down_time_(0)
  , max_ms_(Q_INT64_C(500))
  , max_distance_(5.0)
  {
    ui_.setupUi(config_widget_);

    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Window, Qt::white);
    config_widget_->setPalette(p);
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::darkGreen);
    ui_.status->setPalette(p3);
    QObject::connect(ui_.publish, SIGNAL(clicked()), this,
                     SLOT(PublishWaypoints()));
    QObject::connect(ui_.stop, SIGNAL(clicked()), this,
                     SLOT(Stop()));
    QObject::connect(ui_.clear, SIGNAL(clicked()), this,
                     SLOT(Clear()));
    QObject::connect(this,
                     SIGNAL(VisibleChanged(bool)),
                     this,
                     SLOT(VisibilityChanged(bool)));
    QObject::connect(ui_.depth_editor, SIGNAL(valueChanged(double)), this,
                     SLOT(DepthChanged(double)));
  }

  CougWaypointsPlugin::~CougWaypointsPlugin()
  {
    if (map_canvas_)
    {
      map_canvas_->removeEventFilter(this);
    }
  }

  void CougWaypointsPlugin::VisibilityChanged(bool visible)
  {
    if (visible)
    {
      map_canvas_->installEventFilter(this);
    }
    else
    {
      map_canvas_->removeEventFilter(this);
    }
  }

  void CougWaypointsPlugin::PublishWaypoints()
  {

    if (ui_.topic->text().isEmpty())
    {
      PrintError("No topic");
      return;
    }

    if (waypoints_topic_ != ui_.topic->text().toStdString())
    {
      waypoints_topic_ = ui_.topic->text().toStdString();
      waypoints_pub_.reset();
      waypoints_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
        waypoints_topic_,
        rclcpp::QoS(1));
    }

    auto pose_array = std::make_unique<geometry_msgs::msg::PoseArray>();
    pose_array->header.frame_id = target_frame_;
    pose_array->header.stamp = node_->now();

    // Transform waypoints into the 'map' frame before publishing
    pub_waypoints_ = waypoints_;
    stu::Transform transform;
    if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform))
    {
      for (size_t i = 0; i < waypoints_.size(); i++)
      {
        tf2::Vector3 position(
            waypoints_[i].position.x,
            waypoints_[i].position.y,
            0.0);
        position = transform * position;
        pub_waypoints_[i].position.x = position.x();
        pub_waypoints_[i].position.y = position.y();
      }
    } else {
      PrintError("No transform");
      return;
    }

    pose_array->poses = pub_waypoints_;

    waypoints_pub_->publish(*pose_array);
    if (waypoints_.empty())
    {
      PrintWarning("Stop requested");
    } else {
      PrintInfo("Published " + std::to_string(waypoints_.size()) + " waypoint(s)");
    }
  }

  void CougWaypointsPlugin::Stop()
  {

    if (ui_.topic->text().isEmpty())
    {
      PrintError("No topic");
      return;
    }

    if (waypoints_topic_ != ui_.topic->text().toStdString())
    {
      waypoints_topic_ = ui_.topic->text().toStdString();
      waypoints_pub_.reset();
      waypoints_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
        waypoints_topic_,
        rclcpp::QoS(1));
    }

    auto pose_array = std::make_unique<geometry_msgs::msg::PoseArray>();
    pose_array->header.frame_id = target_frame_;
    pose_array->header.stamp = node_->now();

    waypoints_pub_->publish(*pose_array);
    PrintWarning("Stop requested");
  }

  void CougWaypointsPlugin::Clear()
  {
    waypoints_.clear();
    ui_.depth_editor->setValue(0.0);
    selected_point_ = -1;
    dragged_point_ = -1;
    ui_.depth_editor->setEnabled(false);
    PrintInfo("Click to add waypoints");
  }

  void CougWaypointsPlugin::DepthChanged(double value)
  {
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < waypoints_.size())
    {
      waypoints_[selected_point_].position.z = value;
    }
  }

  void CougWaypointsPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message, 1.0);
  }

  void CougWaypointsPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message, 1.0);
  }

  void CougWaypointsPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message, 1.0);
  }

  QWidget* CougWaypointsPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool CougWaypointsPlugin::Initialize(QGLWidget* canvas)
  {
    map_canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
    map_canvas_->installEventFilter(this);

    initialized_ = true;
    return true;
  }

  bool CougWaypointsPlugin::eventFilter(QObject *object, QEvent* event)
  {
    switch (event->type())
    {
      case QEvent::MouseButtonPress:
        return handleMousePress(dynamic_cast<QMouseEvent*>(event));
      case QEvent::MouseButtonRelease:
        return handleMouseRelease(dynamic_cast<QMouseEvent*>(event));
      case QEvent::MouseMove:
        return handleMouseMove(dynamic_cast<QMouseEvent*>(event));
      default:
        return false;
    }
  }

  bool CougWaypointsPlugin::handleMousePress(QMouseEvent* event)
  {
    dragged_point_ = -1;
    int closest_point = -1;
    double closest_distance = std::numeric_limits<double>::max();

    QPointF point = event->localPos();
    stu::Transform transform;
    if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform))
    {
      for (size_t i = 0; i < waypoints_.size(); i++)
      {
        tf2::Vector3 waypoint(waypoints_[i].position.x, waypoints_[i].position.y, 0);
        waypoint = transform * waypoint;
        QPointF transformed = map_canvas_->FixedFrameToMapGlCoord(QPointF(waypoint.x(), waypoint.y()));
        double distance = QLineF(transformed, point).length();
        if (distance < closest_distance)
        {
          closest_distance = distance;
          closest_point = static_cast<int>(i);
        }
      }
    }

    if (event->button() == Qt::LeftButton)
    {
      is_mouse_down_ = true;
      mouse_down_pos_ = event->localPos();
      mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();
      if (closest_distance < 15)
      {
        dragged_point_ = closest_point;
        return true;
      }
    } else if (event->button() == Qt::RightButton) {
      if (closest_distance < 15)
      {
        waypoints_.erase(waypoints_.begin() + closest_point);
        if (selected_point_ == closest_point)
        {
          selected_point_ = -1;
          ui_.depth_editor->setEnabled(false);
        }
        return true;
      }
    }
    return false;
  }

  bool CougWaypointsPlugin::handleMouseRelease(QMouseEvent* event)
  {
    qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
    qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

    if (dragged_point_ != -1)
    {
      if (distance <= max_distance_ && msecsDiff < max_ms_)
      {
        // This was a click, not a drag. Select the point.
        selected_point_ = dragged_point_;
        ui_.depth_editor->setEnabled(true);
        ui_.depth_editor->blockSignals(true);
        ui_.depth_editor->setValue(waypoints_[selected_point_].position.z);
        ui_.depth_editor->blockSignals(false);
      }
      // If it was a drag, the point is already moved. We just stop dragging.
      dragged_point_ = -1;
      return true;
    }

    if (is_mouse_down_)
    {
      if (msecsDiff < max_ms_ && distance <= max_distance_)
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
        stu::Transform transform;
        if (tf_manager_->GetTransform(stu::_wgs84_frame, target_frame_, transform))
        {
          tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
          position = transform * position;
          geometry_msgs::msg::Pose pose;
          pose.position.x = position.x();
          pose.position.y = position.y();
          pose.position.z = ui_.depth_editor->value();
          waypoints_.push_back(pose);
        }
      }
    }
    is_mouse_down_ = false;
    dragged_point_ = -1;
    return false;
  }

  bool CougWaypointsPlugin::handleMouseMove(QMouseEvent* event)
  {
    if (dragged_point_ >= 0)
    {
      if (selected_point_ != -1) {
          selected_point_ = -1;
          ui_.depth_editor->setEnabled(false);
      }

      QPointF point = event->localPos();
      stu::Transform transform;
      if (tf_manager_->GetTransform(stu::_wgs84_frame, target_frame_, transform))
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        waypoints_[dragged_point_].position.x = position.x();
        waypoints_[dragged_point_].position.y = position.y();
      }
      return true;
    }
    return false;
  }

  void CougWaypointsPlugin::Draw(double x, double y, double scale)
  {
    stu::Transform transform;
    if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform))
    {

      glLineWidth(2);
      glColor4f(0.0, 0.0, 1.0, 1.0);
      glBegin(GL_LINE_STRIP);

      for (size_t i = 0; i < waypoints_.size(); ++i)
      {
        const auto& waypoint = waypoints_[i];
        tf2::Vector3 point(waypoint.position.x, waypoint.position.y, 0);
        point = transform * point;
        glVertex2d(point.x(), point.y());
      }
      glEnd();

      glPointSize(20);
      glBegin(GL_POINTS);

      for (size_t i = 0; i < waypoints_.size(); ++i)
      {
        if (static_cast<int>(i) == selected_point_) {
          glColor4f(1.0, 1.0, 0.0, 1.0);
        } else {
          glColor4f(0.0, 1.0, 1.0, 1.0);
        }
        const auto& waypoint = waypoints_[i];
        tf2::Vector3 point(waypoint.position.x, waypoint.position.y, 0);
        point = transform * point;
        glVertex2d(point.x(), point.y());
      }
      glEnd();
    }
  }

  void CougWaypointsPlugin::Paint(QPainter* painter, double x, double y, double scale)
  {
    painter->save();
    painter->resetTransform();
    painter->setFont(QFont("DejaVu Sans Mono", 10, QFont::Bold));

    stu::Transform transform;
    if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform))
    {
      for (size_t i = 0; i < waypoints_.size(); i++)
      {
        tf2::Vector3 point(waypoints_[i].position.x, waypoints_[i].position.y, 0);
        point = transform * point;
        QPointF gl_point = map_canvas_->FixedFrameToMapGlCoord(QPointF(point.x(), point.y()));

        // --- Draw depth text (white, below) ---
        QPen white_pen(Qt::white);
        painter->setPen(white_pen);
        QPointF depth_text_corner(gl_point.x() - 50, gl_point.y() + 15);
        QRectF depth_text_rect(depth_text_corner, QSizeF(100, 20));
        QString depth_text = QString::number(waypoints_[i].position.z, 'f', 1) + "m";
        painter->drawText(depth_text_rect, Qt::AlignHCenter | Qt::AlignTop, depth_text);

        // --- Draw waypoint number (black, inside) ---
        QPen black_pen(Qt::black);
        painter->setPen(black_pen);
        QPointF num_corner(gl_point.x() - 20, gl_point.y() - 20);
        QRectF num_rect(num_corner, QSizeF(40, 40));
        painter->drawText(num_rect, Qt::AlignHCenter | Qt::AlignVCenter, QString::number(i + 1));
      }
    }
    painter->restore();
  }

  void CougWaypointsPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["waypoints_topic"])
    {
      std::string waypoints_topic = node["waypoints_topic"].as<std::string>();
      ui_.topic->setText(waypoints_topic.c_str());
    }
  }

  void CougWaypointsPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string waypoints_topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "waypoints_topic" << YAML::Value << waypoints_topic;
  }
}   // namespace coug_gui