#pragma once
#include <Eigen/Core>
#include <functional>

template <typename T>
class DiscreteTransformation {
public:
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Matrix4 = Eigen::Matrix<T, 4, 4>;
  DiscreteTransformation() : score(0), level(0), x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}

  DiscreteTransformation(int score) : score(score), level(0), x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}

  DiscreteTransformation(int score, int level, int x, int y, int z, int roll, int pitch, int yaw)
  : score(score),
    level(level),
    x(x),
    y(y),
    z(z),
    roll(roll),
    pitch(pitch),
    yaw(yaw) {}

  ~DiscreteTransformation() {}

  bool operator<(const DiscreteTransformation& rhs) const { return score < rhs.score; }

  bool is_leaf() const { return level == 0; }

  Matrix4 create_matrix(const T trans_res, const Vector3& rpy_res, const Vector3& min_rpy) {
    Eigen::Translation<T, 3> translation(x * trans_res, y * trans_res, z * trans_res);
    Eigen::AngleAxis<T> rollAngle(roll * rpy_res.x() + min_rpy.x(), Vector3::UnitX());
    Eigen::AngleAxis<T> pitchAngle(pitch * rpy_res.y() + min_rpy.y(), Vector3::UnitY());
    Eigen::AngleAxis<T> yawAngle(yaw * rpy_res.z() + min_rpy.z(), Vector3::UnitZ());
    return (translation * yawAngle * pitchAngle * rollAngle).matrix();
  }

  void branch(std::vector<DiscreteTransformation>& b, const int child_level, const int v_rate, const Eigen::Vector3i& num_division,
              const std::function<bool()>& should_stop = {}) {
    for (int i = 0; i < v_rate; i++) {
      for (int j = 0; j < v_rate; j++) {
        for (int k = 0; k < v_rate; k++) {
          for (int l = 0; l < num_division.x(); l++) {
            for (int m = 0; m < num_division.y(); m++) {
              for (int n = 0; n < num_division.z(); n++) {
                if (should_stop && should_stop()) return;
                b.emplace_back(DiscreteTransformation(
                  0,
                  child_level,
                  x * v_rate + i,
                  y * v_rate + j,
                  z * v_rate + k,
                  roll * num_division.x() + l,
                  pitch * num_division.y() + m,
                  yaw * num_division.z() + n));
              }
            }
          }
        }
      }
    }
  }

  std::vector<DiscreteTransformation> branch(const int child_level, const int v_rate, const Eigen::Vector3i& num_division,
              const std::function<bool()>& should_stop = {}) {
    std::vector<DiscreteTransformation> b;
    if (should_stop && should_stop()) return b;
    b.reserve(v_rate * v_rate * v_rate * num_division.x() * num_division.y() * num_division.z());
    for (int i = 0; i < v_rate; i++) {
      for (int j = 0; j < v_rate; j++) {
        for (int k = 0; k < v_rate; k++) {
          for (int l = 0; l < num_division.x(); l++) {
            for (int m = 0; m < num_division.y(); m++) {
              for (int n = 0; n < num_division.z(); n++) {
                if (should_stop && should_stop()) return b;
                b.emplace_back(DiscreteTransformation(
                  0,
                  child_level,
                  x * v_rate + i,
                  y * v_rate + j,
                  z * v_rate + k,
                  roll * num_division.x() + l,
                  pitch * num_division.y() + m,
                  yaw * num_division.z() + n));
              }
            }
          }
        }
      }
    }
    return b;
  }

public:
  int score;
  int level;
  int x;
  int y;
  int z;
  int roll;
  int pitch;
  int yaw;
};
