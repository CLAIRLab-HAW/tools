// A gripper model's linkage: driver joint [rad] <-> clear width between the pads [m], as the TABLE the model file
// carries (clair.robot.contract, profiles/grippers/<model>.yaml: linkage.table), linearly interpolated.
//
// The same interpolation as clair.robot.contract.profile.GripperLinkage, which clearpath's
// robot/contract/tests/test_gripper_linkage_parity.py holds this header to for every model.  The table, not a crank-rocker formula: a recomputed linkage misses both the width
// at q=0 (153.2 mm) and the joint limits (0.0 to 1.25478 rad) on the rg6_v2 and thereby puts the gripper into poses
// that do not exist.

#ifndef GRIPPER_CONTROL__LINKAGE_HPP_
#define GRIPPER_CONTROL__LINKAGE_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace gripper_control
{

class Linkage
{
public:
  //: ``qs`` ascending, ``widths`` falling, the same number of each, two or more.
  Linkage(const std::vector<double> & qs, const std::vector<double> & widths)
  {
    if (qs.size() < 2 || qs.size() != widths.size()) {
      throw std::invalid_argument("a linkage table needs two or more (q, width) points");
    }
    for (std::size_t i = 0; i < qs.size(); ++i) {
      table_.push_back({qs[i], widths[i]});
    }
  }

  double max_width_m() const {return table_.front()[1];}
  double min_width_m() const {return table_.back()[1];}

  //: Clear width [m] at driver joint ``q`` [rad], clamped to the table.
  double width_from_angle(double q) const
  {
    const double x = std::clamp(q, table_.front()[0], table_.back()[0]);
    for (std::size_t i = 1; i < table_.size(); ++i) {
      if (x <= table_[i][0]) {
        const double q0 = table_[i - 1][0], w0 = table_[i - 1][1];
        const double q1 = table_[i][0], w1 = table_[i][1];
        const double t = (q1 - q0) > 0.0 ? (x - q0) / (q1 - q0) : 0.0;
        return w0 + t * (w1 - w0);
      }
    }
    return min_width_m();
  }

  //: Driver joint [rad] for the clear width ``width_m``.  The width is CLAMPED to the representable range -- a
  //: command past the stroke is valid and means "as far as it goes", not NaN.
  double angle_from_width(double width_m) const
  {
    const double w = std::clamp(width_m, min_width_m(), max_width_m());
    for (std::size_t i = 1; i < table_.size(); ++i) {
      if (w >= table_[i][1]) {
        const double q0 = table_[i - 1][0], w0 = table_[i - 1][1];
        const double q1 = table_[i][0], w1 = table_[i][1];
        const double t = (w0 - w1) > 0.0 ? (w0 - w) / (w0 - w1) : 0.0;
        return q0 + t * (q1 - q0);
      }
    }
    return table_.back()[0];
  }

private:
  std::vector<std::array<double, 2>> table_;
};

}  // namespace gripper_control

#endif  // GRIPPER_CONTROL__LINKAGE_HPP_
