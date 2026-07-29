#ifndef PRECISION_LANDING__TYPES_HPP_
#define PRECISION_LANDING__TYPES_HPP_

namespace precision_landing
{

enum class PrecLandState
{
  IDLE,
  START,
  HORIZONTAL_APPROACH,
  DESCEND_ABOVE_TARGET,
  FINAL_APPROACH,
  SEARCH,
  TARGET_LOST,
  FALLBACK,
  DONE
};

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

}  // namespace precision_landing

#endif  // PRECISION_LANDING__TYPES_HPP_
