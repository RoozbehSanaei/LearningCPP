# Modern C++ features used in `calib_eigen_modern.cpp`

This document explains the C++ language and standard-library features used in the calibration implementation, assuming **basic C++ familiarity**.  
All examples are taken directly from the code.

---

## `namespace calib`

Everything is wrapped in a dedicated namespace so names like `Intrinsics`, `homographyDLT`, and `projectPoint` don’t clash when you integrate this into a larger project. The file then opts into the names inside `main()`.

```cpp
namespace calib {
  // ... all calibration code ...
} // namespace calib

int main() {
  using namespace calib;
  // ... uses Intrinsics, Distortion, etc.
}
```

---

## Type aliases with `using`

The code defines short, meaningful aliases for Eigen matrix/vector types. This reduces verbosity in signatures and makes intent clearer throughout the file.

```cpp
using Mat3  = Eigen::Matrix3d;
using Vec2  = Eigen::Vector2d;
using Vec3  = Eigen::Vector3d;
using MatX  = Eigen::MatrixXd;
using VecX  = Eigen::VectorXd;
```

---

## Plain `struct` bundles with in-class default initialization

Instead of constructors, these structs use default member initializers so they start in a safe state automatically. You see this used repeatedly when the code creates `initD{}` (all zeros) and `Extrinsics{}` (zero vectors).

```cpp
struct Intrinsics final {
  double fx {0.0};
  double fy {0.0};
  double cx {0.0};
  double cy {0.0};
  double skew {0.0};
};

struct Distortion final {
  double k1 {0.0};
  double k2 {0.0};
  double p1 {0.0};
  double p2 {0.0};
};

struct Extrinsics final {
  Vec3 rvec {Vec3::Zero()};
  Vec3 tvec {Vec3::Zero()};
};
```

---

## `final` on small data types

`final` communicates that these are not designed for inheritance. It’s a design signal: they’re value-like parameter bundles, not base classes.

```cpp
struct Distortion final {
  double k1 {0.0};
  double k2 {0.0};
  double p1 {0.0};
  double p2 {0.0};
};
```

---

## `[[nodiscard]]` to catch ignored results

Numerical code often returns matrices/vectors that must be used. Marking functions `[[nodiscard]]` asks the compiler to warn you if you call them and discard the returned value.

```cpp
[[nodiscard]] inline Mat3 rodriguesToR(const Vec3& rvec) noexcept { /* ... */ }

[[nodiscard]] inline VecX computeResiduals(
    const VecX& p,
    const Eigen::Ref<const MatX>& objectPts,
    std::span<const MatX> imagePts) { /* ... */ }
```

---

## `inline` for small utilities

Many helpers are `inline`, which is typical for “header-style” utilities. It encourages inlining and avoids separate compilation-unit linkage concerns if you later move code into headers.

```cpp
[[nodiscard]] inline double sqr(const double x) noexcept { return x * x; }

template <typename T>
[[nodiscard]] inline T clamp(const T v, const T lo, const T hi) noexcept {
  return std::clamp(v, lo, hi);
}
```

---

## `noexcept` to separate “pure math” from “validated API”

Math kernels that should not throw are marked `noexcept`. Functions that validate inputs and may throw (like `homographyDLT`) are not `noexcept`. This cleanly separates safe inner loops from boundary checks.

```cpp
[[nodiscard]] inline Vec2 distortNormalized(const Vec2& xy, const Distortion& d) noexcept { /* ... */ }

[[nodiscard]] inline Mat3 homographyDLT(
    const Eigen::Ref<const MatX>& planarXY,
    const Eigen::Ref<const MatX>& imageUV) {
  if (planarXY.rows() < 4) throw std::runtime_error("homographyDLT: need >= 4 points");
  /* ... */
}
```

---

## `const` references and local `const` variables

Parameters are passed by `const&` to avoid copies and to prevent accidental mutation. Inside functions, intermediate values are declared `const` to keep logic stable and readable.

```cpp
[[nodiscard]] inline Vec2 projectPoint(
    const Vec3& X,
    const Intrinsics& K,
    const Distortion& D,
    const Extrinsics& E) noexcept {

  const Mat3 R = rodriguesToR(E.rvec);
  const Vec3 Xc = R * X + E.tvec;
  /* ... */
}
```

---

## Function templates (the `clamp` helper)

The clamp helper is templated so it works for multiple numeric types consistently, but the call sites remain clean.

```cpp
template <typename T>
[[nodiscard]] inline T clamp(const T v, const T lo, const T hi) noexcept {
  return std::clamp(v, lo, hi);
}
```

---

## `std::string_view` for lightweight timer names

The timer stores its label as `std::string_view`, which avoids allocating/copying strings. In this file, the labels are string literals, which are safe to view.

```cpp
struct ScopedTimer final {
  std::string_view name;
  std::chrono::steady_clock::time_point t0 {std::chrono::steady_clock::now()};
  explicit ScopedTimer(std::string_view n) : name(n) {}
  ~ScopedTimer() { /* prints elapsed */ }
};
```

---

## RAII via `ScopedTimer`

The timer starts in the constructor and prints in the destructor. The braces in `main()` guarantee the timer ends exactly where you expect, without manual “stop” calls.

```cpp
{
  ScopedTimer t("bundle_adjust");
  pOpt = levenbergMarquardtPhased(/* ... */);
} // timer prints here automatically
```

---

## `std::span` for non-owning views of contiguous data

`std::span` lets functions accept arrays/vectors without taking ownership or copying. It also carries the size, so it is safer than raw pointers.

```cpp
[[nodiscard]] inline VecX computeResiduals(
    const VecX& p,
    const Eigen::Ref<const MatX>& objectPts,
    std::span<const MatX> imagePts) { /* ... */ }
```

---

## `std::array` for fixed-size stage schedules

The pipeline has exactly three phases (no distortion → radial → tangential). `std::array` expresses that fixed design and avoids heap allocation.

```cpp
const std::array<ActiveMask,3> stages { stage0, stage1, stage2 };
const std::array<int,3> iters { 30, 40, 60 };
```

---

## Lambdas for local helper logic

Small “local” behavior is expressed via lambdas rather than creating extra helper functions and polluting namespace scope. The capture `[&]` is used where the lambda depends on local variables.

```cpp
const auto recompute = [&]() {
  // uses B11..B33 from enclosing scope
  return Intrinsics{alpha, beta, u0, v0, gamma};
};

auto makeMask = [&](const std::vector<int>& idxOn) {
  ActiveMask m(static_cast<std::size_t>(P), 0);
  for (const int idx : idxOn) m.at(static_cast<std::size_t>(idx)) = 1;
  return m;
};
```

---

## Move semantics with `std::move`

When a candidate update is accepted, the code moves it into place to avoid copying large Eigen vectors/matrices. This is particularly helpful inside iterative optimizers.

```cpp
if (costNew < cost) {
  p = std::move(pNew);
  lambda = std::max(opt.minLambda, lambda * opt.lambdaDown);
}
```

And in synthetic data creation:

```cpp
MatX obs(N,2);
/* fill obs */
data.imagePts[v] = std::move(obs);
```

---

## Exceptions for fail-fast validation

The code uses `throw std::runtime_error(...)` where wrong dimensions would lead to silent numerical corruption. This is a practical way to prevent “garbage in, garbage out.”

```cpp
if (planarXY.rows() != imageUV.rows()) throw std::runtime_error("homographyDLT: row mismatch");
if (planarXY.rows() < 4) throw std::runtime_error("homographyDLT: need >= 4 points");
```

---

## Explicit casts with `static_cast`

Size types differ across APIs (`size_t`, `ptrdiff_t`, `int`). The code uses explicit casts to avoid implicit narrowing and to be clear about intent.

```cpp
const int M = static_cast<int>(imagePts.size());
const std::ptrdiff_t N = pts.rows();
```

---

## `constexpr` layout indices as a single source of truth

All offsets into the parameter vector are centralized in `ParamLayout`. This removes magic numbers and makes packing/unpacking consistent.

```cpp
struct ParamLayout final {
  static constexpr int idx_fx = 0;
  static constexpr int idx_fy = 1;
  static constexpr int idx_cx = 2;
  static constexpr int idx_cy = 3;

  static constexpr int idx_k1 = 4;
  static constexpr int idx_k2 = 5;
  static constexpr int idx_p1 = 6;
  static constexpr int idx_p2 = 7;

  [[nodiscard]] static constexpr int poseBase(int viewIdx) noexcept {
    return kHeaderCount + kPoseCount * viewIdx;
  }
};
```

---

## Eigen `Ref` to avoid copies while accepting matrix expressions

`Eigen::Ref<const MatX>` allows passing matrices (and compatible expressions) efficiently without copying them. The function still sees a matrix-like object with stable access.

```cpp
[[nodiscard]] inline Normalization2D normalize2D(const Eigen::Ref<const MatX>& pts) {
  if (pts.cols() != 2) throw std::runtime_error("normalize2D: expected Nx2 matrix");
  /* ... */
}
```
