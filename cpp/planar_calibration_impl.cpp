// calib_eigen_modern.cpp
//
// Modern C++20 + Eigen (header-only) implementation of a strong planar camera calibration pipeline:
//   - Zhang planar initialization (homographies)
//   - Progressive Levenberg–Marquardt bundle adjustment (intrinsics + distortion + poses)
//   - Synthetic test harness included in main()
//
// Build (example):
//   g++ -O2 -std=c++20 calib_eigen_modern.cpp -I /mnt/data/eigen/eigen-5.0.0 -o calib_modern
//
//
// -----------------------------------------------------------------------------
// PSEUDOCODE (Planar Zhang + Progressive Bundle Adjustment)
//
// INPUT:
//   - Views v=1..M
//       - Planar target points (X_i, Y_i) with Z=0
//       - Observed image pixels (u_{v,i}, v_{v,i})
//
// STEP 1: Homographies (planar -> image), one per view
//   for each view v:
//     H_v = NormalizedDLT_Homography( (X_i,Y_i) -> (u_{v,i},v_{v,i}) )
//
// STEP 2: Zhang intrinsics from homographies
//   K = SolveKFromHomographies({H_v})
//
// STEP 3: Initialize per-view extrinsics
//   for each view v:
//     (R_v, t_v) = DecomposeHomography(K, H_v)
//     rvec_v = RodriguesInverse(R_v)
//
// STEP 4: Progressive LM bundle adjustment (intrinsics + distortion + poses)
//   params p = [fx, fy, cx, cy, k1, k2, p1, p2, (rvec_v,t_v)*M]
//
//   Stage 0: optimize {fx, fy, cx, cy, all poses} with distortion fixed to 0
//   Stage 1: additionally optimize {k1, k2}
//   Stage 2: additionally optimize {p1, p2}
//
//   At each LM iteration:
//     r = stacked reprojection residuals
//     J = numeric Jacobian (central differences) for active params
//     Solve (JᵀJ + λI) Δp = -Jᵀr
//     Accept if cost decreases; otherwise increase λ
//
// OUTPUT:
//   - Estimated intrinsics (K)
//   - Estimated distortion (k1,k2,p1,p2)
//   - Estimated extrinsics (R_v,t_v) per view
// -----------------------------------------------------------------------------
//
// Notes:
//   - No OpenCV / CV libraries.
//   - Uses Eigen for linear algebra only.
//   - Designed for clarity and robustness rather than ultimate speed.
//
#include <Eigen/Dense>                     // Eigen matrices/vectors
#include <Eigen/SVD>                       // SVD
#include <Eigen/Cholesky>                  // LDLT
#include <algorithm>                       // std::clamp, std::min, std::max
#include <array>                           // std::array
#include <chrono>                          // timing
#include <cmath>                           // std::sin, std::cos, std::sqrt, std::abs
#include <cstdint>                         // std::uint8_t
#include <iomanip>                         // std::setprecision
#include <iostream>                        // std::cout
#include <numeric>                         // std::iota
#include <optional>                        // std::optional
#include <random>                          // RNG
#include <span>                            // std::span
#include <stdexcept>                       // std::runtime_error
#include <string_view>                     // std::string_view
#include <utility>                         // std::pair
#include <vector>                          // std::vector
//
namespace calib {                                                               // create a namespace to avoid symbol collisions
//
using Mat3  = Eigen::Matrix3d;             // 3x3 double
using Vec2  = Eigen::Vector2d;             // 2x1 double
using Vec3  = Eigen::Vector3d;             // 3x1 double
using MatX  = Eigen::MatrixXd;             // dynamic double matrix
using VecX  = Eigen::VectorXd;             // dynamic double vector
//
// -----------------------------
// Basic parameter bundles
// -----------------------------
struct Intrinsics final {                                                       // open scope
  double fx {0.0};                         // focal length x (pixels)
  double fy {0.0};                         // focal length y (pixels)
  double cx {0.0};                         // principal point x (pixels)
  double cy {0.0};                         // principal point y (pixels)
  double skew {0.0};                       // skew (kept for completeness; often 0)
};                                                                              // close scope
//
struct Distortion final {                  // Brown–Conrady (2 radial + 2 tangential)
  double k1 {0.0};                         // radial k1
  double k2 {0.0};                         // radial k2
  double p1 {0.0};                         // tangential p1
  double p2 {0.0};                         // tangential p2
};                                                                              // close scope
//
struct Extrinsics final {                  // per-view pose
  Vec3 rvec {Vec3::Zero()};                // Rodrigues rotation vector
  Vec3 tvec {Vec3::Zero()};                // translation
};                                                                              // close scope
//
// -----------------------------
// Small utilities
// -----------------------------
[[nodiscard]] inline double sqr(const double x) noexcept { return x * x; }      // attribute: warn if return value is ignored
//
template <typename T>                                                           // function template to support multiple types
[[nodiscard]] inline T clamp(const T v, const T lo, const T hi) noexcept {      // open scope
  return std::clamp(v, lo, hi);                                                 // clamp values to a safe numeric range
}                                                                               // close scope
//
struct ScopedTimer final {                                                      // open scope
  std::string_view name;                                                        // statement
  std::chrono::steady_clock::time_point t0 {std::chrono::steady_clock::now()};  // statement
  explicit ScopedTimer(std::string_view n) : name(n) {}                         // statement
  ~ScopedTimer() {                                                              // open scope
    using namespace std::chrono;                                                // type alias for readability and shorter signatures
    const auto dt = duration_cast<milliseconds>(steady_clock::now() - t0).count(); // const: avoid mutation; enables compiler optimization
    std::cout << "[timer] " << name << ": " << dt << " ms\n";                   // print results / progress
  }                                                                             // close scope
};                                                                              // close scope
//
// -----------------------------
// Rodrigues conversions
// -----------------------------
[[nodiscard]] inline Mat3 rodriguesToR(const Vec3& rvec) noexcept {             // open scope
  const double theta = rvec.norm();                                             // const: avoid mutation; enables compiler optimization
  if (theta < 1e-12) return Mat3::Identity();                                   // return computed value
//
  const Vec3 k = rvec / theta;                                                  // const: avoid mutation; enables compiler optimization
//
  Mat3 K;                                                                       // statement
  K <<  0.0, -k.z(),  k.y(),                                                    // statement
        k.z(),  0.0, -k.x(),                                                    // statement
       -k.y(),  k.x(),  0.0;                                                    // statement
//
  return Mat3::Identity()                                                       // return computed value
       + std::sin(theta) * K                                                    // statement
       + (1.0 - std::cos(theta)) * (K * K);                                     // statement
}                                                                               // close scope
//
[[nodiscard]] inline Vec3 rvecFromR(const Mat3& R) noexcept {                   // open scope
  const double tr = R.trace();                                                  // const: avoid mutation; enables compiler optimization
  double cos_theta = (tr - 1.0) * 0.5;                                          // compute / assign value
  cos_theta = clamp(cos_theta, -1.0, 1.0);                                      // compute / assign value
//
  const double theta = std::acos(cos_theta);                                    // const: avoid mutation; enables compiler optimization
  if (theta < 1e-12) return Vec3::Zero();                                       // return computed value
//
  Vec3 axis;                                                                    // statement
  axis << (R(2,1) - R(1,2)),                                                    // statement
          (R(0,2) - R(2,0)),                                                    // statement
          (R(1,0) - R(0,1));                                                    // statement
  axis *= (1.0 / (2.0 * std::sin(theta)));                                      // compute / assign value
//
  return axis * theta;                                                          // return computed value
}                                                                               // close scope
//
// -----------------------------
// Distortion in normalized coordinates
// -----------------------------
[[nodiscard]] inline Vec2 distortNormalized(const Vec2& xy, const Distortion& d) noexcept { // open scope
  const double x = xy.x();                                                      // const: avoid mutation; enables compiler optimization
  const double y = xy.y();                                                      // const: avoid mutation; enables compiler optimization
//
  const double r2 = sqr(x) + sqr(y);                                            // const: avoid mutation; enables compiler optimization
  const double r4 = sqr(r2);                                                    // const: avoid mutation; enables compiler optimization
//
  const double radial = 1.0 + d.k1 * r2 + d.k2 * r4;                            // const: avoid mutation; enables compiler optimization
//
  const double x_tan = 2.0 * d.p1 * x * y + d.p2 * (r2 + 2.0 * sqr(x));         // const: avoid mutation; enables compiler optimization
  const double y_tan = d.p1 * (r2 + 2.0 * sqr(y)) + 2.0 * d.p2 * x * y;         // const: avoid mutation; enables compiler optimization
//
  return { x * radial + x_tan, y * radial + y_tan };                            // return computed value
}                                                                               // close scope
//
// -----------------------------
// Projection (3D -> 2D pixels)
// -----------------------------
[[nodiscard]] inline Vec2 projectPoint(                                         // attribute: warn if return value is ignored
    const Vec3& X,                                                              // const: avoid mutation; enables compiler optimization
    const Intrinsics& K,                                                        // const: avoid mutation; enables compiler optimization
    const Distortion& D,                                                        // const: avoid mutation; enables compiler optimization
    const Extrinsics& E) noexcept {                                             // open scope
//
  const Mat3 R = rodriguesToR(E.rvec);                                          // const: avoid mutation; enables compiler optimization
  const Vec3 Xc = R * X + E.tvec;                                               // const: avoid mutation; enables compiler optimization
//
  const double z = (std::abs(Xc.z()) < 1e-12) ? (Xc.z() >= 0.0 ? 1e-12 : -1e-12) : Xc.z(); // const: avoid mutation; enables compiler optimization
  const Vec2 xy { Xc.x() / z, Xc.y() / z };                                     // const: avoid mutation; enables compiler optimization
//
  const Vec2 xy_d = distortNormalized(xy, D);                                   // const: avoid mutation; enables compiler optimization
//
  const double u = K.fx * xy_d.x() + K.skew * xy_d.y() + K.cx;                  // const: avoid mutation; enables compiler optimization
  const double v = K.fy * xy_d.y() + K.cy;                                      // const: avoid mutation; enables compiler optimization
//
  return {u, v};                                                                // return computed value
}                                                                               // close scope
//
// -----------------------------
// Hartley normalization (2D)
// -----------------------------
struct Normalization2D final {                                                  // open scope
  MatX pts;                                 // normalized (N,2)
  Mat3 T  {Mat3::Identity()};               // normalization transform (3,3)
};                                                                              // close scope
//
[[nodiscard]] inline Normalization2D normalize2D(const Eigen::Ref<const MatX>& pts) { // open scope
  if (pts.cols() != 2) throw std::runtime_error("normalize2D: expected Nx2 matrix"); // throw exception on invalid input
//
  const std::ptrdiff_t N = pts.rows();                                          // const: avoid mutation; enables compiler optimization
  const double meanx = pts.col(0).mean();                                       // const: avoid mutation; enables compiler optimization
  const double meany = pts.col(1).mean();                                       // const: avoid mutation; enables compiler optimization
//
  double meanDist = 0.0;                                                        // compute / assign value
  for (std::ptrdiff_t i = 0; i < N; ++i) {                                      // open scope
    const double dx = pts(i,0) - meanx;                                         // const: avoid mutation; enables compiler optimization
    const double dy = pts(i,1) - meany;                                         // const: avoid mutation; enables compiler optimization
    meanDist += std::sqrt(dx*dx + dy*dy);                                       // compute / assign value
  }                                                                             // close scope
  meanDist /= std::max<std::ptrdiff_t>(1, N);                                   // compute / assign value
//
  const double s = (meanDist < 1e-12) ? 1.0 : std::sqrt(2.0) / meanDist;        // const: avoid mutation; enables compiler optimization
//
  Normalization2D out;                                                          // statement
  out.T << s,   0.0, -s*meanx,                                                  // statement
           0.0, s,   -s*meany,                                                  // statement
           0.0, 0.0, 1.0;                                                       // statement
//
  out.pts.resize(N, 2);                                                         // statement
  for (std::ptrdiff_t i = 0; i < N; ++i) {                                      // open scope
    const Eigen::Vector3d ph { pts(i,0), pts(i,1), 1.0 };                       // const: avoid mutation; enables compiler optimization
    const Eigen::Vector3d q = out.T * ph;                                       // const: avoid mutation; enables compiler optimization
    out.pts(i,0) = q.x() / q.z();                                               // compute / assign value
    out.pts(i,1) = q.y() / q.z();                                               // compute / assign value
  }                                                                             // close scope
//
  return out;                                                                   // return computed value
}                                                                               // close scope
//
// -----------------------------
// Homography via normalized DLT
// -----------------------------
[[nodiscard]] inline Mat3 homographyDLT(                                        // attribute: warn if return value is ignored
    const Eigen::Ref<const MatX>& planarXY,                                     // const: avoid mutation; enables compiler optimization
    const Eigen::Ref<const MatX>& imageUV) {                                    // open scope
//
  if (planarXY.cols() != 2 || imageUV.cols() != 2) throw std::runtime_error("homographyDLT: expected Nx2 inputs"); // throw exception on invalid input
  if (planarXY.rows() != imageUV.rows()) throw std::runtime_error("homographyDLT: row mismatch"); // throw exception on invalid input
  if (planarXY.rows() < 4) throw std::runtime_error("homographyDLT: need >= 4 points"); // throw exception on invalid input
//
  const auto Xn = normalize2D(planarXY);                                        // const: avoid mutation; enables compiler optimization
  const auto xn = normalize2D(imageUV);                                         // const: avoid mutation; enables compiler optimization
//
  const std::ptrdiff_t N = planarXY.rows();                                     // const: avoid mutation; enables compiler optimization
  MatX A(2*N, 9);                                                               // statement
  for (std::ptrdiff_t i = 0; i < N; ++i) {                                      // open scope
    const double X = Xn.pts(i,0);                                               // const: avoid mutation; enables compiler optimization
    const double Y = Xn.pts(i,1);                                               // const: avoid mutation; enables compiler optimization
    const double u = xn.pts(i,0);                                               // const: avoid mutation; enables compiler optimization
    const double v = xn.pts(i,1);                                               // const: avoid mutation; enables compiler optimization
//
    A.row(2*i + 0) << -X, -Y, -1.0,  0.0,  0.0,  0.0,  u*X, u*Y, u;             // statement
    A.row(2*i + 1) <<  0.0,  0.0,  0.0, -X, -Y, -1.0,  v*X, v*Y, v;             // statement
  }                                                                             // close scope
//
  const Eigen::JacobiSVD<MatX> svd(A, Eigen::ComputeFullV);                     // use Eigen decomposition for linear solves
  const VecX h = svd.matrixV().col(8);                                          // const: avoid mutation; enables compiler optimization
//
  Mat3 Hn;                                                                      // statement
  Hn << h(0), h(1), h(2),                                                       // statement
        h(3), h(4), h(5),                                                       // statement
        h(6), h(7), h(8);                                                       // statement
//
  Mat3 H = xn.T.inverse() * Hn * Xn.T;                                          // compute / assign value
  if (std::abs(H(2,2)) > 1e-12) H /= H(2,2);                                    // branch for stability / correctness
  return H;                                                                     // return computed value
}                                                                               // close scope
//
// -----------------------------
// Zhang intrinsics from homographies
// -----------------------------
[[nodiscard]] inline Eigen::Matrix<double,6,1> vij(const Mat3& H, const int i, const int j) noexcept { // open scope
  Eigen::Matrix<double,6,1> v;                                                  // statement
  v(0) = H(0,i)*H(0,j);                                                         // compute / assign value
  v(1) = H(0,i)*H(1,j) + H(1,i)*H(0,j);                                         // compute / assign value
  v(2) = H(1,i)*H(1,j);                                                         // compute / assign value
  v(3) = H(2,i)*H(0,j) + H(0,i)*H(2,j);                                         // compute / assign value
  v(4) = H(2,i)*H(1,j) + H(1,i)*H(2,j);                                         // compute / assign value
  v(5) = H(2,i)*H(2,j);                                                         // compute / assign value
  return v;                                                                     // return computed value
}                                                                               // close scope
//
[[nodiscard]] inline Intrinsics intrinsicsFromHomographies(std::span<const Mat3> Hs) { // open scope
  if (Hs.size() < 2) throw std::runtime_error("intrinsicsFromHomographies: need >=2 homographies"); // throw exception on invalid input
//
  const std::ptrdiff_t M = static_cast<std::ptrdiff_t>(Hs.size());              // const: avoid mutation; enables compiler optimization
  MatX V(2*M, 6);                                                               // statement
//
  for (std::ptrdiff_t k = 0; k < M; ++k) {                                      // open scope
    V.row(2*k + 0) = vij(Hs[k], 0, 1).transpose();                              // compute / assign value
    V.row(2*k + 1) = (vij(Hs[k], 0, 0) - vij(Hs[k], 1, 1)).transpose();         // compute / assign value
  }                                                                             // close scope
//
  const Eigen::JacobiSVD<MatX> svd(V, Eigen::ComputeFullV);                     // use Eigen decomposition for linear solves
  VecX b = svd.matrixV().col(5);                                                // compute / assign value
//
  // Unpack symmetric B (K^{-T}K^{-1})
  double B11=b(0), B12=b(1), B22=b(2), B13=b(3), B23=b(4), B33=b(5);            // compute / assign value
//
  const auto recompute = [&]() {                                                // open scope
    const double denom = (B11*B22 - B12*B12);                                   // const: avoid mutation; enables compiler optimization
    const double v0 = (B12*B13 - B11*B23) / denom;                              // const: avoid mutation; enables compiler optimization
    const double lambda = B33 - (B13*B13 + v0*(B12*B13 - B11*B23)) / B11;       // const: avoid mutation; enables compiler optimization
//
    const double alpha = std::sqrt(std::abs(lambda / B11));                     // const: avoid mutation; enables compiler optimization
    const double beta  = std::sqrt(std::abs(lambda * B11 / denom));             // const: avoid mutation; enables compiler optimization
    const double gamma = -B12 * alpha*alpha * beta / lambda;                    // const: avoid mutation; enables compiler optimization
    const double u0 = (gamma * v0 / beta) - (B13 * alpha*alpha / lambda);       // const: avoid mutation; enables compiler optimization
//
    return Intrinsics{alpha, beta, u0, v0, gamma};                              // return computed value
  };                                                                            // close scope
//
  Intrinsics K = recompute();                                                   // compute / assign value
//
  // If focal lengths come out non-finite (rare), flip sign
  if (!std::isfinite(K.fx) || !std::isfinite(K.fy)) {                           // open scope
    b = -b;                                                                     // compute / assign value
    B11=b(0); B12=b(1); B22=b(2); B13=b(3); B23=b(4); B33=b(5);                 // compute / assign value
    K = recompute();                                                            // compute / assign value
  }                                                                             // close scope
//
  return K;                                                                     // return computed value
}                                                                               // close scope
//
// -----------------------------
// Extrinsics from K and H
// -----------------------------
[[nodiscard]] inline Extrinsics extrinsicsFromKH(const Intrinsics& intr, const Mat3& H) { // open scope
  Mat3 K;                                                                       // statement
  K << intr.fx, intr.skew, intr.cx,                                             // statement
       0.0,     intr.fy,   intr.cy,                                             // statement
       0.0,     0.0,       1.0;                                                 // statement
//
  const Mat3 invK = K.inverse();                                                // const: avoid mutation; enables compiler optimization
//
  const Vec3 h1 = H.col(0);                                                     // const: avoid mutation; enables compiler optimization
  const Vec3 h2 = H.col(1);                                                     // const: avoid mutation; enables compiler optimization
  const Vec3 h3 = H.col(2);                                                     // const: avoid mutation; enables compiler optimization
//
  const Vec3 r1p = invK * h1;                                                   // const: avoid mutation; enables compiler optimization
  const Vec3 r2p = invK * h2;                                                   // const: avoid mutation; enables compiler optimization
//
  const double s = 1.0 / std::max(1e-12, r1p.norm());                           // const: avoid mutation; enables compiler optimization
  Vec3 r1 = s * r1p;                                                            // compute / assign value
  Vec3 r2 = s * r2p;                                                            // compute / assign value
  Vec3 r3 = r1.cross(r2);                                                       // compute / assign value
//
  Mat3 R;                                                                       // statement
  R.col(0) = r1;                                                                // compute / assign value
  R.col(1) = r2;                                                                // compute / assign value
  R.col(2) = r3;                                                                // compute / assign value
//
  // Orthonormalize to nearest rotation
  const Eigen::JacobiSVD<Mat3> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV); // use Eigen decomposition for linear solves
  R = svd.matrixU() * svd.matrixV().transpose();                                // compute / assign value
  if (R.determinant() < 0) {                                                    // open scope
    Mat3 U = svd.matrixU();                                                     // compute / assign value
    U.col(2) *= -1.0;                                                           // compute / assign value
    R = U * svd.matrixV().transpose();                                          // compute / assign value
  }                                                                             // close scope
//
  Extrinsics ext;                                                               // statement
  ext.rvec = rvecFromR(R);                                                      // compute / assign value
  ext.tvec = s * (invK * h3);                                                   // compute / assign value
  return ext;                                                                   // return computed value
}                                                                               // close scope
//
// -----------------------------
// Parameterization utilities
// p = [fx fy cx cy | k1 k2 p1 p2 | (rvec,tvec)*M]
// -----------------------------
struct ParamLayout final {                                                      // open scope
  static constexpr int kIntrCount = 4;                                          // compute / assign value
  static constexpr int kDistCount = 4;                                          // compute / assign value
  static constexpr int kHeaderCount = kIntrCount + kDistCount; // 8
  static constexpr int kPoseCount = 6;                          // rvec(3)+tvec(3)
//
  static constexpr int idx_fx = 0;                                              // compute / assign value
  static constexpr int idx_fy = 1;                                              // compute / assign value
  static constexpr int idx_cx = 2;                                              // compute / assign value
  static constexpr int idx_cy = 3;                                              // compute / assign value
//
  static constexpr int idx_k1 = 4;                                              // compute / assign value
  static constexpr int idx_k2 = 5;                                              // compute / assign value
  static constexpr int idx_p1 = 6;                                              // compute / assign value
  static constexpr int idx_p2 = 7;                                              // compute / assign value
//
  [[nodiscard]] static constexpr int poseBase(int viewIdx) noexcept {           // open scope
    return kHeaderCount + kPoseCount * viewIdx;                                 // return computed value
  }                                                                             // close scope
};                                                                              // close scope
//
[[nodiscard]] inline VecX packParams(const Intrinsics& intr, const Distortion& dist, std::span<const Extrinsics> extr) { // open scope
  const int M = static_cast<int>(extr.size());                                  // const: avoid mutation; enables compiler optimization
  VecX p(ParamLayout::kHeaderCount + ParamLayout::kPoseCount * M);              // statement
//
  p(ParamLayout::idx_fx) = intr.fx;                                             // compute / assign value
  p(ParamLayout::idx_fy) = intr.fy;                                             // compute / assign value
  p(ParamLayout::idx_cx) = intr.cx;                                             // compute / assign value
  p(ParamLayout::idx_cy) = intr.cy;                                             // compute / assign value
//
  p(ParamLayout::idx_k1) = dist.k1;                                             // compute / assign value
  p(ParamLayout::idx_k2) = dist.k2;                                             // compute / assign value
  p(ParamLayout::idx_p1) = dist.p1;                                             // compute / assign value
  p(ParamLayout::idx_p2) = dist.p2;                                             // compute / assign value
//
  for (int v = 0; v < M; ++v) {                                                 // open scope
    const int base = ParamLayout::poseBase(v);                                  // const: avoid mutation; enables compiler optimization
    p.segment<3>(base + 0) = extr[v].rvec;                                      // compute / assign value
    p.segment<3>(base + 3) = extr[v].tvec;                                      // compute / assign value
  }                                                                             // close scope
//
  return p;                                                                     // return computed value
}                                                                               // close scope
//
inline void unpackParams(const VecX& p, const int nViews, Intrinsics& intr, Distortion& dist, std::vector<Extrinsics>& extr) { // open scope
  intr.fx = p(ParamLayout::idx_fx);                                             // compute / assign value
  intr.fy = p(ParamLayout::idx_fy);                                             // compute / assign value
  intr.cx = p(ParamLayout::idx_cx);                                             // compute / assign value
  intr.cy = p(ParamLayout::idx_cy);                                             // compute / assign value
  intr.skew = 0.0; // enforce zero skew in refinement (common)
//
  dist.k1 = p(ParamLayout::idx_k1);                                             // compute / assign value
  dist.k2 = p(ParamLayout::idx_k2);                                             // compute / assign value
  dist.p1 = p(ParamLayout::idx_p1);                                             // compute / assign value
  dist.p2 = p(ParamLayout::idx_p2);                                             // compute / assign value
//
  extr.assign(nViews, Extrinsics{});                                            // statement
  for (int v = 0; v < nViews; ++v) {                                            // open scope
    const int base = ParamLayout::poseBase(v);                                  // const: avoid mutation; enables compiler optimization
    extr[v].rvec = p.segment<3>(base + 0);                                      // compute / assign value
    extr[v].tvec = p.segment<3>(base + 3);                                      // compute / assign value
  }                                                                             // close scope
}                                                                               // close scope
//
// -----------------------------
// Residuals: stack all (u,v) for all points across all views
// -----------------------------
[[nodiscard]] inline VecX computeResiduals(                                     // attribute: warn if return value is ignored
    const VecX& p,                                                              // const: avoid mutation; enables compiler optimization
    const Eigen::Ref<const MatX>& objectPts,           // (N,3)
    std::span<const MatX> imagePts) {                  // M items of (N,2)
//
  // Unpack parameter vector into structured bundles (intrinsics + distortion + per-view poses)
  const int M = static_cast<int>(imagePts.size());                              // const: avoid mutation; enables compiler optimization
  const int N = static_cast<int>(objectPts.rows());                             // const: avoid mutation; enables compiler optimization
//
  Intrinsics intr;                                                              // statement
  Distortion dist;                                                              // statement
  std::vector<Extrinsics> extr;                                                 // use std::vector for dynamic arrays
  unpackParams(p, M, intr, dist, extr);                                         // statement
//
  // Stack residuals as [ex0 ey0 ex1 ey1 ...] for all points and all views
  VecX r(2LL * M * N);                                                          // statement
  std::int64_t k = 0;                                                           // compute / assign value
//
  // Performance note:
  //   We compute the rotation matrix once per view (Rodrigues) and reuse it for all points in that view.
  for (int v = 0; v < M; ++v) {                                                 // open scope
    const Mat3 R = rodriguesToR(extr[v].rvec);                                  // const: avoid mutation; enables compiler optimization
    const Vec3 t = extr[v].tvec;                                                // const: avoid mutation; enables compiler optimization
//
    for (int i = 0; i < N; ++i) {                                               // open scope
      const Vec3 X = objectPts.row(i).transpose();                              // const: avoid mutation; enables compiler optimization
      const Vec3 Xc = R * X + t;                                                // const: avoid mutation; enables compiler optimization
//
      const double z = (std::abs(Xc.z()) < 1e-12) ? (Xc.z() >= 0.0 ? 1e-12 : -1e-12) : Xc.z(); // const: avoid mutation; enables compiler optimization
      const Vec2 xy { Xc.x() / z, Xc.y() / z };                                 // const: avoid mutation; enables compiler optimization
//
      const Vec2 xy_d = distortNormalized(xy, dist);                            // const: avoid mutation; enables compiler optimization
//
      const double u = intr.fx * xy_d.x() + intr.skew * xy_d.y() + intr.cx;     // const: avoid mutation; enables compiler optimization
      const double vpix = intr.fy * xy_d.y() + intr.cy;                         // const: avoid mutation; enables compiler optimization
//
      const Vec2 obs  = imagePts[v].row(i).transpose();                         // const: avoid mutation; enables compiler optimization
      r(k++) = obs.x() - u;                                                     // compute / assign value
      r(k++) = obs.y() - vpix;                                                  // compute / assign value
    }                                                                           // close scope
  }                                                                             // close scope
//
  return r;                                                                     // return computed value
}                                                                               // close scope
//
// -----------------------------
// Numeric Jacobian (central difference) for active parameters only
// -----------------------------
using ActiveMask = std::vector<std::uint8_t>;                                   // type alias for readability and shorter signatures
//
[[nodiscard]] inline MatX numericJacobianActive(                                // attribute: warn if return value is ignored
    const VecX& p,                                                              // const: avoid mutation; enables compiler optimization
    const ActiveMask& active,                                                   // const: avoid mutation; enables compiler optimization
    const Eigen::Ref<const MatX>& objectPts,                                    // const: avoid mutation; enables compiler optimization
    std::span<const MatX> imagePts,                                             // use std::span for non-owning views of contiguous data
    const double epsScale) {                                                    // open scope
//
  if (static_cast<int>(active.size()) != p.size()) throw std::runtime_error("numericJacobianActive: active size mismatch"); // throw exception on invalid input
//
  const VecX r0 = computeResiduals(p, objectPts, imagePts);                     // const: avoid mutation; enables compiler optimization
  const int m = static_cast<int>(r0.size());                                    // const: avoid mutation; enables compiler optimization
  const int n = static_cast<int>(p.size());                                     // const: avoid mutation; enables compiler optimization
//
  MatX J(m, n);                                                                 // statement
  J.setZero();                                                                  // statement
//
  for (int j = 0; j < n; ++j) {                                                 // open scope
    if (!active[j]) continue;                                                   // branch for stability / correctness
//
    const double step = epsScale * std::max(1.0, std::abs(p(j)));               // const: avoid mutation; enables compiler optimization
    VecX dp = VecX::Zero(n);                                                    // compute / assign value
    dp(j) = step;                                                               // compute / assign value
//
    const VecX rp = computeResiduals(p + dp, objectPts, imagePts);              // const: avoid mutation; enables compiler optimization
    const VecX rm = computeResiduals(p - dp, objectPts, imagePts);              // const: avoid mutation; enables compiler optimization
//
    J.col(j) = (rp - rm) / (2.0 * step);                                        // compute / assign value
  }                                                                             // close scope
//
  return J;                                                                     // return computed value
}                                                                               // close scope
//
// -----------------------------
// Parameter clamps (stability)
// -----------------------------
inline void clampParams(VecX& p, const int imgW, const int imgH) {              // open scope
  p(ParamLayout::idx_fx) = clamp(p(ParamLayout::idx_fx), 100.0, 5000.0);        // compute / assign value
  p(ParamLayout::idx_fy) = clamp(p(ParamLayout::idx_fy), 100.0, 5000.0);        // compute / assign value
  p(ParamLayout::idx_cx) = clamp(p(ParamLayout::idx_cx), 0.0, static_cast<double>(imgW)); // compute / assign value
  p(ParamLayout::idx_cy) = clamp(p(ParamLayout::idx_cy), 0.0, static_cast<double>(imgH)); // compute / assign value
//
  p(ParamLayout::idx_k1) = clamp(p(ParamLayout::idx_k1), -1.0, 1.0);            // compute / assign value
  p(ParamLayout::idx_k2) = clamp(p(ParamLayout::idx_k2), -1.0, 1.0);            // compute / assign value
  p(ParamLayout::idx_p1) = clamp(p(ParamLayout::idx_p1), -0.1, 0.1);            // compute / assign value
  p(ParamLayout::idx_p2) = clamp(p(ParamLayout::idx_p2), -0.1, 0.1);            // compute / assign value
}                                                                               // close scope
//
// -----------------------------
// Progressive Levenberg–Marquardt optimizer
// -----------------------------
struct LMOptions final {                                                        // open scope
  double jacEpsScale {1e-6};                // relative epsilon for numeric Jacobian
  double initLambda {1e-2};                 // initial LM damping
  double lambdaUp {10.0};                   // increase lambda multiplier
  double lambdaDown {0.3};                  // decrease lambda multiplier
  double minLambda {1e-12};                 // lambda lower bound
  double maxLambda {1e12};                  // lambda upper bound
  double maxStepNorm {5.0};                 // cap update norm
  bool verbose {true};                      // logging
};                                                                              // close scope
//
[[nodiscard]] inline VecX levenbergMarquardtPhased(                             // attribute: warn if return value is ignored
    VecX p,                                                                     // statement
    std::span<const ActiveMask> stages,                                         // use std::span for non-owning views of contiguous data
    std::span<const int> stageIters,                                            // use std::span for non-owning views of contiguous data
    const Eigen::Ref<const MatX>& objectPts,                                    // const: avoid mutation; enables compiler optimization
    std::span<const MatX> imagePts,                                             // use std::span for non-owning views of contiguous data
    const int imgW,                                                             // const: avoid mutation; enables compiler optimization
    const int imgH,                                                             // const: avoid mutation; enables compiler optimization
    const LMOptions& opt) {                                                     // open scope
//
  if (stages.size() != stageIters.size()) throw std::runtime_error("LM: stages/iters mismatch"); // throw exception on invalid input
//
  for (std::size_t s = 0; s < stages.size(); ++s) {                             // open scope
    const auto& active = stages[s];                                             // const: avoid mutation; enables compiler optimization
    const int maxIters = stageIters[s];                                         // const: avoid mutation; enables compiler optimization
    double lambda = opt.initLambda;                                             // compute / assign value
//
    for (int it = 0; it < maxIters; ++it) {                                     // open scope
      const VecX r = computeResiduals(p, objectPts, imagePts);                  // const: avoid mutation; enables compiler optimization
      const double cost = r.squaredNorm();                                      // const: avoid mutation; enables compiler optimization
//
      const MatX J = numericJacobianActive(p, active, objectPts, imagePts, opt.jacEpsScale); // const: avoid mutation; enables compiler optimization
      const MatX JTJ = J.transpose() * J;                                       // const: avoid mutation; enables compiler optimization
      const VecX g = J.transpose() * r;                                         // const: avoid mutation; enables compiler optimization
//
      const MatX A = JTJ + lambda * MatX::Identity(p.size(), p.size());         // const: avoid mutation; enables compiler optimization
//
      // Solve (JTJ + lambda I) dp = -g
      VecX dp(p.size());                                                        // statement
      const Eigen::LDLT<MatX> ldlt(A);                                          // use Eigen decomposition for linear solves
      if (ldlt.info() == Eigen::Success) dp = ldlt.solve(-g);                   // branch for stability / correctness
      else dp = A.completeOrthogonalDecomposition().solve(-g);                  // compute / assign value
//
      // Enforce active mask (inactive updates = 0)
      for (int j = 0; j < dp.size(); ++j) if (!active[j]) dp(j) = 0.0;          // loop
//
      // Step cap
      const double stepNorm = dp.norm();                                        // const: avoid mutation; enables compiler optimization
      if (stepNorm > opt.maxStepNorm) dp *= (opt.maxStepNorm / stepNorm);       // branch for stability / correctness
//
      // Candidate
      VecX pNew = p + dp;                                                       // compute / assign value
      clampParams(pNew, imgW, imgH);                                            // statement
//
      const double costNew = computeResiduals(pNew, objectPts, imagePts).squaredNorm(); // const: avoid mutation; enables compiler optimization
//
      if (costNew < cost) {                                                     // open scope
        p = std::move(pNew);                                                    // move to avoid copying
        lambda = std::max(opt.minLambda, lambda * opt.lambdaDown);              // compute / assign value
      } else {                                                                  // close scope
        lambda = std::min(opt.maxLambda, lambda * opt.lambdaUp);                // compute / assign value
      }                                                                         // close scope
//
      if (opt.verbose && (it % 10 == 0)) {                                      // open scope
        std::cout << "stage " << s                                              // print results / progress
                  << " iter " << it                                             // statement
                  << " cost=" << cost << " -> " << costNew                      // compute / assign value
                  << " lambda=" << lambda                                       // compute / assign value
                  << " step=" << dp.norm()                                      // compute / assign value
                  << "\n";                                                      // statement
      }                                                                         // close scope
    }                                                                           // close scope
  }                                                                             // close scope
//
  return p;                                                                     // return computed value
}                                                                               // close scope
//
// -----------------------------
// Synthetic dataset generator (planar grid)
// -----------------------------
struct SyntheticConfig final {                                                  // open scope
  int imgW {1280};                                                              // statement
  int imgH {720};                                                               // statement
  int cols {9};                                                                 // statement
  int rows {6};                                                                 // statement
  double square {0.08};                     // target square size (meters)
  int nViews {20};                                                              // statement
  double noisePx {0.2};                                                         // statement
};                                                                              // close scope
//
struct SyntheticData final {                                                    // open scope
  Intrinsics gtK;                                                               // statement
  Distortion gtD;                                                               // statement
  MatX objectPts;                           // (N,3)
  MatX planarXY;                            // (N,2)
  std::vector<Extrinsics> gtExtr;           // M
  std::vector<MatX> imagePts;               // M of (N,2)
};                                                                              // close scope
//
[[nodiscard]] inline SyntheticData makeSyntheticPlanar(const SyntheticConfig& cfg, const Intrinsics& gtK, const Distortion& gtD) { // open scope
  SyntheticData data;                                                           // statement
  data.gtK = gtK;                                                               // compute / assign value
  data.gtD = gtD;                                                               // compute / assign value
//
  const int N = cfg.cols * cfg.rows;                                            // const: avoid mutation; enables compiler optimization
  data.objectPts.resize(N,3);                                                   // statement
  data.planarXY.resize(N,2);                                                    // statement
//
  int k = 0;                                                                    // compute / assign value
  for (int r = 0; r < cfg.rows; ++r) {                                          // open scope
    for (int c = 0; c < cfg.cols; ++c) {                                        // open scope
      const double X = c * cfg.square;                                          // const: avoid mutation; enables compiler optimization
      const double Y = r * cfg.square;                                          // const: avoid mutation; enables compiler optimization
      data.objectPts.row(k) << X, Y, 0.0;                                       // statement
      data.planarXY.row(k) << X, Y;                                             // statement
      ++k;                                                                      // statement
    }                                                                           // close scope
  }                                                                             // close scope
//
  std::mt19937_64 rng(0);                                                       // statement
  std::normal_distribution<double> n01(0.0, 1.0);                               // statement
  const auto randn = [&]() { return n01(rng); };                                // return computed value
//
  data.gtExtr.resize(cfg.nViews);                                               // statement
  data.imagePts.resize(cfg.nViews);                                             // statement
//
  for (int v = 0; v < cfg.nViews; ++v) {                                        // open scope
    data.gtExtr[v].rvec = 0.6 * Vec3(randn(), randn(), randn());                // compute / assign value
//
    const double gridTx = 0.20 * ((v % 5) - 2);                                 // const: avoid mutation; enables compiler optimization
    const double gridTy = 0.15 * (((v / 5) % 4) - 1.5);                         // const: avoid mutation; enables compiler optimization
//
    const double tz = 1.6 + 0.15 * randn();                                     // const: avoid mutation; enables compiler optimization
    const double tx = gridTx + 0.05 * randn();                                  // const: avoid mutation; enables compiler optimization
    const double ty = gridTy + 0.05 * randn();                                  // const: avoid mutation; enables compiler optimization
//
    data.gtExtr[v].tvec = Vec3(tx, ty, tz);                                     // compute / assign value
//
    MatX obs(N,2);                                                              // statement
    for (int i = 0; i < N; ++i) {                                               // open scope
      const Vec3 X = data.objectPts.row(i).transpose();                         // const: avoid mutation; enables compiler optimization
      Vec2 uv = projectPoint(X, gtK, gtD, data.gtExtr[v]);                      // compute / assign value
      uv += cfg.noisePx * Vec2(randn(), randn());                               // compute / assign value
      obs.row(i) = uv.transpose();                                              // compute / assign value
    }                                                                           // close scope
    data.imagePts[v] = std::move(obs);                                          // move to avoid copying
  }                                                                             // close scope
//
  return data;                                                                  // return computed value
}                                                                               // close scope
//
} // namespace calib
//
// -----------------------------
// Main: end-to-end synthetic test
// -----------------------------
int main() {                                                                    // open scope
  using namespace calib;                                                        // type alias for readability and shorter signatures
//
  const SyntheticConfig cfg{};                                                  // const: avoid mutation; enables compiler optimization
  const Intrinsics gtK{800.0, 820.0, cfg.imgW/2.0, cfg.imgH/2.0, 0.0};          // const: avoid mutation; enables compiler optimization
  const Distortion gtD{-0.12, 0.03, 0.001, -0.001};                             // const: avoid mutation; enables compiler optimization
//
  std::cout << std::fixed << std::setprecision(6);                              // print results / progress
//
  SyntheticData data;                                                           // statement
  {                                                                             // open scope
    ScopedTimer t("synthetic_data");                                            // statement
    data = makeSyntheticPlanar(cfg, gtK, gtD);                                  // compute / assign value
  }                                                                             // close scope
//
  std::vector<Mat3> Hs;                                                         // use std::vector for dynamic arrays
  {                                                                             // open scope
    ScopedTimer t("homographies");                                              // statement
    Hs.reserve(cfg.nViews);                                                     // statement
    for (int v = 0; v < cfg.nViews; ++v) {                                      // open scope
      Hs.push_back(homographyDLT(data.planarXY, data.imagePts[v]));             // statement
    }                                                                           // close scope
  }                                                                             // close scope
//
  Intrinsics initK;                                                             // statement
  std::vector<Extrinsics> initExtr;                                             // use std::vector for dynamic arrays
  {                                                                             // open scope
    ScopedTimer t("zhang_init");                                                // statement
    initK = intrinsicsFromHomographies(std::span<const Mat3>(Hs.data(), Hs.size())); // use std::span for non-owning views of contiguous data
    initK.skew = 0.0; // force skew to 0 (typical)
//
    initExtr.resize(cfg.nViews);                                                // statement
    for (int v = 0; v < cfg.nViews; ++v) {                                      // open scope
      initExtr[v] = extrinsicsFromKH(initK, Hs[v]);                             // compute / assign value
    }                                                                           // close scope
  }                                                                             // close scope
//
  // Initial parameters
  Distortion initD{};                                                           // statement
  VecX p0 = packParams(initK, initD, initExtr);                                 // compute / assign value
  clampParams(p0, cfg.imgW, cfg.imgH);                                          // statement
//
  // Stage masks: p = [fx fy cx cy | k1 k2 p1 p2 | poses...]
  const int P = static_cast<int>(p0.size());                                    // const: avoid mutation; enables compiler optimization
  auto makeMask = [&](const std::vector<int>& idxOn) {                          // open scope
    ActiveMask m(static_cast<std::size_t>(P), 0);                               // statement
    for (const int idx : idxOn) m.at(static_cast<std::size_t>(idx)) = 1;        // loop
    return m;                                                                   // return computed value
  };                                                                            // close scope
//
  std::vector<int> idxStage0 { ParamLayout::idx_fx, ParamLayout::idx_fy, ParamLayout::idx_cx, ParamLayout::idx_cy }; // use std::vector for dynamic arrays
  for (int j = ParamLayout::kHeaderCount; j < P; ++j) idxStage0.push_back(j); // all poses
//
  auto stage0 = makeMask(idxStage0);                                            // compute / assign value
//
  auto idxStage1 = idxStage0;                                                   // compute / assign value
  idxStage1.push_back(ParamLayout::idx_k1);                                     // statement
  idxStage1.push_back(ParamLayout::idx_k2);                                     // statement
  auto stage1 = makeMask(idxStage1);                                            // compute / assign value
//
  auto idxStage2 = idxStage1;                                                   // compute / assign value
  idxStage2.push_back(ParamLayout::idx_p1);                                     // statement
  idxStage2.push_back(ParamLayout::idx_p2);                                     // statement
  auto stage2 = makeMask(idxStage2);                                            // compute / assign value
//
  const std::array<ActiveMask,3> stages { stage0, stage1, stage2 };             // use std::array for fixed-size compile-time arrays
  const std::array<int,3> iters { 30, 40, 60 };                                 // use std::array for fixed-size compile-time arrays
//
  // Optimize
  LMOptions opt;                                                                // statement
  opt.verbose = false;                                                          // compute / assign value
//
  VecX pOpt;                                                                    // statement
  {                                                                             // open scope
    ScopedTimer t("bundle_adjust");                                             // statement
    pOpt = levenbergMarquardtPhased(                                            // compute / assign value
      p0,                                                                       // statement
      std::span<const ActiveMask>(stages.data(), stages.size()),                // use std::span for non-owning views of contiguous data
      std::span<const int>(iters.data(), iters.size()),                         // use std::span for non-owning views of contiguous data
      data.objectPts,                                                           // statement
      std::span<const MatX>(data.imagePts.data(), data.imagePts.size()),        // use std::span for non-owning views of contiguous data
      cfg.imgW,                                                                 // statement
      cfg.imgH,                                                                 // statement
      opt                                                                       // statement
    );                                                                          // statement
  }                                                                             // close scope
//
  // Unpack results
  Intrinsics estK;                                                              // statement
  Distortion estD;                                                              // statement
  std::vector<Extrinsics> estExtr;                                              // use std::vector for dynamic arrays
  unpackParams(pOpt, cfg.nViews, estK, estD, estExtr);                          // statement
//
  const VecX rFinal = computeResiduals(pOpt, data.objectPts, std::span<const MatX>(data.imagePts.data(), data.imagePts.size())); // use std::span for non-owning views of contiguous data
  const double rmse = std::sqrt(rFinal.squaredNorm() / static_cast<double>(rFinal.size())); // const: avoid mutation; enables compiler optimization
//
  std::cout << "\nGround-truth intrinsics: fx=" << data.gtK.fx << " fy=" << data.gtK.fy // print results / progress
            << " cx=" << data.gtK.cx << " cy=" << data.gtK.cy << "\n";          // compute / assign value
  std::cout << "Estimated intrinsics:     fx=" << estK.fx << " fy=" << estK.fy  // print results / progress
            << " cx=" << estK.cx << " cy=" << estK.cy << "\n";                  // compute / assign value
//
  std::cout << "\nGround-truth distortion: k1=" << data.gtD.k1 << " k2=" << data.gtD.k2 // print results / progress
            << " p1=" << data.gtD.p1 << " p2=" << data.gtD.p2 << "\n";          // compute / assign value
  std::cout << "Estimated distortion:    k1=" << estD.k1 << " k2=" << estD.k2   // print results / progress
            << " p1=" << estD.p1 << " p2=" << estD.p2 << "\n";                  // compute / assign value
//
  std::cout << "\nFinal reprojection RMSE (px): " << rmse << "\n";              // print results / progress
//
  return 0;                                                                     // return computed value
}                                                                               // close scope
