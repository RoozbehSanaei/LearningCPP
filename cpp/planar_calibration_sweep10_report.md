# Planar camera calibration sweep report (C++20 + Eigen)

**Date:** 2026-01-12  
**Scope:** Deterministic 10-trial synthetic evaluation of a planar calibration pipeline implemented in C++20 using Eigen for linear algebra (no computer-vision libraries).

---

## 1. Executive summary

A 10-trial synthetic sweep was executed in which the **reference intrinsics** (fx, fy, cx, cy) were re-sampled each trial, a planar grid was projected into 10 images per trial with pixel noise and Brown–Conrady distortion, and the calibration pipeline was run end-to-end.

Key outcomes (10 trials):
- **Noise-limited RMSE (baseline):** 0.498292 ± 0.012309 px
- **Initialization RMSE (homography / Zhang):** 1.705233 ± 0.519229 px
- **Refined RMSE (after phased LM):** 0.621877 ± 0.412107 px
- **Refined-to-baseline ratio:** 1.244274 ± 0.810540

One trial (**run 5**) converged to a poor local minimum (refined RMSE = **1.793087 px**), which inflates the mean and standard deviation of the refined RMSE. A robust view is therefore also reported:
- **Median refined RMSE:** 0.489093 px (MAD = 0.016350 px)
- **Excluding run 5:** refined RMSE 0.491743 ± 0.023293 px, refined/baseline 0.988137 ± 0.032054

---

## 2. Implementation under test

### 2.1 Model
- **Pinhole intrinsics:** fx, fy, cx, cy (skew set to 0).
- **Distortion:** Brown–Conrady with **2 radial** (k1, k2) and **2 tangential** (p1, p2).

### 2.2 Optimization variables
Per trial, the parameter vector is:
- Intrinsics: [fx, fy, cx, cy]
- Distortion: [k1, k2, p1, p2]
- Per-image pose: for each view v, Rodrigues rotation vector rvec_v and translation tvec_v

### 2.3 Pipeline
1. **Homography estimation per image** via normalized DLT (Hartley normalization).
2. **Planar intrinsics initialization** from homographies (Zhang constraints).
3. **Pose initialization** by decomposing K^-1H into R, t, converted to Rodrigues form.
4. **Phased Levenberg–Marquardt refinement** minimizing pixel reprojection residuals:
   - Phase A: optimize (fx, fy, cx, cy) + all poses; distortion fixed to zero.
   - Phase B: additionally optimize (k1, k2).
   - Phase C: additionally optimize (p1, p2).

### 2.4 Numerical details used in this run
- Jacobian: **forward difference** numerical Jacobian for active parameters  
  J(:,j) ≈ ( r(p + h e_j) - r(p) ) / h
- Step scaling: `jacEpsScale = 5e-7` (relative step size).
- Damping: `initLambda = 1e-2`, accept if cost decreases; otherwise increase λ.
- Phase iteration counts: **[5, 7, 10]**.
- Update cap: `maxStepNorm = 2.5`.
- Parameter clamps:
  - fx, fy ∈ [100, 5000]
  - cx ∈ [0, W], cy ∈ [0, H]
  - k1, k2 ∈ [-1, 1], p1, p2 ∈ [-0.1, 0.1]

---

## 3. Synthetic test design

### 3.1 Target and image formation
- Image size: **1280 × 720**
- Planar grid: **9 × 6** points, spacing **0.04**
- Points per image: **54**
- Images per trial: **10**
- Observation noise: i.i.d. Gaussian σ = **0.5 px** added to (u, v) per point

### 3.2 Pose sampling (per image)
For each view:
- Rotation vector: rvec = 0.45 · Normal(0, I)
- Translation:
  - tx ~ Uniform(-0.25, 0.25)
  - ty ~ Uniform(-0.20, 0.20)
  - tz ~ Uniform(0.90, 1.60)
- Rejection criterion: pose accepted only if **all 54 points** project within a **10 px margin** of the image borders
- Max attempts per view: **5000**

### 3.3 Reference intrinsics sweep (per trial)
For trial i:
- fx ~ Uniform(600, 1200)
- fy = fx · a, where a ~ Uniform(0.94, 1.06)
- cx = W/2 + Δx, where Δx ~ Uniform(-50, 50)
- cy = H/2 + Δy, where Δy ~ Uniform(-30, 30)
- skew = 0

### 3.4 Fixed distortion for synthesis
The same distortion coefficients were used in all trials:
- k1 = -0.12, k2 = 0.018, p1 = 0.0012, p2 = -0.0007

### 3.5 Determinism / seeds
- Intrinsics sampler RNG seed: **11**
- Per-trial dataset seed (poses + noise): **1000 + run_index**, where run_index is 0..9

---

## 4. Metrics

### 4.1 RMSE definitions
Each point contributes two residuals (u_obs - u_pred, v_obs - v_pred). Let r be the stacked residual vector.

- **Baseline RMSE** (“floor”): RMSE using the **reference parameters** on the noisy observations  
  RMSE = sqrt( ||r||^2 / len(r) )
- **Initialization RMSE**: RMSE after homography/Zhang initialization (distortion initialized to zero).
- **Refined RMSE**: RMSE after phased LM refinement.
- **Refined/baseline ratio**: RMSE_final / RMSE_floor

### 4.2 Parameter recovery (reported by the program)
- Intrinsics: MAE for fx, fy, cx, cy and relative MAE for fx, fy
- Distortion: MAE for k1, k2, p1, p2

---

## 5. Results

### 5.1 Per-trial table
|       run |       fx_gt |       fy_gt |      cx_gt |      cy_gt |   rmse_floor |   rmse_init |   rmse_final |
|----------:|------------:|------------:|-----------:|-----------:|-------------:|------------:|-------------:|
|  1.000000 |  699.427868 |  722.376867 | 627.802520 | 371.917685 |     0.499879 |    1.880143 |     0.488280 |
|  2.000000 |  635.225330 |  618.759141 | 679.411632 | 370.223325 |     0.498705 |    1.432179 |     0.489906 |
|  3.000000 |  918.051448 |  959.025239 | 617.164855 | 335.916313 |     0.477721 |    1.024511 |     0.462174 |
|  4.000000 | 1130.937002 | 1116.598772 | 615.743967 | 348.956099 |     0.517007 |    1.891883 |     0.525200 |
|  5.000000 |  824.799042 |  775.643657 | 674.649753 | 386.541794 |     0.505165 |    2.936032 |     1.793087 |
|  6.000000 |  780.225451 |  820.916882 | 666.070972 | 381.686013 |     0.500543 |    1.779568 |     0.484461 |
|  7.000000 |  802.073590 |  801.517002 | 633.219712 | 330.525279 |     0.511146 |    1.880217 |     0.498438 |
|  8.000000 |  652.717093 |  661.203107 | 630.510287 | 378.746499 |     0.495480 |    1.454675 |     0.481607 |
|  9.000000 | 1133.450978 | 1189.851555 | 614.146245 | 342.194689 |     0.498011 |    1.441890 |     0.529881 |
| 10.000000 |  744.502714 |  743.926537 | 689.952642 | 332.833964 |     0.479267 |    1.331229 |     0.465738 |

### 5.2 Aggregate metrics (mean ± std)
- Baseline RMSE: **0.498292 ± 0.012309 px**
- Initialization RMSE: **1.705233 ± 0.519229 px**
- Refined RMSE: **0.621877 ± 0.412107 px**
- Refined/baseline: **1.244274 ± 0.810540**

### 5.3 Robust view (local-minimum sensitivity)
Run 5 is an outlier (refined RMSE = **1.793087 px**, refined/baseline = **3.549508**).

- Median refined RMSE: **0.489093 px**
- Median refined/baseline: **0.975967**
- Refined RMSE excluding run 5: **0.491743 ± 0.023293 px**
- Refined/baseline excluding run 5: **0.988137 ± 0.032054**

### 5.4 Parameter recovery summary (from program output)
Intrinsics MAE:
- fx MAE: **20.657992 px**, relative MAE: **0.025017**
- fy MAE: **21.420484 px**, relative MAE: **0.026683**
- cx MAE: **11.478408 px**
- cy MAE: **17.577106 px**

Distortion MAE:
- k1 MAE: **0.029407**
- k2 MAE: **0.074758**
- p1 MAE: **0.001559**
- p2 MAE: **0.001061**

---

## 6. Discussion

### 6.1 Why refined RMSE can exceed baseline RMSE
The baseline RMSE is the error obtained when projecting with the reference parameters onto noisy observations. Because the calibration objective is non-convex and the Jacobian is numerical (forward difference), LM may converge to:
- a near-optimal solution close to the baseline (most runs), or
- a local minimum with substantially higher residual (run 5).

### 6.2 Likely drivers of the outlier run
The outlier behavior is consistent with a combination of:
- limited pose diversity (accepted poses are constrained by the in-frame test),
- nonlinearity introduced by distortion parameters,
- numeric Jacobian sensitivity (step size and forward differencing),
- damping schedule and iteration budget.

### 6.3 Practical mitigations
If the goal is tighter and more consistent convergence across trials:
1. Increase phase iteration budgets (e.g., Phase C beyond 10 iterations).
2. Switch to central-difference Jacobian or implement analytic Jacobians (reduces numerical noise).
3. Add multi-start refinement (restart LM from small perturbations and pick best cost).
4. Use a robust loss (Huber/Cauchy) if outliers are introduced later (not used here).
5. Enforce stronger view diversity (tilt and depth constraints), or accept more extreme poses by relaxing the margin.

---

## 7. Reproducibility

### 7.1 Artifacts
- Source file: `/mnt/data/calib_eigen_sweep10.cpp`
- Eigen headers: `/mnt/data/eigen/eigen-5.0.0/`

### 7.2 Toolchain
- clang version 17.0.0 (https://github.com/swiftlang/llvm-project.git 10999b6d034fe318f3d56c83bddb6572593a8bb0)
- g++ (Debian 12.2.0-14+deb12u1) 12.2.0

### 7.3 Build and run (example)
```bash
clang++ -O2 -std=c++20 /mnt/data/calib_eigen_sweep10.cpp -I /mnt/data/eigen/eigen-5.0.0 -o calib_sweep10_O2_10views
./calib_sweep10_O2_10views
```

### 7.4 Runtime
- elapsed=0.76 s

---

## Appendix A: Raw program output
```text
10-run synthetic sweep (planar calibration)
Image: 1280x720, points/view=54, views/run=10, noise=0.500000 px

run |   fx_gt   fy_gt    cx_gt    cy_gt |  rmse_floor  rmse_init  rmse_final
----+------------------------------------+---------------------------------
  1 | 699.427868 722.376867 627.802520 371.917685 |   0.499879  1.880143   0.488280
  2 | 635.225330 618.759141 679.411632 370.223325 |   0.498705  1.432179   0.489906
  3 | 918.051448 959.025239 617.164855 335.916313 |   0.477721  1.024511   0.462174
  4 | 1130.937002 1116.598772 615.743967 348.956099 |   0.517007  1.891883   0.525200
  5 | 824.799042 775.643657 674.649753 386.541794 |   0.505165  2.936032   1.793087
  6 | 780.225451 820.916882 666.070972 381.686013 |   0.500543  1.779568   0.484461
  7 | 802.073590 801.517002 633.219712 330.525279 |   0.511146  1.880217   0.498438
  8 | 652.717093 661.203107 630.510287 378.746499 |   0.495480  1.454675   0.481607
  9 | 1133.450978 1189.851555 614.146245 342.194689 |   0.498011  1.441890   0.529881
 10 | 744.502714 743.926537 689.952642 332.833964 |   0.479267  1.331229   0.465738

Aggregate over 10 runs
RMSE floor: 0.498292 ± 0.012309 px
RMSE init : 1.705233 ± 0.519229 px
RMSE final: 0.621877 ± 0.412107 px
Final/floor: 1.244274 ± 0.810540

Intrinsics recovery (MAE)
fx MAE: 20.657992 px, relative MAE: 0.025017
fy MAE: 21.420484 px, relative MAE: 0.026683
cx MAE: 11.478408 px
cy MAE: 17.577106 px

Distortion recovery (MAE)
k1 MAE: 0.029407
k2 MAE: 0.074758
p1 MAE: 0.001559
p2 MAE: 0.001061
```
