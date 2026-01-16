# Modern Python features used in the calibration script

## Postponed evaluation of annotations (`from __future__ import annotations`)
This enables type annotations to be treated as “not immediately evaluated,” which prevents forward-reference issues and reduces forward-reference problems. In the script, it lets classes and functions refer to types like `Intrinsics`, `Extrinsics`, and `FloatArray` cleanly throughout the file without worrying about definition order.

```python
from __future__ import annotations
```

---

## `dataclass` with `slots=True`
`@dataclass` removes repetitive boilerplate (writing `__init__`, `__repr__`, etc.) while keeping the code readable. Adding `slots=True` makes instances more memory-efficient and typically faster for attribute access by preventing dynamic attribute creation.

```python
@dataclass(slots=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    skew: float = 0.0
```

---

## Modern typing with `TypeAlias`, `Final`, and NumPy typing
`Final` marks constants that are intended not to change, making intent clearer. `TypeAlias` gives a readable name to a complex type (here, “a NumPy float64 array”), so function signatures stay understandable. `numpy.typing` is used so arrays are typed in a way that aligns with NumPy.

```python
from typing import Final, TypeAlias
import numpy.typing as npt

DTYPE: Final = np.float64
EPS: Final = 1e-12

FloatArray: TypeAlias = npt.NDArray[np.float64]
```

---

## Precise return types using `tuple[...]` and `list[...]` generics
Instead of older `Tuple`/`List` from `typing`, modern Python uses built-in generics like `tuple[...]` and `list[...]`. This makes signatures more direct and requires fewer imports. You see this in functions that return multiple values and in lists of poses or views.

```python
def normalize_2d(pts: FloatArray) -> tuple[FloatArray, FloatArray]:
    ...
    return q2, T

def unpack_params(p: FloatArray, M: int) -> tuple[Intrinsics, Distortion, list[Extrinsics]]:
    ...
    return K, D, extr
```

---

## Type-aware local variables (“typed locals”)
Variables are annotated inside functions to make the code self-documenting and easier to follow. This is not required in Python, but it helps readers and tools (linters, static analyzers) understand what shape/type each value should have.

```python
theta: float = float(np.linalg.norm(rvec))
K_gt: Intrinsics = Intrinsics(fx=820.0, fy=800.0, cx=640.0, cy=360.0)
extr_gt: list[Extrinsics] = []
```

---

## Structured, explicit program entry and exit (`main() -> int` + `SystemExit`)
Rather than executing logic directly at module level, the code uses a `main()` function returning an exit code, and then exits explicitly via `SystemExit`. This is a clean style borrowed from larger Python applications and makes the module easier to import without side effects.

```python
def main() -> int:
    ...
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
```

---

## Boolean masks with NumPy and typed mask creation
The optimizer uses boolean masks to activate subsets of parameters in phases. The code uses an inner helper that takes an `Iterable[int]`, builds a `bool` NumPy array, and returns it. This is a modern and expressive approach to “feature gating” without complex conditionals.

```python
def make_mask(idxs: Iterable[int]) -> npt.NDArray[np.bool_]:
    m: npt.NDArray[np.bool_] = np.zeros(P, dtype=bool)
    m[list(idxs)] = True
    return m
```

---

## Iterable unpacking (`[*a, *b]`) to build index lists
Instead of concatenating lists with `+` repeatedly, the code uses iterable unpacking to assemble stage masks in a readable way. This is idiomatic modern Python and keeps the mask definitions compact.

```python
stage0: npt.NDArray[np.bool_] = make_mask([*intr_idx, *pose_idx])
stage1: npt.NDArray[np.bool_] = make_mask([*intr_idx, *rad_idx, *pose_idx])
stage2: npt.NDArray[np.bool_] = make_mask([*intr_idx, *rad_idx, *tan_idx, *pose_idx])
```

---

## Cleaner paired iteration with `zip`
When iterating over per-view observations and corresponding poses, `zip` makes the relationship explicit and avoids indexing errors. This improves readability and reduces boilerplate.

```python
for obs, E in zip(image_pts_views, extr_list):
    pred: FloatArray = project_points(object_pts, K, D, E)
    diff: FloatArray = (np.asarray(obs, dtype=DTYPE) - pred).astype(DTYPE, copy=False)
```

---

## Vectorization to reduce Python loops
A major “modern Python” practice in numerical code is pushing work into NumPy operations instead of looping point-by-point in Python. The newer version projects all points for a view at once and applies distortion to all points in a vectorized way.

```python
Xc: FloatArray = (object_pts @ R.T + E.tvec.reshape(1, 3)).astype(DTYPE, copy=False)
xy: FloatArray = (Xc[:, :2] / z_safe).astype(DTYPE, copy=False)
xy_d: FloatArray = distort_normalized_xy(xy, D)
```

---

## Careful dtype control (`astype(..., copy=False)`)
The script forces float64 consistently and frequently uses `copy=False` to avoid unnecessary allocations where NumPy can reuse memory. This is a performance- and stability-oriented habit common in production numerical Python.

```python
Xc: FloatArray = (object_pts @ R.T + E.tvec.reshape(1, 3)).astype(DTYPE, copy=False)
J: FloatArray = np.zeros((r0.size, p.size), dtype=DTYPE)
```

---

## Exception-based fallback pattern for numerical robustness
When solving linear systems during LM, the code tries a direct solve first, and falls back to a least-squares solution if the system is ill-conditioned. This is a practical robustness pattern for numerical optimization code.

```python
try:
    dp: FloatArray = np.linalg.solve(A, -g).astype(DTYPE, copy=False)
except np.linalg.LinAlgError:
    dp = np.linalg.lstsq(A, -g, rcond=None)[0].astype(DTYPE, copy=False)
```

---

## Small, local “utility” definitions (e.g., formatting lambda)
The code uses a small lambda for formatting numeric outputs. This keeps printing concise without introducing a separate function that would only be used once.

```python
fmt = lambda x: f"{float(x): .6f}"
```
