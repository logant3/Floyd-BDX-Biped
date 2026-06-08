# Transformation Matrices - Floyd_URDF

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | HipRollMountLeft |
| $L_{2}$ | HipRollMountRight |
| $L_{3}$ | RS03_LeftHipPitch |
| $L_{4}$ | RS03_RightKneePitch |
| $L_{5}$ | RS02_LeftAnklePitch |
| $L_{6}$ | RS02_RightAnklePitch |
| $L_{7}$ | FootPadLeft |
| $L_{8}$ | FootBaseRight |
| $L_{9}$ | FootCoverRight |
| $L_{10}$ | FootPadRight |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | LeftHipRoll | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | RightHipRoll | continuous (rad) | $L_{0}$ | $L_{2}$ |
| $q_{3}$ | LeftHipPitch | continuous (rad) | $L_{1}$ | $L_{3}$ |
| $q_{4}$ | RightHipPitch | continuous (rad) | $L_{2}$ | $L_{4}$ |
| $q_{5}$ | LeftKneePitch | continuous (rad) | $L_{3}$ | $L_{5}$ |
| $q_{6}$ | RightKneePitch | continuous (rad) | $L_{4}$ | $L_{6}$ |
| $q_{7}$ | LeftAnklePitch | continuous (rad) | $L_{5}$ | $L_{7}$ |
| $q_{8}$ | RightAnklePitch | continuous (rad) | $L_{6}$ | $L_{8}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  |-- [continuous] LeftHipRoll (q1)
  |   L1: HipRollMountLeft
  |     +-- [continuous] LeftHipPitch (q3)
  |         L3: RS03_LeftHipPitch
  |           +-- [continuous] LeftKneePitch (q5)
  |               L5: RS02_LeftAnklePitch
  |                 +-- [continuous] LeftAnklePitch (q7)
  |                     L7: FootPadLeft
  +-- [continuous] RightHipRoll (q2)
      L2: HipRollMountRight
        +-- [continuous] RightHipPitch (q4)
            L4: RS03_RightKneePitch
              +-- [continuous] RightKneePitch (q6)
                  L6: RS02_RightAnklePitch
                    +-- [continuous] RightAnklePitch (q8)
                        L8: FootBaseRight
                          |-- [fixed] Rigid_45
                          |   L9: FootCoverRight
                          +-- [fixed] Rigid_46
                              L10: FootPadRight
```

## Transforms

## LeftHipRoll

$L_{0}$ **base_link** -> $L_{1}$ **HipRollMountLeft** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0.098165, 0.045368, 0.0363) m
- **origin rpy**: (0, -1.570795, -1.570796) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 1 & 0 & 0.098165 \\
-0.000001 & 0 & 1 & 0.045368 \\
1 & 0 & 0.000001 & 0.0363 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & s_{1} & 0 & 0 \\
-s_{1} & c_{1} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## RightHipRoll

$L_{0}$ **base_link** -> $L_{2}$ **HipRollMountRight** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0.098165, 0.045368, -0.0637) m
- **origin rpy**: (-3.141593, 1.570795, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 1 & 0 & 0.098165 \\
0.000001 & 0 & -1 & 0.045368 \\
-1 & 0 & -0.000001 & -0.0637 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & -s_{2} & 0 & 0 \\
s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## LeftHipPitch

$L_{1}$ **HipRollMountLeft** -> $L_{3}$ **RS03_LeftHipPitch** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0.07125, 0, 0.012092) m
- **origin rpy**: (-1.570796, 0.698132, 1.570796) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{1}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & 0.07125 \\
0.766044 & -0.642788 & 0 & 0 \\
-0.642788 & -0.766044 & 0 & 0.012092 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & s_{3} & 0 & 0 \\
-s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## RightHipPitch

$L_{2}$ **HipRollMountRight** -> $L_{4}$ **RS03_RightKneePitch** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (0.07125, 0, -0.012092) m
- **origin rpy**: (1.570796, -0.698132, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{2}_{4}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & 1 & 0.07125 \\
0.766044 & -0.642788 & 0 & 0 \\
0.642788 & 0.766044 & 0 & -0.012092 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & -s_{4} & 0 & 0 \\
s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## LeftKneePitch

$L_{3}$ **RS03_LeftHipPitch** -> $L_{5}$ **RS02_LeftAnklePitch** (continuous)
  Variable: $q_{5}$

- **origin xyz**: (0, -0.111, 0) m
- **origin rpy**: (3.141593, 0, -0.261799) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{3}_{5}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.965926 & -0.258819 & 0 & 0 \\
-0.258819 & -0.965926 & 0 & -0.111 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
c_{5} & s_{5} & 0 & 0 \\
-s_{5} & c_{5} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## RightKneePitch

$L_{4}$ **RS03_RightKneePitch** -> $L_{6}$ **RS02_RightAnklePitch** (continuous)
  Variable: $q_{6}$

- **origin xyz**: (0, -0.111, 0) m
- **origin rpy**: (-3.141593, 0, -0.261799) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{4}_{6}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.965926 & -0.258819 & 0 & 0 \\
-0.258819 & -0.965926 & 0 & -0.111 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
c_{6} & s_{6} & 0 & 0 \\
-s_{6} & c_{6} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## LeftAnklePitch

$L_{5}$ **RS02_LeftAnklePitch** -> $L_{7}$ **FootPadLeft** (continuous)
  Variable: $q_{7}$

- **origin xyz**: (0.135254, 0.036254, 0.0025) m
- **origin rpy**: (0.436332, -1.570796, 0) rad
- **axis**: (1, 0, 0)

### Local Transform

$T^{5}_{7}(q_{7}) = T_{fixed} \cdot R_{axis}(q_{7})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.422618 & -0.906308 & 0.135254 \\
0 & 0.906308 & -0.422618 & 0.036254 \\
1 & 0 & 0 & 0.0025 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{7}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{7} & -s_{7} & 0 \\
0 & s_{7} & c_{7} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## RightAnklePitch

$L_{6}$ **RS02_RightAnklePitch** -> $L_{8}$ **FootBaseRight** (continuous)
  Variable: $q_{8}$

- **origin xyz**: (0.135254, 0.036254, -0.0025) m
- **origin rpy**: (-0.436332, 1.570796, 0) rad
- **axis**: (-1, 0, 0)

### Local Transform

$T^{6}_{8}(q_{8}) = T_{fixed} \cdot R_{axis}(q_{8})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.422618 & 0.906308 & 0.135254 \\
0 & 0.906308 & 0.422618 & 0.036254 \\
-1 & 0 & 0 & -0.0025 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{8}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{8} & s_{8} & 0 \\
0 & -s_{8} & c_{8} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_45

$L_{8}$ **FootBaseRight** -> $L_{9}$ **FootCoverRight** (fixed)

- **origin xyz**: (-0.118071, -0.137704, -0.226786) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{8}_{9} = \begin{bmatrix}
1 & 0 & 0 & -0.118071 \\
0 & 1 & 0 & -0.137704 \\
0 & 0 & 1 & -0.226786 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_46

$L_{8}$ **FootBaseRight** -> $L_{10}$ **FootPadRight** (fixed)

- **origin xyz**: (-0.118071, -0.138154, -0.226886) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{8}_{10} = \begin{bmatrix}
1 & 0 & 0 & -0.118071 \\
0 & 1 & 0 & -0.138154 \\
0 & 0 & 1 & -0.226886 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{3})\quad (L_0 \to L_{3}: \text{RS03_LeftHipPitch})$$

$$T^{0}_{4} = T^{0}_{2}(q_{2}) \cdot T^{2}_{4}(q_{4})\quad (L_0 \to L_{4}: \text{RS03_RightKneePitch})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{3}) \cdot T^{3}_{5}(q_{5})\quad (L_0 \to L_{5}: \text{RS02_LeftAnklePitch})$$

$$T^{0}_{6} = T^{0}_{2}(q_{2}) \cdot T^{2}_{4}(q_{4}) \cdot T^{4}_{6}(q_{6})\quad (L_0 \to L_{6}: \text{RS02_RightAnklePitch})$$

$$T^{0}_{7} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{3}) \cdot T^{3}_{5}(q_{5}) \cdot T^{5}_{7}(q_{7})\quad (L_0 \to L_{7}: \text{FootPadLeft})$$

$$T^{0}_{8} = T^{0}_{2}(q_{2}) \cdot T^{2}_{4}(q_{4}) \cdot T^{4}_{6}(q_{6}) \cdot T^{6}_{8}(q_{8})\quad (L_0 \to L_{8}: \text{FootBaseRight})$$

$$T^{0}_{9} = T^{0}_{2}(q_{2}) \cdot T^{2}_{4}(q_{4}) \cdot T^{4}_{6}(q_{6}) \cdot T^{6}_{8}(q_{8}) \cdot T^{8}_{9}\quad (L_0 \to L_{9}: \text{FootCoverRight})$$

$$T^{0}_{10} = T^{0}_{2}(q_{2}) \cdot T^{2}_{4}(q_{4}) \cdot T^{4}_{6}(q_{6}) \cdot T^{6}_{8}(q_{8}) \cdot T^{8}_{10}\quad (L_0 \to L_{10}: \text{FootPadRight})$$

