# Transformation Matrices - Floyd_URDF

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | RS02 |
| $L_{2}$ | BellyBase |
| $L_{3}$ | FrontBase |
| $L_{4}$ | Back_Base |
| $L_{5}$ | RS03_LeftHipRoll |
| $L_{6}$ | RS03_RightHipRoll |
| $L_{7}$ | FrontPanel |
| $L_{8}$ | LowerPanel |
| $L_{9}$ | TopBase |
| $L_{10}$ | Tail_Base |
| $L_{11}$ | Battery_Mount |
| $L_{12}$ | HipRollMountLeft |
| $L_{13}$ | HipRollMountRight |
| $L_{14}$ | TopRightPanel |
| $L_{15}$ | TopLeftPanel |
| $L_{16}$ | Battery |
| $L_{17}$ | RS03_LeftHipPitch |
| $L_{18}$ | RS03_RightHipPitch |
| $L_{19}$ | ThighMountFemaleLeft |
| $L_{20}$ | ThighMountFemaleRight |
| $L_{21}$ | ThighMountMaleLeft |
| $L_{22}$ | RS03_LeftKneePitch |
| $L_{23}$ | ThighMountMaleRight |
| $L_{24}$ | RS03_RightKneePitch |
| $L_{25}$ | LowerLegLeft |
| $L_{26}$ | LowerLegRight |
| $L_{27}$ | RS02_LeftAnklePitch |
| $L_{28}$ | RS02_RightAnklePitch |
| $L_{29}$ | FootBaseLeft |
| $L_{30}$ | FootBaseRight |
| $L_{31}$ | FootCoverLeft |
| $L_{32}$ | FootPadLeft |
| $L_{33}$ | FootCoverRight |
| $L_{34}$ | FootPadRight |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | LeftHipRoll | continuous (rad) | $L_{5}$ | $L_{12}$ |
| $q_{2}$ | RightHipRoll | continuous (rad) | $L_{6}$ | $L_{13}$ |
| $q_{3}$ | LeftHipPitch | continuous (rad) | $L_{12}$ | $L_{17}$ |
| $q_{4}$ | RightHipPitch | continuous (rad) | $L_{13}$ | $L_{18}$ |
| $q_{5}$ | LeftKneePitch | continuous (rad) | $L_{22}$ | $L_{25}$ |
| $q_{6}$ | RightKneePitch | continuous (rad) | $L_{24}$ | $L_{26}$ |
| $q_{7}$ | LeftAnklePitch | continuous (rad) | $L_{27}$ | $L_{29}$ |
| $q_{8}$ | RightAnklePitch | continuous (rad) | $L_{28}$ | $L_{30}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  |-- [fixed] Rigid_47
  |   L1: RS02
  |-- [fixed] Rigid_48
  |   L2: BellyBase
  |     |-- [fixed] Rigid_58
  |     |   L7: FrontPanel
  |     +-- [fixed] Rigid_63
  |         L8: LowerPanel
  |-- [fixed] Rigid_49
  |   L3: FrontBase
  |     +-- [fixed] Rigid_66
  |         L9: TopBase
  |           |-- [fixed] Rigid_56
  |           |   L14: TopRightPanel
  |           +-- [fixed] Rigid_57
  |               L15: TopLeftPanel
  |-- [fixed] Rigid_50
  |   L4: Back_Base
  |     |-- [fixed] Rigid_51
  |     |   L10: Tail_Base
  |     +-- [fixed] Rigid_52
  |         L11: Battery_Mount
  |           +-- [fixed] Rigid_53
  |               L16: Battery
  |-- [fixed] Rigid_54
  |   L5: RS03_LeftHipRoll
  |     +-- [continuous] LeftHipRoll (q1)
  |         L12: HipRollMountLeft
  |           +-- [continuous] LeftHipPitch (q3)
  |               L17: RS03_LeftHipPitch
  |                 +-- [fixed] Rigid_59
  |                     L19: ThighMountFemaleLeft
  |                       |-- [fixed] Rigid_17
  |                       |   L21: ThighMountMaleLeft
  |                       +-- [fixed] Rigid_64
  |                           L22: RS03_LeftKneePitch
  |                             +-- [continuous] LeftKneePitch (q5)
  |                                 L25: LowerLegLeft
  |                                   +-- [fixed] Rigid_23
  |                                       L27: RS02_LeftAnklePitch
  |                                         +-- [continuous] LeftAnklePitch (q7)
  |                                             L29: FootBaseLeft
  |                                               |-- [fixed] Rigid_21
  |                                               |   L31: FootCoverLeft
  |                                               +-- [fixed] Rigid_25
  |                                                   L32: FootPadLeft
  +-- [fixed] Rigid_55
      L6: RS03_RightHipRoll
        +-- [continuous] RightHipRoll (q2)
            L13: HipRollMountRight
              +-- [continuous] RightHipPitch (q4)
                  L18: RS03_RightHipPitch
                    +-- [fixed] Rigid_61
                        L20: ThighMountFemaleRight
                          |-- [fixed] Rigid_29
                          |   L23: ThighMountMaleRight
                          +-- [fixed] Rigid_65
                              L24: RS03_RightKneePitch
                                +-- [continuous] RightKneePitch (q6)
                                    L26: LowerLegRight
                                      +-- [fixed] Rigid_28
                                          L28: RS02_RightAnklePitch
                                            +-- [continuous] RightAnklePitch (q8)
                                                L30: FootBaseRight
                                                  |-- [fixed] Rigid_45
                                                  |   L33: FootCoverRight
                                                  +-- [fixed] Rigid_46
                                                      L34: FootPadRight
```

## Transforms

## Rigid_47

$L_{0}$ **base_link** -> $L_{1}$ **RS02** (fixed)

- **origin xyz**: (0.01375, 0.200264, 0.01185) m
- **origin rpy**: (-1.570796, 0, -1.570796) rad

### Local Transform

$$
T^{0}_{1} = \begin{bmatrix}
0 & 0 & 1 & 0.01375 \\
-1 & 0 & 0 & 0.200264 \\
0 & -1 & 0 & 0.01185 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_48

$L_{0}$ **base_link** -> $L_{2}$ **BellyBase** (fixed)

- **origin xyz**: (-0.000393, 0.133156, 0.057146) m
- **origin rpy**: (-1.570796, 0, 0) rad

### Local Transform

$$
T^{0}_{2} = \begin{bmatrix}
1 & 0 & 0 & -0.000393 \\
0 & 0 & 1 & 0.133156 \\
0 & -1 & 0 & 0.057146 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_49

$L_{0}$ **base_link** -> $L_{3}$ **FrontBase** (fixed)

- **origin xyz**: (-0.000393, 0.133156, 0.056746) m
- **origin rpy**: (-1.570796, 0, 0) rad

### Local Transform

$$
T^{0}_{3} = \begin{bmatrix}
1 & 0 & 0 & -0.000393 \\
0 & 0 & 1 & 0.133156 \\
0 & -1 & 0 & 0.056746 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_50

$L_{0}$ **base_link** -> $L_{4}$ **Back_Base** (fixed)

- **origin xyz**: (0, 0, 0.00015) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{4} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0.00015 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_54

$L_{0}$ **base_link** -> $L_{5}$ **RS03_LeftHipRoll** (fixed)

- **origin xyz**: (0.05005, 0.102099, -0.090118) m
- **origin rpy**: (0.000001, 0, -1.570796) rad

### Local Transform

$$
T^{0}_{5} = \begin{bmatrix}
0 & 1 & -0.000001 & 0.05005 \\
-1 & 0 & 0 & 0.102099 \\
0 & 0.000001 & 1 & -0.090118 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_55

$L_{0}$ **base_link** -> $L_{6}$ **RS03_RightHipRoll** (fixed)

- **origin xyz**: (-0.04995, 0.102099, -0.090118) m
- **origin rpy**: (-0.000001, 0, 1.570796) rad

### Local Transform

$$
T^{0}_{6} = \begin{bmatrix}
0 & -1 & -0.000001 & -0.04995 \\
1 & 0 & 0 & 0.102099 \\
0 & -0.000001 & 1 & -0.090118 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_58

$L_{2}$ **BellyBase** -> $L_{7}$ **FrontPanel** (fixed)

- **origin xyz**: (0.000443, -0.000104, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{2}_{7} = \begin{bmatrix}
1 & 0 & 0 & 0.000443 \\
0 & 1 & 0 & -0.000104 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_63

$L_{2}$ **BellyBase** -> $L_{8}$ **LowerPanel** (fixed)

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{2}_{8} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_66

$L_{3}$ **FrontBase** -> $L_{9}$ **TopBase** (fixed)

- **origin xyz**: (0, 0, -0.0002) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{3}_{9} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & -0.0002 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_51

$L_{4}$ **Back_Base** -> $L_{10}$ **Tail_Base** (fixed)

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{4}_{10} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_52

$L_{4}$ **Back_Base** -> $L_{11}$ **Battery_Mount** (fixed)

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{4}_{11} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## LeftHipRoll

$L_{5}$ **RS03_LeftHipRoll** -> $L_{12}$ **HipRollMountLeft** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0, 0, 0.0566) m
- **origin rpy**: (3.141593, 0, 1.570796) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{5}_{12}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
1 & 0 & 0 & 0 \\
0 & 0 & -1 & 0.0566 \\
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

$L_{6}$ **RS03_RightHipRoll** -> $L_{13}$ **HipRollMountRight** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0, 0, 0.0566) m
- **origin rpy**: (0, 0, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{6}_{13}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & 0 \\
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0.0566 \\
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

## Rigid_56

$L_{9}$ **TopBase** -> $L_{14}$ **TopRightPanel** (fixed)

- **origin xyz**: (0.000443, -0.000104, 0) m
- **origin rpy**: (-3.141593, 0, 3.141593) rad

### Local Transform

$$
T^{9}_{14} = \begin{bmatrix}
-1 & 0 & 0 & 0.000443 \\
0 & 1 & 0 & -0.000104 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_57

$L_{9}$ **TopBase** -> $L_{15}$ **TopLeftPanel** (fixed)

- **origin xyz**: (0.000443, -0.000104, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{9}_{15} = \begin{bmatrix}
1 & 0 & 0 & 0.000443 \\
0 & 1 & 0 & -0.000104 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_53

$L_{11}$ **Battery_Mount** -> $L_{16}$ **Battery** (fixed)

- **origin xyz**: (0.00005, 0.005043, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{11}_{16} = \begin{bmatrix}
1 & 0 & 0 & 0.00005 \\
0 & 1 & 0 & 0.005043 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## LeftHipPitch

$L_{12}$ **HipRollMountLeft** -> $L_{17}$ **RS03_LeftHipPitch** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0.07125, 0, 0.012093) m
- **origin rpy**: (-1.570796, 0.698132, 1.570796) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{12}_{17}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & 0.07125 \\
0.766044 & -0.642788 & 0 & 0 \\
-0.642788 & -0.766044 & 0 & 0.012093 \\
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

$L_{13}$ **HipRollMountRight** -> $L_{18}$ **RS03_RightHipPitch** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (0.07125, 0, -0.012092) m
- **origin rpy**: (1.570796, 0.698132, -1.570796) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{13}_{18}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & 0.07125 \\
-0.766044 & -0.642788 & 0 & 0 \\
-0.642788 & 0.766044 & 0 & -0.012092 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & s_{4} & 0 & 0 \\
-s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_59

$L_{17}$ **RS03_LeftHipPitch** -> $L_{19}$ **ThighMountFemaleLeft** (fixed)

- **origin xyz**: (-0.058592, 0, 0.07125) m
- **origin rpy**: (0, 1.570796, 0) rad

### Local Transform

$$
T^{17}_{19} = \begin{bmatrix}
0 & 0 & 1 & -0.058592 \\
0 & 1 & 0 & 0 \\
-1 & 0 & 0 & 0.07125 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_61

$L_{18}$ **RS03_RightHipPitch** -> $L_{20}$ **ThighMountFemaleRight** (fixed)

- **origin xyz**: (0.058592, 0, 0.07125) m
- **origin rpy**: (0, 1.570796, 0) rad

### Local Transform

$$
T^{18}_{20} = \begin{bmatrix}
0 & 0 & 1 & 0.058592 \\
0 & 1 & 0 & 0 \\
-1 & 0 & 0 & 0.07125 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_17

$L_{19}$ **ThighMountFemaleLeft** -> $L_{21}$ **ThighMountMaleLeft** (fixed)

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{19}_{21} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_64

$L_{19}$ **ThighMountFemaleLeft** -> $L_{22}$ **RS03_LeftKneePitch** (fixed)

- **origin xyz**: (0.12785, -0.111, 0.058592) m
- **origin rpy**: (0, -1.570796, 0) rad

### Local Transform

$$
T^{19}_{22} = \begin{bmatrix}
0 & 0 & -1 & 0.12785 \\
0 & 1 & 0 & -0.111 \\
1 & 0 & 0 & 0.058592 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_29

$L_{20}$ **ThighMountFemaleRight** -> $L_{23}$ **ThighMountMaleRight** (fixed)

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{20}_{23} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_65

$L_{20}$ **ThighMountFemaleRight** -> $L_{24}$ **RS03_RightKneePitch** (fixed)

- **origin xyz**: (0.12785, -0.111, -0.058593) m
- **origin rpy**: (0, 1.570796, 0) rad

### Local Transform

$$
T^{20}_{24} = \begin{bmatrix}
0 & 0 & 1 & 0.12785 \\
0 & 1 & 0 & -0.111 \\
-1 & 0 & 0 & -0.058593 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## LeftKneePitch

$L_{22}$ **RS03_LeftKneePitch** -> $L_{25}$ **LowerLegLeft** (continuous)
  Variable: $q_{5}$

- **origin xyz**: (0, 0, 0.0566) m
- **origin rpy**: (-1.047198, 1.570796, 0) rad
- **axis**: (-1, 0, 0)

### Local Transform

$T^{22}_{25}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.866025 & 0.5 & 0 \\
0 & 0.5 & 0.866025 & 0 \\
-1 & 0 & 0 & 0.0566 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{5} & s_{5} & 0 \\
0 & -s_{5} & c_{5} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## RightKneePitch

$L_{24}$ **RS03_RightKneePitch** -> $L_{26}$ **LowerLegRight** (continuous)
  Variable: $q_{6}$

- **origin xyz**: (0, 0, -0.0566) m
- **origin rpy**: (1.047198, -1.570796, 0) rad
- **axis**: (1, 0, 0)

### Local Transform

$T^{24}_{26}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.866025 & -0.5 & 0 \\
0 & 0.5 & -0.866025 & 0 \\
1 & 0 & 0 & -0.0566 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & c_{6} & -s_{6} & 0 \\
0 & s_{6} & c_{6} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_23

$L_{25}$ **LowerLegLeft** -> $L_{27}$ **RS02_LeftAnklePitch** (fixed)

- **origin xyz**: (-0.0015, -0.140028, -0.000013) m
- **origin rpy**: (-1.570796, -0.261799, -1.570796) rad

### Local Transform

$$
T^{25}_{27} = \begin{bmatrix}
0 & 0 & 1 & -0.0015 \\
-0.965926 & -0.258819 & 0 & -0.140028 \\
0.258819 & -0.965926 & 0 & -0.000013 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_28

$L_{26}$ **LowerLegRight** -> $L_{28}$ **RS02_RightAnklePitch** (fixed)

- **origin xyz**: (-0.0015, -0.140028, 0.000013) m
- **origin rpy**: (1.570796, 0.261799, -1.570796) rad

### Local Transform

$$
T^{26}_{28} = \begin{bmatrix}
0 & 0 & -1 & -0.0015 \\
-0.965926 & -0.258819 & 0 & -0.140028 \\
-0.258819 & 0.965926 & 0 & 0.000013 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## LeftAnklePitch

$L_{27}$ **RS02_LeftAnklePitch** -> $L_{29}$ **FootBaseLeft** (continuous)
  Variable: $q_{7}$

- **origin xyz**: (0, 0, 0.004) m
- **origin rpy**: (0.436332, -1.570796, 0) rad
- **axis**: (1, 0, 0)

### Local Transform

$T^{27}_{29}(q_{7}) = T_{fixed} \cdot R_{axis}(q_{7})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.422618 & -0.906308 & 0 \\
0 & 0.906308 & -0.422618 & 0 \\
1 & 0 & 0 & 0.004 \\
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

$L_{28}$ **RS02_RightAnklePitch** -> $L_{30}$ **FootBaseRight** (continuous)
  Variable: $q_{8}$

- **origin xyz**: (0, 0, -0.004) m
- **origin rpy**: (-0.436332, 1.570796, 0) rad
- **axis**: (-1, 0, 0)

### Local Transform

$T^{28}_{30}(q_{8}) = T_{fixed} \cdot R_{axis}(q_{8})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.422618 & 0.906308 & 0 \\
0 & 0.906308 & 0.422618 & 0 \\
-1 & 0 & 0 & -0.004 \\
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

## Rigid_21

$L_{29}$ **FootBaseLeft** -> $L_{31}$ **FootCoverLeft** (fixed)

- **origin xyz**: (-0.118071, -0.137704, 0.226786) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{29}_{31} = \begin{bmatrix}
1 & 0 & 0 & -0.118071 \\
0 & 1 & 0 & -0.137704 \\
0 & 0 & 1 & 0.226786 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_25

$L_{29}$ **FootBaseLeft** -> $L_{32}$ **FootPadLeft** (fixed)

- **origin xyz**: (-0.118071, -0.138154, 0.226886) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{29}_{32} = \begin{bmatrix}
1 & 0 & 0 & -0.118071 \\
0 & 1 & 0 & -0.138154 \\
0 & 0 & 1 & 0.226886 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_45

$L_{30}$ **FootBaseRight** -> $L_{33}$ **FootCoverRight** (fixed)

- **origin xyz**: (-0.118071, -0.137704, -0.226786) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{30}_{33} = \begin{bmatrix}
1 & 0 & 0 & -0.118071 \\
0 & 1 & 0 & -0.137704 \\
0 & 0 & 1 & -0.226786 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_46

$L_{30}$ **FootBaseRight** -> $L_{34}$ **FootPadRight** (fixed)

- **origin xyz**: (-0.118071, -0.138154, -0.226886) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{30}_{34} = \begin{bmatrix}
1 & 0 & 0 & -0.118071 \\
0 & 1 & 0 & -0.138154 \\
0 & 0 & 1 & -0.226886 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{7} = T^{0}_{2} \cdot T^{2}_{7}\quad (L_0 \to L_{7}: \text{FrontPanel})$$

$$T^{0}_{8} = T^{0}_{2} \cdot T^{2}_{8}\quad (L_0 \to L_{8}: \text{LowerPanel})$$

$$T^{0}_{9} = T^{0}_{3} \cdot T^{3}_{9}\quad (L_0 \to L_{9}: \text{TopBase})$$

$$T^{0}_{10} = T^{0}_{4} \cdot T^{4}_{10}\quad (L_0 \to L_{10}: \text{Tail_Base})$$

$$T^{0}_{11} = T^{0}_{4} \cdot T^{4}_{11}\quad (L_0 \to L_{11}: \text{Battery_Mount})$$

$$T^{0}_{12} = T^{0}_{5} \cdot T^{5}_{12}(q_{1})\quad (L_0 \to L_{12}: \text{HipRollMountLeft})$$

$$T^{0}_{13} = T^{0}_{6} \cdot T^{6}_{13}(q_{2})\quad (L_0 \to L_{13}: \text{HipRollMountRight})$$

$$T^{0}_{14} = T^{0}_{3} \cdot T^{3}_{9} \cdot T^{9}_{14}\quad (L_0 \to L_{14}: \text{TopRightPanel})$$

$$T^{0}_{15} = T^{0}_{3} \cdot T^{3}_{9} \cdot T^{9}_{15}\quad (L_0 \to L_{15}: \text{TopLeftPanel})$$

$$T^{0}_{16} = T^{0}_{4} \cdot T^{4}_{11} \cdot T^{11}_{16}\quad (L_0 \to L_{16}: \text{Battery})$$

$$T^{0}_{17} = T^{0}_{5} \cdot T^{5}_{12}(q_{1}) \cdot T^{12}_{17}(q_{3})\quad (L_0 \to L_{17}: \text{RS03_LeftHipPitch})$$

$$T^{0}_{18} = T^{0}_{6} \cdot T^{6}_{13}(q_{2}) \cdot T^{13}_{18}(q_{4})\quad (L_0 \to L_{18}: \text{RS03_RightHipPitch})$$

$$T^{0}_{19} = T^{0}_{5} \cdot T^{5}_{12}(q_{1}) \cdot T^{12}_{17}(q_{3}) \cdot T^{17}_{19}\quad (L_0 \to L_{19}: \text{ThighMountFemaleLeft})$$

$$T^{0}_{20} = T^{0}_{6} \cdot T^{6}_{13}(q_{2}) \cdot T^{13}_{18}(q_{4}) \cdot T^{18}_{20}\quad (L_0 \to L_{20}: \text{ThighMountFemaleRight})$$

$$T^{0}_{21} = T^{0}_{5} \cdot T^{5}_{12}(q_{1}) \cdot T^{12}_{17}(q_{3}) \cdot T^{17}_{19} \cdot T^{19}_{21}\quad (L_0 \to L_{21}: \text{ThighMountMaleLeft})$$

$$T^{0}_{22} = T^{0}_{5} \cdot T^{5}_{12}(q_{1}) \cdot T^{12}_{17}(q_{3}) \cdot T^{17}_{19} \cdot T^{19}_{22}\quad (L_0 \to L_{22}: \text{RS03_LeftKneePitch})$$

$$T^{0}_{23} = T^{0}_{6} \cdot T^{6}_{13}(q_{2}) \cdot T^{13}_{18}(q_{4}) \cdot T^{18}_{20} \cdot T^{20}_{23}\quad (L_0 \to L_{23}: \text{ThighMountMaleRight})$$

$$T^{0}_{24} = T^{0}_{6} \cdot T^{6}_{13}(q_{2}) \cdot T^{13}_{18}(q_{4}) \cdot T^{18}_{20} \cdot T^{20}_{24}\quad (L_0 \to L_{24}: \text{RS03_RightKneePitch})$$

$$T^{0}_{25} = T^{0}_{5} \cdot T^{5}_{12}(q_{1}) \cdot T^{12}_{17}(q_{3}) \cdot T^{17}_{19} \cdot T^{19}_{22} \cdot T^{22}_{25}(q_{5})\quad (L_0 \to L_{25}: \text{LowerLegLeft})$$

$$T^{0}_{26} = T^{0}_{6} \cdot T^{6}_{13}(q_{2}) \cdot T^{13}_{18}(q_{4}) \cdot T^{18}_{20} \cdot T^{20}_{24} \cdot T^{24}_{26}(q_{6})\quad (L_0 \to L_{26}: \text{LowerLegRight})$$

$$T^{0}_{27} = T^{0}_{5} \cdot T^{5}_{12}(q_{1}) \cdot T^{12}_{17}(q_{3}) \cdot T^{17}_{19} \cdot T^{19}_{22} \cdot T^{22}_{25}(q_{5}) \cdot T^{25}_{27}\quad (L_0 \to L_{27}: \text{RS02_LeftAnklePitch})$$

$$T^{0}_{28} = T^{0}_{6} \cdot T^{6}_{13}(q_{2}) \cdot T^{13}_{18}(q_{4}) \cdot T^{18}_{20} \cdot T^{20}_{24} \cdot T^{24}_{26}(q_{6}) \cdot T^{26}_{28}\quad (L_0 \to L_{28}: \text{RS02_RightAnklePitch})$$

$$T^{0}_{29} = T^{0}_{5} \cdot T^{5}_{12}(q_{1}) \cdot T^{12}_{17}(q_{3}) \cdot T^{17}_{19} \cdot T^{19}_{22} \cdot T^{22}_{25}(q_{5}) \cdot T^{25}_{27} \cdot T^{27}_{29}(q_{7})\quad (L_0 \to L_{29}: \text{FootBaseLeft})$$

$$T^{0}_{30} = T^{0}_{6} \cdot T^{6}_{13}(q_{2}) \cdot T^{13}_{18}(q_{4}) \cdot T^{18}_{20} \cdot T^{20}_{24} \cdot T^{24}_{26}(q_{6}) \cdot T^{26}_{28} \cdot T^{28}_{30}(q_{8})\quad (L_0 \to L_{30}: \text{FootBaseRight})$$

$$T^{0}_{31} = T^{0}_{5} \cdot T^{5}_{12}(q_{1}) \cdot T^{12}_{17}(q_{3}) \cdot T^{17}_{19} \cdot T^{19}_{22} \cdot T^{22}_{25}(q_{5}) \cdot T^{25}_{27} \cdot T^{27}_{29}(q_{7}) \cdot T^{29}_{31}\quad (L_0 \to L_{31}: \text{FootCoverLeft})$$

$$T^{0}_{32} = T^{0}_{5} \cdot T^{5}_{12}(q_{1}) \cdot T^{12}_{17}(q_{3}) \cdot T^{17}_{19} \cdot T^{19}_{22} \cdot T^{22}_{25}(q_{5}) \cdot T^{25}_{27} \cdot T^{27}_{29}(q_{7}) \cdot T^{29}_{32}\quad (L_0 \to L_{32}: \text{FootPadLeft})$$

$$T^{0}_{33} = T^{0}_{6} \cdot T^{6}_{13}(q_{2}) \cdot T^{13}_{18}(q_{4}) \cdot T^{18}_{20} \cdot T^{20}_{24} \cdot T^{24}_{26}(q_{6}) \cdot T^{26}_{28} \cdot T^{28}_{30}(q_{8}) \cdot T^{30}_{33}\quad (L_0 \to L_{33}: \text{FootCoverRight})$$

$$T^{0}_{34} = T^{0}_{6} \cdot T^{6}_{13}(q_{2}) \cdot T^{13}_{18}(q_{4}) \cdot T^{18}_{20} \cdot T^{20}_{24} \cdot T^{24}_{26}(q_{6}) \cdot T^{26}_{28} \cdot T^{28}_{30}(q_{8}) \cdot T^{30}_{34}\quad (L_0 \to L_{34}: \text{FootPadRight})$$

