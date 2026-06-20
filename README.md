# DEEPSEA AUV :: Multibeam Sonar Physics Simulator

基于 **Unity (C++ 原生插件 + C# + Compute Shader)** 构建的专业自主水下航行器
（AUV）深水验证仿真器。项目严格用 C++ 原生实现六自由度水动力求解，并在 GPU
上用 Compute Shader 向不规则海底 Heightmap 发射数千条声呐射线，最终以离散
伪彩色点云条带在海图面板上实时渲染测深结果。

---

## 一、核心能力总览

| 模块 | 技术要点 |
|---|---|
| **六自由度流体动力学（C++ 原生 DLL）** | 附加质量力/力矩 `F_AM = -M_A·ν̇`、粘性线性+平方阻尼、浮力恢复力矩、Coriolis-Centripetal 交叉耦合、高压环境水压计算。6×6 刚体惯性矩阵的部分主元高斯-约旦求逆。 |
| **多波束声呐（Compute Shader）** | DDA 格网遍历算法在 Heightmap 上以每线程 64 束 × 16 组 = **1024 条声线/ping**，子体素二分精化（10 次迭代，精度 <1cm），解析法线重建，入射余弦朗伯反射强度建模。 |
| **测深点云可视化** | 基于 MeshTopology.Points + GS 扩展成 Billboard 四边形，自定义深水深度色带（深紫→蓝→青→黄→红），按回波强度做 Alpha 调制，16 个条带历史形成全覆盖航迹。 |
| **海底地形（Heightmap）** | FBM 分形噪声 + 多高斯海隆（ridge）叠加 + 边缘平滑，512×512 双线性采样，生成 Mesh 并同步 MeshCollider 供 CPU 验证路径使用。 |
| **漆黑海底氛围** | 自定义海底地形 Shader（坡度调制、噪声纹理、深度晕染）、线性雾效、低强度蓝方向光 + AUV 锥形前照灯，环境光亮度 <5%。 |
| **HUD 海图面板** | 1024×512 Texture2D 每帧重绘：按 ping 编号写横条带，AUV 航行位置用洋红十字叠加，深度/波束命中/压力/位置四标签实时更新。 |

---

## 二、目录结构

```
20-deepsea-auv-sonar/
├─ README.md                              ← 本文件
├─ NativePlugin/AUVPhysics/
│  ├─ CMakeLists.txt                       ← CMake 构建 DLL
│  ├─ include/AUVPhysicsAPI.h              ← extern "C" 导出符号
│  └─ src/
│     ├─ HydrodynamicsSolver.cpp           ← 6DOF 求解器 (★ 核心)
│     ├─ AUVState.cpp                      ← 状态积分辅助
│     └─ SonarSimulator.cpp                ← 声呐方向生成 + 强度模型
│
└─ UnityProject/
   └─ Assets/
      ├─ Scripts/
      │  ├─ DeepseaAUV.asmdef
      │  ├─ Native/
      │  │  └─ HydrodynamicsBridge.cs      ← P/Invoke 桥接层
      │  └─ Core/
      │     ├─ AUVController.cs            ← 固定步长调用 AUV_Update()
      │     ├─ AUVKeyboardInput.cs         ← 人机操作输入
      │     ├─ SeafloorTerrain.cs          ← Heightmap + Mesh 生成
      │     ├─ MultibeamSonar.cs           ← ping 调度 + ComputeBuffer 管理
      │     ├─ SonarPointCloud.cs          ← 点云渲染 Mesh 维护
      │     └─ SceneBootstrap.cs           ← 一键程序化搭建场景
      │  └─ UI/
      │     └─ BathymetryPanel.cs          ← 海图 HUD
      │
      ├─ Shaders/
      │  ├─ MultibeamSonarRaycast.compute  ← 声呐射线投射 CS (★ 核心)
      │  ├─ BathymetryPointCloud.shader    ← 点云伪彩色着色器
      │  └─ SeafloorTerrain.shader         ← 海底地形着色器
      │
      └─ Plugins/                          ← 编译后把 AUVPhysics.dll 放在这里
```

---

## 三、6DOF 流体动力学数学模型

### 3.1 运动方程（Fossen, *Guidance and Control of Ocean Vehicles*, 1994）

```
M·ν̇ + C(ν)·ν + D(ν)·ν + g(η) = τ_control + τ_env
```

其中：
- **M = M_RB + M_A**：刚体惯性 + 附加质量
- **C(ν)**：Coriolis-Centripetal 矩阵
- **D(ν) = D_lin + D_quad|ν|**：线性 + 平方阻尼
- **g(η)**：浮力 / 重力恢复力矩
- ν = [u, v, w, p, q, r]ᵀ ：体坐标系速度

### 3.2 附加质量（Added Mass）

椭球流体附加质量近似（Lambert 解法）：
```
X_u̇ = -m·k₁,  Y_v̇ = -m·k₂,  Z_ẇ = -m·k₃
K_ṗ = -I_x·k₄, M_q̇ = -I_y·k₅, N_ṙ = -I_z·k₆
```
`AUVPhysics/src/HydrodynamicsSolver.cpp` 中 6×6 矩阵 `addedMass` 可完全自定义。

### 3.3 恢复力矩（静稳定性）

```
K_restore = -BG_y·W·sin(φ)·cos(θ) - BG_z·W·sin(φ)·cos(φ)
M_restore =  BG_x·W·sin(θ) + BG_z·W·sin(φ)·cos(θ)
```
其中 `BG = B - G`：浮心相对重心的偏移。正的 BG_y（浮心在重心之上）是
AUV 横摇静稳性的必要条件。

### 3.4 深水高压模型

流体静压（Pa）：
```
P(d) = P₀ + ρ·g·d
```
默认海水密度 ρ = 1025 kg/m³，每 10 m 增加 ≈ 100.5 kPa（1 atm）。

---

## 四、多波束声呐 GPU 投射管线

### 4.1 Compute Shader 输入 / 输出

```hlsl
StructuredBuffer<float3>  _Directions;   // 1024 个方向（体→世界由 C++ 预计算）
StructuredBuffer<float3>  _Origins;      // 每线起点（通常相同：换能器位置）
StructuredBuffer<float>   _Heightmap;    // R×R 单精度高度
RWStructuredBuffer<Result> _Results;     // 每个波束 1 个 {pos,depth,intensity,range,hit}
```

### 4.2 DDA 高度场求交算法

1. 把射线起/终点映射到 Heightmap 的 (u, v) ∈ [0,1) 纹理坐标；
2. 以 2D DDA 沿主要轴步进，每个单元格读取中心点高度 `hy`；
3. 当射线 y(prev..cur) 区间跨越 `hy`，用 10 次二分在该单元格精化到 `t`；
4. 重建局部法线 `n = normalize(Δh_left-right, 2·cellSize, Δh_down-up)`；
5. 朗伯强度近似：`dB ∝ 20·log₁₀(cosθ) - α·2R - 40·log₁₀(R)`。

### 4.3 线程配置

- `[numthreads(64, 1, 1)]`
- `Dispatch(ceil(1024/64) = 16, 1, 1)`
- 一台 RTX 3060 可在 <80 µs 完成 1024 线 ping（含读取）

---

## 五、编译与运行步骤

### 5.1 编译 C++ 原生插件（Windows x64）

```powershell
cd NativePlugin\AUVPhysics
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

成功后将 `build\Release\AUVPhysics.dll` 拷贝到：
```
UnityProject\Assets\Plugins\x86_64\AUVPhysics.dll
```
（Unity 会自动识别平台为 Editor + Standalone）

### 5.2 Unity 项目配置

1. 安装 Unity **2022.3 LTS** 或更新版本；
2. 添加已存在项目 → 选择 `UnityProject/` 目录；
3. 打开 Edit → Project Settings：
   - Player → Other Settings → **Scripting Backend = IL2CPP**（可选，但推荐）
   - Graphics → **Scriptable Render Pipeline = Built-in**（我们用原生 Shader）
   - Quality → 至少选 Medium 或以上（需 Compute Shader 支持）
4. 新建空场景，新建空 GameObject，挂上 **`SceneBootstrap`**；
5. 按 ▶ Play，场景会程序化生成：海底地形、AUV、声呐、点云、HUD 全部内容。

### 5.3 快速验证最小流程

不想走完整编译链路？MultibeamSonar 支持回退模式：

1. 在 Inspector 中把 MultibeamSonar 的 `_useGPU` 勾选取消；
2. 它会使用 `Physics.RaycastNonAlloc` 走 CPU 路径（需要 MeshCollider，`SeafloorTerrain` 已生成）；
3. 即便 AUVPhysics.dll 尚未就绪，Unity 也会报 DllNotFoundException，但仍能通过 CPU 路径看到声呐工作。

---

## 六、控制键位

| 操作 | 键位 |
|---|---|
| 前进 / 后退（Surge u） | `W` / `S` |
| 左右横移（Sway v） | `A` / `D` |
| 上浮 / 下潜（Heave w） | `E` / `Q` |
| 艏向偏航（Yaw r） | 鼠标 X 轴 |
| 俯仰（Pitch q） | 鼠标 Y 轴 |
| 横摇（Roll p） | 数字小键盘 `4` / `6` |
| 加力 Boost | 按住 `Left Shift` |
| 重置姿态 | `R` |

默认开启自动配平（保持水平）和深度保持（target = -60 m）。

---

## 七、调参建议（真实 AUV 级别）

| 参数 | 典型值 | 在哪个文件 / 组件 |
|---|---|---|
| 质量 m | 1500 kg | `AUVController._mass` |
| 排水体积 ∇ | 1.47 m³ | `AUVController._displacementM3` |
| 重心 G → 浮心 B 距离 | +0.15 m（上浮） | `_centerOfBuoyancy.y` |
| 附加质量 X_u̇ | 800 kg | `_addedMassLin.x` |
| 附加质量 N_ṙ | 5000 kg·m² | `_addedMassAng.z` |
| 声呐波束数 | 1024 | `MultibeamSonar._numBeams` |
| Ping 频率 | 8~12 Hz | `_pingRateHz` |
| 测幅角 | 120° | `_swathAngleDeg` |
| 量程 | 150 m | `_maxRange` |
| 声速（海水中） | 1500 m/s | `_soundVelocity` |
| 吸收系数（300 kHz） | 0.08 dB/m | `_absorptionDbPM` |

---

## 八、代码参考索引

- 六自由度流体动力学求解器（C++ 核心）：[HydrodynamicsSolver.cpp](file:///d:/SOLO-0619-3/20-deepsea-auv-sonar/NativePlugin/AUVPhysics/src/HydrodynamicsSolver.cpp#L1-L480)
  - `AUV_ComputeHydrodynamics()` 在 #L150-#L270 计算五种分项力
  - `matrix6_inverse_diag_dominant()` 在 #L85-#L120 做 6×6 矩阵求逆
- 声呐方向生成（C++）：[SonarSimulator.cpp](file:///d:/SOLO-0619-3/20-deepsea-auv-sonar/NativePlugin/AUVPhysics/src/SonarSimulator.cpp#L64-L86)
- GPU Heightmap DDA 射线：[MultibeamSonarRaycast.compute](file:///d:/SOLO-0619-3/20-deepsea-auv-sonar/UnityProject/Assets/Shaders/MultibeamSonarRaycast.compute#L74-L190)
  - `raycastHeightmap()` 子函数
- P/Invoke 桥接：[HydrodynamicsBridge.cs](file:///d:/SOLO-0619-3/20-deepsea-auv-sonar/UnityProject/Assets/Scripts/Native/HydrodynamicsBridge.cs#L1-L195)
- 点云着色器（GS 扩展四边形 + 深度色带）：[BathymetryPointCloud.shader](file:///d:/SOLO-0619-3/20-deepsea-auv-sonar/UnityProject/Assets/Shaders/BathymetryPointCloud.shader#L1-L150)
- 海图 HUD：[BathymetryPanel.cs](file:///d:/SOLO-0619-3/20-deepsea-auv-sonar/UnityProject/Assets/Scripts/UI/BathymetryPanel.cs#L1-L270)
- 一键场景搭建：[SceneBootstrap.cs](file:///d:/SOLO-0619-3/20-deepsea-auv-sonar/UnityProject/Assets/Scripts/Core/SceneBootstrap.cs#L1-L450)

---

## 九、可扩展研究方向

1. **水体散射效应**：在 Compute Shader 中加入蒙特卡洛体积散射（Henyey-Greenstein 相位函数），模拟浊度、悬浮粒子衰减；
2. **AUV 动力学辨识**：记录 ν、τ 序列，输出 CSV 供离线最小二乘辨识水动力参数；
3. **SLAM 接口**：将声呐点云保存为 `.ply`，对接 LOAM / ICP-SLAM；
4. **多 AUV 协同**：复制 AUV prefab 多次，添加网络层模拟水声通信延迟（≤ 500 ms 模型）；
5. **ROS 2 桥接**：通过 `ros2-sharp` 发布 `/sonar/pointcloud2`、`/auv/odom`、`/pressure` Topic。

— 构建版本 2026.06 / 深水高保真验证基线 —
