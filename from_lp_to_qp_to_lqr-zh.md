# 从 LP 到 QP 到 LQR：控制中的优化简史

**LP 回答"最便宜的路怎么走"。QP 回答"最便宜的路怎么走，但步子别太大"。LQR 回答"对一个动态系统，最优的一串步子怎么迈"。MPC 把三者串起来 —— 在每一步，把 LQR 变成一个 QP 来解。**

---

## 1. 优化问题的版图

想象你在解决三个问题：

1. **运输调度。** 5 个工厂，200 个仓库。每条工厂→仓库路线运费不同。每个工厂出多少货、运到哪个仓库，才能用最低总成本满足所有需求？

2. **机器人抓取。** 一条 7 关节的机械臂，要控制末端到达目标位姿。但你希望关节运动尽量平滑（别抽搐），而且每个关节都不能超限位。

3. **电机伺服跟踪。** 你给电压，转轴就动。接下来 100 个时间步里，你希望位置误差最小、控制功耗最小。但功放只能输出 ±12V。每步该给多大电压？

问题 1 是一个**线性规划（Linear Program, LP）**。问题 2 是一个**二次规划（Quadratic Program, QP）**。问题 3 是一个**有限时域 LQR** —— 而我们会看到，它可以被转换成一个 QP。

这三类问题构成了一个表达能力递增的层级结构。LP 最简单。QP 加了一个二次项，编码了"步子别太大"的偏好。LQR 则是把 QP 施加到一个随时间演化的动态系统上 —— 当约束进场后，它就成了模型预测控制（MPC）核心的那个 QP。

本文追溯这个层级结构，从 Dantzig 1947 年的单纯形法，一路讲到现代基于 QP 的 MPC 和机器人逆运动学。

---

## 2. 线性规划（LP）

### 2.1 标准形式

标准形式的线性规划长这样：

$$\begin{aligned}
\min_x \quad & c^T x \\
\text{s.t.} \quad & A x = b \\
& x \geq 0
\end{aligned}$$

其中 $x \in \mathbb{R}^n$ 是决策变量，$c$ 是代价向量，$A \in \mathbb{R}^{m \times n}$ 定义等式约束，$b \in \mathbb{R}^m$ 是右边项。$x \geq 0$ 表示 $x$ 的每个分量非负。

更通用的 LP 允许不等式约束 $A x \leq b$ 和自由变量，但这些都可以通过添加松弛变量、把自由变量拆成正部和负部来转成标准形式。

**LP 的定义性特征：** 目标和所有约束对 $x$ 都是*线性*的。没有平方项，没有乘积，没有指数函数 —— 只有内积。

### 2.2 历史：Dantzig 与单纯形法（1947）

二战期间，美军面临巨大的物流问题 —— 就是第 1 节里那个"往前线运物资"的问题。这些问题被建模为线性规划，但没人知道怎么高效求解。

1947 年，George Dantzig —— 当时是美国空军的数学顾问 —— 发明了**单纯形法（simplex method）**。他的核心几何洞察是：LP 的可行域是一个**凸多面体（convex polytope）**，如果最优解存在，那么至少有一个最优解落在多面体的**顶点**上。

单纯形法利用了这个性质。它从一个顶点出发，然后沿着边"旋转（pivot）"到相邻的、使目标更优的顶点。如此反复，直到没有任何相邻顶点能改进目标 —— 此时当前顶点就是全局最优。算法沿着多面体的边行走，始终走下坡路。

```
已知：A, b, c（标准形式 LP）

1. 找一个初始基本可行解（一个顶点）。
2. 计算 "reduced costs" —— 沿着每条边离开当前顶点，
   目标函数会变化多少。
3. 若所有 reduced costs ≥ 0：停止（已最优）。
4. 否则：选一个负的 reduced cost，沿那条边行进，
   直到碰到边界（另一顶点），回到步骤 2。
```

尽管最坏情况下复杂度是指数级的，单纯形法在实践中通常能求解成千上万个变量和约束的 LP。它的平均性能出奇地好 —— 通常只需 $O(m)$ 到 $O(3m)$ 次迭代，其中 $m$ 是约束数量。

Dantzig 的单纯形法让 LP 成为运筹学的主力工具。1950 年代，石油公司用它做炼油厂调度，航空公司用它排机组班表。它至今仍是人类发明的最重要的算法之一。

### 2.3 几何直观

考虑一个两变量的简单 LP：

$$\begin{aligned}
\max_{x_1, x_2} \quad & 3x_1 + 2x_2 \\
\text{s.t.} \quad & x_1 + x_2 \leq 4 \\
& 2x_1 + x_2 \leq 5 \\
& x_1, x_2 \geq 0
\end{aligned}$$

可行域是 $(x_1, x_2)$ 平面上的一个多边形。目标 $3x_1 + 2x_2$ 定义了一族平行直线。最优解是多边形中碰到"最高"那条直线的点 —— 永远是一个顶点。

这个几何图景可以推广到 $n$ 维：可行集是一个凸多面体，目标是一个超平面，最优解（如果唯一的话）是一个顶点。单纯形法就是系统性地在顶点之间跳转。

> **配套演示：** `lp_geometry_demo.py` 绘制了这个 LP —— 可行多边形、目标等高线、单纯形旋转路径 O → A → B，以及一个放大插图展示最优顶点处的 KKT 最优性条件。运行：`.venv/Scripts/python.exe lp_geometry_demo.py`，输出到 `lp_geometry_demo.png`。

### 2.4 内点法（Karmarkar, 1984）

1984 年，贝尔实验室的 Narendra Karmarkar 发表了一个根本性不同的 LP 算法。与沿着边界从一个顶点走到另一个顶点不同，他的**内点法（interior point method）**直接从多面体的中间穿过去。

思路是：从一个严格位于可行域内部的点出发。做一个投影变换，把当前点放在多面体的"中心"。沿最速下降方向迈一步。再变回去。如此反复。

Karmarkar 证明了多项式时间收敛 —— $O(n^{3.5} L)$，其中 $L$ 是输入的比特长度 —— 而且更"挑衅"的是，他声称在大问题上他的方法比单纯形法更快。

这引发了一场革命。Mehrotra 的预测-校正方法（1992）等变体成为现代 LP 求解器的基石。今天，单纯形法和内点法并存：单纯形法擅长重新求解（数据小改后从上次结果热启动），内点法擅长从头开始求解大规模稀疏问题。

> **配套演示：** `interior_point_demo.py` 实现了原始对数障碍法 —— 在同一 LP 上追踪中心路径，从解析中心 $(0.76, 1.12)$ 随 $t \to \infty$ 收敛到 LP 最优解 $(1, 3)$，并展示了中间 $t$ 值处的 Newton 中心化步骤。运行：`.venv/Scripts/python.exe interior_point_demo.py`，输出到 `interior_point_demo.png`。

### 2.5 LP 是静态的

一个关键观察：**LP 解的是静态问题。** 它找的是单个向量 $x$，在约束下最小化 $c^T x$。没有时间。没有动力学。没有"我做了这个决策之后会发生什么"。LP 是一张快照 —— 一个瞬间的最优资源分配。

这既是它的威力，也是它的局限。对运输调度问题，它完美匹配 —— 你只需要一次分配。对控制电机随时间的演化，你需要更多东西。

---

## 3. 二次规划（QP）

### 3.1 QP 多了什么

二次规划将 LP 推广，在目标中增加了一个二次项：

$$\begin{aligned}
\min_x \quad & \frac{1}{2} x^T H x + c^T x \\
\text{s.t.} \quad & A x = b \\
& G x \leq h
\end{aligned}$$

其中 $H \in \mathbb{R}^{n \times n}$ 是对称且（对凸 QP）半正定的。约束仍然是线性的 —— 只有目标多了曲率。

如果 $H = 0$，这就是一个 LP。所以 **QP 严格推广了 LP。**

### 3.2 二次项的意义

线性项 $c^T x$ 说"往这个方向推"。二次项 $\frac{1}{2} x^T H x$ 说"但也别太远 —— 越远代价越高"。

这编码了一个根本性的偏好：**小调整比大调整好。** 在机器人的例子中，$c^T x$ 编码了末端执行器的误差（到达目标），而 $x^T H x$ 惩罚了大幅关节运动（平稳移动）。QP 找到了两者之间的平衡。

具体来说，如果 $H$ 是对角阵，每个 $\frac{1}{2} H_{ii} x_i^2$ 随 $x_i$ 二次增长。优化器会跨多个变量分散"努力"，而不是让某个变量独占 —— 这个性质叫**正则化（regularization）**。

### 3.3 LP vs QP：视觉对比

考虑最小化 $f(x) = \frac{1}{2} x^2 + c x$，约束 $-1 \leq x \leq 1$。

- 若线性项 $c$ 很大且为负（$c = -5$），最优解在约束边界 $x = 1$。二次项"想"让 $x = 0$，但线性项一直把它推到墙上。

- 若 $c$ 较小（$c = -0.5$），无约束最优在 $x = 0.5$ —— 在可行域内部。二次曲率阻止了解碰到边界。

这是关键行为：**LP 的解活在约束边界上。QP 的解可以活在内部。** LP 是"要么全有要么全无" —— 最优解永远在顶点。QP 允许"一些，但别太多" —— 最优解可以出现在可行集内任何位置，由线性推力与二次回拉的权衡决定。

这在控制中非常重要。LP 控制器会不停地在极限之间猛撞。QP 控制器只在必要时才碰到极限。

### 3.4 如何求解 QP

QP 的求解方法与 LP 平行，但因为二次项的存在而多了些复杂性：

**有效集法（active-set methods）**推广了单纯形法。每次迭代，先猜哪些不等式约束是"活动的"（绑定的等式），解一个等式约束 QP（通过 KKT 条件化为线性系统），然后更新活动集。这些方法适合中小规模 QP，尤其在热启动很重要时。

**内点法**将 Karmarkar 的 LP 算法推广到 QP。用对数障碍项替换不等式约束，然后随着障碍权重的减小追踪"中心路径"。这些方法适合大规模稀疏 QP。

**算子分裂法**（ADMM、OSQP）将问题拆成较简单的子问题迭代求解。OSQP（Stellato et al., 2020）使用交替方向法，尤其快于 MPC 中产生的 QP —— 可以在微秒级求解小问题，且能很好地利用热启动。

**面向小规模 QP 的有效集法**如 DAQP（dual active-set QP），专门为中小规模嵌入式 QP 设计。它使用对偶有效集方法直接处理盒约束 —— 正好是 MPC 中输入限幅产生的那类约束。与通用求解器不同，DAQP 利用了 MPC 约束往往是简单上下界 $u_{\min} \leq u_k \leq u_{\max}$ 这一事实。

---

## 4. LQR 作为动态优化

### 4.1 简要回顾：Bellman → Riccati → u = -Kx

线性二次调节器（Linear Quadratic Regulator）求解：

$$\begin{aligned}
\min_{u_0, \ldots, u_{N-1}} \quad & x_N^T Q_f x_N + \sum_{k=0}^{N-1} \left( x_k^T Q x_k + u_k^T R u_k \right) \\
\text{s.t.} \quad & x_{k+1} = A x_k + B u_k
\end{aligned}$$

这是一个**动态优化** —— 决策变量 $(u_0, \ldots, u_{N-1})$ 通过动力学约束 $x_{k+1} = A x_k + B u_k$ 耦合在一起。每个 $u_k$ 不仅影响第 $k$ 步的代价，还通过递推关系影响所有未来状态 $x_{k+1}, x_{k+2}, \ldots$。

Bellman 的最优性原理把它分解为反向递推，导出 Riccati 方程和线性反馈律 $u_k = -K_k x_k$。对无限时域情形，$K$ 是常矩阵，一次求解即可。（完整推导见 `bellman_to_lqr.md`。）

### 4.2 无约束假定

LQR 推导中没有不等式约束。最优的 $u_k = -K_k x_k$ 可以是任意实数。如果 $K$ 要求 12V 的功放给出 100V，LQR 毫不关心 —— 它给你的是*无约束*问题的数学最优解。

Riccati 方程中没有 $u_{\min}$ 或 $u_{\max}$ 的位置。代价函数 $J = \sum (x^T Q x + u^T R u)$ 是纯二次的；对 $u$ 的最小化是无约束的，直接令梯度为零即得。约束一旦出现，这个闭式解立刻失效。

---

## 5. 为什么实践中几乎没人直接用 $u = -Kx$

### 5.1 约束无处不在

每个物理系统都有极限：

| 极限 | 约束的内容 |
|------|-----------|
| 功放饱和 | $u_{\min} \leq u_k \leq u_{\max}$ |
| 摆率限制 | $|u_{k+1} - u_k| \leq \Delta u_{\max}$ |
| 电流限制 | $|i| \leq i_{\max}$（状态约束） |
| 位置限制 | $\theta_{\min} \leq \theta_k \leq \theta_{\max}$ |
| 热限制 | $\sum |u_k|^2 \leq P_{\text{热}}$ |

LQR 全都不管。增益矩阵 $K$ 是在假设控制量可以取任意值的前提下算出来的。当它不行时，闭环行为会恶化 —— 有时是灾难性的。

朴素的补救是饱和：算出 $u = -Kx$，再削到 $[u_{\min}, u_{\max}]$。但这有两个问题：

1. **积分饱和（windup）：** 积分器（LQI 中的）在饱和期间持续累积误差，然后在退出饱和时大幅超调。
2. **控制器不知道自己被饱和了。** 它计算 $u$ 时就当自己有无限的输出能力，因此不会为约束而做规划。结果是超调和振荡 —— 正如 `servo_qp_mpc.py` 所演示的那样。

### 5.2 模型精度永远不够

$A$ 和 $B$ 是模型，不是现实。电机有齿槽转矩、摩擦非线性、随温度变化的电阻，以及众多未建模的动力学。LQR 增益 $K$ 对*模型*是最优的 —— 但在真实硬件上，可能还不如一个凭经验手调的 PID。

这条模型与现实的鸿沟，正是自适应控制和鲁棒控制存在的原因。但它也促使了 MPC 的诞生：**通过在每一步用最新测量重新求解优化，MPC 可以部分补偿模型误差。** 滚动时域的反馈提供了纯开环最优 $u = -Kx$ 所不具备的一定程度的鲁棒性。

### 5.3 无限时域 vs. 有限时域

教科书上的 LQR —— $u = -Kx$ 搭配常值 $K$ —— 是无限时域的。它假设代价累积于 $t \in [0, \infty)$。这数学上很优雅，但实践中有限制：

- 真实任务有有限的时长（在 1 秒内转到目标位置）。
- 真实参考输入会变（电机不只是调节到零）。
- 真实目标会变（跟踪 vs. 调节 vs. 路径跟踪）。

真正需要的是带时变增益 $K_k$ 的有限时域 LQR。而有限时域 LQR 正是 MPC 所求解的东西 —— 再加上约束。

---

## 6. 从 LQR 到 QP：压缩（Condensing）

### 6.1 核心洞见

考虑有 $N$ 步的离散时间有限时域 LQR 问题：

$$\begin{aligned}
\min_{u_0, \ldots, u_{N-1}} \quad & x_N^T P x_N + \sum_{k=0}^{N-1} \left( x_k^T Q x_k + u_k^T R u_k \right) \\
\text{s.t.} \quad & x_{k+1} = A x_k + B u_k
\end{aligned}$$

决策变量是 $(u_0, \ldots, u_{N-1})$ —— 因为，给定了 $x_0$ 和控制序列，所有状态都完全被动力学决定。我们可以**消去状态变量**，把整个代价表达为纯粹关于控制序列的二次函数。

这就叫**压缩（condensing）。** 它把动态优化转成了一个 $N \cdot m$ 变量（$m$ 是控制输入维数）的静态 QP。

### 6.2 数学：沿时域的状态预测

给定了初始状态 $x_0$ 和控制序列 $U = [u_0^T, u_1^T, \ldots, u_{N-1}^T]^T$，我们可以写出每一步未来的状态：

$$\begin{aligned}
x_1 &= A x_0 + B u_0 \\
x_2 &= A^2 x_0 + A B u_0 + B u_1 \\
x_3 &= A^3 x_0 + A^2 B u_0 + A B u_1 + B u_2 \\
&\vdots \\
x_N &= A^N x_0 + A^{N-1} B u_0 + \cdots + B u_{N-1}
\end{aligned}$$

把所有预测状态堆成 $\mathbf{X} = [x_1^T, \ldots, x_N^T]^T$。用矩阵形式：

$$\mathbf{X} = \mathcal{A} x_0 + \mathcal{B} U$$

其中 $\mathcal{A} \in \mathbb{R}^{N n \times n}$ 和 $\mathcal{B} \in \mathbb{R}^{N n \times N m}$ 是**压缩预测矩阵：**

$$\mathcal{A} = \begin{bmatrix} A \\ A^2 \\ \vdots \\ A^N \end{bmatrix}, \qquad
\mathcal{B} = \begin{bmatrix}
B & 0 & \cdots & 0 \\
A B & B & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
A^{N-1} B & A^{N-2} B & \cdots & B
\end{bmatrix}$$

$\mathcal{B}$ 是块下三角的 —— 用矩阵形式表达因果性：$u_k$ 只能影响 $x_{k+1}$ 及以后的状态，绝不回馈到过去。

注意 $\mathcal{B}$ 和控制论中能控性矩阵 $\mathcal{C} = [B, AB, \ldots, A^{n-1}B]$ 共用同一组脉冲响应序列 $\{B, AB, A^2B, \ldots\}$ 作为积木，但它们是不同的矩阵。能控性矩阵把这些块水平拼接，测试状态空间的能达性；$\mathcal{B}$ 则把它们织成块 Toeplitz 结构，强制因果性。

### 6.3 代入代价函数

用 $\mathbf{X}$ 和 $U$ 表达的代价函数：

$$J = x_0^T Q x_0 + \mathbf{X}^T \bar{Q} \mathbf{X} + U^T \bar{R} U$$

其中 $\bar{Q} = \text{blkdiag}(Q, Q, \ldots, Q, P)$ 和 $\bar{R} = \text{blkdiag}(R, R, \ldots, R)$ 是块对角矩阵，将阶段代价矩阵重复 $N$ 次，终端代价 $P$ 在 $\bar{Q}$ 的最后一块。

代入 $\mathbf{X} = \mathcal{A} x_0 + \mathcal{B} U$：

$$\begin{aligned}
J &= x_0^T Q x_0 + (\mathcal{A} x_0 + \mathcal{B} U)^T \bar{Q} (\mathcal{A} x_0 + \mathcal{B} U) + U^T \bar{R} U \\
&= x_0^T Q x_0 + x_0^T \mathcal{A}^T \bar{Q} \mathcal{A} x_0 + 2 x_0^T \mathcal{A}^T \bar{Q} \mathcal{B} U + U^T (\mathcal{B}^T \bar{Q} \mathcal{B} + \bar{R}) U
\end{aligned}$$

$x_0^T Q x_0$ 和 $x_0^T \mathcal{A}^T \bar{Q} \mathcal{A} x_0$ 不依赖于 $U$，可以从最小化中丢掉。得到：

$$\min_U \quad \frac{1}{2} U^T H U + (F^T x_0)^T U$$

其中：

$$H = 2 (\mathcal{B}^T \bar{Q} \mathcal{B} + \bar{R}), \qquad F = 2 \mathcal{B}^T \bar{Q} \mathcal{A}$$

这是一个 $N \cdot m$ 变量的**标准 QP**。Hessian 矩阵 $H$ 是 $N m \times N m$ 正定的（因为 $R \succ 0$）。线性项 $F^T x_0$ 编码了初始状态 —— 每个时间步只有线性项改变，$H$ 不变。

### 6.4 加入约束

现在迈出关键一步：加入控制的盒约束：

$$u_{\min} \leq u_k \leq u_{\max}, \quad k = 0, \ldots, N-1$$

这些就是 $U$ 上的线性不等式约束。完整的 QP 变成：

$$\begin{aligned}
\min_U \quad & \frac{1}{2} U^T H U + (F^T x_0)^T U \\
\text{s.t.} \quad & u_{\min} \leq u_k \leq u_{\max}, \quad k = 0, \ldots, N-1
\end{aligned}$$

这就是 QP 求解器在每一步 MPC 中实际求解的问题。当没有约束活动时，解为 $U^* = -H^{-1} F^T x_0$，其中 $u_0^*$ 精确匹配 LQR 增益 $u_0 = -K_0 x_0$。当约束变为活动时，求解器找到带约束的最优 —— 在限制范围内你能做到的最好结果。

你还可以加入状态约束（$x_{\min} \leq x_k \leq x_{\max}$）和变化率约束（$|u_{k+1} - u_k| \leq \Delta u_{\max}$），全都表达为 $U$ 上的线性不等式。

### 6.5 压缩在代码中

`servo_qp_mpc.py:118-138` 的 Python 实现做的就是这件事：

```python
# 预测矩阵（压缩后的）
A_aug = [A; A^2; ...; A^N]          # 竖直堆叠
B_aug = 如上所述的块下三角矩阵     # N 块 × N 块

# 块对角代价矩阵
Qbar = blkdiag(Q, Q, ..., Q, P)     # 最后一块为终端 P
Rbar = blkdiag(R, R, ..., R)

# QP 矩阵
H = B_aug.T @ Qbar @ B_aug + Rbar
H = 0.5 * (H + H.T)                 # 确保对称性
F = A_aug.T @ Qbar @ B_aug          # 将 x0 映射为线性代价项

# 每个时间步求解：
#   min  0.5 * U^T H U + (F^T x0)^T U
#   s.t. u_min <= U <= u_max
```

$H$ 只计算一次（它不依赖 $x_0$）。每个时间步只有线性项 $F^T x_0$ 改变。这是关键效率所在：QP 的 Hessian 是固定的，求解器可以分解 $H$ 一次，然后重用分解。

### 6.6 这就是 MPC

压缩后的 QP 是**模型预测控制（Model Predictive Control）**的计算引擎：

```
每个时间步 k：
  1. 测量（或估计）当前状态 x_k
  2. 求解压缩 QP：  min ½U^T H U + (F^T x_k)^T U
                      s.t. 约束
  3. 只执行 u*_0（最优序列的第一个控制量）
  4. k ← k+1，回到步骤 1
```

这就是**滚动时域控制（receding-horizon control）。** 你规划了一条完整的 $N$ 步轨迹，但只执行第一步。然后你重新测量、重新优化、重新执行。通过重新测量引入的反馈为模型误差提供了鲁棒性。重新优化确保控制策略适应的是实际状态，而非预测状态。

没有约束时，MPC 精确还原 LQR。有了约束，MPC 将 LQR 扩展到极限存在的物理现实中。

---

## 7. 机器人逆运动学中的 QP

### 7.1 IK 问题作为优化

一条 $n$ 关节的机械臂，构型为 $q \in \mathbb{R}^n$。正运动学将关节角度映射到末端执行器位姿：$p = f(q)$，其中 $p \in \mathbb{R}^6$（位置 + 姿态）。

**逆运动学（Inverse Kinematics, IK）**的问题是：给定期望末端位姿 $p^*$，求关节角度 $q$ 使得 $f(q) = p^*$。

对冗余机械臂（$n > 6$），有无限多解。你需要从中选一个 —— 这就是 QP 的用武之地。

### 7.2 将 IK 表述为 QP

在当前构型 $q_0$ 处线性化正运动学：

$$f(q) \approx f(q_0) + J(q_0) \cdot \Delta q$$

其中 $J = \frac{\partial f}{\partial q}$ 是 $6 \times n$ 的 Jacobian。IK 问题变成：找 $\Delta q$ 使得 $J \Delta q = p^* - f(q_0)$。

这是一个欠定线性系统（变量多于方程）。QP 表述为：

$$\begin{aligned}
\min_{\Delta q} \quad & \frac{1}{2} \Delta q^T W \Delta q \\
\text{s.t.} \quad & J \Delta q = \Delta p \\
& q_{\min} - q_0 \leq \Delta q \leq q_{\max} - q_0
\end{aligned}$$

其中 $W$ 是权重矩阵（通常为对角 —— 越重的关节越不爱动），盒约束强制了关节限位。这是一个**凸 QP** —— 在结构上与 MPC 问题完全相同，只是 $H$ 和约束编码的内容不同。

### 7.3 Mink 与 DAQP

[Mink](https://github.com/kevinzakka/mink) 是一个 Python 逆运动学库，将 IK 表述为带约束的优化问题。它的默认求解器是 **DAQP**（Dual Active-set Quadratic Programming） —— 一个面向中小规模简单约束问题的专用 QP 求解器。

DAQP 很适合 IK，因为：

- **IK 的 QP 很小。** 7 自由度臂只有 7 个变量。Jacobian 是 $6 \times 7$。以优化标准来看这个 QP 很小。
- **约束大多是上下界。** 关节限位是盒约束 $q_{\min} \leq q \leq q_{\max}$。这是最简单的一类 QP 约束，对偶有效集方法处理起来效率极高。
- **热启动至关重要。** 每个控制周期（通常 100 Hz–1 kHz），QP 都从前一个解出发求解。活动集变化缓慢。DAQP 充分利用了这一点。

与 MPC 的联系是直接的：两者都在每个时间步求解带盒约束的凸 QP，两者都使用前一个解的热启动，两者都需要微秒级的求解时间才能实现实时运行。

---

## 8. 全景图

### 8.1 统一视角

| 类别 | 目标 | 约束 | 时间 | 解的位置 |
|------|------|------|------|---------|
| **LP** | 线性 | 线性 | 静态 | 顶点 |
| **QP** | 二次 + 线性 | 线性 | 静态 | 可行集内任意位置 |
| **LQR**（无限时域） | 二次（动态） | 无 | 动态（一次求解） | $u = -Kx$，线性反馈 |
| **MPC**（压缩 QP） | 二次（动态） | 线性（盒/不等式） | 动态（每步求解） | 滚动时域带约束最优 |

这一递进是表达能力不断增强的过程：

- **LP → QP：** 目标获得了曲率，允许"偏好小步移动"的正则化。解不再被逼到顶点。
- **静态 QP → LQR：** 优化跨过一个时域。决策不再是单个向量，而是被动力学链接的序列。消去状态得到一个静态 QP（压缩）。
- **LQR → MPC：** 优化变成在线、带约束的。每个时间步求解一个压缩 QP。约束被显式处理，而非打补丁。

### 8.2 LP 和 QP 是静态优化的两大主力

几乎每个资源分配、排程、投资组合优化或网络流问题都是 LP 或 QP。LP 用单纯形法或内点法求解。QP 加入了编码"调整代价"的二次项 —— 在你需要平滑、分散的解而不是 bang-bang 极端时必不可少。

### 8.3 LQR 和 MPC 是施加到动态系统上的 QP

有限时域 LQR 就是一个 $N \cdot m$ 变量的 QP —— 你没看出来只是因为在无约束情况下，Riccati 递推解得比它更优雅。但 Riccati 方法处理不了不等式约束，这就是为什么 MPC 直接解 QP。

**LQR 是 MPC 在约束全部不活动时的特例。** 这很容易验证：跑一个无约束 QP-MPC，比较 $u_0^*$ 和 $-K_0 x_0$。它们完全相同。

### 8.4 这为什么重要

如果你理解了 LP 和 QP，你就理解了现代控制的计算引擎。MPC 不是一个根本性的新数学对象 —— 它是一个 QP，在线求解，动力学通过压缩折叠进了 Hessian。MPC 的"魔法"不在于优化（那是标准操作），而在于架构：建模、预测、带约束优化、滚动。

同理，如果你理解了 QP，你就理解了现代机器人 IK。唯一的区别是 $H$、$F$ 和约束编码了什么 —— 末端误差 vs. 状态调节，关节限位 vs. 电压限幅。求解器不在乎。

---

## 9. 参考文献

1. **Dantzig, G.B. (1947).** "Maximization of a linear function of variables subject to linear inequalities." Chapter XXI in *Activity Analysis of Production and Allocation* (T.C. Koopmans, ed.). —— 原始的单纯形法。

2. **Karmarkar, N. (1984).** "A New Polynomial-Time Algorithm for Linear Programming." *Combinatorica.* —— 掀起内点法革命的论文。

3. **Wright, S.J. (1997).** *Primal-Dual Interior-Point Methods.* SIAM. —— 关于 LP 和 QP 内点算法的权威参考。

4. **Nocedal, J. & Wright, S.J. (2006).** *Numerical Optimization.* Springer. —— 优化算法的全面覆盖，含 QP 方法。

5. **Boyd, S. & Vandenberghe, L. (2004).** *Convex Optimization.* Cambridge University Press. —— 凸优化的现代基石，含 LP 和 QP。

6. **Maciejowski, J.M. (2002).** *Predictive Control with Constraints.* Prentice Hall. —— 标准 MPC 教材，涵盖了压缩和带约束 QP 表述。

7. **Stellato, B., Banjac, G., Goulart, P., Bemporad, A., & Boyd, S. (2020).** "OSQP: An Operator Splitting Solver for Quadratic Programs." *Mathematical Programming Computation.* —— OSQP 求解器，广泛用于嵌入式 MPC。

8. **Arnström, D., Bemporad, A., & Axehill, D. (2022).** "A Dual Active-Set Solver for Embedded Quadratic Programming." *arXiv:2203.02599.* —— DAQP 求解器，用于 Mink 和其他嵌入式应用。

9. **Zakka, K. et al. (2024).** "Mink: A Python Library for Inverse Kinematics." *GitHub: kevinzakka/mink.* —— 通过带约束 QP 做 IK；尤其参见 DAQP 集成。

10. **Bertsekas, D.P. (2012).** *Dynamic Programming and Optimal Control, Vol. I.* Athena Scientific. —— 串起从 Bellman 到 LQR 到带约束最优控制的线。

11. **Borrelli, F., Bemporad, A., & Morari, M. (2017).** *Predictive Control for Linear and Hybrid Systems.* Cambridge University Press. —— 现代 MPC，含显式 MPC 和混合系统。

---

*本文档是 `Controllers-from-PID-to-QP_MPC` 仓库的一部分。从 Bellman 原理到 LQR 的推导，见 `bellman_to_lqr.md`。交互式仿真器：打开 `lqr_explorer.html`（无约束 LQR）和 `servo_qp_mpc.html`（带约束 QP-MPC）。Python 演示 `servo_qp_mpc.py` 展示了压缩实现及 OSQP vs. DAQP 的对比。*
