# PID控制中的抗积分饱和：综述、分析与新调参方向

> **原文**: Anti-Windup in PID Control: Review, Analysis, and New Tuning Directions
>
> **作者**: M. Caparroz<sup>a</sup>, K. Soltesz<sup>b</sup>, T. Hägglund<sup>b</sup>, J. L. Guzmán<sup>a</sup>
>
> <sup>a</sup>西班牙阿尔梅里亚大学信息学系；<sup>b</sup>瑞典隆德大学自动控制系
>
> **来源**: arXiv:2606.01959v1 [eess.SY], 2026年6月1日

---

## 摘要

执行器饱和是一种基本的非线性特性，它通过引起积分器饱和（windup）显著降低PID控制系统的性能，导致超调、恢复缓慢甚至不稳定。尽管已有大量抗积分饱和策略被提出，但其实际调参在很大程度上仍依赖经验方法，在许多工业场景中难以达到最优。本文对经典和先进的抗积分饱和技术在PI控制的一阶加纯滞后（FOPDT）过程中的应用进行了全面的比较研究，涵盖广泛的工况条件。分析内容包括动态和瞬时回算（back-calculation）、条件积分（conditional integration）以及改进方案。此外，本文提出了一种新颖的混合抗积分饱和策略，将条件积分与动态回算相结合，以提高饱和期间的响应能力，同时保持平滑的恢复动态。本文的一个关键贡献是为回算方案中的跟踪时间常数开发了系统化的调参规则，专门针对负载扰动抑制进行了优化。这些规则源自一项广泛的优化研究，考虑了饱和比、控制器激进程度和扰动特性。所得指南提供了简单而有效的公式，无需复杂计算即可实现接近最优的性能。仿真结果表明，所提方法显著优于常用的启发式规则，特别是在扰动抑制场景中，并为工业应用中抗积分饱和策略的选择和调参提供了清晰、实用的建议。

**关键词**: PID控制；饱和系统；抗积分饱和；跟踪；扰动抑制

---

## 1. 引言

比例-积分-微分（PID）控制器是工业自动化的基石，由于其固有的简单性、鲁棒性和在多种过程动态中的有效性，始终保持着最广泛使用的控制策略的地位（Rojas, Arrieta, & Vilanova, 2021; Visioli, 2006; Willis, 1999; Åström & Hägglund, 2006）。尽管先进控制技术不断涌现，PID的易于实现以及工业界对其三项结构的熟悉确保了其持续的主导地位。

然而，PID控制的实际应用经常受到物理约束的挑战，其中最显著的是执行器饱和（Hui & Chan, 1997; Zaccarian & Teel, 2011）。执行器饱和是一种关键的非线性特性，它通过引起积分器饱和和延长扰动恢复时间来降低反馈控制系统的性能（Kothare & Morari, 1997）。当控制器输出达到执行器的物理极限时，反馈回路变为无效，系统进入开环状态：执行器无法再响应控制信号的进一步增加。在此期间，积分项继续累积误差，导致积分饱和。这种累积在系统返回线性区域后可能导致显著的性能退化，表现为过大的超调、恢复时间延长，在严重情况下甚至导致闭环不稳定（Åström & Murray, 2021; Kothare, Campo, Morari, & Nett, 1994; Åström & Hägglund, 2006）。

为了缓解这些影响，文献中已开发了各种抗积分饱和策略，以在饱和期间维持控制器的性能和完整性。经典方法如回算和积分器箝位由于其简单性和低计算成本，在工业中仍被广泛使用（Okelola, Aborisade, & Adewuyi, 2020; Åström & Hägglund, 2006），而更先进的方法则将这些思想扩展到更复杂的控制器结构和架构。

尽管已有大量研究工作，但在实践中选择合适的抗积分饱和策略并调参其参数仍然是一项具有挑战性且往往是临时性的任务。多年来提出了大量的抗积分饱和技术，但其实际实现和调参仍然缺乏系统的、广泛接受的指南（Amiri, Theodoridis, Müller, & Holderbaum, 2024; Okelola et al., 2020; Sarhadi, 2026; Yapp, Hoo, & Lai, 2024）。给定技术的性能不仅取决于控制器设计和过程动态，还取决于特定的控制目标（如设定值跟踪或扰动抑制）以及所涉及扰动的性质（Cao, Lin, & Ward, 2004; Torstensson, 2013; Wang, 2016）。因此，在一种场景中表现良好的方法在另一种场景中可能导致显著的性能退化，特别是在存在不确定性、非线性和变化工况的情况下。

特别是在回算这一类方法中，跟踪时间常数的选择对确定从饱和中快速恢复与闭环稳定性之间的权衡起着关键作用。尽管简单的启发式规则（如将跟踪时间设置为积分时间）在实践中常用，但它们往往无法在不同工况下提供令人满意的性能（B˘usek, Vyhlídal, & Zítek, 2017; Kumar & Negi, 2012; Markaroglu, Guzelkaya, Eksin, & Yesil, 2006; Rundqwist, 1990）。

系统化指南的缺乏造成了抗积分饱和技术的广泛可用性与其在实际系统中有效应用之间的差距。从业者经常面临多种设计选择：选择抗积分饱和结构、调参其参数、适应不同类型的控制问题，却没有明确的标准来指导这些决策。

在此背景下，本文旨在为选择和调参抗积分饱和策略提供一个全面且实用的框架。首先，对几种经典和最新提出的技术在广泛的工况条件下进行回顾和比较，包括设定值跟踪和扰动抑制，具有不同的过程动态和饱和水平。然后，提出了一种新的混合抗积分饱和方案，将回算和条件积分相结合，以在饱和期间改善性能，同时保持平滑的恢复动态。

此外，还为回算方案中的跟踪时间常数开发了一套系统化的调参规则。所提出的规则源自一项在广泛工况条件下进行的优化程序，以饱和比、控制器激进程度和扰动与过程动态之比为特征，并通过带有PI控制器的一阶加纯滞后过程的仿真研究进行了验证。所得指南使从业者能够在保持实现简单性的同时实现接近最优的性能。

本文结构如下：第2节描述积分饱和问题及其在不同控制问题中对过程输出的影响。第3节对已有解决方案进行综述，涵盖经典和最新提出的算法。第4节深入描述将要比较的方案。第5节描述新提出的解决方案和本文的主要贡献，包括将回算与条件积分相结合的方案，以及使用回算时Tt的新调参规则。第6节对所有规则进行比较，分析各种问题，最终给出帮助读者为其特定问题选择最佳抗积分饱和解决方案的实用规则。最后，第7节给出结论。

---

## 2. 积分饱和问题

执行器饱和给反馈控制系统引入了基本的非线性特性，改变了控制器设计时所假设的标称闭环行为。图1展示了设定值跟踪和扰动抑制问题的标准反馈配置，其中w是受控变量y的设定值，e是w与y之间的误差，d是负载扰动，u_c是控制器输出，u_sat是饱和后的实际控制作用，P(s)和C(s)分别是拉普拉斯域中的过程和PID控制器。

在标称（未饱和）条件下，控制器根据以下公式计算控制信号u_c：

$$C(s) = \frac{u_c(s)}{e(s)} = K_p + \frac{K_i}{s} + K_d s \tag{1}$$

其中K_p、K_i和K_d分别是比例、积分和微分增益，必须正确选择以获得适当的闭环系统动态。

当PID控制器在工业中使用时，必须考虑执行器饱和，最终控制作用(u_sat)始终受限于执行器的极限。当控制器输出u_c超过执行器极限时，执行器将保持在其极限位置，系统以开环方式运行。此外，只要误差非零，积分作用就会持续增加。这导致该项变得非常大，这种效应称为"积分饱和"。

为了说明不同工况下执行器饱和的影响，图2和图3展示了三种将在本文中分析的典型控制场景。这些场景反映了工业实践中遇到的常见情况，并突出了积分饱和的不同表现形式。

在所有情况下，被分析的过程都是一个一阶加纯滞后（FOPDT）系统，具有单位增益K、三秒时间常数T和0.5秒滞后L：

$$P(s) = \frac{K}{Ts + 1} e^{-Ls} = \frac{1}{3s + 1} e^{-0.5s} \tag{2}$$

对于一阶系统，大多数情况下推荐使用PI控制器（Skogestad, 2003; Åström & Hägglund, 2006）。在所展示的示例中，C(s)使用λ方法调参，对于FOPDT系统，该方法建立：

$$K_p = \frac{T}{K(\lambda + L)}, \quad K_i = \frac{K_p}{T_i}$$

其中 $T_i = T$。期望的闭环时间常数为 $\lambda = 0.2T$。

前两种情况对应于设定值跟踪问题。在图2左侧的示例中，饱和发生在设定值变化后的瞬态过程中，控制作用暂时达到执行器极限但最终返回线性区域。这导致控制信号在饱和期间不断增长，造成恢复缓慢和过程输出超调。相反，图2右侧的示例展示了执行器在稳态下饱和的情况，即达到参考值所需控制力超过执行器极限，使设定值无法实现。

最后一种情况对应于扰动抑制问题。图3展示了对负载扰动的响应，控制器必须克服扰动，这通常导致持续饱和。在扰动期间，控制作用超过执行器极限，一旦扰动结束，积分作用已变得过大，导致过程输出的恢复比无饱和时慢得多。

在所有情况下，饱和导致控制器输出与实际控制信号之间的不匹配，使积分项以与实际系统动态不一致的方式累积。由此产生的积分饱和效应根据控制目标和系统配置的不同而有不同的表现，这促使需要针对每种场景定制抗积分饱和策略和调参规则。

很明显，执行器饱和降低了系统性能，必须应用抗积分饱和技术来缓解其影响。然而，选择和调参抗积分饱和方案并非易事，它取决于多个因素，包括过程和扰动动态以及控制器调参方法（Bohn & Atherton, 1995）。因此，下一节将对几种现有的抗积分饱和技术进行文献研究，并讨论其优缺点。

---

## 3. 抗积分饱和技术综述

执行器饱和时的积分饱和问题催生了各种抗积分饱和策略，它们在复杂性、通用性和饱和下的性能方面各不相同。工业中最广泛实施的抗积分饱和策略是**回算（Back-Calculation）**，也称为**跟踪（Tracking）**，其中基于饱和与未饱和控制信号之差向积分项引入反馈校正（Peng, Vrancic, & Hanus, 1996; Shin, 1998; Åström & Hägglund, 2006）。图4展示了应用于PID控制器的经典回算实现，其工作原理如下：当执行器饱和时，饱和误差(e_sat = u_sat - u_c)以增益1/T_t加权后从积分项中减去，其中T_t是跟踪时间常数。这样，积分作用被重置到执行器的极限，但不是在饱和时立即执行。当控制器输出在执行器边界内时（e_sat = 0），该反馈回路不干扰PID行为。

在此方案中，积分作用u_i由下式获得：

$$u_i(s) = \left( K_i e(s) + \frac{1}{T_t} e_{sat}(s) \right) \frac{1}{s} \tag{4}$$

在离散域中实现时，可以如下实现：

$$u_i(k) = u_i(k-1) + K_i T_s e(k) + \frac{1}{T_t} T_s e_{sat}(k) \tag{5}$$

其中T_s是采样时间，k是离散时间时刻。积分项在时刻k更新，使用前一采样时刻k-1的值，加上误差和饱和误差的贡献，两者分别乘以各自的增益和采样时间。当饱和误差为零时，积分项以正常方式计算。

关于跟踪时间常数，虽然没有选择其值的特定规则，但一些作者建议对于PID控制器应选择 $T_t = \sqrt{T_i T_d}$，当没有微分作用时选择 $T_t = T_i$（Morohoshi & Deng, 2025; Åström & Hägglund, 2006）。其他研究表明应选择低于T_i的值（Caparroz, Soltesz, Hägglund, & Guzmán, 2026; Markaroglu et al., 2006; Rundqwist, 1990）。然而，过小的跟踪时间常数会过快地重置控制器，导致严重的性能恶化（Rundqwist, 1990）。该参数在从饱和中快速恢复与闭环稳定性之间的权衡中起着关键作用。

另一种常用的抗积分饱和方案是**控制信号箝位（control signal clamping）**，其中允许积分项增长到饱和极限。因此，当达到饱和时，积分器仍以误差为输入，但饱和误差e_sat被完全移除。如果分析图4中的方案，控制信号箝位也可以看作是饱和误差反馈增益为无穷大时的回算，即对应T_t = 0。在离散实现中，这对应于T_t = T_s。在这种情况下，积分作用立即重置为其极限值。

这种技术的特点是快速退出执行器饱和，但通常导致设定值跟踪性能下降。然而，它的优势是不需要调参的参数，易于实现，且计算成本低。

因此，本文中回算和控制信号箝位将分别称为**"动态"和"瞬时"回算**。这主要是因为两种策略都基于饱和误差回算积分器输入。它们之间的唯一区别是，在动态回算中，校正使用T_t指定的动态来应用，而在瞬时回算中，饱和误差立即从积分作用中移除。

在（Bohn & Atherton, 2002; Glattfelder & Schaufelberger, 1986）中，提出了一种改进的动态回算方案，适用于PI和PID控制器。作者解决了选择小T_t值导致的系统响应缓慢问题。为了缓解这种影响，对比例-微分作用应用了额外的饱和限制，然后用于生成抗积分饱和反馈回路。这种额外的限制允许使用更低的跟踪时间常数而不会将积分器驱动得太远，从而避免缓慢的响应。此外，该方法对系统参数变化的敏感性较低，使其更易于调参，降低了因参数选择不当而导致性能下降的风险。

**条件积分（Conditional integration）**，也称为**积分器箝位（integrator clamping）**，是另一种众所周知且广泛实施的抗积分饱和策略，当满足某个条件时关闭积分（Hanus, Kinnaert, & Henrotte, 1987; Kumar & Negi, 2012; Åström & Hägglund, 2006）。条件积分的一些例子包括将积分项限制为某个值、当误差超过选定阈值时停止积分、或当控制器饱和时停止积分。在（Visioli, 2003）中，条件积分与动态回算相结合。具体来说，仅在满足以下条件时使用动态回算：系统误差与操纵变量同号，且系统输出已离开其先前设定值。此条件允许在过程输出瞬态尚未开始时让积分项增长。这在高滞后过程中特别有用，但当滞后较小时，该技术与标准动态回算的性能几乎相同。

在（Bruciapaglia & Apolˆonio, 1986）中，作者提出了一种抗积分饱和策略，修改当前控制信号以保持控制器输出（未饱和）与实际控制信号（饱和）之间的一致性。误差重计算通过将控制环视为一致的代数关系系统来执行。该算法不是简单地在控制器输出达到物理极限时进行裁剪，而是反向工作：它取实际饱和控制值(u_sat)并使用控制器的内部方程来确定输入误差(e)和内部状态应该是什么才能恰好产生该值。然后，这个重计算的误差用于更新控制器在下一时间步的记忆。通过有效地将控制器"重置"以与执行器的物理现实对齐，该方法确保控制环在数值上保持良好状态，并在系统退出饱和区域后立即准备恢复正常操作（Silva et al., 2017）。然而，该方案不适用于扰动抑制，并且可能对建模误差敏感。

虽然经典的抗积分饱和技术（如上述技术）通常由积分器饱和现象驱动，但在实践中，执行器饱和下的性能退化并非完全由积分项引起。特别是，当控制结构中存在额外的动态元件时，其他内部状态也可能与饱和控制信号不一致，导致不期望的瞬态。

在这方面，（Goodwin, Graebe, Salgado, et al., 2001）提出了一种实现回算的通用方案。在该方案中，控制器被分解为直通项c_∞和严格正则传递函数C̄(s)，使得完整控制器可以写为：

$$C(s) = c_\infty + \bar{C}(s) \tag{6}$$

其中 $c_\infty = \lim_{s \to \infty} C(s)$ 表示控制器的高频增益，C̄(s)包含其所有动态组件。

基于这种分解，通过将实际控制信号的差值通过以下动态补偿器反馈来构建抗积分饱和机制：

$$[\bar{C}(s)]^{-1} - c_\infty^{-1} \tag{7}$$

这产生了一种广义回算方案，其中校正项作用于完整控制器动态而非仅作用于积分分量（Hanus et al., 1987; Kothare et al., 1994; K. Walgama & Sternby, 1990; K. S. Walgama, Rännbäck, & Sternby, 1992）。所得结构（如图5所示）允许控制作用不仅在幅度上（饱和）受到约束，还在速率上（速率限制）受到约束，同时保持与控制器内部动态的一致性。

这种公式化的主要优势之一是其通用性。与隐式假设积分器是饱和唯一来源的经典回算不同，该方法适用于任意阶的控制器，包括具有滤波微分作用的PID控制器和更复杂的补偿器。因此，控制器的所有内部状态都由饱和控制信号适当地驱动，防止了可能导致执行器退出饱和时不期望瞬态的隐藏动态累积。此外，该方法不需要调参跟踪参数，因为抗积分饱和补偿器直接从控制器本身导出。这提供了系统化且理论上一致的设计程序。

然而，这些好处的代价是增加了实现复杂性。计算和实现控制器逆动态的需求可能引入数值和实际困难，特别是在离散时间实现中或当控制器模型是高阶或不确定的时候。

在前馈方案的特定情况下，（Hoyo, Hägglund, Guzmán, & Moreno, 2023）解决了经典的抗积分饱和限制问题，提出了一种包含前馈补偿的控制架构的抗积分饱和方案。该方案将抗积分饱和机制扩展到反馈控制器之外，确保饱和执行器信号与所有贡献控制路径之间的一致性。

总体而言，所综述的技术可以解释为在控制系统的不同层级逐步强制一致性。经典方法关注积分器状态；广义回算将此校正扩展到完整控制器动态；更先进的方法则纳入了多个控制路径（如前馈作用）。这种演变反映了实现简单性与处理饱和下日益复杂控制架构能力之间的权衡。然而，本文将关注经典PI控制器的抗积分饱和技术，其中饱和效应完全由积分器引起。

---

## 4. 比较的抗积分饱和方案

根据上一节的综述，本节重点介绍将通过仿真研究详细分析和比较的抗积分饱和策略子集。这些方法的选择不仅受到其理论相关性的指导，还受到其在工业实践中广泛采用的影响。

以下各节描述的方案将在使用λ方法调参的PI控制器的FOPDT系统中进行比较，并在各种工况条件下进行评估。

比较将包括设定值跟踪和扰动抑制问题。此外，将展示不同饱和条件和控制器调参下的示例。最终目标是根据系统动态、控制器和控制问题确定应优先应用哪种抗积分饱和技术。

### 4.1 经典方案

特别是，动态回算和瞬时回算等经典抗积分饱和技术仍然是工业PID控制器中最常用的解决方案，因为它们简单、计算成本低且易于集成到现有控制架构中。这些抗积分饱和方案已在上文描述并在图4中展示。如第3节所述，两种抗积分饱和方案类似，但在动态回算中，T_t值必须由用户选择，而在瞬时回算中，跟踪时间常数在连续域中必须等于零，在离散域中等于采样时间T_s。

对于动态回算方案，将测试几种T_t调参规则。第一种是众所周知的经验法则，其中T_t = T_i，由多位作者建议（Åström & Hägglund, 2006）。这可以如算法1所示实现，使用与瞬时回算相同的代码，只需更改T_t的值。

条件积分策略也经常被使用，特别是在偏好基于逻辑的简单解决方案的应用中。该方案如图6所示，其中开关根据饱和误差控制积分器的输入。

要实现此方案，积分作用必须如算法2所示计算，这是对算法1第12行的唯一更改。虽然该方案可以用if/else语句实现（选项1），但也提出了单行替代方案以实现更快的代码评估（选项2）。

**算法1：带回算抗积分饱和的PI控制器**

```
1. 初始化：
2.   选择采样时间：T_s
3.   选择控制器调参：K_p, K_i
4.   选择抗积分饱和调参：T_t
5.   对每个采样时刻 k：
6.     1. 测量
7.       定义参考值 w(k)
8.       测量输出 y(k)
9.       计算误差 e(k)
10.    2. 控制作用计算
11.      计算比例作用 u_p(k) = K_p * e(k)
12.      计算带回算抗积分饱和的积分作用：
13.        u_i(k) = u_i(k-1) + K_i * T_s * e(k) + (1/T_t) * T_s * e_sat(k-1)
14.      计算总控制作用 u_c(k) = u_p(k) + u_i(k)
15.      应用执行器饱和 u_sat(k) = min(max(u_c(k), u_min), u_max)
16.      计算饱和误差 e_sat(k) = u_sat(k) - u_c(k)
17.      应用控制作用 u_sat(k)
```

**算法2：条件积分抗积分饱和的积分作用计算修改**

```
1. 计算带条件积分的积分作用 u_i(k)：
   选项1：
     如果 e_sat(k-1) == 0：
       u_i(k) = u_i(k-1) + K_i * T_s * e(k)
     否则：
       u_i(k) = u_i(k-1)
   选项2：
     u_i(k) = u_i(k-1) + (e_sat(k-1) == 0) * K_i * T_s * e(k)
```

### 4.2 混合方案

除了标准方法外，还提出了结合多种技术优势的混合方案，以改善饱和下的性能。其中，（Visioli, 2003）引入的将条件积分与回算相结合的方法代表了简单性和有效性之间的实用折中。该方案是图4和图6所示方案的组合，其中作者建议的跟踪时间常数为T_t = 0.03T_i，积分作用由下式获得：

$$u_i(k) = \begin{cases} u_i(k-1) + \frac{K_p}{T_i} T_s e(k) + \frac{1}{T_t} T_s e_{sat}(k-1), & \text{if condition} \\ u_i(k-1) + \frac{K_p}{T_i} T_s e(k), & \text{otherwise} \end{cases} \tag{8}$$

其中条件在控制信号饱和、系统误差与操纵变量同号、且系统输出已离开其先前设定值时满足。数学上可表示为：

$$[u(k-1) \neq u_{sat}(k-1)] \wedge [u(k-1) \times e(k-1) > 0] \tag{9}$$

且

$$\begin{cases} y(k) > y(k-2) & \text{if } y(k-1) > y(k-2) \\ y(k) < y(k-2) & \text{if } y(k-1) < y(k-2) \end{cases} \tag{10}$$

### 4.3 回算的其他调参规则

如第3节所讨论的，除经典解外，一些作者提出了增强执行器饱和下控制器性能的新策略。动态回算方案的性能将与其他调参规则进行比较，包括众所周知的经验法则。

在设定值跟踪问题中，将测试（Markaroglu et al., 2006）中提出的T_t调参规则。该规则利用了选择大值和小值的优势。当控制器输出饱和时，选择足够大的T_t值以允许过程输出非常快速地增加（T_t^1 = 10T_i），同时在饱和中保持更长时间。当测量输出达到系统参考值的某个百分比值c时，T_t减小到T_t^new = βT_i，通过抑制积分项来避免大的超调。c的值取决于开环静态增益K和饱和输出的最大值u_max：

$$c = \begin{cases} \left(-0.5 \frac{u_{max} K}{w} + 1.4\right) \times 100 & \text{for } 1 \leq R_c \leq 2.6 \\ 10 & \text{for } 2.6 < R_c \end{cases}$$

其中 $R_c = u_{max} K / w$。此外，β取决于T/L比：

$$\beta = 0.59 - 0.65 \cdot e^{-0.09 \frac{T}{L}} \tag{11}$$

**算法3：更改T_t的带回算抗积分饱和PI控制器修改**

```
1. 计算 c 并选择 T_t
   R_c = u_max * K / w(k)
   c = (R_c > 2.6) * 0.1 + (R_c ≤ 1) + (R_c > 1 && R_c ≤ 2.6) * (-0.5R_c + 1.4)
   T_t = (y(k) ≤ w(k) * c) * T_t^1 + (y(k) > w(k) * c) * T_t^new
```

另一方面，对于扰动抑制问题，第5节将为无前馈补偿器的方案中的负载扰动提出新的T_t调参规则。

---

## 5. 新提出的解决方案

本节提出了解决积分饱和的新思路。第一个解决方案提出了一种结合现有方案的新抗积分饱和策略，第二个解决方案基于优化程序为回算方案中使用的跟踪时间常数提供了新的调参规则。

### 5.1 新的混合抗积分饱和方案

本节提出了一种新方法，其中两个互补的抗积分饱和机制在每个控制周期内依次应用。在计算未饱和控制器输出后，算法首先检查饱和误差e_sat是否与当前积分增量Δu_i = K_i T_s e(k)符号相反。如果此条件成立，表明积分器正在主动将控制信号进一步推入饱和。如果是这种情况，则应用直接校正：控制器输出被移动饱和过量的绝对值和积分步长绝对值中的较小者，方向为减少过量的方向。这种条件裁剪有效地防止积分器在饱和边界之外实时累积，同时在系统在线性范围内运行时不干扰积分作用。

在此条件校正之后，应用经典的动态回算项，将饱和误差乘以相应增益1/T_t加到积分项上。这提供了通过跟踪时间常数可调的、平滑的剩余积分饱和放电。组合产生两级校正：第一级是激进的和瞬时的，在同一采样内对大饱和事件做出反应，而第二级以平滑和可调的方式控制恢复动态。它们共同旨在将积分器状态保持在饱和边界附近，而不是允许其远远超出。

该混合方法的主要优势是其响应能力：条件裁剪阶段在积分饱和开始时立即反应，无需调参，而回算阶段为设计者提供了塑造恢复行为的自由度，使该方法比单独的回算对突然的大设定值变化更具鲁棒性。

必须注意的是，要实现此抗积分饱和方案，需要增量PI算法。因此，算法4展示了控制作用计算。

**算法4：带混合抗积分饱和的增量PI控制器**

```
对于每个采样时刻 k：
  1. 测量
  2. 控制作用计算
    计算比例作用增量 Δu_p = K_p * (e(k) - e(k-1))
    计算积分作用增量 Δu_i = K_i * T_s * e(k)
    计算总控制作用 u_c(k) = u_c(k-1) + Δu_p + Δu_i
    应用执行器饱和 u_sat(k) = min(max(u_c(k), u_min), u_max)
    计算饱和误差 e_sat(k) = u_sat(k) - u_c(k)
    检查积分作用方向并校正：
      如果 sign(e_sat(k)) == sign(Δu_i(k))：
        u_c(k) = u_c(k) - sign(e_sat(k)) * min(|e_sat(k)|, |Δu_i(k)|)
    应用抗积分饱和第二阶段：u_c(k) = u_c(k) - min(T_s/T_t, 1) * e_sat(k)
    应用执行器饱和 u_sat(k) = min(max(u_c(k), u_min), u_max)
    应用控制作用 u_sat(k)
```

在应用抗积分饱和方案的第二阶段时，可以选择动态回算（选择T_t > T_s）或瞬时回算（选择T_t < T_s）。

### 5.2 负载扰动抑制回算跟踪时间常数调参规则

最后，本节包含一套回算方案中跟踪时间常数的调参规则。尽管该技术在工业中广泛使用，但跟踪参数的选择通常基于启发式指南。所提出的规则旨在提供更系统的、面向性能的调参程序，特别是在扰动抑制是主要目标的场景中。

在图8中，展示了激励所提调参规则改进的示例。系统性能在无饱和、有饱和和动态回算（T_t = T_i）、有饱和和瞬时回算（T_t = T_s）、以及有饱和和使用最优跟踪时间常数的动态回算（T_t = T_t^opt）的情况下进行了比较。如前所述，扰动为双阶跃形状，只要它持续存在，控制作用就保持饱和。因此，所开发的规则有望使系统在扰动消退后更快地返回工作点。

提出了几种规则和指南供用户应用，取决于可用信息。第一步是表征系统并收集相关信息。允许用户选择更好跟踪时间常数的变量是饱和比R_S、控制器激进程度x和扰动与过程动态之比。

**饱和比**是执行器饱和程度的定量度量，定义为：

$$R_S = \frac{u_f - u_{lim}}{u_f - u_0} \tag{12}$$

其中u_f是施加扰动时无饱和情况下的控制作用最终值，u_lim是饱和极限，u_0是扰动前的工作点（Hoyo et al., 2023）。

**控制器激进程度**是闭环与开环时间常数之比，x = λ/T，其中λ是闭环时间常数。最后，**扰动与过程动态之比**是脉冲持续时间与开环时间常数之比D_d/T，其中D_d是脉冲持续时间。

规则的目标是找到因子α的表达式，使跟踪时间常数可计算为T_t = αT_i，从而实现更好的性能。为此，针对各种情况启动了优化问题，其中特征参数被变化。该研究包括R_S从0.05到0.95、x从0.2到1、D_d/T从1/3到10的值。此程序创建了多个曲面和一个三维空间，可以拟合曲线。最后，简化该表达式，在并非所有信息都可用时提供简单解。

最优α值(α*)的曲面如图9所示。一般来说，α*总是小于1，意味着跟踪时间常数应总是小于积分时间。此外，对于小的R_S值，α* = T_s/T_i，意味着T_t = T_s且最优策略是瞬时回算。对于中到高R_S值，α*的最优值在T_s/T_i和约0.9之间变化。最后，随着控制器激进程度的降低（更高的x），α值应更小。

α始终可计算为：

$$\alpha = \max\left\{ f(R_S, D_d, T, x), \frac{T_s}{T_i} \right\} \tag{13}$$

其中f因规则而异。

**规则1**：第一条规则利用了表征系统的所有参数：

$$f(R_S, D_d, T, d_x) = -1.2 + 3.3(R_S - d_x) - 1.26(R_S - d_x)^2 - 0.6 e^{-1.2 D_d/T} \tag{14}$$

其中 $d_x(x) = -0.28 + 0.8x - 0.3x^2$ 表示曲面在不同x值下在R_S轴上的位移。

**规则2**：第二条规则适用于事先不知道扰动持续时间的情况：

$$f(R_S, x) = -0.3 - 0.63x + 1.5 R_S \tag{15}$$

**简单指南**：最后，对于只能估计R_S的情况，提出了简单指南。从图9可以看出，每个x值都有一个R_S值，低于该值时α* = T_s/T_i。高于此值（称为R_Slim），α*取T_s/T_i和最大值α_max之间的值，α_max始终小于1。图10显示了两个极限作为x函数的曲线。对于低于R_Slim的饱和比，最优抗积分饱和策略是瞬时回算。另一方面，高于这些值时，用户可以选择α_max曲线以下的α值，尽管不能获得最优行为，但与使用α = 1相比仍能获得显著改善。

---

## 6. 策略比较

本节将在各种条件下比较第4和第5节描述的所有抗积分饱和技术和调参规则，旨在找到简单的指南，允许用户在每种情况下选择最佳的抗积分饱和配置。

**表1：策略编码**

| 策略名称 | 编码 |
|---------|------|
| 动态回算（T_t = T_i） | DBC1 |
| 瞬时回算 | IBC |
| 条件积分 | CI |
| 条件积分与回算（Visioli, 2003） | H1 |
| 条件积分与回算（本文提出） | H2 |
| 动态回算，(Markaroglu et al., 2006)规则 | DBC_STr |
| 动态回算，扰动抑制规则1 | DBC_R1 |
| 动态回算，扰动抑制规则2 | DBC_R2 |

如前所述，将研究设定值跟踪和扰动抑制问题。此外，设定值跟踪包括控制作用在瞬态期间和稳态下饱和的问题。为了获得尽可能通用的结论，分析了三种类型的过程，其中T/L比分别取1/6、1/2和1。此外，在每种情况下，参数按照表2变化，以确保研究了广泛的条件。

**表2：抗积分饱和方案比较的研究条件**

| 参数 | 值 |
|------|-----|
| **设定值跟踪 - 瞬态期间饱和** | |
| x | 0.2; 0.5; 0.8 |
| R_S | 从R_Smin到0.95 |
| **设定值跟踪 - 稳态饱和** | |
| x | 0.2; 0.5; 0.8 |
| R_S | 从0.05到0.95 |
| **扰动抑制** | |
| x | 0.2; 0.5; 0.8 |
| R_S | 0.35; 0.55; 0.8 |
| D_d/T | 从1/3到10 |

所有策略根据输出性能进行比较，计算IAE（绝对误差积分）：

$$IAE = \sum_{k=1}^{N} |e(k)| \tag{16}$$

其中k是实际时间时刻，从1到仿真长度N。

### 6.1 设定值跟踪问题

#### 6.1.1 瞬态期间饱和

对于控制信号瞬态饱和的情况，比较的策略是DBC1、IBC、CI、DBC_STr、H1和H2。图12显示了T/L = 1/6时每种策略的结果。每个控制器激进程度由一个子图表示，其中归一化IAE作为饱和比的函数绘制。性能指标相对于DBC1（最经典和最广泛实施的策略）进行归一化。

可以看出：
- 对于L/T = 1/6的过程，动态回算始终是最佳抗积分饱和方案，DBC1对于x = 0.2（无论饱和比如何）是最佳选择
- DBC_STr实现了类似的行为，在饱和比超过0.3时IAE仅高不到10%
- 其余方案（IBC、CI、H1和H2）表现较差，IBC的IAE最高可高65%

对于滞后半时间常数的过程（L/T = 1/2），DBC_STr从来不是最佳选择。最优策略因情况而异：对于x = 0.2，如果R_S ≤ 0.5，IBC、CI、H1和H2可实现高达8%的改善。对于更高的饱和比，DBC1获得最佳性能。

对于L = T的平衡过程，所有策略都优于DBC1，在x = 0.2时获得高达7%的性能改善。

**定值跟踪瞬态饱和的指南**：
- **高滞后主导过程（L/T = 1/6）**：激进控制器使用DBC1，保守调参使用DBC_STr
- **中等滞后时间常数比（L/T = 1/2）**：激进控制器在低饱和比时使用IBC/CI/H1/H2，高饱和比时使用DBC1
- **平衡过程（L = T）**：在所有条件下IBC、CI、H1和H2更优
- 最多可观察到70%的性能改善，选择错误的技术在最坏情况下可能使性能下降15%

#### 6.1.2 稳态饱和

在设定值跟踪问题中可能遇到的第二种饱和类型中，参考值无法达到，因为执行器极限小于实现目标所需的最终控制作用。DBC_STr不包含在此分析中。

图17显示了根据控制器激进程度、滞后主导性和饱和比，各策略实现的归一化IAE。可以看出：
- IBC、H1和H2在所有可能的组合下实现相同的IAE性能
- DBC1始终是激进控制器（x = 0.2）、中等激进控制器（x = 0.5）和中到高滞后主导性（L/T ≥ 1/2）的最佳选择
- 对于保守调参，在中到低R_S值时，IBC、CI、H1和H2是最佳方案
- DBC1通常是最优策略，除非对于具有中到保守控制器调参的滞后主导过程，CI在所有R_S范围内产生最小IAE（最多改善25%）
- 在高饱和比情况下选择错误策略可能导致性能比最优差15倍

### 6.2 扰动抑制问题

在扰动抑制问题中，比较的策略是DBC1、IBC、CI、DBC_R1、DBC_R2和H2。

**对于低L/T比的滞后主导过程（L/T = 1/6）**：
- 无论控制器激进程度如何，对于低到中等饱和比，DBC1始终是最差选择
- 对于激进控制器调参（x = 0.2），DBC_R1在整个扰动持续时间范围内实现最佳行为
- DBC_R2在D_d/T > 1时获得相同行为，但对较短扰动表现较差
- IBC和H1在低饱和比下实现接近最优的性能，但在中到高R_S值时表现不佳

**对于中等滞后时间常数比（L/T = 1/2）**：
- DBC1从来不是最佳选择
- 对于低饱和比，其余策略在所有x值下表现几乎相同
- 对于中等R_S值，IBC是IAE最低的最佳选择
- 对于x = 0.2和R_S = 0.8，IBC变为最差选择之一，H2对于D_d/T ≤ 1呈现最优行为

**对于平衡过程（T = L）**：
- 对于低饱和比，除DBC1外所有策略行为相似，性能比DBC1好15%
- 随着饱和比增加，IBC和H2获得最佳性能，改善高达20%

**扰动抑制的实用指南**：
- DBC_R1和DBC_R2在扰动持续时间已知时特别有效
- H2在各种条件下表现稳健
- IBC在中等饱和比时通常表现良好
- DBC1在大多数扰动抑制场景中不是最佳选择

---

## 7. 结论

本文对PI控制系统在执行器饱和下的抗积分饱和策略进行了广泛分析，重点关注实际适用性和性能优化。结果强调了抗积分饱和方案的选择和调参在决定闭环性能中的关键作用，以及常用的经验法则往往远离最优。

经典技术如动态回算和条件积分由于其简单性仍然具有竞争力，但其有效性强烈取决于过程动态、控制器激进程度和饱和性质。瞬时回算和条件方案在强滞后主导或低饱和水平的场景中往往表现更好，而动态回算通常更适合激进控制器和更高的饱和比。

所提出的混合抗积分饱和策略（将条件积分与回算相结合）展示了在饱和事件期间改善的响应能力和对大扰动增强的鲁棒性，在简单性和性能之间提供了实用的折中。

本文的一个主要贡献在于为回算方案中的跟踪时间常数开发了系统化的调参规则。这些规则源自大规模优化研究，基于可测量的系统特性（如饱和比、控制器激进程度和扰动动态）提供明确的指南。结果表明，最优跟踪时间常数通常小于积分时间，并且随工况条件显著变化，这与传统的启发式建议相矛盾。

比较研究进一步表明，没有单一的抗积分饱和策略是普遍最优的；相反，最佳选择取决于特定的控制目标（设定值跟踪vs.扰动抑制）、过程特性和可用的系统信息。然而，所提出的调参规则和选择指南使从业者能够以最小的努力实现接近最优的性能。总体而言，本文通过提供直观的、可实现的、面向性能的抗积分饱和设计和调参解决方案，弥合了理论发展与工业实践之间的差距。

---

## 致谢

本工作由以下项目资助：西班牙科学部资助的PID2023-150739-I00和PDC2025-165379-I00。隆德大学合作者是ELLIIT战略研究领域的成员。Malena Caparroz感谢西班牙科学、创新和大学部在FPU23/02235资助下的财政支持。

---

## 参考文献

1. Alexis, E., Cardelli, L., & Papachristodoulou, A. (2022). On the design of a PID bio-controller with set point weighting and filtered derivative action. *IEEE Control Systems Letters*, 6, 3134–3139.
2. Alfaro, V. M., & Vilanova, R. (2013). Performance and robustness considerations for tuning of proportional integral/proportional integral derivative controllers with two input filters. *Industrial & Engineering Chemistry Research*, 52(51), 18287–18302.
3. Amiri, P., Theodoridis, T., Müller, M., & Holderbaum, W. (2024). Anti-windup methods for integrators of integral controllers for robotic applications. In *2024 IEEE 22nd Jubilee International Symposium on Intelligent Systems and Informatics (SISY)* (pp. 000267–000276).
4. Åström, K. J., & Murray, R. (2021). *Feedback systems: an introduction for scientists and engineers*. Princeton University Press.
5. Bohn, C., & Atherton, D. (1995). An analysis package comparing PID anti-windup strategies. *IEEE Control Systems Magazine*, 15(2), 34–40.
6. Bohn, C., & Atherton, D. P. (2002). An analysis package comparing PID anti-windup strategies. *IEEE Control Systems Magazine*, 15(2), 34–40.
7. Bruciapaglia, A., & Apolˆonio, R. (1986). Uma estratégia de eliminação da sobrecarga da ação integral para controladores PID discretos. In *II Congresso Latino Americano de Controle Automático* (pp. 519–524).
8. B˘usek, J., Vyhlídal, T., & Zítek, P. (2017). IAE based tuning of controller anti-windup schemes for first order plus dead-time system. In *2017 21st International Conference on Process Control (PC)* (pp. 18–23).
9. Cao, Y.-Y., Lin, Z., & Ward, D. G. (2004). Anti-windup design of output tracking systems subject to actuator saturation and constant disturbances. *Automatica*, 40(7), 1221–1228.
10. Caparroz, M., Soltesz, K., Hägglund, T., & Guzmán, J. L. (2026). A novel tuning rule for the tracking constant parameter in back-calculation anti-windup schemes. In *Proceedings of the 23rd IFAC World Congress*. (Accepted for publication / In press)
11. Glattfelder, A., & Schaufelberger, W. (1986). Start-up performance of different proportional-integral-anti-wind-up regulators. *International Journal of Control*, 44(2), 493–505.
12. Goodwin, G. C., Graebe, S. F., Salgado, M. E., et al. (2001). *Control system design* (Vol. 240). Prentice Hall Upper Saddle River.
13. Hägglund, T. (2013). A unified discussion on signal filtering in PID control. *Control Engineering Practice*, 21(8), 994–1006.
14. Hanus, R., Kinnaert, M., & Henrotte, J.-L. (1987). Conditioning technique, a general anti-windup and bumpless transfer method. *Automatica*, 23(6), 729–739.
15. Hoyo, A., Hägglund, T., Guzmán, J. L., & Moreno, J. C. (2023). A practical solution to the saturation problem in feedforward control for measurable disturbances. *Control Engineering Practice*, 139, 105636.
16. Hui, K., & Chan, C. (1997). New design methods of actuator saturation compensators for proportional, integral and derivative controllers. *Proceedings of the Institution of Mechanical Engineers, Part I*, 211(4), 269–280.
17. Kothare, M. V., Campo, P. J., Morari, M., & Nett, C. N. (1994). A unified framework for the study of anti-windup designs. *Automatica*, 30(12), 1869–1883.
18. Kothare, M. V., & Morari, M. (1997). Stability analysis of anti-windup control systems: A review and some generalizations. In *1997 European Control Conference (ECC)* (pp. 2156–2161).
19. Kumar, S., & Negi, R. (2012). A comparative study of PID tuning methods using anti-windup controller. In *2012 2nd International Conference on Power, Control and Embedded Systems* (pp. 1–4).
20. Markaroglu, H., Guzelkaya, M., Eksin, I., & Yesil, E. (2006). Tracking Time Adjustment In Back Calculation Anti-Windup Scheme. In *ECMS 2006 Proceedings* (pp. 613–618).
21. Morohoshi, Y., & Deng, M. (2025). Nonlinear back-calculation anti-windup based on operator theory. *Processes*, 13(5), 1266.
22. Okelola, M. O., Aborisade, D. O., & Adewuyi, P. A. (2020). Performance and configuration analysis of tracking time anti-windup PID controllers. *Jurnal Ilmiah Teknik Elektro Komputer dan Informatika*, 6(2), 20–29.
23. Peng, Y., Vrancic, D., & Hanus, R. (1996). Anti-windup, bumpless, and conditioned transfer techniques for PID controllers. *IEEE Control Systems Magazine*, 16(4), 48–57.
24. Rojas, J. D., Arrieta, O., & Vilanova, R. (2021). *Industrial PID controller tuning*. Springer.
25. Rundqwist, L. (1990). Anti-reset windup for PID controllers. *IFAC Proceedings Volumes*, 23(8), 453–458.
26. Sarhadi, P. (2026). Simple yet effective anti-windup techniques for amplitude and rate saturation: An autonomous underwater vehicle case study. *arXiv preprint arXiv:2601.01302*.
27. Shin, H.-B. (1998). New antiwindup PI controller for variable-speed motor drives. *IEEE Transactions on Industrial Electronics*, 45(3), 445–450.
28. Silva, L. R. d., et al. (2017). Controle de sistemas com atraso e saturação: estudo comparativo entre um GPC e um DTC com anti-windup.
29. Skogestad, S. (2003). Simple analytic rules for model reduction and PID controller tuning. *Journal of Process Control*, 13(4), 291–309.
30. Sundström, E., Bauer, M., Guzmán, J. L., Hägglund, T., & Soltesz, K. (2026). A practical guide to PID controller implementation. *arXiv preprint arXiv:2604.15918*.
31. Torstensson, E. (2013). Comparison of schemes for windup protection.
32. Visioli, A. (2003). Modified anti-windup scheme for PID controllers. *IEE Proceedings - Control Theory and Applications*, 150(1), 49–54.
33. Visioli, A. (2006). *Practical PID control*. London, England: Springer.
34. Walgama, K., & Sternby, J. (1990). Inherent observer property in a class of anti-windup compensators. *International Journal of Control*, 52(3), 705–724.
35. Walgama, K. S., Rännbäck, S., & Sternby, J. (1992). Generalisation of conditioning technique for anti-windup compensators. In *IEE Proceedings D (Control Theory and Applications)* (Vol. 139, pp. 109–118).
36. Wang, L. (2016). Control design methods with anti-windup mechanism for disturbance rejection and reference following. *Transactions of the Institute of Measurement and Control*, 38(6), 625–639.
37. Willis, M. J. (1999). Proportional-integral-derivative control. *Dept. of Chemical and Process Engineering University of Newcastle*, 6, 28.
38. Yapp, K. K., Hoo, C. L., & Lai, C. H. (2024). New anti-windup proportional-integral-derivative for motor speed control. *Asian Journal of Control*, 26(6), 2854–2866.
39. Zaccarian, L., & Teel, A. R. (2011). *Modern anti-windup synthesis: control augmentation for actuator saturation*. Princeton University Press.
40. Åström, K. J., & Hägglund, T. (2006). *Advanced PID control*. Research Triangle Park, NC: ISA.
