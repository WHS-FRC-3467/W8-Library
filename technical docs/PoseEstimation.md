> **Scope:** A living document summarizing the strategy and mathematics behind our pose estimation strategies. 

    Note that all vector components that are angles are in radians unless expressed otherwise.

## 1. The estimation problem and notation

The overall goal is to estimate the robot's field-relative pose using two imperfect sources: wheel/gyro odometry and vision. Odometry can estimate where the robot is through tracking wheel spin/direction and time; vision can estimate where the robot is through capturing images of fiducials installed at known locations around the field. The strategy will be to fuse the readings from these two sources in some coherent way. Before we start, we'll review some fundamental notation and definitions. It's not important to understand all of these parameters immediately -- we'll elaborate as they come up in the review.

| Symbol | Meaning |
| --- | --- |
| $\hat{\mathbf{x}}=[\hat x,\hat y,\hat\theta]^T$ | Estimated field-relative robot pose. <br><br> **Notes:** <br>(1) The subscript **k** represents the _current_ robot loop. <br>(2) The subscript **k+1** represents the _next_ robot loop. <br>(3) The superscript **-** represents _before_ a vision correction. <br>(4) The superscript **+** represents _after_ a vision correction.
| $\mathbf{z}=[x_v,y_v,\theta_v]^T$ | Field-relative pose reported by vision |
| $\mathbf{u}=[dx, dy, d\theta]^T$ | Body-fixed frame odometry twist (i.e. twist coordinates) |
| $P$ | Estimated covariance of the pose error; a $3\times3$ matrix |
| $R$ | Estimated covariance of a given individual vision measurement |
| $Q$ | Process-noise covariance for one odometry interval, in twist coordinates |
| $F$ | Jacobian of the motion model with respect to the **previous state** |
| $G$ | Jacobian mapping odometry/process noise into the field-relative state |
| $H$ | Observation matrix selecting the vision-measured state components |
| $K$ | Kalman gain: maps the innovation into a pose correction |

Covariance is a **model of estimation error (also referred to as uncertainty or noise)**. Fundamentally, this matrix is tracking what it believes its error bounds and error covariances to be based on the estimator's calibrated inputs provided at construction and hardware measurements over the course of the match. Note that this is fundamentally different from the error measured against "pose ground truth" during a match, because pose ground truth during a match is not known. For an error vector $\mathbf e=\hat{\mathbf x}-\mathbf x_{\mathrm{true}}$ with approximately zero mean (i.e. the error is unbiased):

$$
P=\mathrm{Cov}(\mathbf e)=\mathbb E[\mathbf e\mathbf e^T]
=\begin{bmatrix}
\sigma_x^2&P_{xy}&P_{x\theta}\\
P_{xy}&\sigma_y^2&P_{y\theta}\\
P_{x\theta}&P_{y\theta}&\sigma_\theta^2
\end{bmatrix}.
$$

The diagonals are variances, so their square roots are standard deviations. Assuming approximately Gaussian, unbiased estimation errors and a properly calibrated covariance model, each diagonal variance defines a marginal three-sigma confidence interval:

$$
\begin{aligned}
x_{\text{true}} &\in [\hat{x}_k - 3\sqrt{\sigma_x^2},\ \hat{x}_k + 3\sqrt{\sigma_x^2}] \\
y_{\text{true}} &\in [\hat{y}_k - 3\sqrt{\sigma_y^2},\ \hat{y}_k + 3\sqrt{\sigma_y^2}] \\
\theta_{\text{true}} &\in [\hat{\theta}_k - 3\sqrt{\sigma_\theta^2},\ \hat{\theta}_k + 3\sqrt{\sigma_\theta^2}]
\end{aligned}
$$

As the diagonal variances approach zero, the estimator reports increasing confidence in its pose. This confidence is meaningful only to the extent that the underlying process and measurement uncertainty models are properly calibrated.


The off-diagonal elements of $P_k$ represent **covariance between estimation errors**. For example, assuming unbiased (zero-mean) errors:

$$
P_{y\theta} = \mathrm{Cov}(\delta y,\delta\theta)
= \mathbb{E}[\delta y\,\delta\theta]
\approx \frac{1}{N}\sum_{i=1}^{N}\delta y_i\,\delta\theta_i
$$

Here, $\mathbb{E}$ is the expected-value operator (the probability-weighted population mean), and $\delta y$ and $\delta\theta$ are the unknown errors in estimated Y position and heading. A positive covariance indicates that these errors tend to have the same sign; a negative covariance indicates opposite signs. This off-diagonal covariance relationship is important because it indicates that degree to which errors are correlated. This will be important when we later combine measurements (specifically, these off-diagonals feed _additional_ corrections into the various independent state variables during the Kalman Gain calculation step -- more on this later).

 Now we'll examine two methods for estimating our robot pose: a _Simplified Kalman Filter (SKF)_ and an _Extended Kalman Filter (EKF)_. We begin with the SKF.

## 2. Simplified Kalman Filter (SKF)

### 2.1 Derivation

Consider one coordinate, $x$. Suppose odometry predicts $x_o$ and vision measures $x_v$. We need to reconcile these two measurements. One approach is a weighted fusion, expressed as:

$$
\boxed{\hat x_{\mathrm{fused}}=(1-k)x_o+kx_v=x_o+k(x_v-x_o).}
$$

Here $k=0$ ignores vision; $k=1$ fully adopts vision. To choose $k$ mathematically, write the individual estimates as the true position plus errors:

$$
x_o=x_{\mathrm{true}}+e_o,\qquad x_v=x_{\mathrm{true}}+e_v.
$$

Substitution gives the fused error:

$$
e_{\mathrm{fused}}=(1-k)e_o+ke_v.
$$

For **unbiased, mutually independent** errors with _constant_ variances $P=\mathrm{Var}(e_o)=\sigma_o^2$ and $R=\mathrm{Var}(e_v)=\sigma_v^2$, the covariance cross-term is zero:
$$
\begin{aligned}
\mathrm{Var}(e_{\mathrm{fused}})
&=(1-k)^2P+k^2R\\
&\quad+2k(1-k)\mathrm{Cov}(e_o,e_v)\\
&=(1-k)^2P+k^2R.
\end{aligned}
$$

To find the local minimum in error, differentiate with respect to $k$ and set the derivative to zero:

$$
\frac{d\mathrm{Var}(e_{\mathrm{fused}})}{dk}
=-2(1-k)P+2kR=0.
$$

Thus:

$$
\boxed{k=\frac{P}{P+R}.}
$$

This **scalar Kalman gain** is an uncertainty-weighted average and represents the mathematically optimal weighting to minimize uncertainty within the SKF framework. A larger odometry variance gives vision more influence; a larger vision variance gives it less. Within this approach, $P$ is usually assumed constant and its diagonal equals the numerical _odometry error_ variance. In EKF, this will generalize to the _fused pose covariance_. $R$ is also usually assumed constant and its diagonal equals of the numerical _vision error_ variance. Both are measured under representative driving conditions. The off-diagonal terms of $P$ and $R$ are zeroed in this approach, however, the matrices themselves are still technically referred to as covariance matrices. 

### 2.2 Numerical Example

We'll baseline the application of the SKF through the following numerical example. Let's assume the following:

- Initial robot pose: $(2.00\ \mathrm m,\,3.00\ \mathrm m,\,0^\circ)$.
- Wheel/gyro odometry reports $0.06\ \mathrm m$ forward in $\Delta t=0.020\ \mathrm s$, with no heading change, so the predicted pose is $(2.06,3.00,0^\circ)$ and speed is $3\ \mathrm{m/s}$.
- A two-tag camera observation at the same timestamp reports $(2.16\ \mathrm m,3.04\ \mathrm m,10^\circ)$. Average tag distance is $d=3\ \mathrm m$.
- Slightly anisotropic odometry variances are as follows (however, note that it's very common to assume complete isotropy such that $\sigma_x^2 = \sigma_y^2$):

$$
P_{\mathrm{fixed}}=\mathrm{diag}(0.0037002,\ 0.00370164,\ 0.0004002).
$$

- Isotropic vision variances are as follows: 

$$
R_{\mathrm{baseline}}=\mathrm{diag}(0.0004,\ 0.0004,\ 0.0036).
$$

- Vision observations that utilize numerous tags at a short distance are taken as more reliable than the inverse. Therefore, we'll trust high-quality measurements (and distrust low-quality measurements) more according to the following scaling rule:
$$
\sigma_{xy,v}=\frac{\sigma_{xy,v,baseline}d^2}{n},
\qquad
\sigma_{\theta,v}=\frac{\sigma_{\theta,v,baseline}d^2}{n}
$$

To begin, let's scale our baseline vision uncertainty according to our captured frame conditions ($d=3\ \mathrm m$ & $n=2$):

$$
\sigma_{xy,v}=\frac{0.02(3)^2}{2}=0.09\ \mathrm m,
\qquad
\sigma_{\theta,v}=\frac{0.06d^2}{n}=0.27\ \mathrm{rad}.
$$

This produces our corrected vision uncertainty for use in the fusion operation:

$$
R_{\mathrm{corrected}}=\mathrm{diag}(0.0081,\ 0.0081\ 0.0729).
$$

Apply the scalar gain independently to each coordinate:

$$
\begin{aligned}
k_x&=\frac{0.0037002}{0.0037002+0.0081}=0.313571,\\
k_y&=\frac{0.00370164}{0.00370164+0.0081}=0.313655,\\
k_\theta&=\frac{0.0004002}{0.0004002+0.0729}=0.00545974.
\end{aligned}
$$

With the camera's innovation $(0.10\ \mathrm m,\ 0.04\ \mathrm m,\ 10^\circ)$, this produces:

$$
\boxed{\hat{\mathbf x}_{\mathrm{SKF}}\approx
\begin{bmatrix}2.091357\ \mathrm m\\3.012546\ \mathrm m\\0.054597^\circ\end{bmatrix}.}
$$

SKF adjusts the estimate toward vision by a fraction determined by configured variances, but it does not inherently remember how uncertainty evolved since the previous correction or how heading and position errors became correlated.

### 2.3 Limitations of SKF

- **Odometry uncertainty changes over time.** Driving farther without vision, slipping, and heading errors all affect the reliability of the predicted pose -- these dynamics increase modeled error (i.e. the diagonals of $P$) over time. Naturally, the representative fixed $P_{SKF}$ utilized above does not explicitly accumulate these effects. Such a fixed value *can* approximate a steady-state operating regime, but not arbitrary motion history. As we'll see, this approach is most viable when the robot is receiving high-quality vision measurements at sufficient FPS such that accumulated uncertainty is consistently decimated from loop-to-loop. 

- **State errors are correlated.** After driving forward, a heading error induces lateral displacement error. Three independent scalar weights ignore the resulting $P_{y\theta}$ terms. Neglecting these effects is most viable with high-quality vision at sufficient FPS.

- **Vision observations are not equally reliable.** A baseline $d^2/n$ model describes some variation, but robot motion, camera configuration, and observation quality can also matter. A fixed baseline coefficient may have absorbed their *typical* effects during tuning, but don't represent arbitrary vision conditions.

Fundamentally, SKF uses a blanket **representative uncertainty** rather than dynamically predicting its evolution and correcting it. EKF, derived below, takes a seeded uncertainty from a reset pose and evolves / corrects it as a function of time.

## 3. Extended Kalman Filter 

This is a more advanced approach that initially seeds and evolves $P$ recursively through prediction and correction and generates $R$ independently for each vision observation. 

### 3.1 Robot Motion Model

To begin, we should review the robot's motion model. The most general motion the robot can execute is an odometry **twist**. A twist occurs when the robot simultaneously translates and rotates. After one of these profiles, the robot swerve drive returns a $[dx,dy,d\theta]^T$ in the form of WPILib's `Twist2d`. These components don't represent a field-relative displacement -- they represent transform coordinates in robot's time-varying body-fixed frame over the course of the motion from $t_0$ to $t_f$ quantified through drive and steer-encoder readings from all four modules. However, because the body-fixed frame is rotating while it's translating, these components aren't expressed in the $t_0$ robot-frame either. 

To express this twist in the $t_0$ robot frame, we essentially need to play back all of the rotation matrices for each state variable at every given time $t \in [t_0, t_f]$ and sum the results together. This is done automatically by applying an integration matrix $V$. Let $\phi=\Delta\theta$ and:

$$
R(\theta)=\begin{bmatrix}\cos\theta&-\sin\theta\\\sin\theta&\cos\theta\end{bmatrix},
\qquad
V(\phi)=\begin{bmatrix}
\frac{\sin\phi}{\phi}&-\frac{1-\cos\phi}{\phi}\\
\frac{1-\cos\phi}{\phi}&\frac{\sin\phi}{\phi}
\end{bmatrix}.
$$

With $R(\theta)$ being the rotation matrix for a given heading $\theta$ and $V(\phi)$ being the twist integration matrix as provided by WPILib. $R(\theta_k)$ rotates the resulting displacement into field coordinates where $\theta_k$ is the robot's starting heading before the execution of the twist. 

$$
\begin{bmatrix}\Delta x_r\\\Delta y_r\end{bmatrix}=V(\phi)\begin{bmatrix}dx\\dy\end{bmatrix}=\begin{bmatrix}a\\b\end{bmatrix}
$$

where $[a, b]^T$ is utilized for brevity. We can rotate this $t_0$ robot-frame displacement into field coordinates using $R(\theta_k)$ such that:

$$
\begin{bmatrix}\Delta x_f\\\Delta y_f\end{bmatrix}=R(\theta_k)\begin{bmatrix}a\\b\end{bmatrix}
$$

then the predicted state is:

$$
\boxed{
\begin{aligned}
x_{k+1}&=x_k+a\cos\theta_k-b\sin\theta_k,\\
y_{k+1}&=y_k+a\sin\theta_k+b\cos\theta_k,\\
\theta_{k+1}&=\theta_k+\phi.
\end{aligned}}
$$

This is the planar rigid-body exponential mapping implemented through WPILib's `Pose2d.exp(twist)`. To see why this matters, imagine moving the robot forward by a body twist of $1\ \mathrm m$ while turning $90^\circ$, starting at $\theta=0$ -- the equations above predict a field displacement $(2/\pi,2/\pi)\ \mathrm m$, not $(1,0)\ \mathrm m$.

### 3.2 Deriving the State Jacobian $F$

A covariance requires a model of how small existing pose errors influence the next pose. To determine this sensitivity, differentiate the motion model with respect to the prior state $[x,y,\theta]^T$, holding the measured twist fixed:

$$
F=\frac{\partial\mathbf x_{k+1}}{\partial\mathbf x_k}
=\begin{bmatrix}
1&0&-a\sin\theta-b\cos\theta\\
0&1&a\cos\theta-b\sin\theta\\
0&0&1
\end{bmatrix}.
$$

As demonstrated above, our **field-relative** displacement is $\Delta x_f=a\cos\theta_k-b\sin\theta_k$ and $\Delta y_f=a\sin\theta_k+b\cos\theta_k$. Therefore:

$$
\boxed{F=\begin{bmatrix}
1&0&-\Delta y_f\\0&1&\Delta x_f\\0&0&1
\end{bmatrix}.}
$$

For example, $F_{y\theta}=\Delta x_f$ means an uncertain heading produces uncertain lateral position when driving in field X such that $\partial y_{k+1} \approx \Delta x_f\partial\theta_k$. Even with a zero-mean heading error, its **variance** can generate lateral position variance and a Y/heading covariance.

### 3.3 Generating the Covariance Matrix $P$

Linearize the evolution of state-estimation error about the estimated pose:

$$
\delta\mathbf x_{k+1}\approx F_k\mathbf \delta\mathbf x_k+G_k\mathbf w_k,
$$

where $\mathbf w_k$ is the current-step zero-mean, independent process noise with covariance $Q_k=\mathbb E[\mathbf w_k\mathbf w_k^T]$. The _total_ predicted pose error at the next time step consists of the existing estimation error, propagated through the motion model by $F_k$, plus new process noise introduced during the latest odometry interval and mapped into field coordinates by $G_k$.

Form the expected outer product:

$$
\begin{aligned}
P'&=\mathbb E[\delta\mathbf x_{k+1}{\delta\mathbf x_{k+1}}^T]\\
&\approx F\mathbb E[\delta\mathbf x_k{\delta\mathbf x_k}^T]F^T
+G_k\mathbb E[\mathbf w_k\mathbf w_k^T]G_k^T.
\end{aligned}
$$

The cross terms vanish if prior estimation errors and new process noise are uncorrelated. Therefore:

$$
\boxed{P_{k+1}^-=F_kP_k^+F_k^T+G_kQ_kG_k^T.}
$$

The transposes follow from the covariance definition.

### 3.4 Noise mapping $G$ and the interval noise $Q$

Odometry noise is initially modeled in robot/twist coordinates. The implementation uses the approximate noise Jacobian

$$
\boxed{G=\begin{bmatrix}R(\theta)V(\phi)&\mathbf0_{2\times1}\\\mathbf0_{1\times2}&1\end{bmatrix}.}
$$

This rotates/transforms translational noise into the field frame; heading noise is carried directly into heading. Note that this particular $G$ omits the instantaneous effect of uncertainty in the twist's $\Delta\theta$ on its integrated translation. That is an approximation of the full input Jacobian, not an assertion that the coupling is physically zero.

For one odometry interval, `EkfPoseEstimator.Config` defines

$$
\boxed{
Q_k=\mathrm{diag}(q_{xy},q_{xy},q_\theta),\quad
\begin{aligned}
q_{xy}&=k_d\sqrt{\Delta x_r^2+\Delta y_r^2}+k_t\Delta t,\\
q_\theta&=k_\theta\Delta t.
\end{aligned}}
$$

The default coefficients for our non-tipped example are $k_d=0.02\ \mathrm{m^2/m}$, $k_t=10^{-5}\ \mathrm{m^2/s}$, and $k_\theta=10^{-5}\ \mathrm{rad^2/s}$. **These are variance rates, not standard-deviation rates.** This $Q_k$ is newly calculated for each interval, not accumulated since the last vision frame.

## 4. Predicting uncertainty in our example

Start with a pose reset to $(2,3,0)$ and reset standard deviations $\sigma_{xy}=0.05\ \mathrm m$ and $\sigma_\theta=0.02\ \mathrm{rad}$. Thus

$$
P_0^+=\mathrm{diag}(0.0025,\ 0.0025,\ 0.0004).
$$

The robot moves forward $0.06\ \mathrm m$ in $\Delta t=0.02\ \mathrm s$ with $\phi=0$ and $\theta=0$. Therefore $R(0)=V(0)=I$, the field displacement is $(0.06,0)$, and

$$
F=\begin{bmatrix}1&0&0\\0&1&0.06\\0&0&1\end{bmatrix},\qquad G=I.
$$

The interval noise is

$$
\begin{aligned}
q_{xy}&=0.02(0.06)+10^{-5}(0.02)=0.0012002,\\
q_\theta&=10^{-5}(0.02)=0.0000002,\\
Q&=\mathrm{diag}(0.0012002,0.0012002,0.0000002).
\end{aligned}
$$

First propagate the existing uncertainty:

$$
FP_0^+F^T=\begin{bmatrix}
0.0025&0&0\\0&0.00250144&0.000024\\0&0.000024&0.0004
\end{bmatrix}.
$$

Then add the new process noise:

$$
\boxed{P_1^-=\begin{bmatrix}
0.0037002&0&0\\0&0.00370164&0.000024\\0&0.000024&0.0004002
\end{bmatrix}.}
$$

The $0.000024$ Y/heading covariance and the additional $0.00000144$ Y variance come **from the existing heading uncertainty being carried through the forward motion**, even before new odometry noise is added. In the special case of motion $d$ along field X,

$$
P_{yy}'=P_{yy}+2dP_{y\theta}+d^2P_{\theta\theta}+(GQG^T)_{yy}.
$$

The covariance term $2dP_{y\theta}$ can reinforce or partly cancel prior position uncertainty depending on the direction of motion and existing error correlation. The pose itself becomes

$$
\boxed{\hat{\mathbf x}_1^-=[2.06,\ 3.00,\ 0]^T.}
$$

## 5. A camera measurement has its own covariance $R$

Vision supplies $\mathbf z=[x_v,y_v,\theta_v]^T$ and standard deviations estimated for **this frame**. Unlike $P$, $R$ does not accumulate across frames.

In the `TrustModel` discussed for this estimator, the baseline standard deviations are

$$
\begin{aligned}
\sigma_{xy,\mathrm{base}}&=\max\left(0.02,\frac{0.02d^2}{n}\right),\\
\sigma_{\theta,\mathrm{base}}&=\max\left(1.5^\circ\ \text{in radians},\frac{0.06d^2}{n}\right).
\end{aligned}
$$

Here $d$ is average camera-to-tag distance in meters and $n$ is the number of contributing tags. Both baseline coefficients, the floor values, and the exact dependence on $d$ and $n$ are **empirical model choices, not physical laws**. For example, $1/n$ on **standard deviation** is more aggressive than the $1/\sqrt n$ behavior of ideal independent measurements in sample statistics; real multi-tag pose solutions also depend on tag geometry.

The model multiplies the baseline standard deviations by

$$
\boxed{f=f_{\mathrm{camera}}f_{\mathrm{quality}}f_{\mathrm{motion}}.}
$$

The quality factor can combine edge proximity, reprojection error, decision margin, and single-tag pose ambiguity; each input is an observed/reported metric, while its threshold and gain are tunable heuristics. The motion factor is

$$
f_{\mathrm{motion}}=1+0.15\lvert v\rvert+0.4\lvert\omega\rvert.
$$

For the **same two-tag camera observation** as Section 2, assume a camera factor of $1$, a hypothetical reprojection error $e=0.15$ pixels with `reprojScalePx = 1.5`, no other quality penalties, speed $v=3\ \mathrm{m/s}$, and yaw rate $\omega=0$:

$$
\begin{aligned}
f_{\mathrm{quality}}&=1+\frac{0.15}{1.5}=1.10,\\
f_{\mathrm{motion}}&=1+0.15(3)+0.4(0)=1.45,\\
f&=(1)(1.10)(1.45)=\boxed{1.595}.
\end{aligned}
$$

At $d=3\ \mathrm m$ and $n=2$,

$$
\begin{aligned}
\sigma_{xy,\mathrm{base}}&=0.09\ \mathrm m,\\
\sigma_{\theta,\mathrm{base}}&=0.27\ \mathrm{rad},\\
\sigma_{xy}&=0.09(1.595)=\boxed{0.14355\ \mathrm m},\\
\sigma_\theta&=0.27(1.595)=\boxed{0.43065\ \mathrm{rad}\approx24.67^\circ}.
\end{aligned}
$$

The measurement covariance squares those standard deviations:

$$
\boxed{R_1=\begin{bmatrix}
0.0206066025&0&0\\0&0.0206066025&0\\0&0&0.1854594225
\end{bmatrix}.}
$$

A 10% increase in the **standard deviation** from the quality factor increases the corresponding **variance** by $1.1^2=1.21$, or 21%.

For a single-tag observation, this implementation ordinarily ignores camera heading by masking it with $H$.

## 6. Generalizing scalar fusion to the matrix Kalman gain

### 6.1 Innovation and observation model

The camera measures field pose directly. For our full multi-tag observation,

$$
H=I,\qquad
\boxed{\mathbf r=\mathbf z-H\hat{\mathbf x}^-},
$$

where $\mathbf r$ is the **innovation**. The heading component is wrapped to the shortest signed angular difference. When only X and Y are measured, the implementation uses $H=\mathrm{diag}(1,1,0)$ and zeroes the unmeasured innovation component.

The innovation covariance is

$$
\boxed{S=HP^-H^T+R.}
$$

### 6.2 Gain and state correction

The scalar gain $P/(P+R)$ generalizes to matrices as

$$
\boxed{K=P^-H^T(HP^-H^T+R)^{-1}=P^-H^TS^{-1}.}
$$

The corrected estimate is

$$
\boxed{\hat{\mathbf x}^+=\hat{\mathbf x}^-+K\mathbf r.}
$$

For $H=I$, $K=P^-(P^-+R)^{-1}$. Unlike three independent scalar weights, the off-diagonal entries of $K$ allow a measured Y discrepancy to correct heading, or vice versa, whenever the predicted state errors are correlated.

### 6.3 Covariance correction: Joseph form

The corrected estimation error is approximately $(I-KH)\mathbf e^- - K\mathbf v$, where $\mathbf v$ is independent vision error with covariance $R$ (the sign does not affect its covariance). Taking the covariance gives

$$
\boxed{P^+=(I-KH)P^-(I-KH)^T+KRK^T.}
$$

This is the **Joseph form**, used by `EkfPoseEstimator.java`. The first term carries residual prior error; the second carries vision measurement error introduced through the gain. With the optimal Kalman gain, it is algebraically equivalent to

$$
P^+=P^--P^-H^TS^{-1}HP^-=P^--KSK^T.
$$

Since $KSK^T$ is positive semidefinite, $P^+\preceq P^-$: every **diagonal variance** can only decrease or stay unchanged in an ordinary valid measurement correction. Individual off-diagonal entries, however, can increase, decrease, or change sign. Prediction may increase uncertainty again in the next time step.

## 7. Complete EKF correction: the same numerical example

Our odometry prediction and vision measurement are

$$
\hat{\mathbf x}_1^-=\begin{bmatrix}2.06\\3.00\\0\end{bmatrix},\qquad
\mathbf z_1=\begin{bmatrix}2.16\\3.04\\\pi/18\end{bmatrix},
$$

$$
P_1^-=\begin{bmatrix}
0.0037002&0&0\\0&0.00370164&0.000024\\0&0.000024&0.0004002
\end{bmatrix},\qquad
R_1=\mathrm{diag}(0.0206066025,0.0206066025,0.1854594225).
$$

Because this is a multi-tag observation, $H=I$. The innovation and its covariance are

$$
\boxed{\mathbf r=\begin{bmatrix}0.10\\0.04\\0.174533\end{bmatrix}},\qquad
\boxed{S=\begin{bmatrix}
0.0243068025&0&0\\0&0.0243082425&0.000024\\0&0.000024&0.1858596225
\end{bmatrix}.}
$$

Calculate $K=P^-S^{-1}$:

$$
\boxed{K\approx\begin{bmatrix}
0.152229&0&0\\
0&0.152279&0.000109466\\
0&0.000985194&0.00215311
\end{bmatrix}.}
$$

The X gain is $0.152229$, so vision corrects approximately 15.2% of the 10 cm X discrepancy. The relatively small heading gain reflects the much smaller predicted gyro/heading variance compared with the vision heading variance. The nonzero $K_{\theta y}$ is the payoff from retaining Y/heading covariance during prediction.

The full correction is

$$
K\mathbf r\approx\begin{bmatrix}
0.015222899\\0.006110270\\0.0004151965
\end{bmatrix},
$$

so

$$
\boxed{\hat{\mathbf x}_1^+\approx
\begin{bmatrix}2.075223\ \mathrm m\\3.006110\ \mathrm m\\0.00041520\ \mathrm{rad}\end{bmatrix}
= (2.075223\ \mathrm m,\ 3.006110\ \mathrm m,\ 0.023789^\circ).}
$$

Using the Joseph form,

$$
\boxed{P_1^+\approx\begin{bmatrix}
0.0031369223&0&0\\
0&0.0031379550&0.0000203015\\
0&0.0000203015&0.0003993147
\end{bmatrix}.}
$$

The X and Y standard deviations decrease from approximately 6.083 cm and 6.084 cm to **5.601 cm and 5.602 cm**; heading uncertainty barely changes, from approximately $1.146^\circ$ to $1.145^\circ$.

### 7.1 Is the camera observation plausible?

The implementation uses the **normalized innovation squared** (NIS), which measures the surprise of the observation relative to both uncertainty models:

$$
\boxed{\mathrm{NIS}=\mathbf r^TS^{-1}\mathbf r\approx0.641.}
$$

The configured three-axis gate in our reviewed code is 16.27, so $0.641<16.27$ and this example is accepted.

### 7.2 Comparison at the same instant

| Result | Illustrative old fixed-weight model | EKF with dynamic uncertainty and quality multiplier |
| --- | ---: | ---: |
| Predicted pose | $(2.06,3.00,0^\circ)$ | $(2.06,3.00,0^\circ)$ |
| Camera measurement | $(2.16,3.04,10^\circ)$ | $(2.16,3.04,10^\circ)$ |
| Vision position $\sigma$ | $0.0900\ \mathrm m$ | $0.14355\ \mathrm m$ |
| X correction gain | $0.313571$ | $0.152229$ |
| Fused X | $2.091357\ \mathrm m$ | $2.075223\ \mathrm m$ |
| Fused Y | $3.012546\ \mathrm m$ | $3.006110\ \mathrm m$ |
| Fused heading | $0.054597^\circ$ | $0.023789^\circ$ |
| Y–heading error coupling | Ignored in scalar calculation | Retained via $P_{y\theta}$ and $K_{\theta y}$ |

**Do not interpret this single comparison as an accuracy contest.** The true pose is unknown, and the SKF uses a different $R$ by design. If the two models are assigned exactly the same independent diagonal $P$ and $R$, their scalar and matrix fusion results coincide on those axes. The EKF's advantage is the ability to **evolve $P$, change $R$ frame by frame, and retain cross-covariances**.

## 8. Engineering interpretation and tuning

A plausible limiting explanation for why SKF can work well is a repeatable operating regime with frequent high-quality vision. Starting from

$$
P_{k+1}^-=FP_k^+F^T+GQG^T,
$$

if vision drives $P_k^+$ sufficiently low that $FP_k^+F^T$ is negligible, and if $GQG^T$ is relatively consistent across update intervals, then $P_{k+1}^-\approx GQG^T\approx\mathrm{constant}$. This **does not require $F=I$**, but it does require the amplified residual covariance term to stay small. So, SKF can work reasonably well when:

- High-quality vision results regularly arrive at sufficient FPS
- Tuning $R_{SKF}$ & $P_{SKF}$ are performed under reasonably representative driving conditions
- The Jacobian has a relatively small norm

In cases where the above is satisfied, SKF may be sufficient. In cases where it is not, the additional complexity of EKF may be greatly beneficial. 

As mentioned above, $\sigma_{v,SKF}$ is tuned to implicitly represent typical camera, motion, and quality conditions. The new model exposes these influences separately. A disciplined calibration approach is to first validate the baseline across controlled tag distances/counts with $f\approx1$, and then fit the multipliers against observed changes in error. There is **no requirement** that $d^2/n\approx1$ during this process; its value simply follows the test geometry. Heading calibration should use suitable multi-tag observations, since the implementation ordinarily ignores single-tag heading.

**The EKF is a more mathematically-expressive uncertainty model, but that is not an automatic guarantee of better localization.** Poorly calibrated $Q_{EKF}$, $R_{EKF}$, or observation-quality heuristics can perform worse than well-tuned fixed weights in SKF. Improperly quantified EKF tunables can produce an estimator input set that doesn't capture dynamic effects that would otherwise have representation in the generalized tuning utilized in SKF. However, both operating optimially, EKF appears to have a much more robust representation of pose estimation than SKF.

## 9. Scope and future revisions

This document currently omits camera capture latency, timestamped history replay, rejection-recovery logic, and full test/validation results. Those deserve a separate subsequent revision. Other potential refinements include a more complete $G$ Jacobian, anisotropic vision covariance, a measured justification for the $d^2/n$ baseline model, and test-backed calibration recommendations.
