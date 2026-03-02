# 🌀 EKF Oscillator Tracking (EKFOT)

MATLAB implementation of an **Extended Kalman Filter (EKF)** for tracking  
time-varying **frequency**, **damping**, and **amplitude** of narrowband oscillations  
using a reparametrized AR(2) resonator model.

The repository includes:

- ✅ EKF-based oscillator tracking  
- ✅ Genetic Algorithm (GA) hyperparameter optimization  
- ✅ Multi-channel / multi-trial support  
- ✅ Innovation-based cost function  

---

## 📁 Repository Structure

```
/ [root]
├── code
│   ├── EKFOT.m          % EKF oscillator tracking cost (used by GA)
│   ├── GA_EKFOT.m       % GA objective across channels/trials
│   ├── SIM_EKFOT.m      % EKF tracking returning full state trajectories
│   ├── run_EKFOT.m      % Wrapper to run tracking on multichannel data
│   └── wrapToPiLocal.m  % Helper for frequency wrapping
├── README.md
└── LICENSE
```

---

## 🧠 Model Description

The observed signal is modeled as a noisy AR(2) resonator:

State vector:

```
z(k) = [ x(k), x(k-1), omega(k), alpha(k) ]
```

Measurement equation:

```
y(k) = x(k) + v(k)
```

AR(2) coefficients are parameterized as:

```
a1 =  2*exp(-alpha*Ts)*cos(omega*Ts)
a2 = -exp(-2*alpha*Ts)
```

where:

- `omega` = instantaneous angular frequency (rad/s)  
- `alpha` = damping coefficient (1/s)  
- `Ts` = sampling period  

An **Extended Kalman Filter** estimates:

- Oscillator states `x(k), x(k-1)`
- Instantaneous frequency `omega(k)`
- Damping `alpha(k)`
- State covariance evolution

---

## ⚙️ Hyperparameter Optimization (GA)

The EKF parameters are optimized using a **Genetic Algorithm**.

Optimized parameter vector:

```
X = [R2_ct, Q_ct, omega0, alpha0, P0scale]
```

Where:

- `R2_ct`  → continuous-time measurement noise intensity  
- `Q_ct`   → continuous-time process noise intensity  
- `omega0` → initial angular frequency (rad/s)  
- `alpha0` → initial damping  
- `P0scale` → initial covariance scaling  

Continuous-to-discrete scaling:

```
R2 = R2_ct * Ts
R1 = Q_ct  * Ts * eye(4)
```

---


# 🔁 Optimization Strategy and Data Usage

## 🔹 Optimization on Short Calibration Segments

Hyperparameter optimization does **not** require the full dataset.

The Genetic Algorithm (GA) can be run on:

- A small subset of trials  
- A short calibration segment (e.g., first 30–60 seconds)  
- A representative continuous data block  

This significantly reduces computational load while still providing stable parameter estimates.

After optimal parameters are found, they can be applied to the **full dataset** for tracking.

---

## 🔹 Trial-Based vs Continuous Data

The optimization input `ytr` is expected to have shape:

```
[M x T x nTrials]
```

where:
- `M` = channels
- `T` = time samples
- `nTrials` = number of trials

However, the tracking input `sig_in` (used in `run_EKFOT`) can be provided as:

```
[M x T]
```

and may represent either:

- Epoched trial data (concatenated or individual trials)
- Fully continuous recordings

---

## ⭐ Recommended Practice

While both formats are supported, it is **preferable that `sig_in` is provided as continuous data** when possible.

Continuous data:

- Avoids repeated EKF re-initialization across trials  
- Produces smoother frequency trajectories  
- Improves stability of damping estimates  
- Better captures slow adaptation or learning effects  
- Is more suitable for online and real-time BCI applications  

The training data `ytr` used during hyperparameter optimization can, however, be structured as trials.  
Using epoched trial data for optimization can be beneficial when the goal is to make the algorithm focus specifically on task-related dynamics (e.g., motor imagery periods, cue-locked activity, or event-related oscillatory changes).

A practical strategy is therefore:

1. Use task-relevant trials (`ytr`) to optimize EKF hyperparameters.
2. Apply the optimized parameters to continuous data (`sig_in`) for stable and physiologically meaningful tracking.

This separation between optimization and full-data tracking allows both computational efficiency and improved sensitivity to relevant neural dynamics.

---

## 🔬 Practical Workflow Example

1. Select a short continuous segment (e.g., first 60 seconds)
2. Run GA optimization on that segment
3. Apply the optimized parameters to the full continuous recording
4. Analyze frequency drift, amplitude changes, and uncertainty

This two-step approach balances computational efficiency and estimation accuracy.

---


## 🎯 Cost Function

The optimization minimizes normalized innovation energy:

```
J = norm(innovation(ignore:end)) / norm(signal(ignore:end))
```

Where:
- `innovation` = EKF prediction error
- `ignore` = number of initial samples discarded (transient removal)

---

## 🚀 Example Usage

### 1️⃣ Optimize EKF parameters

```matlab
Fs     = 100;      % sampling rate
ignore = 200;      % discard transient samples

[lamm, err] = optimize_EKFOT(ytr, ignore, Fs);
```

Expected shape:

```
ytr : [M x T x nTrials]    or    [M x T] 
  M:       channels
  T:       time points
  nTrias:  number of trials
```

---

### 2️⃣ Run EKF oscillator tracking

```matlab
polf = run_EKFOT(sig_in, lamm, ignore, Fs);

M = size(sig_in,1);

A   = polf(1:M, :);        % Amplitude proxy
f   = polf(M+1:2*M, :);    % Frequency (Hz)
ptr = polf(2*M+1:3*M, :);  % Uncertainty proxy
```

---

## 📊 Outputs

For each channel:

### 🔹 Amplitude estimate

```
A(k) = sqrt(x1(k)^2 + x2(k)^2)
```

### 🔹 Frequency estimate

```
f(k) = abs(omega(k)) / (2*pi)
```

### 🔹 Uncertainty proxy

```
ptr(k) = norm(P(k),'fro')
```

---

## 🧪 Recommended Workflow

1. Bandpass filter signal around rhythm of interest  
2. Optimize EKF hyperparameters via GA  
3. Run EKF tracking  
4. Analyze frequency drift, amplitude modulation, and uncertainty  

---

## ⚠️ Notes

- Designed for **narrowband oscillatory signals**
- Works best after preprocessing (bandpass filtering)
- `ignore` should remove filter + EKF initialization transients
- Requires `wrapToPiLocal.m` (or replace with MATLAB `wrapToPi`)

---

## 📚 Citation

To be announced

---

