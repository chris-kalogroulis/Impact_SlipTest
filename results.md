# Optimisation Results

## Nealder-Mead | Terrain 40 | Velocity 2

### Optimisation Settings
```python
x0 = np.array([100000.0, 50.0, 3000.0, 3.0])  # initial guess
bounds = [(1, 150000), (1, 10000), (0.1, 5000), (0.1, 1000)]
```
### Simulation Settings
```python
Rail Damping = 30.50

mass = 5 # kg

test_params = {
    "duration": 1.5,
    "y_pos0": 0.645,
    "y_vel0": 0.0,
    "z_pos0": -0.65,
    "z_vel0": 2.0,
    "slope": 40,
}
```
### Terrain Settings
```python
base_size=(1.5, 0.5, 0.05),
base_color=(0.5, 0.5, 0.5, 1.0),
top_color=(0.4, 0.4, 0.7, 1.0),
n=40,
top_box_x=0.01,
top_box_z=0.005,
```

### Simulation Results

| Component | Parameter | Value | Units |
| :--- | :--- | :--- | :--- |
| **Knee** | Stiffness ($k$) | 85460 | N/m |
| **Knee** | Damping ($d$) | 3630 | Ns/m |
| **Ankle** | Stiffness ($k$) | 69.4 | Nm/rad |
| **Ankle** | Damping ($d$) | 2.03 | Nms/rad |

```python
x_optimal = [0.855e5, 3.63e3, 0.694e2, 2.03e0]
```

**Final Position:**
> **Y Pos:** 0.04695 m

<!-- ---

## Powell | Terrain 40 | Velocity 2

### Optimisation Settings
```python
x0 = np.array([100000.0, 50.0, 3000.0, 3.0])  # initial guess
bounds = [(1, 150000), (1, 10000), (0.1, 5000), (0.1, 1000)]
```
### Simulation Settings
```python
Rail Damping = 30.50

mass = 5 # kg

test_params = {
    "duration": 1.5,
    "y_pos0": 0.645,
    "y_vel0": 0.0,
    "z_pos0": -0.65,
    "z_vel0": 2.0,
    "slope": 40,
}
```
### Terrain Settings
```python
base_size=(1.5, 0.5, 0.05),
base_color=(0.5, 0.5, 0.5, 1.0),
top_color=(0.4, 0.4, 0.7, 1.0),
n=40,
top_box_x=0.01,
top_box_z=0.005,
```

### Simulation Results

| Component | Parameter | Value | Units |
| :--- | :--- | :--- | :--- |
| **Knee** | Stiffness ($k$) | 235.9 | N/m |
| **Knee** | Damping ($d$) | 138.1 | Ns/m |
| **Ankle** | Stiffness ($k$) | 6.133 | Nm/rad |
| **Ankle** | Damping ($d$) | 38.22 | Nms/rad |

**Final Position:**
> **Y Pos:** 0.3284 m -->

---

## Nealder-Mead | Terrain 60 | Velocity 2

### Optimisation Settings
x0 was taken from the optimal result from result of **Terrain 40 | Velocity 2**
```python
x0 = np.array([8.54580366e+04, 6.93604482e+01, 3.62780675e+03, 2.02675029e+00])  # initial guess
bounds = [(1, 150000), (1, 10000), (0.1, 5000), (0.1, 1000)]
```
### Simulation Settings
```python
Rail Damping = 30.50

mass = 5 # kg

test_params = {
    "duration": 1.0,
    "y_pos0": 0.645,
    "y_vel0": 0.0,
    "z_pos0": -0.65,
    "z_vel0": 2.5,
    "slope": 40,
}
```
### Terrain Settings
```python
base_size=(1.5, 0.5, 0.05),
base_color=(0.5, 0.5, 0.5, 1.0),
top_color=(0.4, 0.4, 0.7, 1.0),
n=60,
top_box_x=0.01,
top_box_z=0.005,
```

### Simulation Results

| Component | Parameter | Value | Units |
| :--- | :--- | :--- | :--- |
| **Knee** | Stiffness ($k$) | 110000 | N/m |
| **Knee** | Damping ($d$) | 1770 | Ns/m |
| **Ankle** | Stiffness ($k$) | 136.2 | Nm/rad |
| **Ankle** | Damping ($d$) | 1.787 | Nms/rad |

```python
x_optimal = [1.100e5, 1.770e3, 1.362e2, 1.787e0]
```

**Final Position:**
> **Y Pos:** 0.03373 m

---

## Nealder-Mead | Terrain 60 | Velocity 2.5

### Optimisation Settings
x0 was taken from the optimal result from result of **Terrain 60 | Velocity 2**
```python
x0 = np.array([8.54580366e+04, 6.93604482e+01, 3.62780675e+03, 2.02675029e+00])  # initial guess
bounds = [(1, 150000), (1, 10000), (0.1, 5000), (0.1, 1000)]
```
### Simulation Settings
```python
Rail Damping = 30.50

mass = 5 # kg

test_params = {
    "duration": 1.5,
    "y_pos0": 0.645,
    "y_vel0": 0.0,
    "z_pos0": -0.65,
    "z_vel0": 2.0,
    "slope": 40,
}
```
### Terrain Settings
```python
base_size=(1.5, 0.5, 0.05),
base_color=(0.5, 0.5, 0.5, 1.0),
top_color=(0.4, 0.4, 0.7, 1.0),
n=60,
top_box_x=0.01,
top_box_z=0.005,
```

### Simulation Results

| Component | Parameter | Value | Units |
| :--- | :--- | :--- | :--- |
| **Knee** | Stiffness ($k$) | 107500 | N/m |
| **Knee** | Damping ($d$) | 1720 | Ns/m |
| **Ankle** | Stiffness ($k$) | 153.2 | Nm/rad |
| **Ankle** | Damping ($d$) | 1.598 | Nms/rad |

```python
x_optimal = [1.075e5, 1.720e3, 1.532e2, 1.598e0]
```

**Final Position:**
> **Y Pos:** 0.1494 m

---

## Nealder-Mead | Terrain 60 | Velocity 2.5

### Optimisation Settings
x0 was taken from the optimal result from result of **Terrain 60 | Velocity 2**
```python
x0 = np.array([8.54580366e+04, 6.93604482e+01, 3.62780675e+03, 2.02675029e+00])  # initial guess
bounds = [(1, 150000), (1, 10000), (0.1, 5000), (0.1, 1000)]
```
### Simulation Settings
```python
Rail Damping = 30.50

mass = 5 # kg

test_params = {
    "duration": 1.5,
    "y_pos0": 0.645,
    "y_vel0": 0.0,
    "z_pos0": -0.65,
    "z_vel0": 2.0,
    "slope": 40,
}
```
### Terrain Settings
```python
base_size=(1.5, 0.5, 0.05),
base_color=(0.5, 0.5, 0.5, 1.0),
top_color=(0.4, 0.4, 0.7, 1.0),
n=60,
top_box_x=0.01,
top_box_z=0.005,
```

### Simulation Results

| Component | Parameter | Value | Units |
| :--- | :--- | :--- | :--- |
| **Knee** | Stiffness ($k$) | 129600 | N/m |
| **Knee** | Damping ($d$) | 2677 | Ns/m |
| **Ankle** | Stiffness ($k$) | 132.0 | Nm/rad |
| **Ankle** | Damping ($d$) | 2.344 | Nms/rad |

```python
x_optimal = [1.296e5, 2.677e3, 1.320e2, 2.344e0]
```

**Final Position:**
> **Y Pos:** 0.3911 m

---

## Nealder-Mead | Terrain 30 Random | Velocity 2.5

### Optimisation Settings
x0 was taken from the optimal result from result of **Terrain 60 | Velocity 2**
```python
x0 = np.array([8.54580366e+04, 6.93604482e+01, 3.62780675e+03, 2.02675029e+00])  # initial guess
bounds = [(1, 150000), (1, 10000), (0.1, 5000), (0.1, 1000)]
```
### Simulation Settings
```python
Rail Damping = 30.50

mass = 5 # kg

test_params = {
    "duration": 1.5,
    "y_pos0": 0.645,
    "y_vel0": 0.0,
    "z_pos0": -0.65,
    "z_vel0": 2.0,
    "slope": 40,
}
```
### Terrain Settings
```python
base_size=(1.5, 0.5, 0.05),
base_color=(0.5, 0.5, 0.5, 1.0),
top_color=(0.4, 0.4, 0.7, 1.0),
n=60,
top_box_x=0.01,
top_box_z=0.005,
```

### Simulation Results

| Component | Parameter | Value | Units |
| :--- | :--- | :--- | :--- |
| **Knee** | Stiffness ($k$) | 127000 | N/m |
| **Knee** | Damping ($d$) | 1860 | Ns/m |
| **Ankle** | Stiffness ($k$) | 119.0 | Nm/rad |
| **Ankle** | Damping ($d$) | 1.57 | Nms/rad |

```python
x_optimal = [1.27e5, 1.86e3, 1.19e2, 1.57e0]
```

**Final Position:**
> **Y Pos:** 0.8021 m

---

# Discussion

The trend is that all optimal impedance values were of consistent magnitudes, as shown below:

```python
x_optimal_1 = [0.855e5, 3.630e3, 0.694e2, 2.030e0]
x_optimal_2 = [1.100e5, 1.770e3, 1.362e2, 1.787e0]
x_optimal_3 = [1.075e5, 1.720e3, 1.532e2, 1.598e0]
x_optimal_4 = [1.296e5, 2.677e3, 1.320e2, 2.344e0]
x_optimal_5 = [1.270e5, 1.860e3, 1.190e2, 1.570e0]

x_opt_avg   = [1.082e5, 2.449e3, 1.227e2, 1.940e0]
```