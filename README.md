# CarND-Controls-MPC (Project Write-up)
---

## The Model
*Student describes their model in detail. This includes the state, actuators and update equations.*

I used the Global Kinematic Model presented in the course.

State:

  1. `x`: Position of the car on the horizontal axis of the map, in meters.
  2. `y`: Position of the car on the vertical axis of the map, in meters.
  3. `psi`: Angle between the horizontal axis of the map and the direction of the car, in radians.
  4. `v`: Speed of the car, in meters per second.
  5. `cte`: Cross Track Error, a measure of distance between the reference track and the actual track of the car.
  6. `epsi`: Measure of distance between the desired/reference angle and the actual angle of the car. The reference angle is the tangential angle of the track function `f` evaluated at `x`.

Actuators:

  1. `delta`: Change in angle.
  2. `a`: Acceleartion; change in speed.

Update Equations:
```
x' = x + v * cos(psi) * dt
y' = y + v * sin(psi) * dt
psi' = psi + v / Lf * delta * dt
v' = v + a * dt
cte' = f(x) - y + v * sin(epsi) * dt
epsi' = psi - desired_psi + v / Lf * delta * dt
```
(where `f` is a polynomial function that approximates the correct track, usually of degree 3.)

## Timestep Length and Elapsed Duration (N & dt)
*Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.*

My final values are `N = 10` and `dt = 0.1`. When choosing these values, we are making a trade-off between speed and accuracy; higher values of `N` and lower values of `dt` require more computations. I tried, for instance, `N = 25`, and `dt = 0.05` (as suggested in the *Mind the Line* quiz), but the results are not necessarily better.

## Polynomial Fitting and MPC Preprocessing
*A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.*

First, I converted the waypoints from map coordinates to car coordinates, as advised in the YouTube Q&A. I did so by re-centering the car at position `(x, y, psi) = (0, 0, 0)` and applying the necessary transformations. Then I fed the resulting points to `polyfit` in order to fit a polynomial of degree `3` that approximates the reference track.

## Model Predictive Control with Latency
*The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.*

In order to deal with latency, I ran the MPC simulation starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.