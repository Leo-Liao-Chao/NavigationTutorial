# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog],
and this project adheres to [Semantic Versioning].

## [Unreleased]

## [0.0.5] - 2024-09-25

### Added

- fix the chance constraint
  - fix the theta to (theta)
  - fix the chance constraint the covirance matrix .

## [0.0.4] - 2024-09-03

### Added

- add the chance constraint.
- add parameter.py to contain all of the parameters.
- comple the full function.
  - path follow
    - there is no obstacles in the area of 30 m.
    - the average compute time is 0.03s.
  - avoid obstacle
    - there is obstacles in the area of 30 m.
    - the average compute time :
      - one obtsacle is 0.07s
      - two obstacle is 0.10-0.20s
  - compare to the CILQR
    - path follow
      - the average compute time is 0.01-0.03s.
    - avoid obstacle
      - the average compute time is 0.01-0.03s.

### Bug

- Meet the 4th obstacle , it will roll away to avoid obstacle.
- Meet the 7th obstacle , it will crash to solve problem.

## [0.0.3] - 2024-09-03

### Added

- complete the avoid obstacle function.
- comple the full function.
  - path follow
    - there is no obstacles in the area of 30 m.
    - the average compute time is 0.03s.
  - avoid obstacle
    - there is obstacles in the area of 30 m.
    - the average compute time :
      - one obtsacle is 0.15-0.30s
      - two obstacle is 0.25-0.40s
  - compare to the CILQR
    - path follow
      - the average compute time is 0.01-0.03s.
    - avoid obstacle
      - the average compute time is 0.01-0.03s.

### Fixed

- fix the bug of the avoid obstacle function.

### Todo

- add the chance constraint.

## [0.0.2] - 2024-09-01

### Added

- adapt the function into ros, to path tracking.
- adjust the spline to poly.

### Fixed

- Fix reference trajectory bug.
  - Find the closest part of global trajectory, and use add the current position to the the start of the local trajectory, and use spline or function to fit it.
- adjust the spline to poly.

## [0.0.1] - 2024-08-30

### Added

- Use Casadi to employ NMPC ,which is used to PathTracking.
  - The model is state :[x,y,v,θ] control : [a,ω]
    x(next) = x(now) + v * dt * cos(θ) * dt + 0.5 * a * dt * dt * cos(θ)
    y(next) = y(now) + v * dt * sin(θ) * dt + 0.5 * a * dt * dt * sin(θ)
    v(next) = v(now) + a * dt
    θ(next) = θ(now) + ω * dt
  - The object function is
    J = C_state + C_control
    C_state:
    state_error = state_prediction - state_reference
    Q = [2.0,2.0,0.5,0.0]
    C_state = stat_error * Q * stat_error
    C_control:
    control_errot = control_prediction
    R = [1.0,3.0]
    C_control = control_error * R * control_error
  - Constraint
    - Varible constraint:
      - control constraint:
        acc:[-acc_max,acc_max]
        ω:[-ω_max,ω_max]
      - state constraint:
        x:[-inf,inf]
        y:[-inf,inf]
        v:[0,v_max]
        θ:[-inf,inf]
    - Other constraint:
      - Equation constraint:
        - start point constraint:
          X(statr) = Reference(start)
        - position continious:
          f(next) = delata + f(now)

### Todo

- Fix reference trajectory bug.
  - Find the closest part of global trajectory, and use add the current position to the the start of the local trajectory, and use spline or function to fit it.
  - The bug if the local reference trajectory size is small than N will crash, because the dimension is not match.
- Add avoid obstacle function.
- Add chance constraint.
