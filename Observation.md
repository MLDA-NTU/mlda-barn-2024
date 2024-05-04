- Position cost
  - Allow backtracking because it only cares about position
  - It forces the calculated velocity to match the spacing of the global plan (fixed delta t)
- Velocity ref
  - Tell the robot to follow the global plan at a certain speed
  - Might not allow backtracking because the ref velocity is positve
- Acceleration
  - Smooth velocity change
  - Might not stop fast enough to avoid collision
- Theta error

- omega W

  - Make the lidar and the map misalign
  - But limit the curvature of the turn
    - It makes the robot act like an ackermann
    - Trade-off

- It is better to make the global planner slow

- Decrease acceleration is at odd with following the position of the trajectory

- Sometimes, putting the maximum hinder the solver from finding a better solution

  - Trade off between the "obvious" fast solution and safety

- Constraining the last horizon step

  - Might result in the bending of the mpc plan if it wants to match a certain velocity reference, because going off course will give higher velocity

- Add time as an optimization variable

- N = 10 calculate faster, the local controller is faster
  - Slow global planner with local planner sticking to the global is bad
  - Hence global planner should update fast as well
- Constraint at the end of the horizon is still good
- How to solve the problem of the velocity being bounded by the spacing of the trajectory? Change the delta t?

- Add the map cloud to +90 -90 so that it is added in the contraints as well
- Remove smooth acceleration
- Change max acceleration prevent the robot from planning unrealistic acceleration
