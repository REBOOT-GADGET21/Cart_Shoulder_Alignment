# Snapshot body-axis alignment

`/shoulder_line` supplies four RGB-D points in odom: left shoulder, right
shoulder, **eye midpoint**, pelvis midpoint. Vision deprojects each eye using
its own depth before averaging. The controller checks 3-D shoulder width
(0.17 m), eye–pelvis length (0.56 m), and shoulder–pelvis length (0.31 m).
The initial rejection tolerance is 0.08 m; it is not a positioning tolerance
or a measured sensor accuracy. Inspect rejection logs before tuning it.

During CAPTURE the cart and mannequin must remain still. At least 10 unique
samples over 5 seconds are required by the current shared configuration.
Coordinate medians fix the shoulder centre and floor-projected pelvis-to-eye
axis. The approach side is the side containing the cart at capture time.
The target is shoulder centre + axis * (front clearance + rear-pivot-to-front).
Shoulder slope does not determine heading. All later feedback is odometry;
the snapshot does not track person movement or obstacles.

DRIVE uses polar Lyapunov feedback, with a fixed virtual reverse heading if
the target starts behind the cart. With a=bearing-robot_yaw and
b=bearing-goal_yaw (both virtual headings for reverse),

    v = kr*rho*cos(a)
    w = ka*a + kr*sinc(a)*cos(a)*(a + kb*b)
    V = (rho^2 + a^2 + kb*b^2)/2

In the ideal continuous unicycle model away from angle branch cuts,
Vdot = -kr*rho^2*cos(a)^2 - ka*a^2. Speed limits scale both commands by the
same positive factor to preserve this property. No minimum-speed floor is
applied. Finite sampling, actuator dead zones, odometry error, switching and
angle wrapping still require physical validation; this is not a shortest-path
guarantee. FINAL_HEADING aligns the final yaw and HOLD stops the cart.

`camera_elevation_from_down_rad=1.0472` means 60 degrees above vertically down,
equivalently 30 degrees below horizontal. It replaces the ambiguous shared
`camera_pitch_down_rad` key. ROS positive Ry points a forward ray downward;
both hardware TF and Python geometry use that convention.

Build: `colcon build --packages-select shoulder_align_controller vision geometry alignment zlac8015d_driver simulation --symlink-install`.
Tests: `ctest --test-dir build/shoulder_align_controller --output-on-failure`.
Launch remains `ros2 launch zlac8015d_driver real_alignment.launch.py`.
