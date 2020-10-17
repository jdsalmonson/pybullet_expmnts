========================
Pybullet Experiments Log
========================

8/8/2020

Trying to understand robot dynamics.  Ran ``python kuka_apply_torque.py``
without touching any keys to apply torques.  Joints 1 & 3 move as the arm
relaxes under gravity.  Run ``python kuka_log_replay.py`` to watch a replay,
and ``python plot_log.py`` to plot the position, q, velocity, u, and torque, t,
of each joint.

It is interesting that joints 4,5,6 will have a large (sometimes > 100 N-m) torque
spike when one of the joints hits its maximum value or the arm hits an external
object like a table, but joints 0,1,2,3 don't; their torques simply jump to a
new value, even though there may also be a velocity jump (acceleration spike)
because of the obstacle.  The one counter example: letting the arm lean over
onto the table with joint 3 swung away such that the "upper" arm, before
joint 3, contacted the table made joint 3 exhibit a small torque spike.
Typically, one of these obstructions causes a torque and velocity discontinuity
(thus a large acceleration spike) on the inner joints, but the outer joints
have a torque spike but very little (and only transient) change to the velocity
(a small pos/neg S-curve oscillations in acceleration).

The torque on a joint is the sum of unbalanced torques, for e.g. gravity + motor
+ obstructions.  So when the robot leaned onto the table by rotating joint 1,
the torque gradually increased as the sin(angle) with gravity increased.
When the arm made contact with the table, the torque immediately dropped to
nearly zero (about 0.01).  So the balanced torque of gravity and the table sum
to (about) zero.  It is more tricky because joints 0, 2, 4 can swivel to relax
a bit further, and given enough time the bot swivels and falls off the table.
