# Kinematics

The forward and inverse kinematics GOIK uses for a three joint leg (coxa, femur, tibia). They are not tied to a
particular servo type, but assume that the servo midpoint is at 0 degrees. The inverse kinematics equations are in
[solver.go](../robot/solver.go) and should be easy to port to other languages and platforms.

## Typical kinematic diagram for a pod leg

This kinematic diagram describes a typical pod leg (Servo horns should face in the positive z- direction of their respective reference frame
).

![kinematic diagram](../pictures/kinematic_diagram.png)

The first leg segment is the _coxa_. This has a rotation angle $\theta_1$ around $z_0$. The length of the coxa is $r_2$.

The second leg segment is the _femur_. This has a rotation angle $\theta_2$ around $z_1$. The length of the femur is $r_3$.

The third leg segment is the _tibia_. This has a rotation angle $\theta_3$ around $z_2$. The length of the tibia is $r_4$.

The end effector is located at the origin of the $[x_3,y_3,z_3]$ reference frame.

$r_1$ is assumed to be of zero length in this pod example. (If $\theta1, \theta2$ and $\theta3$ are all zero, $x_0, x_1$ and $x_3$ all align in the same direction at $z_0=0$ and $y_0=0$).

## Forward kinematics

All leg segments rotate around z in their corresponding reference frame. We will therefore rely on the z rotation matrix for all joints:

$`R_{z}(\theta) = \begin{bmatrix}
Cos(\theta) & -Sin(\theta) & 0\\
Sin(\theta) & Cos(\theta) & 0\\
0 & 0 & 1
\end{bmatrix}
`$

By multiplying the rotation matrix with a projection matrix, we can get a new rotation matrix that also reflect the rotation of the reference frame compared to the previous reference frame.

If the reference frames are perfectly aligned, the projection matrix is the identity matrix (which will not change the rotation matrix). The exception in this case is the projection of the femur reference frame to the coxa reference frame. This gives us the following projection matrices:

$`P_{coxa}^{femur} = \begin{bmatrix}
1 & 0 & 0\\
0 & 0 &-1\\
0 & 1 & 0\\
\end{bmatrix}
`$

$`P_{femur}^{tibia} = P_{tibia}^{end effector} \begin{bmatrix}
1 & 0 & 0\\
0 & 1 &0\\
0 & 0 & 1\\
\end{bmatrix}
`$

The final rotation matrix for each frame is:

$`R(\theta) = R_{z}(\theta) \times P`$

A displacement vector describes the offset between reference frames. The displacement vector has to be valid for all values of $\theta$.

For this kinematic diagram, the displacements vectors are :

$`D_{coxa}= \begin{bmatrix}
r_2 \times cos(\theta_1)\\
r_2 \times sin(\theta_1)\\
0
\end{bmatrix}
`$

$`D_{femur}= \begin{bmatrix}
r_3 \times cos(\theta_2)\\
r_3 \times sin(\theta_2)\\
0
\end{bmatrix}
`$

$`D_{tibia}= \begin{bmatrix}
r_4 \times cos(\theta_3)\\
r_4 \times sin(\theta_3)\\
0
\end{bmatrix}
`$

We can then compose homgeneous transformation matrices for transforming from one frame to the next.

From frame zero to frame 1 :

$`H^0_1= \begin{bmatrix}
R_{coxa}[0,0] & R_{coxa}[0,1] & R_{coxa}[0,2] & D_{coxa}[0]\\
R_{coxa}[1,0] & R_{coxa}[1,1] & R_{coxa}[1,2] & D_{coxa}[1]\\
R_{coxa}[2,0] & R_{coxa}[2,1] & R_{coxa}[2,2] & D_{coxa}[2]\\
0 & 0 & 0 & 1
\end{bmatrix}
`$

From frame 1 to frame 2 :

$`H^1_2= \begin{bmatrix}
R_{femur}[0,0] & R_{femur}[0,1] & R_{femur}[0,2] & D_{femur}[0]\\
R_{femur}[1,0] & R_{femur}[1,1] & R_{femur}[1,2] & D_{femur}[1]\\
R_{femur}[2,0] & R_{femur}[2,1] & R_{femur}[2,2] & D_{femur}[2]\\
0 & 0 & 0 & 1
\end{bmatrix}
`$

From frame 2 to frame 3 :

$`H^2_3= \begin{bmatrix}
R_{tibia}[0,0] & R_{tibia}[0,1] & R_{tibia}[0,2] & D_{tibia}[0]\\
R_{tibia}[1,0] & R_{tibia}[1,1] & R_{tibia}[1,2] & D_{tibia}[1]\\
R_{tibia}[2,0] & R_{tibia}[2,1] & R_{tibia}[2,2] & D_{tibia}[2]\\
0 & 0 & 0 & 1
\end{bmatrix}
`$

Since each leg originates at a different [X,Y,Z] coordinate in the base reference frame, we also introduce a transformation matrix for the base reference frame. The origin of this frame is the center of the robot body.

Example: Here each leg origin is arranged at a distance _d_ separated by _a_ degrees in the X/Y plane at Z=0 in the base reference frame.

$`H^{base}_0= \begin{bmatrix}
cos(a) & -sin(a) & 0 & d \times cos(a)\\
sin(a) & cos(a) & 0 & d \times  sin(a)\\
0 & 0 & 1 & 0\\
0 &  0 &  0 &  1
\end{bmatrix}
`$

If we want to know the end effector's position in the $[x_{base},y_{base},z_{base}]$ reference frame, we simply take the matrix product of all the homogeneous transformation matrices and extract the X,Y,Z values from the resulting matrix from row 0-2 in the rightmost column.

$`H^{base}_3 = H^{base}_0 \times H^0_1 \times H^1_2 \times H^2_3`$

End effector coordinates (after matrix multiplication) in the base reference frame:

$`H^{base}_3: \begin{bmatrix}
. & . & . & x\\
. & . & . & y\\
. & . & . & z\\
. &  . &  . &  .
\end{bmatrix}
`$

## Inverse kinematics

### Pod leg top view

![leg top view](../pictures/top_view.png)

The angle $\theta_1$ is given by the formula

(1) $`\theta_1 = arctan(\frac{y} {x})`$

### Pod leg side view

![leg side view](../pictures/side_view.png)

(2) $`L=\sqrt(L_2^2+(L_1-coxa)^2)`$

(3) $`\alpha_1=arccos(\frac{L_2}{L})`$

(4) $`\alpha_2=arccos(\frac{tibia^2-femur^2-L^2}{-2 \times femur \times L})`$

(5) $`\alpha = \alpha_1 + \alpha_2`$

(6) $`\beta=arccos(\frac{L^2 - tibia^2 - femur^2}{-2 \times tibia \times femur})`$


$\theta_2 = 90 - \alpha`$

$\theta_3 = 180 - \beta`$

>Note: It is assumed that the neutral position (centered servos) are at 0 degrees. If this is not the case, a zero offset has to be introduced.

## Twisted joints

The leg above is planar: the coxa axis is vertical, and the femur and tibia axes are horizontal and parallel. A
[twisted joint](designing-a-pod.md#twisted-joints) turns a joint's axis about the link leading into it. In the
kinematic chain ([robot/twist.go](../robot/twist.go)) each twist is a rotation about the link's X axis:

| Frame | Transformation |
|---|---|
| Mount | T(coxa joint position) Rz(mount angle) Rx(coxa twist) |
| Coxa joint | Rz(coxa angle) Tx(coxa length) Rx(90 + femur twist) |
| Femur joint | Rz(femur angle) Tx(femur length) Rx(tibia twist) |
| Tibia joint | Rz(tibia angle) Tx(tibia length) |

With all twists 0 this is the chain above: the Rx(90) turns the femur axis horizontal.

A twisted leg is no longer planar, so the inverse kinematics has no simple closed form. It is solved numerically with
damped least squares (Levenberg-Marquardt): the Jacobian is exact (column j is joint j's axis crossed with the lever
arm from the joint to the foot), and each step solves a 3 x 3 system. Starting from the leg's current angles keeps the
solution on the same branch (the knee stays on the same side) from one tick to the next, and it converges in a few
iterations: about 0.8 µs per solve on a desktop PC, against 0.03 µs for the closed form of an untwisted leg. Untwisted
legs still use the closed form.

The mirror image of a leg (in a design) has the opposite twists, the opposite coxa angle and the same femur and tibia
angles.

## Gaits

Moving the pod involves one or more of the legs of the robot being lifted off the ground and being moved in a desired direction. These legs are in the "swing phase". The legs that provide stabilization and are still on the ground are said to be in the "stance phase".

Given the position of a leg on the robot, the length of the leg segments and the angles of coxa, femur and tibia joints, we can know the exact location of the end effector (the tip of the robot leg) in XYZ space in the base reference frame. This is called forward kinematics.

Assuming that the positive y direction is the robot forward direction, a foot that should move to a new position gives a new coordinate in XYZ space in the base reference frame. From this (and the lengths of the leg segments) we can then use inverse kinematics to find the servo angles neccesary for moving the end effector to the new location.

The legs in the swing phase can not move directly to the new position, but have to be lifted from the ground before they move. As the legs in the swing phase move forward, the legs in the stance phase move back, carrying the body forward.

> The gait engine ([gait-engine.md](gait-engine.md)) does this continuously: every leg has a phase in the gait cycle, the
> feet in stance move opposite to the body's velocity, and swinging feet follow an arc to where they should land. The gait
> patterns below define when each leg swings.

Example layout of a hexapod

![xyview](../pictures/xyview.png)

The metachronal gait (`gait metachronal`) is generated from where the legs are: a wave of steps runs along each side
from the front to the back, and the two sides are half a cycle apart. For segmented bodies a leg's place in the wave
is its segment ([centipede.md](centipede.md)).

The patterns below are for a hexapod. Pentapods and heptapods have hand written wave gait patterns, and other numbers
of legs get generated ones (wave gait, tripod gait for an even number of legs, ripple gait for a multiple of 3, see
[robot/gaits.go](../robot/gaits.go)). All of them assume that the legs are numbered in order around the body.

We can define the folling gait patterns
### Tripod gait

![Tripod gait](../pictures/tripod_gait.png)


### Wave gait

![Wave gait](../pictures/wave_gait.png)

### Ripple gait

![Ripple gait](../pictures/ripple_gait.png)

