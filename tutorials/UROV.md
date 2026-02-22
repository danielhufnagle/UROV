# Introduction to robotic manipulation
For a more in-depth exploration into these topics, take ME 449 - robotic manipulation
## Key concepts for today
- Frames
- Rotation and transformation matrices
- Screws and twists
- Exponential coordinates of rigid body motion
- Forward kinematics
## Frames
These are just your basic $x$, $y$, and $z$ axes and where you place them and how you orient them. On a robot, you'll typically see a *space frame* $\{s\}$ - a fixed global frame from which we define the robot's configuration, and a *body frame* $\{b\}$ - a frame attached to the robot itself. Commonly, you'll also end up seeing frames attached to the robot's joints and end effector as well.

## Rotation and transformation matrices
We've already defined the space and body frame. So now the question is how we describe these frames relative to each other and move a frame from one place to another. In robotics, we do this with matrices. The rotation matrix describes the orientation of one frame relative to another.
$$R_{sb} = \begin{bmatrix}\hat{x}_b & \hat{y}_b & \hat{z}_b\end{bmatrix}=\begin{bmatrix}x_{bx} & y_{bx} & z_{bx} \\ x_{by} & y_{by} & z_{by} \\ x_{bz} & y_{bz} & z_{bz}\end{bmatrix}\in SO(3)$$
Note that the first subscript is the frame of reference and the second is the frame whose orientation is being represented. $\hat{x}_b$, $\hat{y}_b$, and $\hat{z}_b$ are the unit coordinate axes of $\{b\}$ expressed in $\{s\}$ written as column vectors.

Some basic operations and properties of rotation matrices are shown here:
*Inverse and transpose*
$$R^{-1}=R^T$$
*Composition and subscript cancellation*
$$R_{ab}=R_{as}R_{ab}$$
*Representing a point in a reference frame*
$$p_s = R_{sb}p_b$$
*Rotation*
Let $R_{sb}=R$ be a rotation matrix
$$p'_s=Rp_s=R_{sb}p_s$$
is the rotation of point $p_s$
$$RR_{sc}=R_{sb}R_{sc}$$
rotates frame $\{c\}$ about the axes of $\{s\}$
$$R_{sc}R=R_{sc}R_{sb}$$
rotates frame $\{c\}$ about the axes of $\{c\}$

Now, it's one thing to express and modify orientations, but we need to also be able to specify positions too. This is where the transformation matrix comes in.
$$T_{sb}=\begin{bmatrix}R_{sb} & p_{sb} \\ 0 & 1\end{bmatrix}=\begin{bmatrix}r_{11} & r_{12} & r_{13} & p_1 \\ r_{21} & r_{22} & r_{23} & p_2 \\ r_{31} & r_{32} & r_{33} & p_3 \\ 0 & 0 & 0 & 1\end{bmatrix}\in SE(3)$$
$R_{sb}$ is the rotation matrix specifying the orientation of $\{b\}$ in $\{s\}$ and $p_{sb}$ is the position of the origin of $\{b\}$ in $\{s\}$. Note the bottom row is the way it is to simplify matrix operations.

Transformation matrices follow much of the same properties and operations of a rotation matrix
*Inverse*
$$T^{-1} = \begin{bmatrix}R^T & -R^Tp \\ 0 & 1\end{bmatrix}$$
*Composition and subscript cancellation*
$$T_{ab} = T_{as}T_{sb}$$
*Representing a point in a frame*
$$p_{s}=T_{sb}\begin{bmatrix}p_b \\ 1\end{bmatrix}$$
Because the point is a 3-vector, we need to append a 1 so that we can perform the multiplication.
*Transformation*

$$TT_{sc}=T_{sb}T_{sc}$$
transforms frame $\{c\}$ by first rotating about the axes of $\{s\}$ and then translating along the axes of $\{s\}$
$$T_{sc}T=T_{sc}T_{sb}$$
transforms frame $\{c\}$ by first translating along the axes of $\{c\}$ and then rotating about the axes of $\{c\}$
## Screws and twists
In robotics, we define motion as rotation about a screw axis. A screw is defined as such
$$\mathcal{S}=\begin{bmatrix}\mathcal{S}_{\omega} \\ \mathcal{S}_{v}\end{bmatrix}=\begin{bmatrix}\mathcal{S}_{\omega x} \\ \mathcal{S}_{\omega y} \\ \mathcal{S}_{\omega z} \\ \mathcal{S}_{vx} \\ \mathcal{S}_{vy} \\ \mathcal{S}_{vz}\end{bmatrix}$$
$\mathcal{S}_{\omega}$ is the angular velocity when the speed of rotation about it $\dot{\theta}=1$ and $\mathcal{S}_v$ is the linear velocity of the origin when $\dot{\theta}=1$

$\mathcal{S}_\omega$ can be better defined as the axis of rotation (as a unit vector or the zero vector) and $\mathcal{S}_v$ is the is the vector tangent to the circle whose origin is at the screw axis and touches the origin of the frame we are describing the axis in in the direction of rotation about the axis.

Note that if the pitch of a screw is infinite, the rotational component will be 0 and the linear component will be a unit vector corresponding to the direction of linear motion. 

The twist is the representation of velocity for frames / rigid bodies, given by
$$\mathcal{V}=\mathcal{S}\dot{\theta}$$
$$\mathcal{V}t=\mathcal{S}\theta$$
This is to say that following a twist for a certain period of time is equivalent to rotating about a screw axis for a certain angle.

To change a twist's frame of reference, we need to adjust for the dimension mismatch as a transformation matrix is 4x4 and a twist is a 6-vector. To do this, we use the *adjoint representation* of the transformation matrix
$$[Ad_T]=\begin{bmatrix}R & 0 \\ [p]R & R\end{bmatrix}$$
where
$$[p] = \begin{bmatrix}0 & -p_z & p_y \\ p_z & 0 & -p_x \\ -p_y & p_x & 0\end{bmatrix}$$
So now we can say
$$\mathcal{V}_s=[Ad_T{_{sb}}]\mathcal{V}_b$$
## Exponential coordinates of rigid body motion
Moving a frame about a screw axis for a certain angle, or having a frame experience a twist for a period of time, is represented with *exponential coordinates* using the *matrix exponential*, shown below.
$$e^{[\mathcal{S}]\theta}=\mathcal{e}^{[\mathcal{V}]t}$$
where $$[\mathcal{V}]=\begin{bmatrix}[\omega] & v \\ 0 & 0\end{bmatrix}$$
where $$[\omega]=\begin{bmatrix}0 & -\omega_z & \omega_y \\ \omega_x & 0 & -\omega_z \\ -\omega_y & \omega_x & 0\end{bmatrix}$$
The notation for $[\mathcal{S}]$ is the same.
The actual closed forms of the matrix exponentials are given below
*purely translational motion*
$$e^{[\mathcal{S}]\theta}=\begin{bmatrix}I & \mathcal{S}_v\theta \\ 0 & 1\end{bmatrix}$$
*rotation + translation*
$$e^{[\mathcal{S}]\theta}=\begin{bmatrix}e^{[\mathcal{S}_\omega]\theta} & (I\theta+(1-cos\theta)[\mathcal{S}_\omega]+(\theta-sin\theta)[\mathcal{S}_\omega]^2)\mathcal{S}_v \\ 0 & 1\end{bmatrix}$$
where
$$e^{[\mathcal{S}_\omega]\theta}=I+sin\theta[\mathcal{S}_\omega]+(1 - cos\theta)[\mathcal{S}_\omega]^2$$
The matrix exponential yields a transformation matrix. 
If the screw axis is represented in the body frame
$$T_{sb'}=T_{sb}e^{[\mathcal{S}_b]\theta}=T_{sb}e^{[\mathcal{V}_b]t}$$
If the screw axis is represented in the space frame
$$T_{sb'}=e^{[\mathcal{S_s}]\theta}T_{sb}=e^{[\mathcal{V}_s]t}T_{sb}$$
## Forward kinematics
Using the matrix exponential, we can represent the forward kinematics of a robot. Let's first define the matrix $M$ as the home configuration of the robot - the transformation $T_{sb}$ when all joint angles $\theta=0$.

Given every joint is a screw axis, with a list of joint angles, we can drive the robot to its final configuration.
If the screw axes are represented in $\{s\}$
$$T(\theta)=e^{[\mathcal{S}_1]\theta}...e^{[\mathcal{S}_n]\theta}M$$
If the screw axes are represented in $\{b\}$
$$T(\theta)=Me^{[\mathcal{S}_1]\theta}...e^{[\mathcal{S}_n]\theta}$$
# Pioneer Mission
[Video](https://www.youtube.com/watch?v=8zsv5DNZ_fQ)
[PDF version](https://20693798.fs1.hubspotusercontent-na1.net/hubfs/20693798/2026/Manuals/2026%20PIONEER%20Manual_updated_12_17_Cover.pdf)
## Task 1: A Kaleidoscope of corals in cold water
1. Collect species from the coral garden
	1. Collect two basket stars and return them to the surface, side of the pool
	2. Collect two coral species and return them to the surface, side of the pool
2. Coral garden ridge modelling
	1. Via photogrammetry, autonomously create a scaled 3D model of the coral garden
		1. Create a 3D model of the coral garden
		2. Measure the length of the coral garden within 5 cm
		3. Scale the 3D model using the length of the coral garden
		4. Use the properly scaled 3D model to estimate the height of the coral garden
	2. Manually (CAD) create a scaled 3D model of the coral garden
		1. Measure the length of the coral garden
		2. Measure the height of the coral garden
		3. Create a scaled 3D model of the coral restoration area displaying the length and height measurements
	3. Manually create a 3-view technical drawing of the coral restoration area
		1. Measure the length of the coral garden
		2. Measure the height of the coral garden
		3. Create a technical drawing on paper of the coral garden, including the measured length and height with at least 3 views (top, front, side)
3. Fly a transect over the coral garden
	1. Fly a transect to create a video of the coral garden
## Task 2: Better information, better decisions
1. Mitigate invasive species
	1. Determine the number of invasive European Green crabs in the sample
		1. Using image recognition
		2. Manually
	2. Upload the data to the invasive species form
2. Iceberg tracking
	1. Track icebergs headed towards offshore oil platforms
		1. Survey iceberg at five points around its perimeter
			1. Survey all 5 points
			2. Survey one to four points
		2. Measure the keep depth of the iceberg within 10 cm of true depth
		3. Use the location, heading, and keel depth to determine the threat level of the iceberg to the four area oil platforms
			1. Determine the threat level to the offshore platform
			2. Determine the threat level to the offshore platform's subsea assets
3. Testing whale safe fishing gear
	1. Turn the handle to simulate the acoustic release of the retrieval buoy
		1. Recover the lobster pot off the bottom
		2. Return the lobster pot to the surface, side of the pool
4. Recover the buoy anchor
	1. Attach a recovery line to the buoy anchor
	2. Return the line to the surface, side of the pool
5. Service the Holyrood subsea observatory
	1. Recover the old eDNA sensor
		1. Recover the old sensor to the surface, side of the pool
		2. Analyze the sensor's data to determine the percent frequency seen of various organisms
	2. Install a new eDNA sensor
		1. Place the sensor in the designated area
		2. Connect the sensor to the Holyrood subsea observatory
	3. Remove biofouling from the Holyrood subsea observatory camera
## Task 3: Wind-powered offshore oil platform: scalable solutions for global energy needs
1. Micropile installation
	1. Install a micropile to the seafloor to secure the wind turbine
		1. Place a bubble curtain device around the micropile designated location
		2. Guide a micropile to the designated location
		3. Pull. a pin to release the micropile
2. Powering an oil platform from a wind turbine
	1. Connect the wind farm power connector to the oil platform
		1. Retrieve the power connector from the wind farm subsea station
		2. Lay the power connector cable through a waypoint away from the micropile
		3. Remove the cover from the oil platform port
		4. Install the power connector into the oil platform port
## Task 4: MATE floats under ice
1. Prior to competition, design and construct a vertical profiling float
2. Float communicates with the station prior to descending
3. Float completes two vertical profiles under the ice
	1. Profile 1
		1. Float completes first vertical profile
			1. Using a buoyancy engine
			2. Using a different mechanism
		2. Float maintains a depth of 2.5 meters for 30 seconds
		3. Float maintains a depth of 40 cm for 30 seconds
			1. Do not let float break the surface or contact the ice sheet
	2. Profile 2
		1. Same as profile 1
4. After recovery, float communicates with (transmits data autonomously to) the station
	1. Float communicates to the mission station
		1. Float communicates all data packets
		2. Float communicate at least one data packet
	2. Profile is graphed as depth over time
5. Company does not design and construct a vertical profiling float, or float does not communicate data to the mission station after recovery