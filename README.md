# Robotics
**All code in MATLAB**

**1) Dinosaur** - Simulate the robot motion and determine if the the dinosaur is looking at the window GH. Used Forward Kinematics and tracked end-effector position wrt to base and used that to track GH's position wrt to camera frame.

<img width="400" alt="Screen Shot 2022-05-25 at 11 50 56 PM" src="https://user-images.githubusercontent.com/39920489/170412953-17074b16-aa4a-48dc-9398-13e0abd25d6f.png">

**2) Trajectory Generation** - Simulate the planar robot workspace. Account for each joint max and min range.
<img width="300" alt="Screen Shot 2022-05-25 at 11 51 15 PM" src="https://user-images.githubusercontent.com/39920489/170413160-9e5b7b1f-c32c-4166-8ca7-3e15687b1866.png">

**3) Inverse Kinematics** - Given a transformation matrix, use inverse kinematics using Jacobian Inverse to get the required pose of the robot. 

**4) Dynamics** - Generate the dynamics of the robot - inertia, coriolios, and gravity matrices - and use that to plot end-effector trajectory for torques sent at 200 Hz and 1KHz.

<img width="310" alt="Screen Shot 2022-05-26 at 12 04 30 AM" src="https://user-images.githubusercontent.com/39920489/170413569-968c129a-84db-4248-b393-4bf07e742225.png">


**5) Project** - Drive a KUKA robot from its initial configuration to a final configuration. Final configuration is the robot arm placing a wooden block over a target. Used forward kinematics to track end effector pose, inverse kinematics to generate desired pose, and cubic polynomial trajectory generation.

<img width="300" alt="Screen Shot 2022-05-26 at 12 08 37 AM" src="https://user-images.githubusercontent.com/39920489/170414392-fae097fb-9642-4cba-85fe-0f9118d2f729.png"> <img width="360" alt="Screen Shot 2022-05-26 at 12 09 05 AM" src="https://user-images.githubusercontent.com/39920489/170414405-356cc7e1-db82-44d9-8b3d-9e9f9db88c2e.png">
