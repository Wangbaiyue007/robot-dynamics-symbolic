# Symbolic Dynamics
Can machines beat humans in the symbolic space?  
Some documentation is on the [Notion](https://www.notion.so/Symbolic-Dynamics-48e1fd577cab490193e373d776ac2949) page.
## Note
- **Currently only support urdf that satisfy the following condition:**
1. There are only revolute joints.
2. The transformations between joints in the **home configuration** are orthognal (the rotations are multiple of $\pi/2$).
- The animation is currenly irrelavent and not done yet.
- Running this code requires the [symbolic toolbox](https://www.mathworks.com/products/symbolic.html) and the [robotics toolbox](https://www.mathworks.com/products/robotics.html).
- [`DynamicsSym.SaveFunction()`](https://github.com/roahmlab/dynamics-symbolic/blob/main/test.m#L17) may take tremendous amout of time. Avoid using it if you can.
## Instruction
Run `startup.m` to set path. Run `test.m` to see the comparison between the symbolic dynamics and the ground truth.
## Results
The comparison between the calculation of the symbolic function (using substitution) and the matlab toolbox is shown here.  
1. Mass matrix
<p align="center">
    <img src="figs/compare_massmatrix.png">
<p/>
2. Transformation matrix
<p align="center">
    <img src="figs/compare_transform.png">
<p/>
3. Gravitation torque matrix
<p align="center">
    <img src="figs/compare_gravitation.png">
<p/>
4. Forward dynamics
<p align="center">
    <img src="figs/result.png">
<p/>

## Reference
1. [Robot Modeling and Control](https://www.wiley.com/en-us/Robot+Modeling+and+Control%2C+2nd+Edition-p-9781119524045)