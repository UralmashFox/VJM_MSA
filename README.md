# Report
The goal of this work was creating of Virtual Joint Modeling (VJM) and Matrix structure analysis (MSA) models.
- [x] Develop VJM model of the robot with flexible links and joints
- [x]  Develop MSA model of the robot with flexible links and joints
- [x]  Implement linear VJM and MSA model in Matlab 
- [x] Create deflection maps for 100N force in X, Y, Z directions
- [x] Compare VJM and MSA results

## VJM:
Firstly was solved inverse kinematics by atan2 function. Were found all the rotation angles per each joint. Assuming, that translational joints move the same as end-effector: each axis depends on one of the coordinate of the platform. Platform is a point.
Based on the inverse kinematics we can obtain:
![](https://i.imgur.com/kMGF7se.jpg)

this is for 1st (primary) configuration, when end-effector is in (1,1,1) position.

For full positions, described in work:
![](https://i.imgur.com/XcJtXpz.jpg)
Please, pay attention: red lines - link from base to middle joint. Green lines - link from middle joint to the end-effector. Yellow - second link over first.
Then K-matrices were found, depending on primary information about material and shape of links.

Then deflection was found by following formula:
```
delta_t = inv(Kc)*F
```
There were used 5 different forces:
- no force:
```
F = [0 0 0 0 0 0]'
```
- 100 N along x-axis:
```
F = [100 0 0 0 0 0]'
```
- 100 N along y-axis:
```
F = [0 100 0 0 0 0]'
```
- 100 N along z-axis:
```
F = [0 0 100 0 0 0]'
```
- 100 N along all-axis:
```
F = [100 100 100 0 0 0]'
```
 Results:
![](https://i.imgur.com/fEYXQvr.jpg)
![](https://i.imgur.com/SRoJii5.jpg)
![](https://i.imgur.com/X6W6i6u.jpg)
![](https://i.imgur.com/BLKu2Id.jpg)
![](https://i.imgur.com/uxEM7KO.jpg)

## MSA:

Again, were found all k-matrices and overall stiffness matrix for links

upper shows around or along which axis each movement is

Aggregation and it's reduction.

Total stiffness matrix was found from following assumption:
```
Kc = D1 - C1*inv(A1)*B1
```

Applying wrench (force for our task) find deflection and it's magnitude. Forces were the same as for VJM.
Plots:
![](https://i.imgur.com/huaLIZV.jpg)
![](https://i.imgur.com/0El1isj.jpg)
![](https://i.imgur.com/dUB9HZJ.jpg)
![](https://i.imgur.com/cCdlJ5d.jpg)
![](https://i.imgur.com/MuzFaeE.jpg)
## Comparision
If to compare VJM with MSA results, it may be said that ~~I didn't understand the MSA topic~~ MSA doesn't do it's work well, because of really high numbers of deflection. VJM shows much better results. Moreover, they're much more realistic, because it may be detected the peak of deflection (about 0.100000012 in the point (10,10,10)). That shows, that in the farthest distance the deflection is bigger, what is obviouse.
## VJM model
![](https://i.imgur.com/Ax7xpbF.jpg)
Where Ps - passive joint, circle with spring - virtual spring, EE - end-effector (platform in our case)

## MSA model
![](https://i.imgur.com/pCfeoaM.jpg)
Where the part from base to link is an elastic support, links are elastically connected between each other and with platform.



