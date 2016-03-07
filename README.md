ICP-test
========

This is simple ICP registration codes by using PCL 1.7 APIs.

Requirements
------------
- pcl > 1.7
- cmake > 2.8
- eigen > 3

Codes
-----
- `icp1_simple.cpp`
  simple registration with ICP. Only results are shown.
- `icp2_iterative_view.cpp`
  simple registration with ICP. Animated registration process is shown.
- `icp3_with_normal_iterative_view.cpp`
  registration by using ICP with normal vector information. Animated registration process is shown.
- `icp4_after_feature_registration.cpp`
  registration by using feature detection, description, matching, followed by ICP for fine registration.
- `transform_estimation`
  estimation of R and t (and scale) from given two sets of points

Build
-----

```
mkdir build
cd build
cmake ..
make
```

Run
----

```
cd build
```

### sample 1

Simple registration with ICP. Only results are shown.

```
$ ./icp1_simple ../data/bun{000,045}mesh.ply 
Converged. score =1.19601e-05
   0.827246  0.00948285   -0.561763   0.0341258
  -0.012711     0.99992 -0.00183846 0.000735376
     0.5617  0.00866113    0.827298   0.0383121
          0           0           0           1
```

### sample 2

Simple registration with ICP. Animated registration process is shown.

```
$ ./icp2_iterative_view ../data/bun{000,045}mesh.ply 
0.000115243
6.63744e-05
3.57769e-05
2.65457e-05
1.99575e-05
1.64426e-05
1.42573e-05
1.27708e-05
1.17543e-05
1.10658e-05
1.0597e-05
1.02805e-05
1.00704e-05
9.92269e-06
9.82098e-06
9.74741e-06
........(omit)
```
press `q` to stop.


### sample 3

Registration by using ICP with normal vector information. Animated registration process is shown.

```
$ ./icp3_with_normal_iterative_view ../data/bun{000,045}mesh.ply 
0.000228403
8.08289e-05
2.11145e-05
2.11849e-05
1.23206e-05
1.10054e-05
1.09729e-05
1.09652e-05
1.09648e-05
1.09648e-05
1.09648e-05
1.09648e-05
1.09648e-05
1.09648e-05
1.09648e-05
........(omit)
```
press `q` to stop.


### sample 4

Registration by using ICP with normal vector information. Animated registration process is shown.

```
$ ./icp4_after_feature_registration ../data/pcl_data/milk.pcd ../data/pcl_data/milk_cartoon_all_small_clorox.pcd 
scale: 0.00639819
detection
number of source keypoints found: 224
number of target keypoints found: 29130
description
Estimating transformation
  0.968323  -0.130764   0.212724   -0.16276
  0.129827   0.991365  0.0184329   0.205639
 -0.213298 0.00976849   0.976938 -0.0389752
         0          0          0          1
```

+ Correspondences (matches) are shown with lines.
  There  are many outliers.
  press `q` to proceed.
+ Outlier matches are rejected.
  press `q` to proceed.
+ Aligned the milk to the scene with matched points.
  press `q` to proceed.
+ Final refinement by PCL.
  Estimated R and t are shown.
  press `q` to stop.




### sample : R and t estimation

Estimation of R and t (and scale) from given two sets of points.
- One is randomly generated points, and 
- the other is transfomed by R, t (and optionaly s).

Note that resutls changes every time because of the use of random().


+ R, t and s are estimatd if no arguments are given.

```
$ ./transform_estimation 
true R
   0.148915   -0.563813    0.812366           0
  -0.106047   -0.825894   -0.553762           0
   0.983147 -0.00368515   -0.182779           0
          0           0           0           1
true T
1.43107
1.16595
1.87135
true sR
  0.455674   -1.72524     2.4858          0
 -0.324499    -2.5272   -1.69449          0
   3.00839 -0.0112764  -0.559295          0
         0          0          0          1
true scale 3.05996
true transformation
  0.455674   -1.72524     2.4858    1.43107
 -0.324499    -2.5272   -1.69449    1.16595
   3.00839 -0.0112764  -0.559295    1.87135
         0          0          0          1
estimated scale 3.05996
estimated transformation 
  0.455674   -1.72524     2.4858    1.43107
 -0.324499    -2.5272   -1.69449    1.16595
   3.00839 -0.0112764  -0.559296    1.87135
         0          0          0          1
```


+ R, t are estimatd if any arguments are given.

```
$ ./transform_estimation noscale
true R
  0.255721  -0.744612   0.616571          0
  0.158815  -0.596756   -0.78655          0
  0.953616   0.299058 -0.0343475          0
         0          0          0          1
true T
1.82391
1.52619
1.27993
true transformation
  0.255721  -0.744612   0.616571    1.82391
  0.158815  -0.596756   -0.78655    1.52619
  0.953616   0.299058 -0.0343475    1.27993
         0          0          0          1
estimated transformation 
  0.255721  -0.744612   0.616571    1.82391
  0.158815  -0.596756   -0.78655    1.52619
  0.953617   0.299058 -0.0343475    1.27993
         0          0          0          1
```
