ICP-test
========

This is simple ICP registration codes by using PCL 1.7 APIs.

Requirements
------------
pcl > 1.7
cmake > 2.8


Codes
-----
- icp1_simple.cpp
  simple registration with ICP. Only results are shown.
- icp2_iterative_view.cpp
  simple registration with ICP. Animated registration process is shown.
- icp3_with_normal_iterative_view.cpp
  registration by using ICP with normal vector information. Animated registration process is shown.
- icp4_after_feature_registration.cpp
  registration by using feature detection, description, matching, followed by ICP for fine registration.
