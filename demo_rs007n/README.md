# Demo of rs007n

## Adept motion with Python interface

### How to run

![rs007n_screenshot](doc/rs007n_screenshot.png)

```
roslaunch khi_rs007n_moveit_config demo.launch
rosrun moveit_tutorials demo_adept_motion_rs007n.py
# Graph PDF is saved to /tmp/demo_adept_motion_rs007n.pdf
```

### [Comparison of time parameterization algorithms](doc/demo_adept_motion_rs007n_all.pdf)

![No Time Parametrization](doc/demo_adept_motion_rs007n.png)

![Iterative Spline Parametrization](doc/demo_adept_motion_rs007n_isp.png)

![Iterative Time Parametrization](doc/demo_adept_motion_rs007n_itp.png)

![Time Optimal Trajectory Generation (New)](doc/demo_adept_motion_rs007n_totg.png)
