\page bayes_tracking_example Example: bayes_tracking_example

This example illustrates how to implement an Extended Kalman Filter (EKF)
and a particle filter (PF) using mrpt::bayes classes, for the problem of
tracking a 2D mobile target with **state space** being its **location** and its
**velocity vector**.

Demo video: [https://www.youtube.com/watch?v=0_gGXYzjcGE](https://www.youtube.com/watch?v=0_gGXYzjcGE)

The equations of this example and the theory behind them are explained in [this tutorial](tutorial-ekf.html).



![bayes_tracking_example screenshot](doc/source/images/bayes_tracking_example_screenshot.png)
C++ example source code:
\include bayes_tracking_example/test.cpp
