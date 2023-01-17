.. _tutorial-motion-models:

===========================================================================
Probabilistic motion models
===========================================================================

.. contents:: :local:

1. Overview
========================================

Within a particle filter, the samples are propagated at each time step using some 
given proposal distribution. 
A common approach for mobile robots is taking the probabilistic motion model directly as this proposal.

In MRPT there are two models for probabilistic motion in SE(2) or SE(3),
implemented in <a class="externallink" title="https://docs.mrpt.org/reference/stable/classmrpt_1_1slam_1_1_c_action_robot_movement2_d.html" href="https://docs.mrpt.org/reference/stable/classmrpt_1_1obs_1_1_c_action_robot_movement2_d.html" rel="nofollow">mrpt::slam::CActionRobotMovement2D</a>. To use them just fill out the option structure "motionModelConfiguration" and select the method in "<a class="externallink" title="https://docs.mrpt.org/reference/stable/structmrpt_1_1slam_1_1_c_action_robot_movement2_d_1_1_t_motion_model_options.html" href="https://docs.mrpt.org/reference/stable/structmrpt_1_1obs_1_1_c_action_robot_movement2_d_1_1_t_motion_model_options.html" rel="nofollow">CActionRobotMovement2D::TMotionModelOptions</a>::modelSelection".

An example of usage would be like:

   using namespace mrpt::slam;
   using namespace mrpt::poses;

   CPose2D actualOdometryReading(0.20f, 0.05f, DEG2RAD(1.2f) );

   // Prepare the "options" structure:
   CActionRobotMovement2D                      actMov;
   CActionRobotMovement2D::TMotionModelOptions opts;

   opts.modelSelection = CActionRobotMovement2D::mmThrun;
   opts.thrunModel.alfa3_trans_trans = 0.10f;

   // Create the probability density distribution (PDF) from a 2D odometry reading:
   actMov.computeFromOdometry( actualOdometryReading, opts );

   // For example, draw one sample from the PDF:
   CPose2D sample;
   actMov.drawSingleSample( sample );


This page provides a description of the internal workings of these methods.


2. Gaussian probabilistic motion model
========================================

Assume the odometry is read as incremental changes in the 2D robot pose. The odometry readings are denoted as

[latex]\left( \Delta^{odo}_x ~ \Delta^{odo}_y ~ \Delta^{odo}_\phi \right)[/latex]

The model for these variables is:
<p style="text-align: center;"><a href="http://www.mrpt.org/wp-content/uploads/2013/10/Motion_model_gauss.png"><img class="aligncenter  wp-image-192" src="http://www.mrpt.org/wp-content/uploads/2013/10/Motion_model_gauss.png" alt="Motion_model_gauss" width="400" height="266" /></a>
<div align="center"> </div>
The equations that relate the prior robot pose $latex \left( x ~ y ~ \phi \right) $ and the new pose $latex \left( x' ~ y' ~ \phi' \right)$ after the incremental change are: (based on the proposal in <a title="Probabilistic_Motion_Models" href="Probabilistic_Motion_Models#References">[1</a>])
[latex]\left(<br />\begin{array}{c}<br />x' \\ y' \\ \phi'<br />\end{array}<br />\right)<br />=<br />\left(<br />\begin{array}{c}<br />x \\ y \\ \phi<br />\end{array}<br />\right)<br />+<br />\left(<br />\begin{array}{ccc}<br />\cos(\phi+\frac{\Delta^{odo}_\phi}{2}) ~ &amp; ~ -\sin(\phi+\frac{\Delta^{odo}_\phi}{2}) ~ &amp; ~ 0 ~ \\<br />\sin(\phi+\frac{\Delta^{odo}_\phi}{2}) ~ &amp; ~ \cos(\phi+\frac{\Delta^{odo}_\phi}{2}) ~ &amp; ~ 0 ~ \\<br />0 ~ &amp; ~ 0 ~ &amp; ~ 1<br />\end{array}<br />\right)<br />\left(<br />\begin{array}{c}<br />\Delta^{odo}_x \\ \Delta^{odo}_y \\ \Delta^{odo}_\phi<br />\end{array}<br />\right)<br />[/latex]
Our aim here is to obtain a multivariate Gaussian distribution of the new pose, given that the prior pose has a known value (it is the particle being propragated). In this case we can just model how to draw samples from a prior pose of $latex (0~0~0)$, and then the samples can be composed using the actual prior pose.
Using this simplification:
[latex]\left(<br />\begin{array}{c}<br />x' \\ y' \\ \phi'<br />\end{array}<br />\right)<br />=<br />\left(<br />\begin{array}{ccc}<br />\cos \frac{\Delta^{odo}_\phi}{2} ~ &amp; ~ -\sin \frac{\Delta^{odo}_\phi}{2} ~ &amp; ~ 0 ~ \\<br />\sin \frac{\Delta^{odo}_\phi}{2} ~ &amp; ~ \cos \frac{\Delta^{odo}_\phi}{2} ~ &amp; ~ 0 ~ \\<br />0 ~ &amp; ~ 0 ~ &amp; ~ 1<br />\end{array}<br />\right)<br />\left(<br />\begin{array}{c}<br />\Delta^{odo}_x \\ \Delta^{odo}_y \\ \Delta^{odo}_\phi<br />\end{array}<br />\right)<br />=<br />H<br />\left(<br />\begin{array}{c}<br />\Delta^{odo}_x \\ \Delta^{odo}_y \\ \Delta^{odo}_\phi<br />\end{array}<br />\right)<br />[/latex]
The mean of the Gaussian can be simply computed from the composition of the prior and the odometry increment. For the covariance, we need to estimate the variances of the three variables of the odometry increment. We model them as having independent, zero-mean Gaussian errors. The errors will be composed of terms that capture imperfect odometry and potential drift effects.
We denote as $latex \Sigma $ the diagonal matrix having the three variances of the odometry variables, modeled as:
[latex]\begin{array}{l}<br />\sigma_{\Delta^{odo}_x} = \sigma_{\Delta^{odo}_y} = \sigma^{min}_{xy} + \alpha_1 \sqrt{ (\Delta^{odo}_x)^2 + (\Delta^{odo}_y)^2} + \alpha_2 | \Delta^{odo}_\phi | \\<br />\sigma_{\Delta^{odo}_\phi} = \sigma^{min}_{\phi} + \alpha_3 \sqrt{ (\Delta^{odo}_x)^2 + (\Delta^{odo}_y)^2} + \alpha_4 | \Delta^{odo}_\phi |<br />\end{array}<br />[/latex]
The default parameters (loaded in the constructor and available in RawLogViewer) are:
[latex]\begin{array}{cc}<br />\alpha_1 =&amp; 0.05 ~ meters/meter \\<br />\alpha_2 =&amp; 0.001 ~ meters/degree \\<br />\alpha_3 =&amp; 5 ~ degrees/meter \\<br />\alpha_4 =&amp; 0.05 ~ degrees/degree \\<br />\sigma^{min}_{xy} =&amp; 0.01 ~ meters \\<br />\sigma^{min}_{\phi} =&amp; 0.20 ~ degrees<br />\end{array}<br />[/latex]
And finally, the covariance of the new pose after the odometry increment ($latex C$) is computed by means of:
 
<center>$latex C = J ~ \Sigma ~ J^t$</center>
where J stands for the Jacobian of H. See [3] for a derivation of this formula for error propagation.
 
The following is an example of samples obtained using this model with the <a title="/Application:RawLogViewer" href="https://www.mrpt.org/list-of-mrpt-apps/rawlogviewer/">RawLogViewer</a> application:
<p style="text-align: center;"><a href="http://www.mrpt.org/wp-content/uploads/2013/10/Screenshot_topic_motion_model_gauss.png"><img class="aligncenter  wp-image-257" src="http://www.mrpt.org/wp-content/uploads/2013/10/Screenshot_topic_motion_model_gauss.png" alt="Screenshot_topic_motion_model_gauss" width="596" height="368" /></a>
<a id="3._Thrun.2C_Fox_.26_Burgard's_book_particle_motion_model" name="3._Thrun.2C_Fox_.26_Burgard's_book_particle_motion_model"></a>


3. Sampling-based model
========================================

As above, denote the odometry readings as $latex \left( \Delta^{odo}_x ~ \Delta^{odo}_y ~ \Delta^{odo}_\phi \right) $,
and let's assume that the prior robot pose is $latex (0~0~0)$, which means that we want to draw samples
of the robot increment, not the final robot pose (to simplify the equations without loss of generality).
Then, the new robot pose, which we want to draw samples from is:

[latex]\left(<br />\begin{array}{c}<br />x' \\ y' \\ \phi'<br />\end{array}<br />\right)<br />=<br />\left(<br />\begin{array}{ccc}<br />\cos \hat\delta_{rot1} ~ &amp; ~ 0 ~ &amp; ~ 0 ~ \\<br />\sin \hat\delta_{rot1} ~ &amp; ~ 0 ~ &amp; ~ 0 ~ \\<br />0 ~ &amp; ~ 1 ~ &amp; ~ 1<br />\end{array}<br />\right)<br />\left(<br />\begin{array}{c}<br />\hat\delta_{trans} \\ \hat\delta_{rot1} \\ \hat\delta_{rot2}<br />\end{array}<br />\right)<br />[/latex]

Where the variables correspond to the robot pose increment as is shown in the figure:
<div align="center"><a href="http://www.mrpt.org/wp-content/uploads/2010/08/Motion_model_thrun.png"><img class="aligncenter  wp-image-651" src="http://www.mrpt.org/wp-content/uploads/2010/08/Motion_model_thrun.png" alt="Motion_model_thrun" width="400" height="266" /></a></div>
Here, the variables $latex \hat\delta_{trans} $, $latex \hat\delta_{rot1} $ and $latex \hat\delta_{rot2} $ are the result of adding a Gaussian, zero-mean random noise to the actual odometry readings:
[latex]\begin{array}{cc}<br />\hat\delta_{trans} =&amp; \delta_{trans} + \epsilon_{trans} ~~~~~~~~ \epsilon_{trans} \sim \mathcal{N}(0, \sigma^2_{trans}) \\<br />\hat\delta_{rot1} =&amp; \delta_{rot1} + \epsilon_{rot1} ~~~~~~~~ \epsilon_{rot1} \sim \mathcal{N}(0, \sigma^2_{rot1}) \\<br />\hat\delta_{rot2} =&amp; \delta_{rot2} + \epsilon_{rot2} ~~~~~~~~ \epsilon_{rot2} \sim \mathcal{N}(0, \sigma^2_{rot2})<br />\end{array}<br />[/latex]
The model described in <a title="Probabilistic_Motion_Models" href="Probabilistic_Motion_Models#References">[2</a>] employs the following approximations for the values of the standard deviations required for the equations above:
[latex]\begin{array}{rl}<br />\sigma_{rot1} =&amp; \alpha_1 |\delta_{rot1}| + \alpha_2 \delta_{trans} \\<br />\sigma_{trans} =&amp; \alpha_3 \delta_{trans} + \alpha_4 ( |\delta_{rot1}| + |\delta_{rot2}| ) \\<br />\sigma_{rot2} =&amp; \alpha_1 |\delta_{rot2}| + \alpha_2 \delta_{trans}<br />\end{array}<br />[/latex]
This is the model implemented in <a class="externallink" title="https://docs.mrpt.org/reference/stable/classmrpt_1_1slam_1_1_c_action_robot_movement2_d.html" href="https://docs.mrpt.org/reference/stable/classmrpt_1_1obs_1_1_c_action_robot_movement2_d.html" rel="nofollow">CActionRobotMovement2D</a> when setting "CActionRobotMovement2D::TMotionModelOptions::modelSelection" to "mmThrun". Actually, a small additional error is summed to each pose component ($latex x,y,\phi $) to avoid that for a null odometry increment the movement for all the particles become exactly zero, which may lead a particle filter to degenerate.
Below it is shown an example of samples generated using this model, for an excessively large value of $latex \alpha_2 $ (a very large "slippage"), generated by <a title="/Application:RawLogViewer" href="https://www.mrpt.org/list-of-mrpt-apps/rawlogviewer/">RawLogViewer</a>:
<p style="text-align: center;"><a href="http://www.mrpt.org/wp-content/uploads/2013/10/Screenshot_topic_motion_model_thrun_0.png"><img class="aligncenter  wp-image-259" src="http://www.mrpt.org/wp-content/uploads/2013/10/Screenshot_topic_motion_model_thrun_0.png" alt="Screenshot_topic_motion_model_thrun_0" width="511" height="337" /></a>



<h2>References</h2>
[1] Eliazar, A.I. and Parr, R. Learning probabilistic motion models for mobile robots, 2004. <a class="externallink" title="http://portal.acm.org/citation.cfm?id=1015330.1015413" href="http://portal.acm.org/citation.cfm?id=1015330.1015413" rel="nofollow">(ACM portal)</a>.
[2] Thrun S. and Burgard W. and Fox D. Probabilistic Robotics (book), 2005.
[3] Arras, K.O., "An Introduction to Error Propagation: Derivation, Meaning, and Examples of Equation cy= fx cx fx", Lausanne: Swiss Federal Institute of Technology Lausanne (EPFL), 1998.
