.. _app_2d-slam-demo:

====================================================
Application: 2d-slam-demo
====================================================


1. Description
-----------------
This GUI application is a demonstration of 2D range-bearing SLAM
under the paradigm of EKF-based SLAM, in a didactic way that 
allows extensive experimentation with data-association and parameter tuning.

It is an extension to a similar Matlab program developed
by `J. Neira <https://webdiis.unizar.es/~neira/>`_ and 
`J. D. Tardós <https://webdiis.unizar.es/~jdtardos/>`_ (University of Zaragoza). 

At the core of this program lies one MRPT class: 
`mrpt::slam::CRangeBearingKFSLAM2D <class_mrpt_slam_CRangeBearingKFSLAM2D.html>`_.

The paper explaining the Matching Likelihood criteron for data association (one of the two choices offered in this program)
is :cite:`blanco2012amd` 
(`BibTex <https://ingmec.ual.es/aigaion2//index.php/export/publication/216/bibtex>`_, 
`Draft PDF <https://ingmec.ual.es/~jlblanco/papers/blanco2012amd.pdf>`_ ).


2. Demo video
----------------

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
          <iframe src="https://www.youtube.com/embed/IHGcW_DCaps" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

3. Command line options
-----------------------------

Invoking the program without any flag will launch it in normal, interactive, mode::


   USAGE: 
   
   2d-slam-demo  [-r] [-n] [-c <params.ini>] [--] [--version] [-h]
   
   
   Where: 
   
   -r,  --norun
       Just load the config file, don't run it.
   
   -n,  --nogui
       Don't stay in the GUI, exit after the experiment.
   
   -c <params.ini>,  --config <params.ini>
       Config file to load
   
   --,  --ignore_rest
       Ignores the rest of the labeled arguments following this flag.
   
   --version
       Displays version information and exits.
   
   -h,  --help
       Displays usage information and exits.
   
   
   2d-slam-demo

