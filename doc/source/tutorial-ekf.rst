.. _tutorial-ekf:

===================================================================
Writing an Extended Kalman Filter (EKF) with mrpt::bayes
===================================================================

.. contents:: :local:

.. toctree::
  :maxdepth: 2

In MRPT, the family of `Kalman Filter algorithms <https://en.wikipedia.org/wiki/Kalman_filter>`_
such as the Extended KF (EKF) or the Iterative EKF (IEKF) are centralized in
one single virtual class, `mrpt::bayes::CKalmanFilterCapable <class_mrpt_bayes_CKalmanFilterCapable.html>`_.

This C++ class keeps the **system state vector** and the system **covariance matrix**, as well as a
generic method to execute one complete iteration of the selected algorithm.

In practice, solving a specific problem requires **deriving a new class** from this virtual class
and implementing a few methods such as transforming the state vector through the transition model,
or computing the Jacobian of the observation model linearized at some given value of the state space.

This page will teach you the implementation details of the 2D target tracking example
shown in this video (`full source code <page_bayes_tracking_example.html>`_):

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/0_gGXYzjcGE" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>



1. Kalman Filters in the MRPT
--------------------------------

A set of parameters that are problem-independent can be changed in the member `KF_options <struct_mrpt_bayes_TKF_options.html>`_ 
of this class, where the most important parameter is the **selection of the KF algorithm**.
Currently it can be chosen from the following ones (``mrpt::bayes::TKFMethod``):

- ``kfEKFNaive``: The basic EKF algorithm.
- ``kfEKFAlaDavison``: An EKF that processes the observations sequentially, one
  scalar component at a time, re-evaluating the prediction with the updated state.
- ``kfIKFFull``: Iterated Kalman Filter (IKF), which re-linearizes the observation
  model around increasingly more accurate values of the state vector
  (``KF_options.IKF_iterations`` times).

.. note:: 
  
  An alternative implementation of Bayesian filtering in MRPT are Particle Filters.


2. Writing a KF class for a specific problem
----------------------------------------------

2.1. Deriving a new class
~~~~~~~~~~~~~~~~~~~~~~~~~~

First a new class must be derived from ``mrpt::bayes::CKalmanFilterCapable``.
A public method must be declared as an entry point for the user, which takes the domain-specific input data
(range observations, sonar measurements, temperatures, etc.) and calls the protected method ``runOneKalmanIteration()``
of the parent class.
There are **two fundamental types of systems** the user can build by deriving a new class:

- **A regular tracking problem**: The size of the state vector remains unchanged over time. This size must be returned by the virtual method `get_vehicle_size()`.
- **A SLAM-like problem**: The size of the state vector grows as new map elements are incorporated over time.
  In this case the first ``get_vehicle_size()`` scalar elements of the state vector correspond to the state of 
  the vehicle/robot/camera/... and the rest of the state vector is a whole number of ``get_feature_size()`` sub-vectors,
  each describing one map element (landmark, feature,...).


2.2. The internal flow of the algorithm
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The method ``mrpt::bayes::CKalmanFilterCapable::runOneKalmanIteration()`` will sequentially call each of the virtual methods,
according to the following order:

- **During the KF prediction stage**:
  
  - ``OnGetAction()``
  - ``OnTransitionModel()``
  - ``OnTransitionJacobian()``
  - ``OnTransitionNoise()``
  - ``OnNormalizeStateVector()``: This can be optionally implemented if required for the concrete problem.

- **During the KF update stage**:

  - ``OnGetObservationNoise()``
  - ``OnObservationModel()``: predicts the observations for all map elements.
  - ``OnPreComputingPredictions()`` (only in SLAM-like problems): decides which predictions
    need their Jacobians and covariances computed.
  - ``OnObservationJacobians()`` (or numerical Jacobians, if not implemented).
  - ``OnGetObservationsAndDataAssociation()``: This is the ideal place for generating observations in those
    applications where it requires an estimate of the current state (e.g. in visual SLAM, to predict where each
    landmark will be found in the image). At this point the internal state vector and covariance contain the
    "prior distribution", i.e. updated through the transition model, and the predicted observations and their
    innovation covariance are provided. This is also the place where data association must be solved (in mapping problems).
  - ``OnSubtractObservationVectors()``: computes the innovation (re-implement it for angular components).
  - ``OnNormalizeStateVector()``
  - If the system implements a mapping problem, and the data association returned by
    ``OnGetObservationsAndDataAssociation()`` indicates the existence of unmapped observations, then the next
    methods will be invoked for each of these new features: ``OnInverseObservationModel()`` and
    ``OnNewLandmarkAddedToMap()``.

- ``OnPostIteration()``: A placeholder for any code the user wants to execute after each iteration (e.g. logging, visualization,...).


3. An example
-------------------

An example of a KF implementation can be found in `bayes_tracking_example <page_bayes_tracking_example.html>`_
for the problem of **tracking a vehicle** from noisy **range and bearing** data.

Here we will derive the required equations to be implemented, as well as how they are actually implemented in C++.
Note that this problem is also implemented as a Particle Filter in the same example in order to visualize side by side
the performance of both approaches.

3.1. Problem Statement
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The problem of **range-bearing tracking** is that of estimating the vehicle state:

.. math::

   \mathbf{x}=(x ~ y ~ v_x ~ v_y)

where x and y are the Cartesian coordinates of the vehicle, and vx and vy are the linear velocities.
Thus, we will use a simple constant velocity model, where the transition function is:

.. math::

   \mathbf{x}_k = f(\mathbf{x}_{k-1}, \Delta t) = \left( \begin{array}{l} x_{k-1} + v_{x,k-1} \Delta t \\ y_{k-1} + v_{y,k-1} \Delta t \\ v_{x,k-1} \\ v_{y,k-1} \end{array} \right)

We will consider the time interval between steps :math:`\Delta t` as the action :math:`u` of the system.
The observation vector :math:`z=(z_b ~ z_r)` consists of the bearing and range of the vehicle from some point (arbitrarily set to the origin):

.. math::

   z_b = atan2(y,x)

.. math::

   z_r = \sqrt{ x^2 + y^2 }

Then, it is straightforward to obtain the required Jacobian of the transition function:

.. math::

   \frac{\partial f}{\partial x} = \left( \begin{array}{cccc} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{array} \right)

and the Jacobian of the observation model:

.. math::

   \frac{\partial h}{\partial x} = \left( \begin{array}{cccc} \frac{-y}{x^2+y^2} & \frac{x}{x^2+y^2} & 0 & 0 \\ \frac{x}{\sqrt{x^2+y^2}} & \frac{y}{\sqrt{x^2+y^2}} & 0 & 0 \end{array} \right)


3.2 Implementation
~~~~~~~~~~~~~~~~~~~~

The most important implemented methods are detailed below. For further details refer to the complete sources of the example.

3.2.1 The transition model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The constant-velocity model is implemented simply as:


.. code-block:: cpp

    /** Implements the transition model \f$ \hat{x}_{k|k-1} = f( \hat{x}_{k-1|k-1}, u_k ) \f$
     * \param in_u The vector returned by OnGetAction.
     * \param inout_x At input has \f$ \hat{x}_{k-1|k-1} \f$, at output must have \f$ \hat{x}_{k|k-1} \f$.
     * \param out_skipPrediction Set this to true if for some reason you want to skip the prediction step
     *  (to do not modify either the vector or the covariance). Default: false
     */
    void CRangeBearing::OnTransitionModel(
        const KFArray_ACT& in_u, KFArray_VEH& inout_x, [[maybe_unused]] bool& out_skipPrediction) const
    {
      // in_u[0] : Delta time
      // in_out_x: [0]:x  [1]:y  [2]:vx  [3]: vy
      inout_x[0] += in_u[0] * inout_x[2];
      inout_x[1] += in_u[0] * inout_x[3];
    }

3.2.2 The transition model Jacobian
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is just the Jacobian of the state propagation equation above:

.. code-block:: cpp

    /** Implements the transition Jacobian \f$ \frac{\partial f}{\partial x} \f$
     * \param out_F Must return the Jacobian.
     *  The returned matrix must be \f$N \times N\f$ with N being either the size
     *  of the whole state vector or get_vehicle_size().
     */
    void CRangeBearing::OnTransitionJacobian(KFMatrix_VxV& F) const
    {
      F.setIdentity();

      F(0, 2) = m_deltaTime;
      F(1, 3) = m_deltaTime;
    }

3.2.3 The observations and the observation model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: cpp

    void CRangeBearing::OnGetObservationsAndDataAssociation(
        vector_KFArray_OBS& out_z,
        std::vector<int>& out_data_association,
        [[maybe_unused]] const vector_KFArray_OBS& in_all_predictions,
        [[maybe_unused]] const KFMatrix& in_S,
        [[maybe_unused]] const std::vector<size_t>& in_lm_indices_in_S,
        [[maybe_unused]] const KFMatrix_OxO& in_R)
    {
      out_z.resize(1);
      out_z[0][0] = m_obsBearing;
      out_z[0][1] = m_obsRange;

      out_data_association.clear();  // Not used
    }

    /** Implements the observation prediction \f$ h_i(x) \f$.
     * \param idx_landmarks_to_predict The indices of the landmarks in the map whose predictions are
     *  expected as output. For non SLAM-like problems, this input value is undefined and the
     *  application should just generate one observation for the given problem.
     * \param out_predictions The predicted observations.
     */
    void CRangeBearing::OnObservationModel(
        [[maybe_unused]] const std::vector<size_t>& idx_landmarks_to_predict,
        vector_KFArray_OBS& out_predictions) const
    {
      // predicted bearing:
      kftype x = m_xkk[0];
      kftype y = m_xkk[1];

      kftype h_bear = atan2(y, x);
      kftype h_range = sqrt(square(x) + square(y));

      // idx_landmarks_to_predict is ignored in NON-SLAM problems
      out_predictions.resize(1);
      out_predictions[0][0] = h_bear;
      out_predictions[0][1] = h_range;
    }

    /** Implements the observation Jacobians \f$ \frac{\partial h_i}{\partial x} \f$
     * and (when applicable) \f$ \frac{\partial h_i}{\partial y_i} \f$.
     */
    void CRangeBearing::OnObservationJacobians(
        [[maybe_unused]] size_t idx_landmark_to_predict,
        KFMatrix_OxV& Hx,
        [[maybe_unused]] KFMatrix_OxF& Hy) const
    {
      kftype x = m_xkk[0];
      kftype y = m_xkk[1];

      Hx.setZero();
      Hx(0, 0) = -y / (square(x) + square(y));
      Hx(0, 1) = x / (square(x) + square(y));

      Hx(1, 0) = x / sqrt(square(x) + square(y));
      Hx(1, 1) = y / sqrt(square(x) + square(y));
    }

    /** Computes A=A-B, wrapping the angular component. */
    void CRangeBearing::OnSubtractObservationVectors(KFArray_OBS& A, const KFArray_OBS& B) const
    {
      A -= B;
      math::wrapToPiInPlace(A[0]);
    }
