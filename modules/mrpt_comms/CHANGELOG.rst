^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_comms
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.4 (2026-09-04)
------------------
* test(mrpt_comms): raise unit test coverage of the TCP, HTTP, serial-port and pub/sub code (26%->76% lines) (`#1391 <https://github.com/MRPT/mrpt/issues/1391>`_).
* fix(mrpt_comms): http_request() returned the payload padded with unused buffer tail instead of trimming to the bytes actually received.
* fix(mrpt_comms): CClientTCPSocket::connect() leaked the socket descriptor and misreported isConnected() when a later step in connect() failed.
* Contributors: Jose Luis Blanco-Claraco

3.1.3 (2026-08-12)
------------------

3.1.2 (2026-07-07)
------------------

3.1.1 (2026-07-04)
------------------

3.1.0 (2026-07-03)
------------------

3.0.4 (2026-06-17)
------------------

3.0.3 (2026-06-15)
------------------

3.0.2 (2026-06-11)
------------------

3.0.1 (2026-06-11)
------------------

3.0.0 (2026-06-06)
------------------

2.20.0 (2026-06-06)
-------------------
* Last release of the 2.x series. Starting from 3.0.0, changes are tracked
  in each module's own CHANGELOG.rst file.
* Contributors: Jose Luis Blanco-Claraco

