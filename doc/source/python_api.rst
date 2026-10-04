.. _python_api:

====================
Python API reference
====================

MRPT is available from Python 3 through one package per C++ module
(``mrpt.poses``, ``mrpt.maps``, ...), all under the ``mrpt`` namespace:

.. code-block:: python

   from mrpt.poses import CPose3D
   from mrpt.maps import CSimplePointsMap

Install them with the ``python3-mrpt`` packages (Debian/Ubuntu), or build MRPT
from sources with ``pybind11-dev`` installed (then ``source install/setup.bash``).
Each package ships PEP 561 type stubs (``.pyi``), so IDEs and type checkers
show signatures and docstrings. Most classes link to the C++ class they wrap,
whose documentation applies to the Python methods with the same name.

See also the :ref:`python_examples`.

.. toctree::
   :maxdepth: 1
   :glob:

   python_api/mrpt/*/index
