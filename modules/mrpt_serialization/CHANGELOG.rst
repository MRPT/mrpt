^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_serialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.4 (2026-09-04)
------------------
* test(mrpt_serialization): add a unit test suite (module had none) covering CArchive, CMessage, and STL/std::optional serialization (`#1390 <https://github.com/MRPT/mrpt/issues/1390>`_).
* fix(mrpt_serialization): CMessage::sendMessage()/receiveMessage() used mismatched byte order for the payload length (no message >= 256 bytes could be received) and misread an empty payload as a framing error; sendMessage() now also rejects payloads too large for the 16-bit length field instead of overflowing the frame buffer.
* fix(mrpt_serialization): operator<<(std::monostate) writes no version byte, but the reader only skipped it for "nullptr", so an empty std::variant failed to deserialize.
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

