\page comms_socket_example Example: comms_socket_example

Example output:

~~~~~~~~~~~~~
[Server] Started
[00:27:20.8565|DEBUG|CServerTCPSocket] [CServerTCPSocket] Listening at 127.0.0.1:15000
[00:27:20.8566|DEBUG|CServerTCPSocket] [CServerTCPSocket::accept] Waiting incoming connections
[Client] Connecting
[Client] Connected. Waiting for a message...
[00:27:20.8766|DEBUG|CServerTCPSocket] [CServerTCPSocket::accept] Incoming connection accepted
[00:27:20.8767|DEBUG|CServerTCPSocket] [CServerTCPSocket::accept] Connection accepted from 127.0.0.1:40794
[Server] Connection accepted
[Client] Message received OK!:
  MSG Type: 16
  MSG Length: 119 bytes
[Client] Parsing payload...
[Client] Received payload: [1.000000 2.000000 3.000000 11.459156 22.918312 34.377468]
[Client] tx payload: [1.000000 2.000000 3.000000 11.459156 22.918312 34.377468]
[Client] Done!!
[Client] Finish
~~~~~~~~~~~~~

Contents of `comms_socket_example/SocketsTest_impl.cpp`:

\include comms_socket_example/SocketsTest_impl.cpp

C++ example source code:
\include comms_socket_example/test.cpp
