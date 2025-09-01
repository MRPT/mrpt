#!/bin/env python3

# from mrpt.core import Clock, WorkerThreadsPool, abs_diff_int, reverse_bytes_u32
import mrpt

# Clock
now_sec = Clock.nowDouble()
tp = Clock.fromDouble(123.456)
print("Current time:", now_sec)

# WorkerThreadsPool
pool = WorkerThreadsPool(4)
pool.enqueue(lambda: print("Hello from a thread!"))
print("Pending tasks:", pool.pendingTasks())

# abs_diff
print(abs_diff_int(10, 3))  # 7

# reverse_bytes
x = 0x12345678
print(hex(reverse_bytes_u32(x)))  # 0x78563412
