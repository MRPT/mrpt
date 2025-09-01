#!/bin/env python3

from mrpt.system import CTicTac, CTimeLogger, compute_CRC16, encodeBase64, unitsFormat
import time

timer = CTicTac()
time.sleep(0.1)
elapsed = timer.Tac()
print(f"Elapsed: {elapsed:.3f} s")

crc_val = compute_CRC16(b"hello")
print(f"CRC16 for 'hello': {crc_val:#06x}")

b64 = encodeBase64(b"hello world")
print(f"base64 for 'hello world': {b64}")

print(f'13445 formatted: {unitsFormat(13445)}')  # "13.44 km"

logger = CTimeLogger(name="Profiler")
logger.enter("my_section")
# do work
logger.leave("my_section")
logger.dumpAllStats()
