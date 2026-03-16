import numpy as np
from ad5941 import AD5941
import time

dev = AD5941(port='/dev/cu.usbmodem101', debug=True)
dev.connect()

# Single OCP
dev.init_ocp()
ocp = dev.measure_ocp_single(npts=10)
print(f"OCP: {ocp:.3f} mV")
dev.reset()

# Use the new function for cycling OCP
print("\n--- Cycling OCP with measure_ocp_cycling ---")
dev.init_ocp()
data = dev.measure_ocp_cycling(-20, 20, 4, timeout=30)
print(f"Received {len(data)} points")
for we, ocp1, ocp2 in data:
    print(f"WE={we:7.2f} mV, AD5940={ocp1:7.2f} mV, RP2040={ocp2:7.2f} mV")

dev.close()