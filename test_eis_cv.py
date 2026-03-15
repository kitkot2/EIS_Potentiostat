import matplotlib.pyplot as plt
import numpy as np
from ad5941 import AD5941

def plot_cv(voltages, currents, title="Cyclic Voltammogram"):
    """Plot CV data (current vs voltage)."""
    fig, ax = plt.subplots(figsize=(8, 6))
    
    ax.plot(voltages, currents, 'b-', linewidth=1.5)
    ax.set_xlabel("Potential (mV)")
    ax.set_ylabel("Current (μA)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax.axvline(x=0, color='k', linestyle='-', linewidth=0.5)
    
    # Add arrow to show scan direction
    if len(voltages) > 1:
        mid_point = len(voltages) // 2
        ax.annotate('', xy=(voltages[mid_point+10], currents[mid_point+10]),
                   xytext=(voltages[mid_point], currents[mid_point]),
                   arrowprops=dict(arrowstyle='->', color='red', lw=2))
    
    plt.tight_layout()
    plt.show()

def plot_eis(frequencies, Z_real, Z_imag):
    """Create Nyquist and Bode plots."""
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
    
    # Nyquist plot (Z' vs -Z'')
    ax1.plot(Z_real, -np.array(Z_imag), 'o-')
    ax1.set_xlabel("Z' (Ω)")
    ax1.set_ylabel("-Z'' (Ω)")
    ax1.set_title("Nyquist Plot")
    ax1.grid(True)
    ax1.axis('equal')
    
    # Bode magnitude
    magnitude = np.sqrt(np.array(Z_real)**2 + np.array(Z_imag)**2)
    ax2.loglog(frequencies, magnitude, 'o-')
    ax2.set_xlabel("Frequency (Hz)")
    ax2.set_ylabel("|Z| (Ω)")
    ax2.set_title("Bode Magnitude")
    ax2.grid(True, which='both')
    
    # Bode phase
    phase = np.arctan2(Z_imag, Z_real) * 180 / np.pi
    ax3.semilogx(frequencies, phase, 'o-')
    ax3.set_xlabel("Frequency (Hz)")
    ax3.set_ylabel("Phase (deg)")
    ax3.set_title("Bode Phase")
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()


dev = AD5941(port='/dev/cu.usbmodem101', debug=True)
dev.connect()
dev.reset()
# Configure
dev.set_verbose(0)
dev.set_use_variable_gain(False)
dev.set_seeedstat_mode(False)
dev.set_freqlo_hz(0.2)
dev.set_freqhi_hz(200000)
dev.set_nfreqs(20)
dev.set_pga_gain(1)
dev.set_tia_rf(1)
dev.set_amplitude_mv(20)
dev.set_vbias_mv(10)
#dev.set_use_variable_gain(True)
dev.set_rcal(10000)

# Measure
#impedances = dev.run_eis_scan(rcal_value=10000, timeout_per_point=60)

impedances = dev.run_seeedstat_scan()
print("\nImpedance results:")
for f, zr, zi in impedances:
    print(f"{f:.3f} Hz: Z'={zr:.2f} Ω, Z''={zi:.2f} Ω")

freqs = [f for f, _, _ in impedances]
z_real = [zr for _, zr, _ in impedances]
z_imag = [zi for _, _, zi in impedances]
plot_eis(freqs, z_real, z_imag)



dev.set_verbose(0)
dev.set_seeedstat_mode(True)  # CV always uses ASCII mode

# CV Parameters - set these directly
start_mv = -200.0
stop_mv = 200.0
step_mv = 2.0
rate_mvs = 100.0
cycles = 5

print(f"\n=== CV Parameters ===")
print(f"Start: {start_mv} mV")
print(f"Stop: {stop_mv} mV")
print(f"Step: {step_mv} mV")
print(f"Rate: {rate_mvs} mV/s")
print(f"Cycles: {cycles}")

dev.reset()
# Run CV scan
print("\nStarting CV scan...")
cv_data = dev.run_cv_scan(
    start_mv=start_mv,
    stop_mv=stop_mv,
    step_mv=step_mv,
    rate_mvs=rate_mvs,
    cycles=cycles,
    timeout_per_step=5
)

print(f"\nCV Results ({len(cv_data)} points):")
print("Voltage (mV), Current (μA)")
for v, i in cv_data[:10]:  # Show first 10 points
    print(f"{v:8.2f}, {i:8.3f}")
if len(cv_data) > 10:
    print("  ...")

# Extract data for plotting
voltages = [v for v, _ in cv_data]
currents = [i for _, i in cv_data]

# Plot CV
plot_cv(voltages, currents, f"CV: {start_mv} to {stop_mv} mV, {rate_mvs} mV/s, {cycles} cycle(s)")
dev.reset()

dev.close()