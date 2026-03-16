import serial
import time
import struct
import re
import math

class AD5941:
    """Controller for AD5941-based potentiostat.
    
    This class provides a Python interface to the HunStat2. 
    It communicates via serial (USB) using the command set defined in the device firmware. 
    All configuration commands are sent as single-character strings followed by parameters; 
    responses are either ASCII text or binary data depending on the mode.
    """
    
    # Conversion constants (from Arduino code)
    DAC12BITVOLT_1LSB = 2200.0 / 4095      # mV per LSB of 12-bit DAC
    DAC6BITVOLT_1LSB = DAC12BITVOLT_1LSB * 64  # mV per LSB of 6-bit DAC
    
    def __init__(self, port='/dev/cu.usbmodem1101', baudrate=1000000, debug=False):
        """Initialize the controller but do not open the port yet.
        
        Args:
            port: Serial port device name (e.g., '/dev/cu.usbmodem1101').
            baudrate: Communication speed; must match device firmware (1 Mbps).
            debug: If True, print all sent commands and received data.
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.response_delay = 1.1  # seconds, from earlier tests
        self.debug = debug
        self._ascii_mode = True

        
    def connect(self):
        """Open the serial connection to the device.
        
        After opening, a short delay allows the device to reset and become ready.
        """
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=3,
            write_timeout=2
        )
        time.sleep(2)
        print(f"Connected to {self.port}")

        
    def close(self):
        """Close the serial connection."""
        if self.ser:
            self.ser.close()
            print("Disconnected")

            
    def write(self, cmd):
        """Send a command that does not expect a response.
        
        The command string is automatically terminated with a newline.
        The input buffer is cleared before sending.
        
        Args:
            cmd: Command string (e.g., 'g2' for PGA gain 2).
        """
        self.ser.reset_input_buffer()
        self.ser.write((cmd + '\n').encode())
        if self.debug:
            print(f"TX: {cmd}")

        
    def query(self, cmd):
        """Send a command and return the response as a string.
        
        Waits for the fixed response delay (1.1 s) and then reads all
        available data. Useful for commands like '?' and '!'.
        
        Args:
            cmd: Command string.
        
        Returns:
            Decoded response string, or None if no response.
        """
        self.ser.reset_input_buffer()
        self.ser.write((cmd + '\n').encode())
        if self.debug:
            print(f"TX: {cmd} (waiting {self.response_delay}s)")
        
        time.sleep(self.response_delay)
        
        response = b''
        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)
            time.sleep(0.1)
            while self.ser.in_waiting > 0:
                response += self.ser.read(self.ser.in_waiting)
                time.sleep(0.1)
        
        if response:
            decoded = response.decode(errors='ignore')
            if self.debug:
                print(f"RX: {len(response)} bytes")
            return decoded
        else:
            if self.debug:
                print("RX: no response")
            return None


    def reset(self):
        """Reset the AD5941 (command 'Z')."""
        self.write('Z')
        time.sleep(0.5)
        if self.debug:
            print("Device reset")
            
            
    # ------------------------------------------------------------------------
    # Parameter setters (write-only commands)
    # ------------------------------------------------------------------------
    
    def set_verbose(self, mask):
        """Set verbosity mask (command '@').
        
        Controls debug output from the device. Higher bits enable more
        detailed logging.
        
        Args:
            mask: Integer bitmask (0 = quiet, 1 = basic info).
        """
        self.write(f'@{mask}')

        
    def set_seeedstat_mode(self, enabled):
        """Set output mode (command 'S').
        
        Args:
            enabled: If True, ASCII mode (human-readable); if False, binary mode.
        """
        self.write(f'S{1 if enabled else 0}')
        self._ascii_mode = enabled
        time.sleep(0.1)

        
    def set_pga_gain(self, gain):
        """Set ADC programmable gain amplifier (command 'g').
        
        Args:
            gain: Integer 0–4:
                0 = 1x
                1 = 1.5x
                2 = 2x
                3 = 4x
                4 = 9x
        """
        if gain not in range(5):
            raise ValueError("PGA gain must be 0-4")
        self.write(f'g{gain}')

        
    def set_tia_rf(self, index):
        """Set transimpedance amplifier feedback resistor (command 'r').
        
        Args:
            index: Integer 0–7:
                0 = 200 Ω
                1 = 1 kΩ
                2 = 5 kΩ
                3 = 10 kΩ
                4 = 20 kΩ
                5 = 40 kΩ
                6 = 80 kΩ
                7 = 160 kΩ
        """
        if index not in range(8):
            raise ValueError("TIA_Rf index must be 0-7")
        self.write(f'r{index}')

        
    def set_constA(self, value):
        """Set constant A for OCP calculation (command 'i').
        
        This enables the use of a linear calibration for open-circuit potential.
        
        Args:
            value: Float constant.
        """
        self.write(f'i{value}')

        
    def set_constB(self, value):
        """Set constant B for OCP calculation (command 'j').
        
        Args:
            value: Float constant.
        """
        self.write(f'j{value}')

        
    def set_eis_mode(self, mode):
        """Set EIS measurement mode (command 'm').
        
        Args:
            mode: 0 = measure unknown impedance (Rz), 1 = measure calibration resistor (RCAL).
        """
        if mode not in (0, 1):
            raise ValueError("EIS mode must be 0 or 1")
        self.write(f'm{mode}')

        
    def set_ocp_npts(self, n):
        """Set number of points to average in OCP measurement (command 'n').
        
        Args:
            n: Non-negative integer.
        """
        if n < 0:
            raise ValueError("OCP_npts must be non-negative")
        self.write(f'n{n}')

        
    def set_vzero(self, value):
        """Set Vzero 6-bit DAC value (command 'a').
        
        Vzero sets the baseline potential for the working electrode.
        
        Args:
            value: Integer 0–63.
        """
        if value not in range(64):
            raise ValueError("Vzero must be 0-63")
        self.write(f'a{value}')

        
    def set_use_variable_gain(self, enable):
        """Enable/disable automatic gain selection (command 's').
        
        When enabled, the device chooses optimal TIA_Rf and PGA gain based on frequency.
        
        Args:
            enable: True = on, False = off.
        """
        self.write(f's{1 if enable else 0}')

        
    def set_nfreqs(self, n):
        """Set number of frequency points in an EIS sweep (command 'y').
        
        Args:
            n: Positive integer (≥1).
        """
        if n < 1:
            raise ValueError("Number of frequencies must be at least 1")
        self.write(f'y{n}')

        
    def set_vbias_code(self, code):
        """Set Vbias 12-bit DAC code directly (command 'b').
        
        Vbias sets the reference electrode potential.
        
        Args:
            code: Integer 0–4095.
        """
        if code not in range(4096):
            raise ValueError("Vbias code must be 0-4095")
        self.write(f'b{code}')

        
    def set_vbias_mv(self, mv):
        """Set Vbias in millivolts (converted to DAC code).
        
        The conversion uses the formula from the Arduino firmware.
        
        Args:
            mv: Desired bias voltage in mV (may be negative).
        """
        code = 1664 + int(-mv * (1850.0 - 1664.0) / 100.0 + 0.5)
        code = max(0, min(4095, code))
        self.set_vbias_code(code)

        
    def set_amplitude_code(self, code):
        """Set sine wave amplitude word (command 'z').
        
        The amplitude word is an 11-bit value (0–2047) that controls the
        zero-to-peak amplitude of the excitation sine wave.
        
        Args:
            code: Integer 0–2047.
        """
        if code not in range(2048):
            raise ValueError("Amplitude code must be 0-2047")
        self.write(f'z{code}')

        
    def set_amplitude_mv(self, mv):
        """Set amplitude in millivolts (zero-to-peak).
        
        Converts mV to the internal 11-bit code using the Arduino formula.
        
        Args:
            mv: Desired amplitude in mV (typically ≤ 800 mV).
        """
        code = int(mv * 126.0 / 50.0 + 0.5)
        code = max(0, min(2047, code))
        self.set_amplitude_code(code)

        
    def set_offset_code(self, code):
        """Set sine wave offset word (command 'v').
        
        The offset is a 12-bit two’s-complement value (0–4095) that adds a DC
        component to the excitation.
        
        Args:
            code: Integer 0–4095 (interpreted as 12-bit two’s complement).
        """
        if code not in range(4096):
            raise ValueError("Offset code must be 0-4095")
        self.write(f'v{code}')

        
    def set_offset_mv(self, mv):
        """Set offset in millivolts.
        
        Converts mV to the internal 12-bit two’s-complement code.
        
        Args:
            mv: Desired offset in mV (may be negative).
        """
        code = (int(mv) - 56) & 0xFFF
        self.set_offset_code(code)

        
    def set_freqlo_hz(self, hz):
        """Set lower frequency of EIS sweep in Hz (command 'W').
        
        Args:
            hz: Frequency in Hz (float). Internally converted to millihertz.
        """
        self.write(f'W{hz}')

        
    def set_freqlo_mhz(self, mhz):
        """Set lower frequency directly in millihertz (command 'w').
        
        Args:
            mhz: Frequency in millihertz (integer).
        """
        self.write(f'w{int(mhz)}')

        
    def set_freqhi_hz(self, hz):
        """Set upper frequency of EIS sweep in Hz (command 'X').
        
        Args:
            hz: Frequency in Hz (float).
        """
        self.write(f'X{hz}')

        
    def set_freqhi_mhz(self, mhz):
        """Set upper frequency directly in millihertz (command 'x').
        
        Args:
            mhz: Frequency in millihertz (integer).
        """
        self.write(f'x{int(mhz)}')

        
    def set_cgmax(self, value):
        """Set maximum combined gain for automatic gain selection (command 't').
        
        Combined gain = TIA_Rf (Ω) x PGA gain (numeric factor).
        
        Args:
            value: Integer maximum.
        """
        self.write(f't{value}')
 
        
    def set_cgmin(self, value):
        """Set minimum combined gain for automatic gain selection (command 'u').
        
        Args:
            value: Integer minimum.
        """
        self.write(f'u{value}')

        
    def set_rcal(self, ohms):
        """Set calibration resistor value in ohms (command 'c').
        
        This value is used in impedance calculations.
        
        Args:
            ohms: Resistor value (float).
        """
        self.write(f'c{ohms}')
        
    # ------------------------------------------------------------------------
    # Query commands
    # ------------------------------------------------------------------------
    
    def get_parameters(self):
        """Send '?' and parse the response into a dictionary.
        
        The device returns a multi-line ASCII dump of all current parameters.
        This method extracts key-value pairs and also captures the displayed
        mV values for bias, amplitude, and offset.
        
        Returns:
            Dictionary where keys are the descriptive parameter names
            (including the command letter in parentheses) and values are
            integers or floats.
        """
        response = self.query('?')
        if not response:
            return {}
        params = {}
        for line in response.split('\n'):
            line = line.strip()
            if '=' in line and not line.startswith('--'):
                parts = line.split('=')
                if len(parts) == 2:
                    key = parts[0].strip()
                    val = parts[1].strip()
                    if ' ' in val:
                        val = val.split(' ')[0]
                    try:
                        if '.' in val:
                            params[key] = float(val)
                        else:
                            params[key] = int(val)
                    except:
                        params[key] = val
            elif line.startswith('Bias'):
                m = re.match(r'Bias\s+\(B\)=([\d.-]+)', line)
                if m:
                    params['Bias_mV'] = float(m.group(1))
            elif line.startswith('Amplitude'):
                m = re.match(r'Amplitude\s+\(Y\)=([\d.-]+)', line)
                if m:
                    params['Amplitude_mV'] = float(m.group(1))
            elif line.startswith('Offset'):
                m = re.match(r'Offset\s+\(V\)=([\d.-]+)', line)
                if m:
                    params['Offset_mV'] = float(m.group(1))
        return params

    
    def get_history(self):
        """Send '!' and return the command history string.
        
        Returns:
            String containing recent commands sent to the device.
        """
        return self.query('!')

    
    def _get_param_by_cmd(self, cmd):
        """Helper to retrieve a single parameter value by its command letter.
        
        Args:
            cmd: Single character command (e.g., 'y' for nfreqs).
        
        Returns:
            Parameter value (int or float), or None if not found.
        """
        params = self.get_parameters()
        for key, val in params.items():
            if f'({cmd})' in key:
                return val
        return None
    
    # ------------------------------------------------------------------------
    # EIS measurement
    # ------------------------------------------------------------------------
    
    @staticmethod
    def _to_float(val):
        """Convert a raw 32-bit DFT register value to a floating-point number.
        
        The AD5940 stores DFT results as 18-bit two’s complement with the lowest
        2 bits representing the fractional part. This matches the Arduino
        ToFloat() function.
        
        Args:
            val: 32-bit unsigned integer from the device.
        
        Returns:
            Floating-point value.
        """
        val &= 0x3FFFF
        frac = val & 3
        integer = val >> 2
        if integer & (1 << 15):
            integer -= (1 << 16)
        return integer + frac / 4.0

    
    def run_eis_scan_binary(self, mode, n_points, timeout_per_point=30):
        """Run an EIS scan in binary mode and return raw (real, imag) pairs.
        
        The scan is started with the 'E' command. The device sends back
        8 bytes per frequency point (4-byte real, 4-byte imaginary,
        little-endian). This method waits until all data is received
        and converts each pair to floats using _to_float().
        
        Args:
            mode: 0 = unknown impedance, 1 = calibration resistor (RCAL).
            n_points: Number of frequency points (must match device setting).
            timeout_per_point: Maximum seconds allowed per point.
        
        Returns:
            List of (real, imag) tuples as floats.
        """
        self.set_eis_mode(mode)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.write('E')
        
        bytes_expected = 8 * n_points
        total_timeout = timeout_per_point * n_points + 10
        start_time = time.time()
        data = b''
        
        if self.debug:
            print(f"Waiting for {bytes_expected} bytes, timeout={total_timeout:.1f}s")
        
        while len(data) < bytes_expected and (time.time() - start_time) < total_timeout:
            if self.ser.in_waiting:
                chunk = self.ser.read(self.ser.in_waiting)
                data += chunk
                if self.debug:
                    print(f"Read {len(chunk)} bytes, total {len(data)}")
            else:
                time.sleep(0.1)
        
        if len(data) < bytes_expected:
            raise RuntimeError(f"Timeout after {time.time()-start_time:.1f}s: "
                               f"expected {bytes_expected} bytes, got {len(data)}")
        
        if len(data) > bytes_expected:
            if self.debug:
                print(f"Warning: received {len(data)} bytes, expected {bytes_expected}. Truncating.")
            data = data[:bytes_expected]
        
        if self.debug:
            print(f"Hex data: {' '.join(f'{b:02x}' for b in data)}")
        
        fmt = '<' + ('II' * n_points)
        values = struct.unpack(fmt, data)
        
        results = [(self._to_float(values[2*i]), self._to_float(values[2*i+1])) for i in range(n_points)]
        return results

  
    def generate_frequencies(self):
        """Generate the logarithmically spaced frequency list from current settings.
        
        Uses the values of freqlo, freqhi, and nfreqs read from the device.
        
        Returns:
            List of frequencies in Hz.
        """
        flo_mhz = self._get_param_by_cmd('w')
        fhi_mhz = self._get_param_by_cmd('x')
        n = self._get_param_by_cmd('y')
        if None in (flo_mhz, fhi_mhz, n):
            raise RuntimeError("Frequency parameters not set")
        flo_hz = flo_mhz / 1000.0
        fhi_hz = fhi_mhz / 1000.0
        if n == 1:
            return [flo_hz]
        log_start = math.log10(flo_hz)
        log_end = math.log10(fhi_hz)
        log_step = (log_end - log_start) / (n - 1)
        return [10 ** (log_start + i * log_step) for i in range(n)]
 
    
    def run_eis_measurement(self, rcal_value=None, timeout_per_point=30):
        """Perform a full impedance measurement (RCAL + unknown) and return results.
        
        The measurement consists of two EIS scans: first the calibration resistor,
        then the unknown. Impedance is calculated using the device’s internal
        formula: Z = (RCAL / unknown) * Rcal, which is the ratio of the two
        complex DFT results. The imaginary part is then flipped to the standard
        EIS convention (negative for capacitive) to produce conventional Nyquist
        and Bode plots.
        
        Args:
            rcal_value: Value of the calibration resistor in ohms. If None,
                        it is read from the device parameters.
            timeout_per_point: Seconds allowed per frequency point in each scan.
        
        Returns:
            List of (frequency, Z_real, Z_imag) tuples.
        """
        self.set_seeedstat_mode(False)
        time.sleep(0.1)
        
        # Get all needed parameters at once
        n_points = self._get_param_by_cmd('y')
        if n_points is None:
            raise RuntimeError("Number of frequencies not set")
        print(f"Number of frequency points: {n_points}")
        
        freqlo_mhz = self._get_param_by_cmd('w')
        freqhi_mhz = self._get_param_by_cmd('x')
        if None in (freqlo_mhz, freqhi_mhz):
            raise RuntimeError("Frequency range not set")
        
        if rcal_value is None:
            rcal_value = self._get_param_by_cmd('c')
            if rcal_value is None:
                raise RuntimeError("RCAL value not set")
            print(f"RCAL value: {rcal_value} ohms")
        
        # Compute frequencies locally
        flo_hz = freqlo_mhz / 1000.0
        fhi_hz = freqhi_mhz / 1000.0
        if n_points == 1:
            freqs = [flo_hz]
        else:
            log_start = math.log10(flo_hz)
            log_end = math.log10(fhi_hz)
            log_step = (log_end - log_start) / (n_points - 1)
            freqs = [10 ** (log_start + i * log_step) for i in range(n_points)]
        
        # Measure RCAL
        print("Measuring RCAL...")
        rcal_data = self.run_eis_scan_binary(mode=1, n_points=n_points,
                                            timeout_per_point=timeout_per_point)
        
        # Measure unknown
        print("Measuring unknown...")
        unknown_data = self.run_eis_scan_binary(mode=0, n_points=n_points,
                                                timeout_per_point=timeout_per_point)
        
        # Compute impedance using device formula: Z = (RCAL / unknown) * Rcal
        impedances = []
        for i, (u_real, u_imag) in enumerate(unknown_data):
            r_real, r_imag = rcal_data[i]
            denom = u_real*u_real + u_imag*u_imag
            if denom == 0:
                z_real = z_imag = float('nan')
            else:
                z_real = (r_real * u_real + r_imag * u_imag) / denom * rcal_value
                z_imag = (r_imag * u_real - r_real * u_imag) / denom * rcal_value
            # Flip imaginary sign to standard EIS convention (negative for capacitive)
            z_imag = -z_imag
            impedances.append((freqs[i], z_real, z_imag))
        
        return impedances


    def run_seeedstat_scan(self, timeout_per_point=30):
        """Perform a SeeedStat scan (command 'P') and return impedance results.
        
        The device runs two EIS scans (RCAL then unknown) and calculates
        Nyquist points using its internal formula. Results are printed as
        ASCII lines "real,imag," one per frequency point. This method
        collects those lines, parses them, and pairs each with the
        corresponding frequency (generated from current device settings).
        
        Args:
            timeout_per_point: Maximum seconds to wait for each point's data.
                            Total timeout is roughly (2 * n_points * timeout_per_point).
        
        Returns:
            List of (frequency, Z_real, Z_imag) tuples.
        """
        # Ensure ASCII mode for readable output
        self.set_seeedstat_mode(True)
        time.sleep(0.1)
        
        # Get number of frequency points and frequency range from device
        n_points = self._get_param_by_cmd('y')
        if n_points is None:
            raise RuntimeError("Number of frequencies not set")
        print(f"Number of frequency points: {n_points}")
        
        freqlo_mhz = self._get_param_by_cmd('w')
        freqhi_mhz = self._get_param_by_cmd('x')
        if None in (freqlo_mhz, freqhi_mhz):
            raise RuntimeError("Frequency range not set")
        
        # Compute frequencies locally (same as in measure_impedance)
        flo_hz = freqlo_mhz / 1000.0
        fhi_hz = freqhi_mhz / 1000.0
        if n_points == 1:
            freqs = [flo_hz]
        else:
            log_start = math.log10(flo_hz)
            log_end = math.log10(fhi_hz)
            log_step = (log_end - log_start) / (n_points - 1)
            freqs = [10 ** (log_start + i * log_step) for i in range(n_points)]
        
        # Start the scan
        self.ser.reset_input_buffer()
        self.write('P')
        
        # Read lines until we have all points or timeout
        data_lines = []
        total_timeout = timeout_per_point * n_points * 2 + 10  # generous
        start_time = time.time()
        
        while len(data_lines) < n_points and (time.time() - start_time) < total_timeout:
            if self.ser.in_waiting:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    if self.debug:
                        print(f"RX line: {line}")
                    data_lines.append(line)
            else:
                time.sleep(0.1)
        
        if len(data_lines) < n_points:
            raise RuntimeError(f"Timeout: expected {n_points} lines, got {len(data_lines)}")
        
        # Parse each line as "real,imag,"
        impedances = []
        for i, line in enumerate(data_lines):
            parts = line.strip(',').split(',')
            if len(parts) >= 2:
                try:
                    z_real = float(parts[0])
                    z_imag = float(parts[1])
                    impedances.append((freqs[i], z_real, -z_imag))
                except ValueError:
                    raise RuntimeError(f"Failed to parse line: {line}")
            else:
                raise RuntimeError(f"Unexpected format: {line}")
        
        return impedances
    
    
    # ------------------------------------------------------------------------
    # CV measurement
    # ------------------------------------------------------------------------
    
    def run_cv_scan(self, start_mv, stop_mv, step_mv, rate_mvs, cycles, timeout_per_step=5):
        """Perform a cyclic voltammetry scan (commands 'D' and 'M').

        The device generates a staircase waveform and measures current at each step.
        Results are streamed as ASCII lines: "voltage_mV,current_uA," one per step.

        After the scan, the device may remain in measurement mode. Call reset()
        to return to idle (green LED).

        Args:
            start_mv: Initial potential in mV.
            stop_mv:  Final potential in mV (peak of the ramp).
            step_mv:  Potential increment per step in mV.
            rate_mvs: Scan rate in mV/s.
            cycles:   Number of cycles (1 = one forward sweep).
            timeout_per_step: Maximum seconds to wait between data lines.
                            Total timeout is estimated based on scan parameters.

        Returns:
            List of (voltage_mV, current_uA) tuples.
        """
        # Ensure ASCII mode for readable output
        self.set_seeedstat_mode(True)
        time.sleep(0.1)
        
        # Build and send the D command with 5 floats
        d_cmd = f"D {start_mv},{stop_mv},{step_mv},{rate_mvs},{cycles}"
        self.write(d_cmd)
        time.sleep(0.2)  # allow device to process
        
        # Estimate total steps for timeout calculation
        # Total voltage range = |stop - start|
        # Steps per direction = |stop - start| / step
        # Total steps = steps_per_direction * 2 * cycles (forward + reverse each cycle)
        steps_per_direction = int(abs(stop_mv - start_mv) / step_mv) + 1
        estimated_steps = steps_per_direction * 2 * cycles
        total_timeout = estimated_steps * timeout_per_step + 30  
        
        if self.debug:
            print(f"Estimated steps: {estimated_steps}, timeout: {total_timeout}s")
        
        # Clear buffer and start scan
        self.ser.reset_input_buffer()
        self.write('M')
        
        # Collect data lines as they arrive
        results = []
        start_time = time.time()
        line_buffer = b''
        
        while (time.time() - start_time) < total_timeout:
            # Read whatever is available
            if self.ser.in_waiting:
                line_buffer += self.ser.read(self.ser.in_waiting)
                
                # Process complete lines (ended with newline)
                while b'\n' in line_buffer:
                    line, line_buffer = line_buffer.split(b'\n', 1)
                    line_str = line.decode(errors='ignore').strip()
                    
                    if line_str:
                        if self.debug:
                            print(f"RX: {line_str}")
                        
                        # Parse "voltage,current,"
                        parts = line_str.strip(',').split(',')
                        if len(parts) >= 2:
                            try:
                                voltage = float(parts[0])
                                current = float(parts[1])
                                results.append((voltage, current))
                                
                                # Reset timeout on successful data
                                start_time = time.time()
                            except ValueError:
                                if self.debug:
                                    print(f"Skipping unparseable: {line_str}")
            else:
                # No data available, short sleep
                time.sleep(0.05)
                
                # If we haven't received data for a while and have some results, assume scan is done
                if len(results) > 0 and (time.time() - start_time) > timeout_per_step * 2:
                    if self.debug:
                        print(f"No data for {timeout_per_step*2}s, scan likely complete")
                    break
        
        self.reset()
        if self.debug:
            print(f"CV scan completed, collected {len(results)} points")
        
        return results

    # ------------------------------------------------------------------------
    # OCP measurement methods
    # ------------------------------------------------------------------------

    def init_ocp(self, we_mv=None):
        """Initialize the device for OCP measurement (command 'I').

        This configures the ADC for measuring the potential between RE and WE.
        If a voltage is provided, it also sets the DAC to that value (calibration mode).

        Args:
            we_mv: If given, sets the working electrode voltage (mV) for calibration.
                Sends a single-float 'M' command.
        """
        if we_mv is not None:
            self.write(f'M{we_mv:.0f}')
            time.sleep(0.1)
        self.write('I')
        time.sleep(0.1)


    def measure_ocp_single(self, npts=None, timeout=10):
        """Perform a single OCP measurement and return the calculated OCP value.

        The measurement uses OCP_npts samples (averaged internally). After
        completion, the device sends a 'Z' character, then the result is
        retrieved via the 'T' command.

        Args:
            npts: Number of points to average (sets OCP_npts). If None, uses current setting.
            timeout: Maximum seconds to wait for the 'Z' completion signal.

        Returns:
            Float OCP value in mV.
        """
        if npts is not None:
            self.set_ocp_npts(npts)
            time.sleep(0.1)

        self.ser.reset_input_buffer()
        self.write('O')

        # Wait for 'Z' character
        start = time.time()
        while time.time() - start < timeout:
            if self.ser.in_waiting:
                ch = self.ser.read(1)
                if ch == b'Z':
                    break
            time.sleep(0.05)
        else:
            raise RuntimeError("Timeout waiting for OCP completion")

        # Retrieve result via 'T'
        result_str = self.query('T')
        if result_str is None:
            raise RuntimeError("No response from 'T' command")
        try:
            ocp = float(result_str.strip())
        except ValueError:
            raise RuntimeError(f"Failed to parse OCP result: {result_str}")
        return ocp


    def measure_ocp_raw_sum(self, npts=None, timeout=10):
        """Perform a single OCP measurement and return the raw ADC sum.

        The measurement uses OCP_npts samples (summed internally). After
        completion, the device sends a 'Z' character, then the raw sum is
        retrieved via the 'U' command (4-byte little-endian).

        Args:
            npts: Number of points to average (sets OCP_npts). If None, uses current setting.
            timeout: Maximum seconds to wait for the 'Z' completion signal.

        Returns:
            Integer raw sum of ADC samples (uint32).
        """
        if npts is not None:
            self.set_ocp_npts(npts)
            time.sleep(0.1)

        self.ser.reset_input_buffer()
        self.write('O')

        # Wait for 'Z'
        start = time.time()
        while time.time() - start < timeout:
            if self.ser.in_waiting:
                ch = self.ser.read(1)
                if ch == b'Z':
                    break
            time.sleep(0.05)
        else:
            raise RuntimeError("Timeout waiting for OCP completion")

        # Retrieve raw sum via 'U'
        self.ser.reset_input_buffer()
        self.write('U')
        time.sleep(0.1)  # small delay for device to respond
        data = b''
        start = time.time()
        while len(data) < 4 and time.time() - start < 2:
            if self.ser.in_waiting:
                data += self.ser.read(self.ser.in_waiting)
            else:
                time.sleep(0.05)
        if len(data) < 4:
            raise RuntimeError("Failed to read raw OCP sum")
        return struct.unpack('<I', data)[0]
        

    def measure_ocp_cycling(self, we_from, we_to, we_step, timeout=30):
        """Perform a cycling OCP measurement and return structured data.
        
        The device steps the working electrode voltage from we_from to we_to
        in increments of we_step (all in mV). At each step, it measures the
        OCP using the internal ADC and calibration constants, returning three
        values: WE voltage (as set), OCP measured by AD5940, and OCP measured
        by RP2040 ADC.
        
        Args:
            we_from: Start voltage in mV (integer or float).
            we_to:   Stop voltage in mV (integer or float).
            we_step: Step size in mV (positive integer or float).
            timeout: Maximum seconds to wait for data.
        
        Returns:
            List of tuples (we_mV, ocp_ad5940_mV, ocp_rp2040_mV) as floats.
        """
        # Save current mode and force ASCII
        original_mode = self._ascii_mode
        self.set_seeedstat_mode(True)
        time.sleep(0.1)
        
        # Initialize OCP system (send 'I')
        self.init_ocp()
        time.sleep(0.2)
        
        # Configure cycling
        cmd = f"M{we_from},{we_to},{we_step}"
        self.ser.reset_input_buffer()
        self.write(cmd)
        time.sleep(0.2)
        
        # Start measurement
        self.write('O')
        
        # Collect lines until timeout or 'Z' received
        results = []
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.ser.in_waiting:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    if self.debug:
                        print(f"RX: {line}")
                    # Stop if device signals end of measurement
                    if line == 'Z':
                        break
                    # Only keep lines that contain exactly two commas (three values)
                    if line.count(',') == 2:
                        try:
                            parts = line.split(',')
                            we = float(parts[0])
                            ocp1 = float(parts[1])
                            ocp2 = float(parts[2])
                            results.append((we, ocp1, ocp2))
                        except ValueError:
                            if self.debug:
                                print(f"Skipping unparseable line: {line}")
            else:
                time.sleep(0.1)
        
        # Restore original mode and reset
        self.set_seeedstat_mode(original_mode)
        self.reset()
        
        return results

    # ------------------------------------------------------------------------
    # Conversions
    # ------------------------------------------------------------------------

    @staticmethod
    def bias_code_to_mv(code):
        """Convert 12-bit bias code to millivolts.
        
        Implements the inverse of the Arduino ConvertUint16BiasToFloat.
        """
        return (code - 1664.0) * 100.0 / (1850.0 - 1664.0)

    
    @staticmethod
    def bias_mv_to_code(mv):
        """Convert millivolts to 12-bit bias code.
        
        Implements the Arduino ConvertFloatBiasToUint16.
        """
        return 1664 + int(-mv * (1850.0 - 1664.0) / 100.0 + 0.5)

    
    @staticmethod
    def amplitude_code_to_mv(code):
        """Convert 11-bit amplitude code to millivolts (zero-to-peak)."""
        return code * 50.0 / 126.0

    
    @staticmethod
    def amplitude_mv_to_code(mv):
        """Convert millivolts to 11-bit amplitude code."""
        return int(mv * 126.0 / 50.0 + 0.5)

    
    @staticmethod
    def offset_code_to_mv(code):
        """Convert 12-bit two’s-complement offset code to millivolts."""
        if code & (1 << 11):
            val = code - (1 << 12)
        else:
            val = code
        return float(val) + 56.0

    
    @staticmethod
    def offset_mv_to_code(mv):
        """Convert millivolts to 12-bit two’s-complement offset code."""
        return (int(mv) - 56) & 0xFFF
