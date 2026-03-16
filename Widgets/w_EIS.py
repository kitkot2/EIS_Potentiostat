import json
import numpy as np
import queue
import threading
from pathlib import Path
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QComboBox,
    QPushButton, QCheckBox, QGroupBox, QGridLayout, QMessageBox, QFileDialog,
    QSizePolicy
)
from PySide6.QtCore import Qt, Signal, Slot, QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

# Constants
INPUT_WIDTH = 100
QUEUE_POLL_INTERVAL_MS = 100

# TIA Rf values (index 0-7)
TIA_RF_VALUES = ["200 Ω", "1 kΩ", "5 kΩ", "10 kΩ", "20 kΩ", "40 kΩ", "80 kΩ", "160 kΩ"]
# PGA Gain values (index 0-4)
PGA_GAIN_VALUES = ["1x", "1.5x", "2x", "4x", "9x"]


class EISWidget(QWidget):
    """Widget for EIS measurements with parameter controls and visualization."""

    measurement_finished = Signal(list, list, list)  # frequencies, Z_real, Z_imag
    error_occurred = Signal(str)

    def __init__(self, ad5941_instance=None, parent=None):
        super().__init__(parent)
        self.ad5941 = ad5941_instance  # instance of AD5941 class
        self.is_measuring = False
        self.data_queue = queue.Queue()

        # Default parameters
        self.freq_low = 0.2      # Hz
        self.freq_high = 200000  # Hz
        self.num_points = 20
        self.tia_rf_index = 1    # 0-7
        self.pga_gain_index = 1  # 0-4
        self.amplitude_mv = 20.0
        self.bias_mv = 0.0      # renamed from vbias_mv
        self.offset_mv = 0.0     # new parameter
        self.rcal_ohms = 10000.0
        self.use_variable_gain = False

        # Plot data storage
        self.frequencies = []
        self.Z_real = []
        self.Z_imag = []
        self.plot_type = "Nyquist"  # "Nyquist" or "Bode"

        # Load saved settings
        self.load_settings()

        self._create_widgets()
        self._layout_widgets()
        self._connect_signals()

    def _get_settings_path(self):
        """Return path to settings file."""
        home = Path.home()
        return home / ".hunstat2_eis_settings.json"

    def load_settings(self):
        """Load saved settings from file."""
        settings_path = self._get_settings_path()
        if not settings_path.exists():
            return
        try:
            with open(settings_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            # Update default parameters
            self.freq_low = data.get('freq_low', self.freq_low)
            self.freq_high = data.get('freq_high', self.freq_high)
            self.num_points = data.get('num_points', self.num_points)
            self.tia_rf_index = data.get('tia_rf_index', self.tia_rf_index)
            self.pga_gain_index = data.get('pga_gain_index', self.pga_gain_index)
            self.amplitude_mv = data.get('amplitude_mv', self.amplitude_mv)
            self.bias_mv = data.get('bias_mv', self.bias_mv)
            self.offset_mv = data.get('offset_mv', self.offset_mv)
            self.rcal_ohms = data.get('rcal_ohms', self.rcal_ohms)
            self.use_variable_gain = data.get('use_variable_gain', self.use_variable_gain)
            self.plot_type = data.get('plot_type', self.plot_type)
        except (json.JSONDecodeError, IOError, OSError) as e:
            print(f"Failed to load settings: {e}")

    def save_settings(self):
        """Save current settings to file."""
        # Ensure internal parameters reflect current UI
        self._sync_parameters_from_ui()
        settings_path = self._get_settings_path()
        data = {
            'freq_low': self.freq_low,
            'freq_high': self.freq_high,
            'num_points': self.num_points,
            'tia_rf_index': self.tia_rf_index,
            'pga_gain_index': self.pga_gain_index,
            'amplitude_mv': self.amplitude_mv,
            'bias_mv': self.bias_mv,
            'offset_mv': self.offset_mv,
            'rcal_ohms': self.rcal_ohms,
            'use_variable_gain': self.use_variable_gain,
            'plot_type': self.plot_type,
        }
        try:
            with open(settings_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
        except (IOError, OSError) as e:
            print(f"Failed to save settings: {e}")

    def _create_widgets(self):
        """Create all sub-widgets."""
        # Left panel: parameter controls
        self.param_group = QGroupBox("Measurement Parameters")
        param_layout = QVBoxLayout()
        param_layout.setContentsMargins(5, 5, 5, 5)
        self._create_parameter_controls(param_layout)
        self.param_group.setLayout(param_layout)
        self.param_group.setMinimumWidth(300)

        # Scan control panel (buttons)
        self.scan_control_group = QGroupBox("Scan Control")
        scan_layout = QVBoxLayout()
        scan_layout.setContentsMargins(5, 5, 5, 5)
        self._create_scan_controls(scan_layout)
        self.scan_control_group.setLayout(scan_layout)
        self.scan_control_group.setMinimumWidth(300)

        # Right panel: plots
        self.plot_group = QGroupBox("Visualization")
        plot_layout = QVBoxLayout()
        plot_layout.setContentsMargins(5, 5, 5, 5)
        self._create_plot_area(plot_layout)
        self.plot_group.setLayout(plot_layout)

    def _create_parameter_controls(self, parent_layout):
        """Create input fields and dropdowns and add them to parent_layout."""
        grid = QGridLayout()

        # Frequency range - two rows
        grid.addWidget(QLabel("Frequency Low (Hz):"), 0, 0)
        self.freq_low_edit = QLineEdit(str(self.freq_low))
        self.freq_low_edit.setMaximumWidth(INPUT_WIDTH)
        grid.addWidget(self.freq_low_edit, 0, 1)
        grid.addWidget(QLabel("Frequency High (Hz):"), 1, 0)
        self.freq_high_edit = QLineEdit(str(self.freq_high))
        self.freq_high_edit.setMaximumWidth(INPUT_WIDTH)
        grid.addWidget(self.freq_high_edit, 1, 1)

        # Number of points
        grid.addWidget(QLabel("Number of points:"), 2, 0)
        self.num_points_edit = QLineEdit(str(self.num_points))
        self.num_points_edit.setMaximumWidth(INPUT_WIDTH)
        grid.addWidget(self.num_points_edit, 2, 1)

        # TIA Rf selection
        grid.addWidget(QLabel("TIA Rf:"), 3, 0)
        self.tia_rf_combo = QComboBox()
        self.tia_rf_combo.addItems(TIA_RF_VALUES)
        self.tia_rf_combo.setCurrentIndex(self.tia_rf_index)
        self.tia_rf_combo.setMaximumWidth(INPUT_WIDTH)
        grid.addWidget(self.tia_rf_combo, 3, 1)

        # PGA Gain selection
        grid.addWidget(QLabel("PGA Gain:"), 4, 0)
        self.pga_gain_combo = QComboBox()
        self.pga_gain_combo.addItems(PGA_GAIN_VALUES)
        self.pga_gain_combo.setCurrentIndex(self.pga_gain_index)
        self.pga_gain_combo.setMaximumWidth(INPUT_WIDTH)
        grid.addWidget(self.pga_gain_combo, 4, 1)

        # Amplitude
        grid.addWidget(QLabel("Amplitude (mV):"), 5, 0)
        self.amplitude_edit = QLineEdit(str(self.amplitude_mv))
        self.amplitude_edit.setMaximumWidth(INPUT_WIDTH)
        grid.addWidget(self.amplitude_edit, 5, 1)

        # Bias V (renamed from Vbias)
        grid.addWidget(QLabel("Bias V (mV):"), 6, 0)
        self.bias_edit = QLineEdit(str(self.bias_mv))
        self.bias_edit.setMaximumWidth(INPUT_WIDTH)
        grid.addWidget(self.bias_edit, 6, 1)

        # Offset V (new)
        grid.addWidget(QLabel("Offset V (mV):"), 7, 0)
        self.offset_edit = QLineEdit(str(self.offset_mv))
        self.offset_edit.setMaximumWidth(INPUT_WIDTH)
        grid.addWidget(self.offset_edit, 7, 1)

        # RCAL
        grid.addWidget(QLabel("RCAL (Ω):"), 8, 0)
        self.rcal_edit = QLineEdit(str(self.rcal_ohms))
        self.rcal_edit.setMaximumWidth(INPUT_WIDTH)
        grid.addWidget(self.rcal_edit, 8, 1)

        # Checkbox for variable gain
        self.variable_gain_check = QCheckBox("Use variable gain")
        self.variable_gain_check.setChecked(self.use_variable_gain)
        grid.addWidget(self.variable_gain_check, 9, 0, 1, 2)

        # Plot type selector
        grid.addWidget(QLabel("Plot type:"), 10, 0)
        self.plot_type_combo = QComboBox()
        self.plot_type_combo.addItems(["Nyquist", "Bode (Magnitude & Phase)"])
        # Map stored plot_type to combo index
        if self.plot_type == "Nyquist":
            self.plot_type_combo.setCurrentIndex(0)
        else:
            self.plot_type_combo.setCurrentIndex(1)
        self.plot_type_combo.setMaximumWidth(INPUT_WIDTH)
        grid.addWidget(self.plot_type_combo, 10, 1)

        # Info label
        self.param_info = QLabel("Set parameters and press 'Run EIS Scan'")
        self.param_info.setStyleSheet("color: gray;")
        grid.addWidget(self.param_info, 11, 0, 1, 2)

        parent_layout.addLayout(grid)

    def _create_plot_area(self, parent_layout):
        """Create matplotlib figure and canvas and add them to parent_layout."""
        layout = QVBoxLayout()
        self.figure = Figure(figsize=(10, 6))
        # Single axes for either Nyquist or Bode
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        parent_layout.addLayout(layout)

    def _create_scan_controls(self, parent_layout):
        """Create scan control buttons and add them to parent_layout."""
        layout = QHBoxLayout()
        self.run_button = QPushButton("Run EIS")
        self.run_button.setStyleSheet("""
            QPushButton {
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                                  stop:0 #5dade2, stop:1 #3498db);
                border: 1px solid #2980b9;
                border-radius: 6px;
                color: white;
                font-weight: bold;
                padding: 6px 12px;
            }
            QPushButton:hover {
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                                  stop:0 #3498db, stop:1 #2980b9);
                border: 1px solid #1c5a7a;
            }
            QPushButton:pressed {
                background-color: #1c5a7a;
                border: 1px solid #154360;
            }
            QPushButton:disabled {
                background-color: #bdc3c7;
                border: 1px solid #95a5a6;
                color: #7f8c8d;
            }
        """)
        self.clear_button = QPushButton("Clear Plot")
        self.export_button = QPushButton("Export Data")

        layout.addWidget(self.run_button)
        layout.addWidget(self.clear_button)
        layout.addWidget(self.export_button)
        layout.addStretch()

        parent_layout.addLayout(layout)

    def _layout_widgets(self):
        """Arrange widgets in main layout."""
        main_layout = QVBoxLayout()
        content_layout = QHBoxLayout()
        content_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        content_layout.setSpacing(10)

        # Left column: parameters and scan controls stacked vertically
        left_column = QVBoxLayout()
        left_column.setAlignment(Qt.AlignmentFlag.AlignTop)
        left_column.setSpacing(5)
        left_column.addWidget(self.param_group)
        left_column.addWidget(self.scan_control_group)
        # No stretch, left column will take minimal vertical space

        # Right column: plot group, should expand vertically
        self.plot_group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Add left and right columns to horizontal layout
        content_layout.addLayout(left_column, 1)   # stretch factor 1
        content_layout.addWidget(self.plot_group, 3)  # stretch factor 3

        main_layout.addLayout(content_layout)
        self.setLayout(main_layout)

    def _connect_signals(self):
        """Connect signals to slots."""
        self.run_button.clicked.connect(self._run_eis_scan_thread)
        self.clear_button.clicked.connect(self._clear_plot)
        self.export_button.clicked.connect(self._export_data)
        self.variable_gain_check.toggled.connect(self._on_variable_gain_toggled)
        self.plot_type_combo.currentTextChanged.connect(self._on_plot_type_changed)
        self.measurement_finished.connect(self._on_measurement_finished)
        self.error_occurred.connect(self._on_error)

        # Timer for checking queue
        self.timer = QTimer()
        self.timer.timeout.connect(self._check_queue)
        self.timer.start(QUEUE_POLL_INTERVAL_MS)  # every 100 ms

        # Initial state of combos
        self._update_combo_states()

    def _update_combo_states(self):
        """Enable/disable TIA Rf and PGA Gain combos based on variable gain checkbox."""
        enabled = not self.variable_gain_check.isChecked()
        self.tia_rf_combo.setEnabled(enabled)
        self.pga_gain_combo.setEnabled(enabled)

    @Slot(bool)
    def _on_variable_gain_toggled(self, checked):
        self._update_combo_states()

    @Slot(str)
    def _on_plot_type_changed(self, text):
        self.plot_type = "Nyquist" if text.startswith("Nyquist") else "Bode"
        self._update_plot()

    def _update_parameters(self):
        """Update internal parameters from UI with validation."""
        try:
            self.freq_low = float(self.freq_low_edit.text())
            self.freq_high = float(self.freq_high_edit.text())
            self.num_points = int(self.num_points_edit.text())
            self.tia_rf_index = self.tia_rf_combo.currentIndex()
            self.pga_gain_index = self.pga_gain_combo.currentIndex()
            self.amplitude_mv = float(self.amplitude_edit.text())
            self.bias_mv = float(self.bias_edit.text())
            self.offset_mv = float(self.offset_edit.text())
            self.rcal_ohms = float(self.rcal_edit.text())
            self.use_variable_gain = self.variable_gain_check.isChecked()
            return True
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid numeric values.")
            return False

    def _sync_parameters_from_ui(self):
        """Sync internal parameters from UI without validation (for saving)."""
        try:
            self.freq_low = float(self.freq_low_edit.text())
        except ValueError:
            pass
        try:
            self.freq_high = float(self.freq_high_edit.text())
        except ValueError:
            pass
        try:
            self.num_points = int(self.num_points_edit.text())
        except ValueError:
            pass
        self.tia_rf_index = self.tia_rf_combo.currentIndex()
        self.pga_gain_index = self.pga_gain_combo.currentIndex()
        try:
            self.amplitude_mv = float(self.amplitude_edit.text())
        except ValueError:
            pass
        try:
            self.bias_mv = float(self.bias_edit.text())
        except ValueError:
            pass
        try:
            self.offset_mv = float(self.offset_edit.text())
        except ValueError:
            pass
        try:
            self.rcal_ohms = float(self.rcal_edit.text())
        except ValueError:
            pass
        self.use_variable_gain = self.variable_gain_check.isChecked()
        # plot_type is updated via signal, but we can get from combo
        if self.plot_type_combo.currentIndex() == 0:
            self.plot_type = "Nyquist"
        else:
            self.plot_type = "Bode"

    @Slot()
    def _run_eis_scan_thread(self):
        """Start EIS measurement in a separate thread."""
        if self.is_measuring:
            QMessageBox.warning(self, "Warning", "Measurement already in progress.")
            return
        if self.ad5941 is None:
            QMessageBox.critical(self, "Error", "AD5941 instance not provided.")
            return
        if not self._update_parameters():
            return
        self.is_measuring = True
        self.run_button.setEnabled(False)
        self.param_info.setText("Measuring...")
        self.param_info.setStyleSheet("color: orange;")

        thread = threading.Thread(target=self._run_eis_scan, daemon=True)
        thread.start()

    def _run_eis_scan(self):
        """Perform EIS measurement using AD5941."""
        try:
            # Configure device
            self.ad5941.set_freqlo_hz(self.freq_low)
            self.ad5941.set_freqhi_hz(self.freq_high)
            self.ad5941.set_nfreqs(self.num_points)
            self.ad5941.set_tia_rf(self.tia_rf_index)
            self.ad5941.set_pga_gain(self.pga_gain_index)
            self.ad5941.set_amplitude_mv(self.amplitude_mv)
            self.ad5941.set_vbias_mv(self.bias_mv)   # note: method name is set_vbias_mv
            self.ad5941.set_offset_mv(self.offset_mv)
            self.ad5941.set_rcal(self.rcal_ohms)
            self.ad5941.set_use_variable_gain(self.use_variable_gain)

            # Run measurement (always binary mode)
            impedances = self.ad5941.run_eis_measurement(rcal_value=self.rcal_ohms)

            # Store results
            freqs = [f for f, _, _ in impedances]
            Z_real = [zr for _, zr, _ in impedances]
            Z_imag = [zi for _, _, zi in impedances]
            self.data_queue.put(('data', (freqs, Z_real, Z_imag)))
        except (ConnectionError, RuntimeError, ValueError) as e:
            self.data_queue.put(('error', str(e)))
        except Exception as e:
            self.data_queue.put(('error', f"Unexpected error: {e}"))
        finally:
            self.data_queue.put(('done', None))

    @Slot()
    def _check_queue(self):
        """Check for results from measurement thread."""
        try:
            while True:
                msg_type, data = self.data_queue.get_nowait()
                if msg_type == 'data':
                    self.measurement_finished.emit(*data)
                elif msg_type == 'error':
                    self.error_occurred.emit(data)
                elif msg_type == 'done':
                    self.is_measuring = False
                    self.run_button.setEnabled(True)
        except queue.Empty:
            pass

    @Slot(list, list, list)
    def _on_measurement_finished(self, freqs, Z_real, Z_imag):
        """Handle measurement completion."""
        self.frequencies = freqs
        self.Z_real = Z_real
        self.Z_imag = Z_imag
        self._update_plot()
        self.param_info.setText("Measurement completed.")
        self.param_info.setStyleSheet("color: green;")

    @Slot(str)
    def _on_error(self, error_msg):
        """Handle measurement error."""
        self.param_info.setText(f"Error: {error_msg}")
        self.param_info.setStyleSheet("color: red;")
        QMessageBox.critical(self, "Measurement Error", error_msg)
        self.is_measuring = False
        self.run_button.setEnabled(True)


    def _clear_axes(self):
        """Clear the figure and create a fresh axes."""
        self.figure.clear()
        self.ax = self.figure.add_subplot(111)

    def _update_plot(self):
        """Update plot according to selected type."""
        # Clear the figure and create a fresh axes to remove any twin axes
        self._clear_axes()

        if not self.frequencies:
            self.canvas.draw()
            return

        if self.plot_type == "Nyquist":
            # Nyquist plot
            self.ax.plot(self.Z_real, [-zi for zi in self.Z_imag], 'o-', color='blue')
            self.ax.set_xlabel("Z' (Ω)")
            self.ax.set_ylabel("-Z'' (Ω)")
            self.ax.set_title("Nyquist Plot")
            self.ax.grid(True)
            self.ax.axis('equal')
            # Place legend in upper left where there are usually fewer points
            self.ax.legend(["Impedance"], loc='upper left')
        else:
            # Bode plot: magnitude on left y-axis, phase on right y-axis
            magnitude = np.sqrt(np.array(self.Z_real)**2 + np.array(self.Z_imag)**2)
            phase = np.arctan2(self.Z_imag, self.Z_real) * 180 / np.pi

            # Magnitude (log scale)
            color_mag = 'blue'
            color_phase = 'red'
            self.ax.set_xlabel("Frequency (Hz)")
            self.ax.set_ylabel("|Z| (Ω)", color=color_mag)
            self.ax.loglog(self.frequencies, magnitude, 'o-', color=color_mag, label='Magnitude')
            self.ax.tick_params(axis='y', labelcolor=color_mag)
            self.ax.grid(True, which='both', alpha=0.3)

            # Create second y-axis for phase
            ax2 = self.ax.twinx()
            ax2.set_ylabel("Phase (deg)", color=color_phase)
            ax2.semilogx(self.frequencies, phase, 's-', color=color_phase, label='Phase')
            ax2.tick_params(axis='y', labelcolor=color_phase)

            # Add legend in lower right to avoid overlapping with data
            lines1, labels1 = self.ax.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            # Combine legends and place in lower right
            self.ax.legend(lines1 + lines2, labels1 + labels2, loc='lower right')
            self.ax.set_title("Bode Plot (Magnitude & Phase)")

        self.figure.tight_layout()
        self.canvas.draw()

    @Slot()
    def _clear_plot(self):
        """Clear plot data."""
        self.frequencies.clear()
        self.Z_real.clear()
        self.Z_imag.clear()
        self._update_plot()
        self.param_info.setText("Plot cleared.")
        self.param_info.setStyleSheet("color: gray;")

    @Slot()
    def _export_data(self):
        """Export data to CSV file."""
        if not self.frequencies:
            QMessageBox.information(self, "No Data", "No measurement data to export.")
            return
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save CSV", "", "CSV files (*.csv);;All files (*.*)"
        )
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write("Frequency (Hz),Z_real (Ohm),Z_imag (Ohm)\n")
                    for fhz, zr, zi in zip(self.frequencies, self.Z_real, self.Z_imag):
                        f.write(f"{fhz},{zr},{zi}\n")
                QMessageBox.information(self, "Success", f"Data exported to {filename}")
            except (IOError, OSError) as e:
                QMessageBox.critical(self, "Export Error", str(e))

    def set_ad5941(self, ad5941_instance):
        """Update the AD5941 instance (for dynamic connection)."""
        self.ad5941 = ad5941_instance