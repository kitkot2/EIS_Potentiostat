import json
import queue
import threading
import time
from pathlib import Path
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QGroupBox, QGridLayout, QMessageBox, QFileDialog,
    QSizePolicy
)
from PySide6.QtCore import Qt, Signal, Slot, QTimer

# Constants
INPUT_WIDTH = 100
QUEUE_POLL_INTERVAL_MS = 100


class OCPWidget(QWidget):
    """Widget for Open Circuit Potential single measurement."""

    measurement_finished = Signal(float)  # OCP value in mV
    error_occurred = Signal(str)

    def __init__(self, ad5941_instance=None, parent=None):
        super().__init__(parent)
        self.ad5941 = ad5941_instance  # instance of AD5941 class
        self.is_measuring = False
        self.data_queue = queue.Queue()

        # Default parameters
        self.ocp_npts = 10           # number of points to average

        # Result storage
        self.last_ocp_mv = None      # last measured OCP value (AD5940)

        # Load saved settings
        self.load_settings()

        self._create_widgets()
        self._layout_widgets()
        self._connect_signals()

    def _get_settings_path(self):
        """Return path to settings file."""
        home = Path.home()
        return home / ".hunstat2_ocp_settings.json"

    def load_settings(self):
        """Load saved settings from file."""
        settings_path = self._get_settings_path()
        if not settings_path.exists():
            return
        try:
            with open(settings_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            # Update default parameters
            self.ocp_npts = data.get('ocp_npts', self.ocp_npts)
        except (json.JSONDecodeError, IOError, OSError) as e:
            print(f"Failed to load settings: {e}")

    def save_settings(self):
        """Save current settings to file."""
        # Ensure internal parameters reflect current UI
        self._sync_parameters_from_ui()
        settings_path = self._get_settings_path()
        data = {
            'ocp_npts': self.ocp_npts,
        }
        try:
            with open(settings_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
        except (IOError, OSError) as e:
            print(f"Failed to save settings: {e}")

    def _create_widgets(self):
        """Create all sub-widgets."""
        # Left panel: parameter controls
        self.param_group = QGroupBox("OCP Parameters")
        param_layout = QVBoxLayout()
        param_layout.setContentsMargins(5, 5, 5, 5)
        self._create_parameter_controls(param_layout)
        self.param_group.setLayout(param_layout)
        self.param_group.setMinimumWidth(300)
        # Ensure same width as EIS/CV widgets

        # Scan control panel (buttons)
        self.scan_control_group = QGroupBox("Scan Control")
        scan_layout = QVBoxLayout()
        scan_layout.setContentsMargins(5, 5, 5, 5)
        self._create_scan_controls(scan_layout)
        self.scan_control_group.setLayout(scan_layout)
        self.scan_control_group.setMinimumWidth(300)

        # Right panel: empty (placeholder)
        self.plot_group = QGroupBox("Visualization")
        plot_layout = QVBoxLayout()
        plot_layout.setContentsMargins(5, 5, 5, 5)
        self._create_plot_area(plot_layout)
        self.plot_group.setLayout(plot_layout)

    def _create_parameter_controls(self, parent_layout):
        """Create input fields and add them to parent_layout."""
        grid = QGridLayout()

        # OCP number of points (averaging)
        grid.addWidget(QLabel("OCP npts:"), 0, 0)
        self.ocp_npts_edit = QLineEdit(str(self.ocp_npts))
        self.ocp_npts_edit.setMaximumWidth(INPUT_WIDTH)
        grid.addWidget(self.ocp_npts_edit, 0, 1)

        # Info label for result
        self.result_label = QLabel("No measurement yet")
        self.result_label.setStyleSheet("color: gray; font-weight: bold;")
        grid.addWidget(QLabel("OCP mV:"), 1, 0)
        grid.addWidget(self.result_label, 1, 1)

        # Info label for status
        self.param_info = QLabel("Set parameters and press 'Run OCP Scan'")
        self.param_info.setStyleSheet("color: gray;")
        grid.addWidget(self.param_info, 2, 0, 1, 2)

        parent_layout.addLayout(grid)

    def _create_plot_area(self, parent_layout):
        """Create empty placeholder (right panel)."""
        layout = QVBoxLayout()
        self.placeholder_label = QLabel("No visualization for OCP")
        self.placeholder_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.placeholder_label.setStyleSheet("color: #888; font-style: italic;")
        layout.addWidget(self.placeholder_label)
        layout.addStretch()
        parent_layout.addLayout(layout)

    def _create_scan_controls(self, parent_layout):
        """Create scan control buttons and add them to parent_layout."""
        layout = QHBoxLayout()
        self.run_button = QPushButton("Run OCP Scan")
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
        self.clear_button = QPushButton("Clear Result")
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

        # Right column: plot group, should expand vertically (empty)
        self.plot_group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Add left and right columns to horizontal layout
        content_layout.addLayout(left_column, 1)   # stretch factor 1
        content_layout.addWidget(self.plot_group, 3)  # stretch factor 3

        main_layout.addLayout(content_layout)
        self.setLayout(main_layout)

    def _connect_signals(self):
        """Connect signals to slots."""
        self.run_button.clicked.connect(self._run_ocp_scan_thread)
        self.clear_button.clicked.connect(self._clear_result)
        self.export_button.clicked.connect(self._export_data)
        self.measurement_finished.connect(self._on_measurement_finished)
        self.error_occurred.connect(self._on_error)

        # Timer for checking queue
        self.timer = QTimer()
        self.timer.timeout.connect(self._check_queue)
        self.timer.start(QUEUE_POLL_INTERVAL_MS)  # every 100 ms

    def _update_parameters(self):
        """Update internal parameters from UI with validation."""
        try:
            self.ocp_npts = int(self.ocp_npts_edit.text())
            return True
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter a valid integer.")
            return False

    def _sync_parameters_from_ui(self):
        """Sync internal parameters from UI without validation (for saving)."""
        try:
            self.ocp_npts = int(self.ocp_npts_edit.text())
        except ValueError:
            pass

    @Slot()
    def _run_ocp_scan_thread(self):
        """Start OCP measurement in a separate thread."""
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

        thread = threading.Thread(target=self._run_ocp_scan, daemon=True)
        thread.start()

    def _run_ocp_scan(self):
        """Perform single OCP measurement using AD5941."""
        try:
            # Initialize OCP (command 'I')
            self.ad5941.init_ocp()
            time.sleep(0.1)

            # Set OCP npts (number of points to average)
            self.ad5941.set_ocp_npts(self.ocp_npts)
            time.sleep(0.1)

            # Run measurement (timeout uses default)
            ocp_value = self.ad5941.measure_ocp_single(npts=self.ocp_npts)
            # Reset device after measurement
            self.ad5941.reset()
            time.sleep(0.1)

            self.data_queue.put(('data', ocp_value))
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
                    self.measurement_finished.emit(data)
                elif msg_type == 'error':
                    self.error_occurred.emit(data)
                elif msg_type == 'done':
                    self.is_measuring = False
                    self.run_button.setEnabled(True)
        except queue.Empty:
            pass

    @Slot(float)
    def _on_measurement_finished(self, ocp_value):
        """Handle measurement completion."""
        self.last_ocp_mv = ocp_value
        self.result_label.setText(f"{ocp_value:.2f} mV")
        self.result_label.setStyleSheet("color: green; font-weight: bold;")
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

    @Slot()
    def _clear_result(self):
        """Clear stored result."""
        self.last_ocp_mv = None
        self.result_label.setText("No measurement yet")
        self.result_label.setStyleSheet("color: gray; font-weight: bold;")
        self.param_info.setText("Result cleared.")
        self.param_info.setStyleSheet("color: gray;")

    @Slot()
    def _export_data(self):
        """Export data to CSV file (single value)."""
        if self.last_ocp_mv is None:
            QMessageBox.information(self, "No Data", "No measurement data to export.")
            return
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save CSV", "", "CSV files (*.csv);;All files (*.*)"
        )
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write("OCP (mV)\n")
                    f.write(f"{self.last_ocp_mv}\n")
                QMessageBox.information(self, "Success", f"Data exported to {filename}")
            except (IOError, OSError) as e:
                QMessageBox.critical(self, "Export Error", str(e))

    def set_ad5941(self, ad5941_instance):
        """Update the AD5941 instance (for dynamic connection)."""
        self.ad5941 = ad5941_instance