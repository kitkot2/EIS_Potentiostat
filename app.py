import sys
import json
from pathlib import Path
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QGroupBox, QGridLayout,
    QMessageBox, QStatusBar, QTabWidget
)
from PySide6.QtCore import Qt
from ad5941 import AD5941
from Widgets.w_EIS import EISWidget
from Widgets.w_CV import CVWidget
from Widgets.w_OCP import OCPWidget


class MainWindow(QMainWindow):
    """Main window for EIS and CV measurements."""

    def __init__(self):
        super().__init__()
        self.ad5941 = None
        self.port = "/dev/cu.usbmodem1101"
        self.eis_widget = None
        self.cv_widget = None
        self.ocp_widget = None

        # Load saved settings (including port)
        self.load_settings()

        self._create_menu()
        self._create_connection_panel()
        self._create_central_widget()
        self._create_status_bar()
        self._connect_signals()

        self.setWindowTitle("HunStat2 Electrochemical Measurement GUI")
        self.resize(1400, 800)

    def _get_settings_path(self):
        """Return path to settings file."""
        home = Path.home()
        return home / ".hunstat2_gui_settings.json"

    def load_settings(self):
        """Load saved settings from file."""
        settings_path = self._get_settings_path()
        if not settings_path.exists():
            return
        try:
            with open(settings_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            # Load port
            self.port = data.get('port', self.port)
        except (json.JSONDecodeError, IOError, OSError) as e:
            print(f"Failed to load settings: {e}")

    def save_settings(self):
        """Save current settings to file."""
        settings_path = self._get_settings_path()
        # Load existing data to preserve other settings (like EIS parameters)
        data = {}
        if settings_path.exists():
            try:
                with open(settings_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
            except (json.JSONDecodeError, IOError, OSError):
                pass
        # Update port
        data['port'] = self.port_edit.text() if hasattr(self, 'port_edit') else self.port
        try:
            with open(settings_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
        except (IOError, OSError) as e:
            print(f"Failed to save settings: {e}")

    def _create_menu(self):
        """Create menu bar."""
        menubar = self.menuBar()
        file_menu = menubar.addMenu("File")
        exit_action = file_menu.addAction("Exit")
        exit_action.triggered.connect(self.close)

        help_menu = menubar.addMenu("Help")
        about_action = help_menu.addAction("About")
        about_action.triggered.connect(self._show_about)

    def _create_connection_panel(self):
        """Panel for serial connection settings."""
        connection_group = QGroupBox("Device Connection")
        layout = QHBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)

        layout.addWidget(QLabel("Port:"))
        self.port_edit = QLineEdit(self.port)
        self.port_edit.setMaximumWidth(200)
        layout.addWidget(self.port_edit)

        self.connect_button = QPushButton("Connect")
        self.connect_button.setStyleSheet("background-color: lightgreen;")
        layout.addWidget(self.connect_button)
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.setEnabled(False)
        layout.addWidget(self.disconnect_button)

        # Status indicator (circle)
        self.status_indicator = QLabel()
        self.status_indicator.setFixedSize(20, 20)
        self._update_status_indicator(False)
        layout.addWidget(self.status_indicator)

        # Optional status text (compact)
        self.status_text = QLabel("Not connected")
        self.status_text.setStyleSheet("color: gray;")
        layout.addWidget(self.status_text)

        layout.addStretch()

        connection_group.setLayout(layout)
        self.connection_group = connection_group
        # Set fixed height to make panel narrower
        self.connection_group.setMaximumHeight(80)

    def _update_status_indicator(self, connected):
        """Update the status indicator circle."""
        color = "green" if connected else "red"
        self.status_indicator.setStyleSheet(
            f"background-color: {color};"
            "border-radius: 10px;"
            "border: 1px solid black;"
        )
        self.status_indicator.setToolTip("Connected" if connected else "Not connected")

    def _create_central_widget(self):
        """Central widget containing connection panel and tabbed measurement widgets."""
        central = QWidget()
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)

        main_layout.addWidget(self.connection_group)

        # Create tab widget
        self.tab_widget = QTabWidget()
        self.eis_widget = EISWidget()
        self.cv_widget = CVWidget()
        self.ocp_widget = OCPWidget()
        self.tab_widget.addTab(self.eis_widget, "EIS")
        self.tab_widget.addTab(self.cv_widget, "CV")
        self.tab_widget.addTab(self.ocp_widget, "OCP")

        main_layout.addWidget(self.tab_widget, 1)

        central.setLayout(main_layout)
        self.setCentralWidget(central)

    def _create_status_bar(self):
        """Status bar at bottom."""
        self.status_bar = QStatusBar()
        self.status_bar.showMessage("Ready")
        self.setStatusBar(self.status_bar)

    def _connect_signals(self):
        """Connect signals to slots."""
        self.connect_button.clicked.connect(self._connect_device)
        self.disconnect_button.clicked.connect(self._disconnect_device)

    def _connect_device(self):
        """Connect to AD5941 device."""
        self.port = self.port_edit.text()
        # Fixed baudrate and debug
        baudrate = 1000000
        debug = False

        try:
            self.ad5941 = AD5941(port=self.port, baudrate=baudrate, debug=debug)
            self.ad5941.connect()
            self.ad5941.reset()
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
            self._update_status_indicator(True)
            self.status_text.setText(f"Connected to {self.port}")
            self.status_text.setStyleSheet("color: green;")
            self.status_bar.showMessage("Device connected successfully.")
            # Pass AD5941 instance to all widgets
            self.eis_widget.set_ad5941(self.ad5941)
            self.cv_widget.set_ad5941(self.ad5941)
            self.ocp_widget.set_ad5941(self.ad5941)
        except Exception as e:
            QMessageBox.critical(self, "Connection Error", f"Failed to connect:\n{e}")
            self.status_bar.showMessage(f"Connection failed: {e}")

    def _disconnect_device(self):
        """Disconnect from device."""
        if self.ad5941:
            try:
                self.ad5941.close()
            except:
                pass
            self.ad5941 = None
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self._update_status_indicator(False)
        self.status_text.setText("Not connected")
        self.status_text.setStyleSheet("color: gray;")
        self.status_bar.showMessage("Device disconnected.")
        self.eis_widget.set_ad5941(None)
        self.cv_widget.set_ad5941(None)
        self.ocp_widget.set_ad5941(None)

    def _show_about(self):
        """Show about dialog."""
        about_text = """HunStat2 Electrochemical Measurement GUI
Version 1.0
Developed for impedance and cyclic voltammetry measurements using AD5941.
"""
        QMessageBox.information(self, "About", about_text)

    def closeEvent(self, event):
        """Handle window close event."""
        if self.ad5941:
            self._disconnect_device()
        if self.eis_widget:
            self.eis_widget.save_settings()
        if self.cv_widget:
            self.cv_widget.save_settings()
        if self.ocp_widget:
            self.ocp_widget.save_settings()
        self.save_settings()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()