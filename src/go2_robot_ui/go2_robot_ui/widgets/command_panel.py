"""
Command Panel Widget

This module provides a Qt widget for Go2 robot command interface including:
- Organized action buttons for robot commands
- Emergency stop functionality
- QR code action highlighting
- Command feedback and status
- Action categorization (Basic, Entertainment, Athletic, etc.)

The command panel emits signals for robot control actions and provides
visual feedback for command execution.
"""

from typing import Dict, List, Tuple
from python_qt_binding.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                                       QLabel, QPushButton, QGroupBox, 
                                       QGridLayout, QScrollArea, QMessageBox)
from python_qt_binding.QtCore import Qt, pyqtSignal, pyqtSlot
from python_qt_binding.QtGui import QFont


class CommandButton(QPushButton):
    """Custom button for robot actions with enhanced styling and feedback."""
    
    def __init__(self, action_name: str, description: str = "", category: str = ""):
        """
        Initialize command button.
        
        Args:
            action_name: Name of the robot action
            description: Description of the action
            category: Category for styling (basic, entertainment, athletic, emergency)
        """
        super().__init__(action_name)
        self.action_name = action_name
        self.description = description
        self.category = category
        
        self.setMinimumHeight(45)
        self.setMaximumHeight(60)
        
        # Set tooltip
        if description:
            self.setToolTip(f"{action_name}: {description}")
        else:
            self.setToolTip(action_name)
        
        self._apply_category_styling()
    
    def _apply_category_styling(self):
        """Apply styling based on button category."""
        base_style = (
            "QPushButton { "
            "font-size: 12px; "
            "font-weight: bold; "
            "border: 2px solid; "
            "border-radius: 8px; "
            "padding: 8px; "
            "margin: 2px; "
            "} "
            "QPushButton:hover { "
            "border-width: 3px; "
            "} "
            "QPushButton:pressed { "
            "padding: 6px; "
            "border-width: 1px; "
            "}"
        )
        
        if self.category == "emergency":
            style = base_style + (
                "QPushButton { "
                "background-color: #8b2635; "
                "color: white; "
                "border-color: #a53545; "
                "} "
                "QPushButton:hover { "
                "background-color: #a53545; "
                "}"
            )
        elif self.category == "basic":
            style = base_style + (
                "QPushButton { "
                "background-color: #2d5016; "
                "color: white; "
                "border-color: #4a7c19; "
                "} "
                "QPushButton:hover { "
                "background-color: #4a7c19; "
                "}"
            )
        elif self.category == "entertainment":
            style = base_style + (
                "QPushButton { "
                "background-color: #1e3a8a; "
                "color: white; "
                "border-color: #3b82f6; "
                "} "
                "QPushButton:hover { "
                "background-color: #3b82f6; "
                "}"
            )
        elif self.category == "athletic":
            style = base_style + (
                "QPushButton { "
                "background-color: #7c2d12; "
                "color: white; "
                "border-color: #ea580c; "
                "} "
                "QPushButton:hover { "
                "background-color: #ea580c; "
                "}"
            )
        else:  # default
            style = base_style + (
                "QPushButton { "
                "background-color: #374151; "
                "color: white; "
                "border-color: #6b7280; "
                "} "
                "QPushButton:hover { "
                "background-color: #6b7280; "
                "}"
            )
        
        self.setStyleSheet(style)
    
    def highlight_for_qr(self):
        """Highlight button when QR code is detected for this action."""
        highlight_style = (
            "QPushButton { "
            "background-color: #fbbf24; "
            "color: #000; "
            "border: 3px solid #f59e0b; "
            "font-size: 12px; "
            "font-weight: bold; "
            "border-radius: 8px; "
            "padding: 8px; "
            "margin: 2px; "
            "animation: pulse 1s infinite; "
            "}"
        )
        self.setStyleSheet(highlight_style)
        
        # Reset after 3 seconds
        from python_qt_binding.QtCore import QTimer
        QTimer.singleShot(3000, self._apply_category_styling)


class CommandPanel(QWidget):
    """
    Widget providing Go2 robot command interface.
    
    This widget provides a comprehensive control interface for robot operations,
    emitting signals for robot control actions and providing visual feedback
    for command execution.
    
    Signals:
        action_triggered: Emitted when user triggers an action (str: action_name)
        emergency_stop: Emitted when emergency stop is pressed
        qr_action_triggered: Emitted when QR-mapped action is triggered (str: action_name)
    """
    
    # Control signals
    action_triggered = pyqtSignal(str)
    emergency_stop = pyqtSignal()
    qr_action_triggered = pyqtSignal(str)
    
    # Robot action definitions with categories and descriptions
    ACTION_DEFINITIONS = {
        # Basic Actions
        "basic": [
            ("StandUp", "Stand up from sitting position"),
            ("Sit", "Sit down from standing position"),
            ("Hello", "Wave hello gesture"),
            ("Stretch", "Perform stretching routine"),
            ("BalanceStand", "Stand in balanced position"),
            ("RecoveryStand", "Recover to standing position"),
        ],
        
        # Entertainment Actions
        "entertainment": [
            ("Dance1", "Perform dance routine 1"),
            ("Dance2", "Perform dance routine 2"),
            ("WiggleHips", "Wiggle hips movement"),
            ("FingerHeart", "Make finger heart gesture"),
        ],
        
        # Athletic Actions
        "athletic": [
            ("FrontFlip", "Perform front flip"),
            ("FrontJump", "Jump forward"),
            ("FrontPounce", "Pounce forward"),
            ("Handstand", "Perform handstand"),
            ("MoonWalk", "Moon walk movement"),
            ("Bound", "Bounding movement"),
            ("CrossWalk", "Cross walking gait"),
        ],
        
        # Emergency Actions
        "emergency": [
            ("StopMove", "Emergency stop all movement"),
        ]
    }
    
    def __init__(self, parent=None):
        """
        Initialize command panel widget.
        
        Args:
            parent: Parent Qt widget (optional)
        """
        super().__init__(parent)
        
        # State tracking
        self.command_buttons = {}  # action_name -> button
        self.last_qr_action = ""
        
        self._setup_ui()
        self._setup_connections()
    
    def _setup_ui(self):
        """Set up the user interface layout."""
        # Main layout with scroll area
        main_layout = QVBoxLayout(self)
        
        # Title
        title_label = QLabel("Go2 Robot Commands")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet(
            "QLabel { "
            "font-size: 16px; "
            "font-weight: bold; "
            "color: #fff; "
            "padding: 10px; "
            "background-color: #2b2b2b; "
            "border: 1px solid #555; "
            "border-radius: 5px; "
            "}"
        )
        main_layout.addWidget(title_label)
        
        # Scroll area for commands
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # Commands widget
        commands_widget = QWidget()
        commands_layout = QVBoxLayout(commands_widget)
        
        # Create action groups
        for category, actions in self.ACTION_DEFINITIONS.items():
            group_box = self._create_action_group(category, actions)
            commands_layout.addWidget(group_box)
        
        # Add stretch to push everything to top
        commands_layout.addStretch()
        
        scroll_area.setWidget(commands_widget)
        main_layout.addWidget(scroll_area)
        
        # Set overall styling
        self.setStyleSheet(
            "CommandPanel { "
            "background-color: #1e1e1e; "
            "} "
            "QScrollArea { "
            "border: none; "
            "background-color: #1e1e1e; "
            "} "
            "QScrollBar:vertical { "
            "background-color: #2b2b2b; "
            "width: 12px; "
            "border: none; "
            "} "
            "QScrollBar::handle:vertical { "
            "background-color: #555; "
            "border-radius: 6px; "
            "} "
            "QScrollBar::handle:vertical:hover { "
            "background-color: #777; "
            "}"
        )
    
    def _create_action_group(self, category: str, actions: List[Tuple[str, str]]) -> QGroupBox:
        """
        Create a group box for a category of actions.
        
        Args:
            category: Category name
            actions: List of (action_name, description) tuples
            
        Returns:
            QGroupBox containing the action buttons
        """
        # Create group box
        group_box = QGroupBox(category.title())
        group_box.setStyleSheet(
            "QGroupBox { "
            "font-size: 14px; "
            "font-weight: bold; "
            "color: #fff; "
            "border: 2px solid #555; "
            "border-radius: 5px; "
            "margin-top: 10px; "
            "padding-top: 10px; "
            "} "
            "QGroupBox::title { "
            "subcontrol-origin: margin; "
            "left: 10px; "
            "padding: 0 5px 0 5px; "
            "}"
        )
        
        # Create grid layout for buttons
        layout = QGridLayout(group_box)
        
        # Add buttons in grid format (2 columns for most, 1 for emergency)
        cols = 1 if category == "emergency" else 2
        
        for i, (action_name, description) in enumerate(actions):
            row = i // cols
            col = i % cols
            
            button = CommandButton(action_name, description, category)
            button.clicked.connect(lambda checked, name=action_name: self._on_action_clicked(name))
            
            layout.addWidget(button, row, col)
            self.command_buttons[action_name] = button
        
        return group_box
    
    def _setup_connections(self):
        """Set up internal signal connections."""
        # Internal connections are handled in button creation
        pass
    
    def _on_action_clicked(self, action_name: str):
        """Handle action button click."""
        try:
            # Emit appropriate signal based on whether this is from QR or manual
            if action_name == self.last_qr_action:
                self.qr_action_triggered.emit(action_name)
                self.last_qr_action = ""  # Clear after use
            else:
                if action_name == "StopMove":
                    self.emergency_stop.emit()
                else:
                    self.action_triggered.emit(action_name)
            
            # Provide visual feedback
            self._provide_button_feedback(action_name)
            
        except Exception as e:
            print(f"Error handling action click: {e}")
    
    def _provide_button_feedback(self, action_name: str):
        """Provide visual feedback for button press."""
        if action_name in self.command_buttons:
            button = self.command_buttons[action_name]
            
            # Temporarily change style to show press
            original_style = button.styleSheet()
            
            feedback_style = (
                "QPushButton { "
                "background-color: #4ade80; "
                "color: #000; "
                "border: 2px solid #22c55e; "
                "font-size: 12px; "
                "font-weight: bold; "
                "border-radius: 8px; "
                "padding: 8px; "
                "margin: 2px; "
                "}"
            )
            
            button.setStyleSheet(feedback_style)
            
            # Reset after 500ms
            from python_qt_binding.QtCore import QTimer
            QTimer.singleShot(500, lambda: button.setStyleSheet(original_style))
    
    @pyqtSlot(str)
    def highlight_qr_action(self, qr_data: str):
        """
        Highlight button corresponding to detected QR code.
        
        Args:
            qr_data: QR code content
        """
        # Map QR data to action (using the same mapping as robot_controller)
        qr_action_map = {
            "HELLO": "Hello", "HI": "Hello", "WAVE": "Hello",
            "SIT": "Sit", "SITDOWN": "Sit",
            "STAND": "StandUp", "STANDUP": "StandUp", "UP": "StandUp",
            "DANCE": "Dance1", "DANCE1": "Dance1", "DANCE2": "Dance2", "PARTY": "Dance1",
            "STRETCH": "Stretch", "EXERCISE": "Stretch",
            "FLIP": "FrontFlip", "FRONTFLIP": "FrontFlip",
            "JUMP": "FrontJump",
            "HEART": "FingerHeart", "LOVE": "FingerHeart",
            "HIPS": "WiggleHips", "WIGGLE": "WiggleHips",
            "HANDSTAND": "Handstand",
            "MOONWALK": "MoonWalk", "MOON": "MoonWalk",
            "STOP": "StopMove", "EMERGENCY": "StopMove",
            "BALANCE": "BalanceStand",
        }
        
        qr_upper = qr_data.upper().strip()
        
        # Find matching action
        action_name = None
        if qr_upper in qr_action_map:
            action_name = qr_action_map[qr_upper]
        else:
            # Try direct match with action names
            for cmd_action in self.command_buttons.keys():
                if cmd_action.upper() == qr_upper:
                    action_name = cmd_action
                    break
        
        # Highlight the corresponding button
        if action_name and action_name in self.command_buttons:
            self.last_qr_action = action_name
            self.command_buttons[action_name].highlight_for_qr()
    
    def get_available_actions(self) -> List[str]:
        """Get list of all available action names."""
        return list(self.command_buttons.keys())
    
    def is_valid_action(self, action_name: str) -> bool:
        """Check if an action name is valid."""
        return action_name in self.command_buttons