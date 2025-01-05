#Markus Buchholz

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import curses
import time
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        # Publisher for /target_position
        self.publisher = self.create_publisher(Float64MultiArray, '/target_position', 10)

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Frame names - replace these with your robot's actual frame names
        self.parent_frame = 'base_link'
        self.child_frame = 'ee_base_link'

        # Initialize current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # Initialize target position (will be set to current position once read)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_initialized = False

        # Define step sizes
        self.step_x = 0.01  # meters
        self.step_y = 0.01  # meters
        self.step_z = 0.005  # meters

        # Define limits
        self.min_limit = -0.15
        self.max_limit = 0.15 + 0.05

        # Initialize curses
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.stdscr.nodelay(True)  # Non-blocking input
        self.stdscr.keypad(True)

        # Hide the cursor
        curses.curs_set(0)

        # Display instructions
        self.display_instructions()
        self.update_display(initial=True)

    def get_current_position(self):
        """Attempt to get the current end-effector position from TF."""
        try:
            # Attempt to get the latest available transform
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )

            # Extract the position (XYZ)
            translation = transform.transform.translation
            self.current_x = translation.x
            self.current_y = translation.y
            self.current_z = translation.z

            # If target position hasn't been set yet, initialize it to current position
            if not self.target_initialized:
                self.target_x = self.current_x
                self.target_y = self.current_y
                self.target_z = self.current_z
                self.publish_position()
                self.target_initialized = True

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # If transform is not available, keep previous current positions
            pass  # Optionally, you can set current positions to None or log a warning

    def display_instructions(self):
        """Display the control instructions on the terminal."""
        self.stdscr.clear()
        instructions = [
            "Keyboard Controller for XYZ Position",
            "------------------------------------",
            "Use the following keys to control each axis:",
            "",
            "X-Axis (Arrow Left/Right):",
            "  Left Arrow  : Decrease X (-0.01 m)",
            "  Right Arrow : Increase X (+0.01 m)",
            "",
            "Y-Axis (Arrow Up/Down):",
            "  Up Arrow    : Increase Y (+0.01 m)",
            "  Down Arrow  : Decrease Y (-0.01 m)",
            "",
            "Z-Axis:",
            "  'e' : Increase Z (+0.005 m)",
            "  'q' : Decrease Z (-0.005 m)",
            "",
            "Other Controls:",
            "  'x' : Reset target position to current position",
            "  'ESC': Quit the program",
            "",
            "Current Position:",
            "  X: 0.000 m",
            "  Y: 0.000 m",
            "  Z: 0.000 m",
            "",
            "Target Position:",
            "  X: 0.000 m",
            "  Y: 0.000 m",
            "  Z: 0.000 m",
            "",
            "Press keys to move the target position...",
        ]
        for idx, line in enumerate(instructions):
            self.stdscr.addstr(idx, 0, line)
        self.stdscr.refresh()

    def update_display(self, initial=False):
        """Update the displayed current and target XYZ positions on the terminal."""
        # Update the current position display
        self.stdscr.addstr(20, 4, f"X: {self.current_x: .3f} m    ")
        self.stdscr.addstr(21, 4, f"Y: {self.current_y: .3f} m    ")
        self.stdscr.addstr(22, 4, f"Z: {self.current_z: .3f} m    ")

        # Update the target position display
        self.stdscr.addstr(24, 4, f"X: {self.target_x: .3f} m    ")
        self.stdscr.addstr(25, 4, f"Y: {self.target_y: .3f} m    ")
        self.stdscr.addstr(26, 4, f"Z: {self.target_z: .3f} m    ")

        # Refresh the screen only if not in initial setup
        if not initial:
            self.stdscr.refresh()

    def publish_position(self):
        """Publish the current target XYZ position to the /target_position topic."""
        msg = Float64MultiArray()
        msg.data = [self.target_x, self.target_y, self.target_z]
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published target_position: [X: {self.target_x:.3f}, Y: {self.target_y:.3f}, Z: {self.target_z:.3f}]'
        )

    def run(self):
        """Main loop to handle keyboard inputs and publish positions."""
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)

                # Update current position from TF
                self.get_current_position()

                # Capture key press
                key = self.stdscr.getch()

                if key != -1:
                    key_handled = False  # Flag to check if a valid key was pressed

                    # Handle Arrow Keys for X and Y axes
                    if key == curses.KEY_LEFT:
                        self.target_x -= self.step_x
                        if self.target_x < self.min_limit:
                            self.target_x = self.min_limit
                        key_handled = True
                    elif key == curses.KEY_RIGHT:
                        self.target_x += self.step_x
                        if self.target_x > self.max_limit:
                            self.target_x = self.max_limit
                        key_handled = True
                    elif key == curses.KEY_UP:
                        self.target_y += self.step_y
                        if self.target_y > self.max_limit:
                            self.target_y = self.max_limit
                        key_handled = True
                    elif key == curses.KEY_DOWN:
                        self.target_y -= self.step_y
                        if self.target_y < self.min_limit:
                            self.target_y = self.min_limit
                        key_handled = True
                    elif key in [ord('e'), ord('E')]:
                        self.target_z += self.step_z
                        if self.target_z > self.max_limit:
                            self.target_z = self.max_limit
                        key_handled = True
                    elif key in [ord('q'), ord('Q')]:
                        self.target_z -= self.step_z
                        if self.target_z < self.min_limit:
                            self.target_z = self.min_limit
                        key_handled = True
                    elif key in [ord('x'), ord('X')]:
                        # Reset target position to current position
                        self.target_x = self.current_x
                        self.target_y = self.current_y
                        self.target_z = self.current_z
                        key_handled = True
                    elif key == 27:  # ESC key
                        break  # Exit the loop

                    if key_handled:
                        # Publish the updated target position
                        self.publish_position()
                        # Update the display to reflect changes
                        self.update_display()

                # Always update the display with the latest current position
                self.update_display()

                # Sleep to control the loop rate
                time.sleep(0.1)  # 10 Hz

        finally:
            # Restore terminal settings
            curses.nocbreak()
            self.stdscr.keypad(False)
            curses.echo()
            curses.endwin()

def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()