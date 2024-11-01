from carsim import Car, TrackSpline

import numpy as np
from direct.task import Task

from panda3d.core import Point3, LineSegs, Vec3
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
import numpy as np

# view_type = "third_person"
view_type = "top_down"


class CarViewer(ShowBase):
    def __init__(self, view_type, track_spline, car, track_width=5):
        super().__init__()
        self.track_spline = track_spline
        self.car = car
        self.track_width = track_width  # Width of the track
        self.view_type = view_type

        # Set up basic camera configuration
        self.disableMouse()  # Disable default mouse camera controls

        if view_type == "third_person":
            self.camera_offset = Vec3(0, -10, 5)  # Offset behind and above the car
            self.camera.setPos(0, -10, 2)
        elif view_type == "top_down":
            self.cam.setPos(0, -10, 25)  # Move the camera back and up
            self.cam.lookAt(0, 0, 0)  # Make the camera look towards the origin
            self.camLens.setFov(90)  # Set a wider field of view for a 2D-like effect

        # Set up a task to update the camera each frame
        self.taskMgr.add(self.update_viewer, "update_viewer")

        # Create car model (this example uses a simple cube as a placeholder for the car)
        self.car_model = self.loader.loadModel("models/box")  # Replace with actual car model if available
        self.car_model.reparentTo(self.render)
        self.car_model.setScale(1, 2, 0.5)  # Adjust the scale to fit a car shape
        self.car_model.setPos(self.car.position[0], self.car.position[1], 0)

        self.camera.reparentTo(self.car_model)

        # Draw the track using LineSegs
        self.track_node = self.render.attachNewNode("track")
        self.draw_track()

    def draw_track(self):
        """Draws the track as parallel lines using the track spline points."""
        track_points = [
            self.track_spline.get_point(t) for t in np.linspace(0, len(self.track_spline.spline_x.c) - 1, 100)
        ]

        # Use LineSegs to draw lines for the track
        track_lines = LineSegs()
        track_lines.setColor(1, 1, 1, 1)  # White track lines

        for i in range(len(track_points) - 1):
            p1 = track_points[i]
            p2 = track_points[i + 1]

            # Calculate the direction of the track segment (p2 - p1)
            direction = np.array(p2) - np.array(p1)
            direction_norm = direction / np.linalg.norm(direction)  # Normalize the direction

            # Calculate the perpendicular vector (normal) to the track direction (rotate by 90 degrees)
            normal = np.array([-direction_norm[1], direction_norm[0]])

            # Calculate left and right track boundaries
            left_p1 = p1 + self.track_width / 2 * normal
            right_p1 = p1 - self.track_width / 2 * normal
            left_p2 = p2 + self.track_width / 2 * normal
            right_p2 = p2 - self.track_width / 2 * normal

            # Draw the left and right lines
            track_lines.moveTo(Point3(left_p1[0], left_p1[1], 0))
            track_lines.drawTo(Point3(left_p2[0], left_p2[1], 0))
            track_lines.moveTo(Point3(right_p1[0], right_p1[1], 0))
            track_lines.drawTo(Point3(right_p2[0], right_p2[1], 0))

        # Attach the generated lines to the scene graph
        self.track_node.attachNewNode(track_lines.create())

    def update_viewer(self, task):
        """Update the car's position and heading each frame."""
        car_pos = self.car.position
        car_heading = self.car.heading  # Heading is assumed to be in radians

        # Update the car model's position
        self.car_model.setPos(car_pos[0], car_pos[1], 0)

        # Update the car's rotation (heading) based on its forward direction
        self.car_model.setHpr(np.degrees(-car_heading), 0, 0)  # Rotate around the Z-axis

        return Task.cont  # Continue the task in future frames


class CarController:
    def __init__(self, visualizer):
        self.visualizer = visualizer
        self.max_speed = 1
        self.max_steering_angle = np.radians(15)  # Maximum steering angle in radians

        # Initialize control states
        self.throttle = 0
        self.brake = 0
        self.steering_input = 0

        # Accept keyboard inputs for controls
        self.visualizer.accept("arrow_up", self.start_accelerate)
        self.visualizer.accept("arrow_up-up", self.stop_accelerate)
        self.visualizer.accept("arrow_down", self.start_brake)
        self.visualizer.accept("arrow_down-up", self.stop_brake)
        self.visualizer.accept("arrow_left", self.start_steer_left)
        self.visualizer.accept("arrow_left-up", self.stop_steer_left)
        self.visualizer.accept("arrow_right", self.start_steer_right)
        self.visualizer.accept("arrow_right-up", self.stop_steer_right)

        # Schedule the update task to continuously apply controls
        self.visualizer.taskMgr.add(self.update_controls, "update_controls")

    def start_accelerate(self):
        self.throttle = 0.01  # Fixed throttle value for acceleration

    def stop_accelerate(self):
        self.throttle = 0  # Stop acceleration

    def start_brake(self):
        self.brake = 0.01  # Fixed brake value

    def stop_brake(self):
        self.brake = 0  # Stop braking

    def start_steer_left(self):
        self.steering_input = -self.max_steering_angle  # Turn left

    def stop_steer_left(self):
        self.steering_input = 0  # Stop turning left

    def start_steer_right(self):
        self.steering_input = self.max_steering_angle  # Turn right

    def stop_steer_right(self):
        self.steering_input = 0  # Stop turning right

    def update_controls(self, task):
        # Apply controls based on current throttle, brake, and steering input
        self.visualizer.car.apply_controls(self.throttle, self.brake, self.steering_input)
        return Task.cont  # Continue the task


def run_simulation():
    # Define a simple track with control points
    control_points = [(0, 0), (50, 100), (100, 0), (150, -50), (200, 0)]
    track = TrackSpline(control_points, track_width=50)

    # Create a car object
    car = Car()

    # Create a viewer
    viewer = CarViewer(view_type, track, car)

    # Create a controller
    controller = CarController(viewer)

    viewer.run()

    # # Demo movement: a sequence of (throttle, brake, steering_input)
    # control_sequence = [
    #     (0.2, 0.0, 0.0),  # Accelerate straight
    #     (0.2, 0.0, 0.8),  # Slight turn
    #     (0.2, 0.0, 0.6),  # Opposite turn
    #     (0.05, 0.2, 0.0),  # Slow down, no turn
    # ] * 250  # Repeat to simulate longer movement
    # target_fps = 60

    # for controls in control_sequence:
    #     for event in pygame.event.get():
    #         if event.type == pygame.QUIT:
    #             pygame.quit()
    #             return

    #     # Apply car controls
    #     throttle, brake, steering = controls
    #     car.apply_controls(throttle, brake, steering)
    #     car.update_drift()
    #     car.detect_boundaries(track)

    #     # Update and draw simulation
    #     viewer.update_display()

    #     # Cap the frame rate
    #     demo_clock.tick(target_fps)

    # pygame.quit()


if __name__ == "__main__":
    run_simulation()
