#!/usr/bin/env python3

import cv2

import rclpy
from rclpy.node import Node


class CameraTestNode(Node):

    def __init__(self):
        super().__init__("camera_test_node")

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter(
            "camera_device",
            "/dev/video0"
        )

        self.declare_parameter(
            "camera_width",
            1280
        )

        self.declare_parameter(
            "camera_height",
            720
        )

        self.declare_parameter(
            "camera_fps",
            30.0
        )

        # Obtener parámetros
        self.camera_device = self.get_parameter(
            "camera_device"
        ).value

        self.camera_width = self.get_parameter(
            "camera_width"
        ).value

        self.camera_height = self.get_parameter(
            "camera_height"
        ).value

        self.camera_fps = self.get_parameter(
            "camera_fps"
        ).value

        # ============================================================
        # CAMERA
        # ============================================================

        self.camera = None

        self._initialize_camera()

        if self.camera is None:
            self.get_logger().error(
                "Camera initialization failed."
            )

            raise RuntimeError(
                "Unable to initialize camera."
            )

        # ============================================================
        # TIMER
        # ============================================================

        timer_period = 1.0 / self.camera_fps

        self.timer = self.create_timer(
            timer_period,
            self._capture_frame
        )

        self.get_logger().info(
            "Camera test node initialized."
        )

    # ================================================================
    # CAMERA INITIALIZATION
    # ================================================================

    def _initialize_camera(self):

        self.get_logger().info(
            f"Opening camera: {self.camera_device}"
        )

        self.get_logger().info(
            f"Requested resolution: "
            f"{self.camera_width}x{self.camera_height}"
        )

        self.get_logger().info(
            f"Requested FPS: {self.camera_fps}"
        )

        # ------------------------------------------------------------
        # Open camera using V4L2
        # ------------------------------------------------------------

        self.camera = cv2.VideoCapture(
            self.camera_device,
            cv2.CAP_V4L2
        )

        if not self.camera.isOpened():

            self.get_logger().error(
                f"Could not open camera "
                f"{self.camera_device}"
            )

            self.camera = None

            return

        # ------------------------------------------------------------
        # Configure MJPEG
        # ------------------------------------------------------------

        self.camera.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG")
        )

        # ------------------------------------------------------------
        # Configure resolution
        # ------------------------------------------------------------

        self.camera.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self.camera_width
        )

        self.camera.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self.camera_height
        )

        # ------------------------------------------------------------
        # Configure FPS
        # ------------------------------------------------------------

        self.camera.set(
            cv2.CAP_PROP_FPS,
            self.camera_fps
        )

        # ------------------------------------------------------------
        # Read actual camera configuration
        # ------------------------------------------------------------

        actual_width = self.camera.get(
            cv2.CAP_PROP_FRAME_WIDTH
        )

        actual_height = self.camera.get(
            cv2.CAP_PROP_FRAME_HEIGHT
        )

        actual_fps = self.camera.get(
            cv2.CAP_PROP_FPS
        )

        self.get_logger().info(
            f"Camera opened successfully."
        )

        self.get_logger().info(
            f"Actual resolution: "
            f"{int(actual_width)}x{int(actual_height)}"
        )

        self.get_logger().info(
            f"Actual FPS: {actual_fps:.2f}"
        )

        # ------------------------------------------------------------
        # Test first frame
        # ------------------------------------------------------------

        ret, frame = self.camera.read()

        if not ret or frame is None:

            self.get_logger().error(
                "Camera opened, but the first frame "
                "could not be acquired."
            )

            self.camera.release()
            self.camera = None

            return

        self.get_logger().info(
            f"First frame received successfully: "
            f"{frame.shape[1]}x{frame.shape[0]}"
        )

        # ------------------------------------------------------------
        # OpenCV window
        # ------------------------------------------------------------

        cv2.namedWindow(
            "RGB Camera",
            cv2.WINDOW_NORMAL
        )

        cv2.resizeWindow(
            "RGB Camera",
            int(actual_width),
            int(actual_height)
        )

    # ================================================================
    # FRAME CAPTURE
    # ================================================================

    def _capture_frame(self):

        if self.camera is None:
            return

        ret, frame = self.camera.read()

        if not ret or frame is None:

            self.get_logger().warn(
                "Failed to read camera frame."
            )

            return

        # ------------------------------------------------------------
        # Display frame
        # ------------------------------------------------------------

        cv2.imshow(
            "RGB Camera",
            frame
        )

        # ------------------------------------------------------------
        # Process OpenCV keyboard events
        #
        # q or ESC -> close node
        # ------------------------------------------------------------

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or key == 27:

            self.get_logger().info(
                "Camera window closed by user."
            )

            self.shutdown_camera()

            rclpy.shutdown()

    # ================================================================
    # CAMERA SHUTDOWN
    # ================================================================

    def shutdown_camera(self):

        if self.camera is not None:

            self.get_logger().info(
                "Releasing camera..."
            )

            self.camera.release()

            self.camera = None

        cv2.destroyAllWindows()

    # ================================================================
    # NODE DESTRUCTOR
    # ================================================================

    def destroy_node(self):

        self.shutdown_camera()

        super().destroy_node()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = CameraTestNode()

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    except Exception as exception:

        if node is not None:

            node.get_logger().error(
                f"Camera node error: {exception}"
            )

        else:

            print(
                f"Camera node error: {exception}"
            )

    finally:

        if node is not None:

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":
    main()