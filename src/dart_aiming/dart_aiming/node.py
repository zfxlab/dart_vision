import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from dart_interfaces.msg import ControllerState, TargetEstimation, AimCommand
from dart_target_estimation.core import fresh
from dart_target_estimation.ros_geometry import seconds, xyz, transform
from .core import command


class Aiming(Node):
    def __init__(self):
        super().__init__("aiming")
        for k, v in dict(
            fixed_frame="dart_base_link",
            launcher_frame="launcher_frame",
            controller_timeout=0.25,
            estimate_timeout=0.25,
            tf_timeout=0.25,
            light_timeout=0.25,
            allow_acquire=True,
        ).items():
            self.declare_parameter(k, v)
        self.p = lambda k: self.get_parameter(k).value
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.controller = self.estimate = None
        self.create_subscription(
            ControllerState,
            "controller_state",
            self.on_controller,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TargetEstimation, "target_estimation", self.on_estimate, 10
        )
        self.pub = self.create_publisher(
            AimCommand, "aim_command", qos_profile_sensor_data
        )
        self.create_timer(0.02, self.tick)

    def on_controller(self, m):
        self.controller = m

    def on_estimate(self, m):
        self.estimate = m

    def tick(self):
        clock = self.get_clock().now()
        now = clock.nanoseconds * 1e-9
        out = AimCommand()
        out.header.stamp = clock.to_msg()
        out.header.frame_id = self.p("launcher_frame")
        c, e = self.controller, self.estimate
        try:
            if (
                c is not None
                and e is not None
                and e.header.frame_id == self.p("fixed_frame")
                and fresh(seconds(c.header.stamp), now, self.p("controller_timeout"))
                and fresh(seconds(e.header.stamp), now, self.p("estimate_timeout"))
            ):
                t = self.tf.lookup_transform(
                    self.p("fixed_frame"), self.p("launcher_frame"), Time()
                )
                if fresh(seconds(t.header.stamp), now, self.p("tf_timeout")):
                    out.state, out.yaw_rad, out.distance_m = command(
                        c.target_id,
                        c.offset_rad,
                        transform(t.transform),
                        xyz(e.base_center),
                        xyz(e.center_light),
                        xyz(e.light_center),
                        clear=e.visibility == 2,
                        base_valid=e.base_valid,
                        light_valid=e.light_valid
                        and fresh(seconds(e.light_stamp), now, self.p("light_timeout")),
                        measured=e.base_source == 1,
                        conflict=e.sensor_conflict,
                        allow_acquire=self.p("allow_acquire"),
                    )
        except (TransformException, ValueError):
            pass
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Aiming()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
