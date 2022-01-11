import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


class StaticCollisionObjectPublisher(Node):
    def __init__(self):
        """
        Publish static collision environment
        :param args: ROS args
        :return: None
        """
        # initialize ROS things
        super().__init__('static_collision_object_publisher')
        self.planning_scene = PlanningScene()
        self.planning_scene_pub = self.create_publisher(PlanningScene, "planning_scene", 1)
        self.timer = self.create_timer(0.5, self.publish_collision_objects)

    def publish_collision_objects(self):
        self.planning_scene_pub.publish(self.planning_scene)

    def add_box(self, position, orientation, dimensions):
        collision_object = CollisionObject()

        # dimensions
        primitive = SolidPrimitive()
        primitive.type = primitive.BOX
        # primitive.dimensions.resize(3)
        primitive.dimensions = dimensions

        # pose
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]

        # create and return object
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)

        self.planning_scene.world.collision_objects.append(collision_object)
        print(self.planning_scene.world.collision_objects)


def main(args=None):
    rclpy.init(args=args)

    static_collision_object_publisher = StaticCollisionObjectPublisher()

    # test box pls ignore
    static_collision_object_publisher.add_box(
        (1.0, 0.0, 0.0),       # position    (x, y, z)
        (0.0, 0.0, 0.0, 1.0),  # orientation (x, y, z, w)
        (10.0, 10.0, 10.0)     # dimensions  (x, y, z)
    )

    rclpy.spin(static_collision_object_publisher)


if __name__ == "__main__":
    main()
