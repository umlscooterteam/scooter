import rclpy
from scooter_interfaces import GoToJointConfig


class JointConfigs:
    fold_config = [0.0263192318379879, 1.2939172983169556, -1.7775753180133265, -1.2353704611407679, -1.5866463820086878, 0.1328919529914856]
    travel_config = [1.585824966430664, 1.1334773302078247, -2.7559364477740687, -0.05789691606630498, -1.5676992575274866, 0.1261676698923111]


def main(args=None):
    rclpy.init(args=args)
    action_client = TestGoToJointConfig()

    while True:
        future = action_client.send_goal(JointConfigs.fold_config)
        rclpy.spin_until_future_complete(action_client, future)

        future = action_client.send_goal(JointConfigs.travel_config)
        rclpy.spin_until_future_complete(action_client, future)


if __name__ == "__main__":
    main()
