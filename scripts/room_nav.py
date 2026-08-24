#!/usr/bin/env python3

import math
import sys
import random

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class RoomNavGA(Node):
    def __init__(self):
        super().__init__('room_nav_ga')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF để lấy vị trí hiện tại của robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Điểm home dự phòng nếu chưa lấy được TF
        self.home = (0.13754701614379883, 0.016991257667541504)
        self.current_start = self.home

        self.rooms = {
            "room_1_a": {"inside": (7.693443, -4.750302), "door": (1.373064, -3.977323)},
            "room_1_b": {"inside": (7.198482, 4.519353), "door": (0.999525, 3.929502)},
            "room_2_a": {"inside": (4.391075, -9.216341), "door": (1.373064, -3.977323)},
            "room_2_b": {"inside": (4.473134, 7.336530), "door": (0.999525, 3.929502)},
            "room_3_a": {"inside": (0.821909, -10.31354), "door": (1.373064, -3.977323)},
            "room_3_b": {"inside": (0.4656100273132324, 9.768363952636719), "door": (0.999525, 3.929502)},
            "room_4_a": {"inside": (-4.560138, -10.35412), "door": (-5.195352, -5.005528)},
            "room_4_b": {"inside": (-5.035061, 9.778931), "door": (-5.3145012855529785, 3.996394634246826)},
            "room_5_a": {"inside": (-14.32598, -8.320415), "door": (-18.82197, -5.281110)},
            "room_5_b": {"inside": (-15.13345, 7.802671), "door": (-18.86230, 4.906578)},
            "room_6_a": {"inside": (-28.23718, -8.702794), "door": (-32.98879, -5.286651)},
            "room_6_b": {"inside": (-28.90996, 7.817585), "door": (-32.82167, 4.820290)},
            "room_7":   {"inside": (-37.04680, -8.319846), "door": (-34.36130, -5.322080)},
            "room_8":   {"inside": (-43.25960, -3.084990), "door": None},
            "room_9":   {"inside": (-35.98862, 8.718700), "door": None},
            "room_10_a": {"inside": (-13.43992, -1.409400), "door": (-8.738978, -3.005652)},
            "room_10_b": {"inside": (-13.21121, 0.947333), "door": (-8.798617, 2.597627)},
            "room_11_a": {"inside": (-20.95772, -1.800810), "door": (-18.85386, -2.974307)},
            "room_11_b": {"inside": (-20.88392, 1.474046), "door": (-18.88599, 2.604129)},
            "room_12_a": {"inside": (-28.05889, -2.119309), "door": (-24.43754, -2.995604)},
            "room_12_b": {"inside": (-26.84202, 1.313752), "door": (-24.57225, 2.574560)},
        }

    # =========================
    # Geometry helpers
    # =========================
    def dist(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def compute_yaw(self, p_from, p_to):
        return math.atan2(p_to[1] - p_from[1], p_to[0] - p_from[0])

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(yaw)
        return pose

    # =========================
    # Get current robot pose
    # =========================
    def get_current_position(self, wait_sec=5.0):
        start_time = self.get_clock().now()

        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < wait_sec:
            try:
                if self.tf_buffer.can_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time()
                ):
                    tf = self.tf_buffer.lookup_transform(
                        'map',
                        'base_link',
                        rclpy.time.Time()
                    )

                    x = tf.transform.translation.x
                    y = tf.transform.translation.y
                    self.get_logger().info(
                        f'Lay vi tri hien tai thanh cong: x={x:.3f}, y={y:.3f}'
                    )
                    return (x, y)

            except (LookupException, ConnectivityException, ExtrapolationException):
                pass

            # cho TF listener có thời gian nhận dữ liệu
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn(
            f'Khong lay duoc vi tri hien tai sau {wait_sec:.1f}s, dung home: {self.home}'
        )
        return self.home

    # =========================
    # GA for room ordering
    # =========================
    def room_sequence_cost(self, room_order):
        total = 0.0
        current = self.current_start

        for room_name in room_order:
            room = self.rooms[room_name]
            door = room["door"]
            inside = room["inside"]

            if door is not None and inside is not None:
                total += self.dist(current, door)
                total += self.dist(door, inside)
                total += self.dist(inside, door)
                current = door
            elif inside is not None:
                total += self.dist(current, inside)
                current = inside

        return total

    def random_chromosome(self, room_names):
        chrom = list(room_names)
        random.shuffle(chrom)
        return chrom

    def crossover(self, parent1, parent2):
        n = len(parent1)
        if n < 2:
            return parent1[:]

        a, b = sorted(random.sample(range(n), 2))
        child = [None] * n
        child[a:b] = parent1[a:b]

        fill = [g for g in parent2 if g not in child]
        idx = 0
        for i in range(n):
            if child[i] is None:
                child[i] = fill[idx]
                idx += 1
        return child

    def mutate(self, chrom):
        if len(chrom) >= 2:
            i, j = random.sample(range(len(chrom)), 2)
            chrom[i], chrom[j] = chrom[j], chrom[i]
        return chrom

    def run_ga(self, room_names, pop_size=20, generations=30):
        if len(room_names) <= 2:
            return room_names, self.room_sequence_cost(room_names)

        population = [self.random_chromosome(room_names) for _ in range(pop_size)]

        for _ in range(generations):
            population.sort(key=self.room_sequence_cost)
            new_pop = population[:5]

            while len(new_pop) < pop_size:
                p1, p2 = random.sample(population[:10], 2)
                child = self.crossover(p1, p2)

                if random.random() < 0.3:
                    child = self.mutate(child)

                new_pop.append(child)

            population = new_pop

        population.sort(key=self.room_sequence_cost)
        return population[0], self.room_sequence_cost(population[0])

    # =========================
    # Build navigation route
    # =========================
    def build_route_from_best_order(self, best_order):
        poses = []

        for i, room_name in enumerate(best_order):
            room = self.rooms[room_name]
            door = room["door"]
            inside = room["inside"]

            if door is not None and inside is not None:
                yaw_door_to_inside = self.compute_yaw(door, inside)

                # door -> inside
                poses.append(self.make_pose(door[0], door[1], yaw_door_to_inside))
                poses.append(self.make_pose(inside[0], inside[1], yaw_door_to_inside))

                # quay lại door và nhìn sang điểm tiếp theo
                if i < len(best_order) - 1:
                    next_door = self.rooms[best_order[i + 1]]["door"]
                    next_inside = self.rooms[best_order[i + 1]]["inside"]
                    next_pt = next_door if next_door is not None else next_inside
                    yaw_back = self.compute_yaw(door, next_pt)
                else:
                    yaw_back = self.compute_yaw(inside, door)

                poses.append(self.make_pose(door[0], door[1], yaw_back))

            elif inside is not None:
                if i < len(best_order) - 1:
                    next_door = self.rooms[best_order[i + 1]]["door"]
                    next_inside = self.rooms[best_order[i + 1]]["inside"]
                    next_pt = next_door if next_door is not None else next_inside
                    yaw_to_next = self.compute_yaw(inside, next_pt)
                else:
                    yaw_to_next = 0.0

                poses.append(self.make_pose(inside[0], inside[1], yaw_to_next))

        return poses

    def remove_duplicate_poses(self, poses, tol=0.05):
        if not poses:
            return poses

        filtered = [poses[0]]
        for p in poses[1:]:
            prev = filtered[-1]
            d = self.dist(
                (prev.pose.position.x, prev.pose.position.y),
                (p.pose.position.x, p.pose.position.y)
            )
            if d > tol:
                filtered.append(p)
        return filtered

    # =========================
    # Send goals sequentially
    # =========================
    def send_one_goal_and_wait(self, pose):
        # Không dùng timeout, chờ action server lên
        self.client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f'Gui goal: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}'
        )

        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal bi tu choi hoac khong co phan hoi.')
            return False

        # Chờ tới khi robot hoàn thành goal hiện tại rồi mới gửi tiếp
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error('Khong nhan duoc ket qua.')
            return False

        self.get_logger().info(f'Finished waypoint with status = {result.status}')
        return result.status == 4  # STATUS_SUCCEEDED

    def execute_best_route(self, selected_rooms):
        invalid_rooms = [r for r in selected_rooms if r not in self.rooms]
        if invalid_rooms:
            self.get_logger().error(
                f'Loi: Cac phong sau khong ton tai trong he thong: {invalid_rooms}'
            )
            return False

        # Lấy vị trí hiện tại để tính cost thay vì dùng home cố định
        self.current_start = self.get_current_position()

        best_order, best_cost = self.run_ga(selected_rooms)

        self.get_logger().info('==============================')
        self.get_logger().info(f'Vi tri bat dau hien tai: {self.current_start}')
        self.get_logger().info(f'Danh sach yeu cau: {selected_rooms}')
        self.get_logger().info(f'Thu tu toi uu theo GA: {best_order}')
        self.get_logger().info(f'Estimated total cost: {best_cost:.3f} meters')
        self.get_logger().info('==============================')

        poses = self.build_route_from_best_order(best_order)
        poses = self.remove_duplicate_poses(poses)

        if not poses:
            self.get_logger().error('Khong tao duoc route.')
            return False

        for idx, pose in enumerate(poses):
            self.get_logger().info(f'--- Di chuyen toi Waypoint {idx+1}/{len(poses)} ---')
            if not self.send_one_goal_and_wait(pose):
                self.get_logger().error(
                    f'Robot that bai tai waypoint {idx+1}. Huy hanh trinh!'
                )
                return False

        self.get_logger().info('HOAN THANH! Da di het tat ca cac diem theo thu tu toi uu.')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = RoomNavGA()

    try:
        if len(sys.argv) < 2:
            node.get_logger().info(
                'Cach dung:\n'
                'ros2 run project_omni room_nav_ga.py <room_A> <room_B> ... <room_N>\n'
                'Vi du: ros2 run project_omni room_nav_ga.py room_1_a room_4_b room_7 room_12_a'
            )
        else:
            selected_rooms = sys.argv[1:]
            node.execute_best_route(selected_rooms)

    except KeyboardInterrupt:
        node.get_logger().info('Chuong trinh bi dung boi nguoi dung.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()