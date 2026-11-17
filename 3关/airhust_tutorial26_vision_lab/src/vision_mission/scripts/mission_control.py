#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import sys

import rospy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

ALTITUDE = 0.8

setpoint_raw = PositionTarget()

# 飞控状态。
current_state = State()


def state_cb(msg):
    global current_state
    current_state = msg


# 里程计与起飞初始位置。
local_pos = Odometry()
roll = pitch = yaw = 0.0
init_position_x_take_off = 0.0
init_position_y_take_off = 0.0
init_position_z_take_off = 0.0
init_yaw_take_off = 0.0
flag_init_position = False


def local_pos_cb(msg):
    global local_pos, roll, pitch, yaw
    global init_position_x_take_off, init_position_y_take_off
    global init_position_z_take_off, init_yaw_take_off, flag_init_position
    local_pos = msg
    q = msg.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    if flag_init_position == False and (local_pos.pose.pose.position.z != 0):
        init_position_x_take_off = local_pos.pose.pose.position.x
        init_position_y_take_off = local_pos.pose.pose.position.y
        init_position_z_take_off = local_pos.pose.pose.position.z
        init_yaw_take_off = yaw
        flag_init_position = True


# 起飞初始位置为平移原点的位置目标。
mission_pos_cruise_last_position_x = 0.0
mission_pos_cruise_last_position_y = 0.0
mission_pos_cruise_flag = False


def mission_pos_cruise(x, y, z, target_yaw, error_max):
    global mission_pos_cruise_flag
    global mission_pos_cruise_last_position_x, mission_pos_cruise_last_position_y
    if not mission_pos_cruise_flag:
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y
        mission_pos_cruise_flag = True
    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048
    setpoint_raw.coordinate_frame = 1
    setpoint_raw.position.x = x + init_position_x_take_off
    setpoint_raw.position.y = y + init_position_y_take_off
    setpoint_raw.position.z = z + init_position_z_take_off
    setpoint_raw.yaw = target_yaw
    rospy.loginfo(
        "now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f), init_pose(%.2f, %.2f, %.2f, %.2f)",
        local_pos.pose.pose.position.x,
        local_pos.pose.pose.position.y,
        local_pos.pose.pose.position.z,
        yaw * 180.0 / math.pi,
        x,
        y,
        z,
        target_yaw * 180.0 / math.pi,
        init_position_x_take_off,
        init_position_y_take_off,
        init_position_z_take_off,
        init_yaw_take_off * 180.0 / math.pi,
    )
    if (
        math.fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off)
        < error_max
        and math.fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off)
        < error_max
        and math.fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off)
        < error_max
        and math.fabs(yaw - target_yaw) < 0.1
    ):
        rospy.loginfo("到达目标点，巡航点任务完成")
        mission_pos_cruise_flag = False
        return True
    return False


# 以首次调用时的当前位置为 XY 平移起点；不进行机体航向旋转。
current_position_cruise_last_position_x = 0.0
current_position_cruise_last_position_y = 0.0
current_position_cruise_flag = False


def current_position_cruise(x, y, z, target_yaw, error_max):
    global current_position_cruise_flag
    global \
        current_position_cruise_last_position_x, \
        current_position_cruise_last_position_y
    if not current_position_cruise_flag:
        current_position_cruise_last_position_x = local_pos.pose.pose.position.x
        current_position_cruise_last_position_y = local_pos.pose.pose.position.y
        current_position_cruise_flag = True
    # float p_xy = 0.8;
    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048
    setpoint_raw.coordinate_frame = 1
    setpoint_raw.position.x = x + current_position_cruise_last_position_x
    setpoint_raw.position.y = y + current_position_cruise_last_position_y
    setpoint_raw.position.z = z
    setpoint_raw.yaw = target_yaw
    rospy.loginfo(
        "now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)",
        local_pos.pose.pose.position.x,
        local_pos.pose.pose.position.y,
        local_pos.pose.pose.position.z,
        yaw * 180.0 / math.pi,
        x + current_position_cruise_last_position_x,
        y + current_position_cruise_last_position_y,
        z,
        target_yaw * 180.0 / math.pi,
    )
    if (
        math.fabs(
            local_pos.pose.pose.position.x - current_position_cruise_last_position_x - x
        )
        < error_max
        and math.fabs(
            local_pos.pose.pose.position.y - current_position_cruise_last_position_y - y
        )
        < error_max
        and math.fabs(local_pos.pose.pose.position.z - z) < error_max
        and math.fabs(yaw - target_yaw) < 0.1
    ):
        rospy.loginfo("到达目标点，巡航点任务完成")
        # current_position_cruise_flag = False;
        setpoint_raw.yaw_rate = 0.0  # 重置yaw_rate
        setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048
        setpoint_raw.yaw = yaw  # 确保最终yaw角度正确
        return True
    return False


# 降落。
precision_land_init_position_x = 0.0
precision_land_init_position_y = 0.0
precision_land_init_position_flag = False
precision_land_last_time = rospy.Time(0)


def precision_land():
    global precision_land_init_position_flag
    global precision_land_last_time
    global precision_land_init_position_x, precision_land_init_position_y
    if not precision_land_init_position_flag:
        precision_land_init_position_x = local_pos.pose.pose.position.x
        precision_land_init_position_y = local_pos.pose.pose.position.y
        precision_land_last_time = rospy.Time.now()
        precision_land_init_position_flag = True
    setpoint_raw.position.x = precision_land_init_position_x
    setpoint_raw.position.y = precision_land_init_position_y
    setpoint_raw.position.z = -0.15
    setpoint_raw.type_mask = 64 + 128 + 256 + 512
    setpoint_raw.coordinate_frame = 1
    if rospy.Time.now() - precision_land_last_time > rospy.Duration(5):
        rospy.loginfo("Precision landing complete.")
        precision_land_init_position_flag = False  # Reset for next landing
        return True
    return False


# 任务计时器
last_time = rospy.Time(0)
lib_time_init_flag = True


def lib_time_record_func(duration, current_time):
    # 静态变量，记录上次触发时间
    global lib_time_init_flag, last_time
    if lib_time_init_flag:
        last_time = current_time
        lib_time_init_flag = False  # 只在第一次调用时初始化
    elapsed = current_time - last_time
    # 如果已经过去超过 duration 秒，返回 True 并更新时间
    if elapsed.to_sec() >= duration:
        lib_time_init_flag = True
        return True
    return False


# 保持当前位置悬停。
hover_started = False
hover_last_time = rospy.Time(0)
hover_x = 0.0
hover_y = 0.0
hover_z = 0.0
hover_yaw = 0.0


def hover(time_duration):
    global hover_started, hover_last_time
    global hover_x, hover_y, hover_z, hover_yaw
    if not hover_started:
        hover_last_time = rospy.Time.now()
        hover_started = True
        hover_x = local_pos.pose.pose.position.x
        hover_y = local_pos.pose.pose.position.y
        hover_z = local_pos.pose.pose.position.z
        hover_yaw = yaw
    elapsed = rospy.Time.now() - hover_last_time
    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048
    setpoint_raw.coordinate_frame = 1
    setpoint_raw.position.x = hover_x
    setpoint_raw.position.y = hover_y
    setpoint_raw.position.z = hover_z
    setpoint_raw.yaw = hover_yaw
    rospy.loginfo(
        "Hovering at (%.2f, %.2f, %.2f) for %.2f seconds",
        hover_x,
        hover_y,
        hover_z,
        elapsed.to_sec(),
    )
    # 如果已经过去超过 time_duration 秒，返回 True 并重置状态
    if elapsed.to_sec() >= time_duration:
        hover_started = False  # Reset for next hover
        return True
    return False


# end header

# 全局变量定义
mission_num = 0
if_debug = 0
err_max = 0.2


def print_param():
    print("=== 控制参数 ===")
    print(f"err_max: {err_max}")
    print(f"ALTITUDE: {ALTITUDE}")
    print(f"if_debug: {if_debug}")
    if if_debug == 1:
        print("自动offboard")
    else:
        print("遥控器offboard")


def main():
    global mission_num, err_max, if_debug, lib_time_init_flag
    # 初始化ROS节点
    rospy.init_node("mission_control")

    # 订阅mavros相关话题
    rospy.Subscriber("mavros/state", State, state_cb, queue_size=10)
    rospy.Subscriber(
        "/mavros/local_position/odom", Odometry, local_pos_cb, queue_size=10
    )

    # 发布无人机多维控制话题
    mavros_setpoint_pos_pub = rospy.Publisher(
        "/mavros/setpoint_raw/local", PositionTarget, queue_size=100
    )
    # 创建服务客户端
    # ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    # ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    rospy.wait_for_service("mavros/set_mode")
    set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
    rospy.wait_for_service("mavros/cmd/arming")
    arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    # 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
    rate = rospy.Rate(20)

    # 参数读取

    err_max = rospy.get_param("err_max", 0.0)
    if_debug = rospy.get_param("if_debug", 0.0)
    print_param()

    choice = 0
    try:
        choice = int(input("1 to go on , else to quit: "))
    except ValueError:
        choice = 0
    if choice != 1:
        return 0

    rate.sleep()

    # 等待连接到飞控
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    # 定义客户端变量，设置为offboard模式
    # mavros_msgs::SetMode offb_set_mode;
    # offb_set_mode.request.custom_mode = "OFFBOARD"
    # resp = set_mode(custom_mode="OFFBOARD")
    # if resp.mode_sent:
    #   rospy.loginfo("Offboard enabled")

    # 定义客户端变量，请求无人机解锁
    # mavros_msgs::CommandBool arm_cmd;
    # arm_cmd.request.value = True
    # resp = arm(value=True)
    # if resp.success:
    #   rospy.loginfo("Vehicle armed")

    # 记录当前时间，并赋值给变量last_request
    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (
            rospy.Time.now() - last_request > rospy.Duration(3)
        ):
            if if_debug == 1:
                if set_mode(custom_mode="OFFBOARD").mode_sent:
                    rospy.loginfo("Offboard enabled")
            else:
                rospy.loginfo("Waiting for OFFBOARD mode")
            last_request = rospy.Time.now()
        else:
            if not current_state.armed and (
                rospy.Time.now() - last_request > rospy.Duration(3)
            ):
                if arm(value=True).success:
                    rospy.loginfo("Vehicle armed")
                last_request = rospy.Time.now()
        # 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
        if math.fabs(
            local_pos.pose.pose.position.z - ALTITUDE
        ) < 0.2 and rospy.Time.now() - last_request > rospy.Duration(1):
            mission_num = 1
            last_request = rospy.Time.now()
            break

        mission_pos_cruise(0, 0, ALTITUDE, 0, err_max)
        mavros_setpoint_pos_pub.publish(setpoint_raw)
        rate.sleep()

    while not rospy.is_shutdown():
        rospy.logwarn(f"mission_num = {mission_num}")

        if mission_num == 1:  # 起飞
            if mission_pos_cruise(0, 0, ALTITUDE, 0, err_max):
                if lib_time_record_func(1.0, rospy.Time.now()):
                    mission_num = 2
                    last_request = rospy.Time.now()
            elif rospy.Time.now() - last_request >= rospy.Duration(3):
                mission_num = 2
                last_request = rospy.Time.now()
                lib_time_init_flag = True
        elif mission_num == 2:  # 世界系前进
            if mission_pos_cruise(1.0, 0.0, ALTITUDE, 1.57, err_max):
                mission_num = 3
                last_request = rospy.Time.now()
        elif mission_num == 3:  # 机体系右移
            if current_position_cruise(0.0, -4.0, ALTITUDE, -1.57, err_max):
                mission_num = 4
                last_request = rospy.Time.now()
        elif mission_num == 4:  # 悬停
            if hover(3.0):
                mission_num = 5
                last_request = rospy.Time.now()
        elif mission_num == 5:  # 降落
            if precision_land():
                mission_num = -1  # 任务结束
                last_request = rospy.Time.now()

        mavros_setpoint_pos_pub.publish(setpoint_raw)
        rate.sleep()

        if mission_num == -1:
            sys.exit(0)
    return 0


if __name__ == "__main__":
    main()
