import rclpy
import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
l_VELOCITY, l_ACC = 250, 250
j_VELOCITY, j_ACC = 60, 60

# 초기화
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("task", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            amovel,
            amovej,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            DR_TOOL,
            set_digital_output,
            wait,
            get_current_posx,
            get_current_posj,
            mwait,
            DR_MV_MOD_REL,
        )
        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return
    
    def grip():
        """grip 동작 수행."""
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    def release():
        """Release 동작 수행."""
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    def move_to_pos(position):
        """지정된 위치로 pos 이동."""
        movel(position, vel=l_VELOCITY, acc=l_ACC, ref=DR_BASE)
        print(f"Moved to position: {position}")

    def move_to_joint(joint):
        """지정된 위치로 joint 이동."""
        movej(joint, vel=j_VELOCITY, acc=j_ACC)
        print(f"Moved to position: {joint}")

    def move_to_place(pos):
        pos[2] += 110
        amovel(posx(pos), vel=l_VELOCITY, acc=l_ACC, ref=DR_BASE)
        wait(0.1)
        pos[2] -= 110
        movel(posx(pos), vel=l_VELOCITY, acc=l_ACC, ref=DR_BASE)
    
    def get_pos(coord):
        x1, y1, z1, rx1, ry1, rz1 = coord
        coordinates = [
            [round(x1, 3), round(y1, 3), round(z1, 3), rx1, ry1, rz1],
            [round(x1, 3), round(y1 - 80, 3), round(z1, 3), rx1, ry1, rz1],
            [round(x1, 3), round(y1 - 160, 3), round(z1, 3), rx1, ry1, rz1],
            [round(x1 + 69.282, 3), round(y1 - 40, 3), round(z1, 3), rx1, ry1, rz1],
            [round(x1 + 69.282, 3), round(y1 - 120, 3), round(z1, 3), rx1, ry1, rz1],
            [round(x1 + 138.564, 3), round(y1 - 80, 3), round(z1, 3), rx1, ry1, rz1]
        ]
        z_offset = 90
        x7 = round((coordinates[0][0] + coordinates[1][0] + coordinates[3][0]) / 3, 3)
        y7 = round((coordinates[0][1] + coordinates[1][1] + coordinates[3][1]) / 3, 3)
        x8 = round((coordinates[1][0] + coordinates[2][0] + coordinates[4][0]) / 3, 3)
        y8 = round((coordinates[1][1] + coordinates[2][1] + coordinates[4][1]) / 3, 3)
        x9 = round((coordinates[3][0] + coordinates[4][0] + coordinates[5][0]) / 3, 3)
        y9 = round((coordinates[3][1] + coordinates[4][1] + coordinates[5][1]) / 3, 3)
        coordinates.append([x7, y7, round(z1 + z_offset, 3), rx1, ry1, rz1])
        coordinates.append([x8, y8, round(z1 + z_offset, 3), rx1, ry1, rz1])
        coordinates.append([x9, y9, round(z1 + z_offset, 3), rx1, ry1, rz1])
        z_offset_2 = z_offset * 2
        x10 = round((x7 + x8 + x9) / 3, 3)
        y10 = round((y7 + y8 + y9) / 3, 3)
        coordinates.append([x10, y10, round(z1 + z_offset_2, 3), rx1, ry1, rz1])
        return coordinates
    
    # 도구와 TCP 설정
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    # 위치 정의
    pick = [377.461, -176.385, 217.806, 79.468, 179.973, 79.457]
    place = [380.462, 165.951, 95.486, 126.914, 179.983, 126.874]

    before_last_pick_j = [-86.631, 47.209, 97.133, 50.728, 107.477, -65.31]
    last_pick_j = [-70.733, 40.898, 113.0, 63.631, 95.48, -68.979]
    before_last_place_j = [-49.732, 31.023, 96.63, 72.528, 123.357, -220.149]
    last_place_j = [-27.728, 31.85, 93.238, 75.512, 122.596, -216.795]

    place_pos = get_pos(place)

    if rclpy.ok():
        memory_pos = pick
        move_to_joint([0,0,90,0,90,0])
        for i in range(10):
            move_to_pos(posx(memory_pos))
            grip()
            wait(1)

            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -60, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
            release_compliance_ctrl()
            mwait()

            current_pos, sol = get_current_posx()
            memory_pos = current_pos

            movel([0, 0, 10, 0, 0, 0], vel=l_VELOCITY, acc=l_ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

            release()
            wait(0.5)

            movel([0, 0, -23, 0, 0, 0], vel=l_VELOCITY, acc=l_ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

            grip()
            wait(0.5)

            amovel([0, 0, 100, 0, 0, 0], vel=l_VELOCITY, acc=l_ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
            wait(0.6)
            amovel([100, 100, 0, 0, 0, 0], vel=l_VELOCITY, acc=l_ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
            wait(0.3)

            move_to_place(place_pos[i])

            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -60, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
            release_compliance_ctrl()
            mwait()

            release()
            wait(0.5)
            amovel([0, 0, 20, 0, 0, 0], vel=l_VELOCITY, acc=l_ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
            wait(0.05)

        move_to_pos(posx(pick))

        amovej(before_last_pick_j, vel=j_VELOCITY, acc=j_ACC)
        wait(1.25)
        move_to_joint(last_pick_j)
        grip()
        wait(1)

        amovej(before_last_place_j, vel=j_VELOCITY, acc=j_ACC)
        wait(2.3)
        move_to_joint(last_place_j)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        release_compliance_ctrl()
        mwait()

        release()
        wait(1)

        move_to_joint(before_last_place_j)
        move_to_joint([-25.964, 7.013, 78.913, -0.011, 94.08, -25.94])

    rclpy.shutdown()

if __name__ == "__main__":
    main()

