# coding:utf-8
import pygame
import time
from pymycobot import ultraArm
import threading

mc = ultraArm("com7")
command = [235, 0, 130]
zero = [235, 0, 130]
action = 0
speed = 30
is_zero = False
lock = threading.Lock()
is_move_jog = False


def control():
    global command, action, is_zero, is_move_jog
    while True:
        if is_zero:
            # if action != 0:
            # print(" action: ", action)
            if action == 1:
                print("x++")
                mc.set_jog_coord(1, 0, speed)
                action = 0
            elif action == 2:
                print("x--")
                mc.set_jog_coord(1, 1, speed)
                action = 0
            elif action == 4:
                print("y++")
                mc.set_jog_coord(2, 0, speed)
                action = 0
            elif action == 3:
                print("y--")
                
                mc.set_jog_coord(2, 1, speed)
                action = 0
            elif action == 5 and is_move_jog == False:
                mc.set_jog_coord(3, 0, speed)
                lock.acquire()
                is_move_jog = True
                action = 0
                lock.release()
                print("z++ ")
                
            elif action == 6  and is_move_jog == False:
                mc.set_jog_coord(3, 1, speed)
                lock.acquire()
                is_move_jog = True
                action = 0
                lock.release()
                print("z-- ")
                
            elif action == 7:
                mc.set_gripper_state(1)
                action = 0
            elif action == 8:
                
                mc.set_gripper_state(0)
                action = 0
            elif action == 9:
                mc.set_gpio_state(0)
                action = 0
            elif action == 10:
                
                mc.set_gpio_state(1)
                action = 0
            elif action == 11:
                time.sleep(2)
                if action == 11:
                    mc.go_zero()
                    action = 0
            elif action == 12:
                time.sleep(2)
                if action == 12:
                    mc.release_all_servos()
                    action = 0
            elif action == 13:
                time.sleep(2)
                if action == 13:
                    mc.set_coords(zero, speed)
                    command = zero.copy()
                    action = 0
            elif action == 14:
                time.sleep(2)
                if action == 14:
                    mc.power_on()
                    action = 0
            elif action == 15:
                
                mc.set_jog_stop()
                lock.acquire()
                is_move_jog = False
                action = 0
                lock.release()
                print("move stop")
        else:
            if action == 11:
                time.sleep(2)
                if action == 11:
                    print("go zero")
                    mc.go_zero()
                    action = 0
                    is_zero = True
        time.sleep(0.05)


def main():
    global action, is_move_jog
    pygame.init()
    pygame.joystick.init()

    try:
        joystick = pygame.joystick.Joystick(0)
    except:
        print("请先连接手柄")
        return
    joystick.init()

    done = False

    start_time = 0
    axis_list = []
    print("请先按下'回到零点'按钮，将设备调整到零点位置, 才能进行下一步的控制")
    while not done:
        for event_ in pygame.event.get():
            if event_.type == pygame.QUIT:
                done = True
            # 按键按下或弹起事件
            elif (
                event_.type == pygame.JOYBUTTONDOWN or event_.type == pygame.JOYBUTTONUP
            ):
                buttons = joystick.get_numbuttons()
                # 获取所有按键状态信息
                for i in range(buttons):
                    button = joystick.get_button(i)
                    # print("button " + str(i) + ": " + str(button))
                    
                    if i == 1:
                        if button == 1:
                            action = 10
                            break
                    if i == 0:
                        if button == 1:
                            print(11)
                            action = 9
                            break
                    if i == 3:
                        if button == 1:
                            action = 8
                            break
                    if i == 2:
                        if button == 1:
                            action = 7
                            break
                    if i == 4:
                        if button == 1:
                            action = 11
                            start_time = time.time()
                            break
                        if start_time != 0 and button == 0:
                            if time.time() - start_time > 2:
                                start_time = 0
                                break
                            else:
                                start_time = 0
                                action = 0
                    if i == 5:
                        if button == 1:
                            action = 13
                            start_time = time.time()
                            break
                        if start_time != 0 and button == 0:
                            if time.time() - start_time > 2:
                                start_time = 0
                                break
                            else:
                                start_time = 0
                                action = 0
            # 轴转动事件
            elif event_.type == pygame.JOYAXISMOTION:
                axes = joystick.get_numaxes()
                # 获取所有轴状态信息
                # while True:
                
                for i in range(axes):
                    axis = joystick.get_axis(i)
                    # res[i] = axis
                    
                    if i == 1:
                        # print("axis " + str(i) + ": " + str(axis) + " "+str(is_move_jog))
                        if axis == -3.0517578125e-05 and is_move_jog:
                            lock.acquire()
                            action = 15
                            lock.release()
                            break
                        if axis == -3.0517578125e-05:
                            continue
                        if axis < -3.0517578125e-05:
                            lock.acquire()
                            action = 5
                            lock.release()
                            if len(axis_list) < 2:
                                axis_list.append(axis)
                            else:
                                # print(axis_list)
                                
                                if axis <= axis_list[1] <= axis_list[0]:
                                    axis_list = []
                                    break
                                else:
                                    lock.acquire()
                                    action = 15
                                    lock.release()
                                    axis_list = []
                                    break
                            break
                        if axis > -3.0517578125e-05:
                            lock.acquire()
                            action = 6
                            lock.release()
                            if len(axis_list) < 2:
                                axis_list.append(axis)
                            else:
                                # print(axis_list)
                                if axis >= axis_list[1] >= axis_list[0]:
                                    
                                    axis_list = []
                                    break
                                else:
                                    lock.acquire()
                                    action = 15
                                    lock.release()
                                    axis_list = []
                                    break
                            break
                    if i == 4:
                        if axis > 0.9:
                            action = 12
                            start_time = time.time()
                            break
                        if start_time != 0 and axis == -1.0:
                            if time.time() - start_time > 2:
                                start_time = 0
                                break
                            else:
                                start_time = 0
                                lock.acquire()
                                action = 15
                                lock.release()
                    if i == 5:
                        if axis > 0.9:
                            action = 14
                            start_time = time.time()
                            break
                        if start_time != 0 and axis == -1.0:
                            if time.time() - start_time > 2:
                                start_time = 0
                                break
                            else:
                                start_time = 0
                                lock.acquire()
                                action = 15
                                lock.release()
            # 方向键改变事件
            elif event_.type == pygame.JOYHATMOTION:
                # hats = joystick.get_numhats()
                # 获取所有方向键状态信息
                # for i in range(hats):
                hat = joystick.get_hat(0)
                print("hat " + str(i) +": " + str(hat))
                if hat == (0, 1):
                    action = 1
                elif hat == (0, -1):
                    action = 2
                elif hat == (-1, 0):
                    action = 3
                elif hat == (1, 0):
                    action = 4
                elif hat == (0, 0):
                    action = 15

        # joystick_count = pygame.joystick.get_count()

    pygame.quit()

if __name__ == "__main__":
    t = threading.Thread(target=control)
    t.daemon = True
    t.start()
    main()
