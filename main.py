#!/usr/bin/env pybricks-micropython
from math import floor, ceil
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase


class Robot:

    def __init__(self):
        self.init_vars()
        self.init_hardware()

        self.open()

        while not self.Button1.pressed():
            ...


    def init_hardware(self):
        self.ev3 = EV3Brick()

        self.left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.B)

        self.mid_motor = Motor(Port.A)

        self.deg_enc = self.left_motor.angle()


        self.LS_right = ColorSensor(Port.S2)
        self.LS_left = ColorSensor(Port.S3)
    
        # LS_cube = ColorSensor(Port.S1)
        # LS_center = ColorSensor(Port.S4)

        self.Button1 = TouchSensor(Port.S1)

        # self.LS_side_right = ColorSensor(Port.S4)
        # self.LS_side_left = ColorSensor(Port.S1)

        self.US = UltrasonicSensor(Port.S4)

        # if self.LS_right.color() == Color.GREEN

        # self.gyro_sensor = GyroSensor(Port.S4)



        self.robot = DriveBase(self.left_motor, self.right_motor, wheel_diameter=62.4, axle_track=185)
        self.robot.settings(straight_speed=self.default_speed)


    def init_vars(self):

        self.timer = StopWatch()

        self.S = 185
        self.d = 62.4

        self.gray = 45

        self.default_speed = 300

        self.number = 0

        self.last_error = 0
        self.error_i = 0

        self.n = 0

        self.filtered = [0, 0, 0]

        self.colors_int = {
            Color.BLACK: 1,
            Color.BLUE: 2,
            Color.GREEN: 3,
            Color.YELLOW: 4,
            Color.RED: 5,
            Color.WHITE: 6,
            Color.BROWN: 7,
            Color.ORANGE: 8,
            Color.PURPLE: 9,
        }
    
    def beep(self):
        self.ev3.speaker.beep(300)


    def mm_to_deg(self, mm):
        return mm * 360 / self.d / 3.1415


    def deg_to_mm(self, deg):
        return deg / 360 * self.d * 3.1415


    def motors_stop(self):
        self.left_motor.stop()
        self.right_motor.stop()


    def ride_mm(self, v, mm):
        deg = self.left_motor.angle()
        while abs(deg - self.left_motor.angle()) < self.mm_to_deg(mm):
            self.left_motor.run(v)
            self.right_motor.run(v)
        self.left_motor.stop()
        self.right_motor.stop()


    def ride_deg(self, v, degrees):
        deg = self.left_motor.angle()
        while abs(deg - self.left_motor.angle()) < degrees:
            self.left_motor.run(v)
            self.right_motor.run(v)
        self.left_motor.stop()
        self.right_motor.stop()


    def ride_S_higher(self, v, S):
        while self.US.distance() > S:
            self.left_motor.run(v)
            self.right_motor.run(v)
        self.left_motor.stop()
        self.right_motor.stop()

    def ride_S_lower(self, v, S):
        while self.US.distance() < S:
            self.left_motor.run(v)
            self.right_motor.run(v)
        self.left_motor.stop()
        self.right_motor.stop() 

    def ride_line(self, v, thereshold):
        while self.LS_left.reflection() > thereshold:
            self.left_motor.run(v)
            self.right_motor.run(v)
        self.left_motor.stop()
        self.right_motor.stop() 

    def curve_v(self, v_left, v_right, mm):
        deg_sum = self.left_motor.angle()+self.right_motor.angle()
        while abs(deg_sum - self.left_motor.angle() - self.right_motor.angle())/2 < self.mm_to_deg(mm):
            self.left_motor.run(v_left)
            self.right_motor.run(v_right)
        self.left_motor.stop()
        self.right_motor.stop() 

    def curve_r(self, v, r, mm):
        value = (r+self.S/2)/(r-self.S/2)
        v_right = v
        v_left = v_right*value
        deg_sum = self.left_motor.angle()+self.right_motor.angle()
        while abs(deg_sum - self.left_motor.angle() - self.right_motor.angle())/2 < self.mm_to_deg(mm):
            self.left_motor.run(v_left)
            self.right_motor.run(v_right)
        self.left_motor.stop()
        self.right_motor.stop() 


    def curve_angle(self, v, r, angle):
        value = (r+self.S/2)/(r-self.S/2)
        v_right = v
        v_left = v_right*value
        deg_sum = self.left_motor.angle()+self.right_motor.angle()
        mm=angle/180*3.1415*r
        while abs(deg_sum - self.left_motor.angle() - self.right_motor.angle())/2 < self.mm_to_deg(mm):
            self.left_motor.run(v_left)
            self.right_motor.run(v_right)
        self.left_motor.stop()
        self.right_motor.stop() 




    def empty_pid_reg_right(self,v,kp,ki,kd, offset = 0):
        error = self.gray - self.LS_right.reflection()
        self.error_i = error + self.error_i
        error_d = error - self.last_error

        p = kp * error
        i = ki * self.error_i
        d = kd * error_d

        u = p + i + d
        
        self.last_error = error

        self.left_motor.run(v + u)
        self.right_motor.run(v - u + offset)
        
        wait(10)


    def empty_pid_reg_left(self,v,kp,ki,kd, offset = 0):
        error = self.LS_left.reflection() - self.gray
        self.error_i = error + self.error_i
        error_d = error - self.last_error

        p = kp * error
        i = ki * self.error_i
        d = kd * error_d

        u = p + i + d
        
        self.last_error = error

        self.left_motor.run(v + u)
        self.right_motor.run(v - u + offset)
        
        wait(10)


    def empty_pid_reg(self,v,kp,ki,kd, offset = 0):
        error = self.LS_left.reflection() - self.LS_right.reflection()
        self.error_i = error + self.error_i
        error_d = error - self.last_error

        p = kp * error
        i = ki * self.error_i
        d = kd * error_d

        u = p + i + d
        
        self.last_error = error

        self.left_motor.run(v + u)
        self.right_motor.run(v - u + offset)
        
        wait(10)

    def empty_pid_reg_inversion(self,v,kp,ki,kd, offset = 0):
        error = self.LS_right.reflection() - self.LS_left.reflection()
        self.error_i = error + self.error_i
        error_d = error - self.last_error

        p = kp * error
        i = ki * self.error_i
        d = kd * error_d

        u = p + i + d
        
        self.last_error = error

        self.left_motor.run(v + u)
        self.right_motor.run(v - u + offset)
        
        wait(10)


    def pid_reg_inversion(self,v,kp,ki,kd,n,ride=40):
        self.error_i = 0
        self.last_error = 0
        for _ in range(n):
            while self.LS_right.reflection() < 30 or self.LS_left.reflection() < 30:
                self.empty_pid_reg_inversion(v,kp,ki,kd)
            if ride:
                self.ride_mm(v, ride)
            self.motors_stop()


    def pid_reg(self,v,kp,ki,kd,n,ride=40):
        self.error_i = 0
        self.last_error = 0
        for _ in range(n):
            while self.LS_right.reflection() > 30 or self.LS_left.reflection() > 30:
                self.empty_pid_reg(v,kp,ki,kd)
            if ride:
                self.ride_mm(v, ride)
            self.motors_stop()


    def pid_reg_deg(self,v,kp,ki,kd,deg):
        old_deg = self.left_motor.angle()
        self.error_i = 0
        self.last_error = 0
        while self.left_motor.angle() - old_deg < deg:
            self.empty_pid_reg(v,kp,ki,kd)
        self.left_motor.stop()
        self.right_motor.stop()


    def pid_reg_deg_right(self,v,kp,ki,kd,deg):
        old_deg = self.left_motor.angle()
        self.error_i = 0
        self.last_error = 0
        while self.left_motor.angle() - old_deg < deg:
            self.empty_pid_reg_right(v,kp,ki,kd)
        self.left_motor.stop()
        self.right_motor.stop()


    def pid_reg_deg_left(self,v,kp,ki,kd,deg):
        old_deg = self.left_motor.angle()
        self.error_i = 0
        self.last_error = 0
        while self.left_motor.angle() - old_deg < deg:
            self.empty_pid_reg_left(v,kp,ki,kd)
        self.left_motor.stop()
        self.right_motor.stop()


    def pid_reg_deg(self,v,kp,ki,kd,deg):
        old_deg = self.left_motor.angle()
        self.error_i = 0
        self.last_error = 0
        while self.left_motor.angle() - old_deg < deg:
            self.empty_pid_reg(v,kp,ki,kd)
        self.left_motor.stop()
        self.right_motor.stop()


    def pid_reg_4sensors(self,v,kp,ki,kd,n):
        self.error_i = 0
        self.last_error = 0
        for _ in range(n):
            while self.LS_right.reflection() > 40 or self.LS_left.reflection() > 40:
                
                error = (self.LS_side_right.reflection() + self.LS_right.reflection()) - (self.LS_left.reflection() + self.LS_side_left.reflection())
                # if abs(error_i) > abs(error + error_i):
                    # error_i = error * 2 + error_i
                # else:
                self.error_i = error + self.error_i
                error_d = error - self.last_error

                p = kp * error
                i = ki * self.error_i
                d = kd * error_d

                u = p + i + d
                
                self.last_error = error

                self.left_motor.run(v + u)
                self.right_motor.run(v - u)
                
                wait(10)
            self.left_motor.run(300)
            self.right_motor.run(300)
            wait(100)
            self.left_motor.stop()
            self.right_motor.stop()


    def filter(self, data, n):
        self.filtered.append(data)
        if len(filtered) >= 2*n:
            self.filtered = self.filtered[-1*n:]
            # ev3.screen.print(filtered[:n])
        return sorted(self.filtered[:n])[n//2]


    def border(self, val, min, max):
        if min > val: val = min
        if max < val: val = max

        return val


    def empty_pid_reg_wall(self,v,kp,ki,kd,S):
        error = self.border(self.US.distance(), 10, S*2) - S
        self.error_i = error + self.error_i
        error_d = error - self.last_error

        p = kp * error
        i = ki * self.error_i
        d = kd * error_d

        u = p + i + d
        
        self.last_error = error

        self.left_motor.run(v + u)
        self.right_motor.run(v - u)
        
        wait(10)


    def wall_ride_line(self,v,kp,ki,kd,S):
        self.error_i = 0
        self.last_error = 0
        while self.LS_left.reflection() > 30:
            self.empty_pid_reg_wall(v,kp,ki,kd,S)
        self.motors_stop()


    def wall_ride_S(self,v,kp,ki,kd,S,end):
        self.error_i = 0
        self.last_error = 0
        while self.US.distance() < end:
            self.empty_pid_reg_wall(v,kp,ki,kd,S)
        self.motors_stop()

    def wall_ride_wall(self,v,kp,ki,kd,S,end):
        self.error_i = 0
        self.last_error = 0
        while self.US2.distance() > end:
            self.empty_pid_reg_wall(v,kp,ki,kd,S)
        self.motors_stop()

    def enter(self):
        while not Button.CENTER in self.buttons.pressed():
            if Button.UP in self.buttons.pressed():
                self.number += 1
                wait(200)
            if Button.DOWN in self.buttons.pressed():
                self.number -= 1
                wait(200)
            if Button.RIGHT in self.buttons.pressed():
                self.number += 10
            if Button.LEFT in self.buttons.pressed():
                self.number -= 10
                wait(50)
            if self.Button1.pressed():
                self.number = self.number * 10
                wait(400)
            self.screen.clear()
            self.screen.print(self.number)
            wait(10)


    def tank_turn(self, degrees):
        deg = self.left_motor.angle()
        while abs(self.left_motor.angle() - deg) < abs(degrees) * 180 / self.d:
            self.left_motor.run(300 * degrees // abs(degrees))
            self.right_motor.run(-300 * degrees // abs(degrees))
        self.left_motor.stop()
        self.right_motor.stop()


    def turn(self, n, v, t, back=0):
        for _ in range(n):
            self.left_motor.run(v)
            self.right_motor.run(-v)
            wait(t)
            while ((self.LS_right.reflection() * (v > 0)) + (self.LS_left.reflection() * (v < 0))) > 16:
                self.left_motor.run(v)
                self.right_motor.run(-v)
            self.left_motor.run(-v)
            self.right_motor.run(v)
            wait(back)
            self.motors_stop()


    def anti_turn(self, n, v, t):
        for _ in range(n):
            self.left_motor.run(v)
            self.right_motor.run(-v)
            wait(t)
            while (self.LS_left.reflection() * (v > 0)) + (self.LS_right.reflection() * (v < 0)) > 20:
                self.left_motor.run(v)
                self.right_motor.run(-v)
            self.motors_stop()


    def open(self):
        self.mid_motor.run_until_stalled(300, duty_limit=50)


    def close(self):
        self.mid_motor.run_until_stalled(-300, duty_limit=50)
        self.mid_motor.hold()



    # def per_count(self,v,kp,ki,kd,ride=500):
    #     pers_list=[]
    #     self.error_i = 0
    #     self.last_error = 0
    #     n=0
    #     while (self.LS_center.reflection() < 75 or self.LS_right.reflection() < 75 or self.LS_left.reflection() < 75): #  or cube detected
    #         old_deg = left_motor.angle()
    #         while (self.LS_right.reflection() > 40 or self.LS_left.reflection() > 40) and (self.LS_center.reflection() < 75 or self.LS_right.reflection() < 75 or self.LS_left.reflection() < 75):
    #             empty_pid_reg(v,kp,ki,kd)
    #             if 30 < LS_center.reflection() < 40 and 30 < LS_right.reflection() < 40 and 30 < LS_left.reflection() < 40: #finish color between (30) and (40) - change
    #                 return [-1, -1]
    #         if ride:
    #             self.left_motor.run(300)
    #             self.right_motor.run(300)
    #             wait(ride)
    #         n+=1
    #         pers_list1.append(self.left_motor.angle() - old_deg)
    #         self.left_motor.stop()
    #         self.right_motor.stop()
    #     self.left_motor.stop()
    #     self.right_motor.stop()
    #     return [pers_list, n]


    def main(self):
        # -----------------------------------------------КОД-----------------------------------------------
        












# конец класса



robot = Robot()
robot.main()

