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

        self.close()
        self.closed_enc = self.mid_motor.angle()
        self.open()

        while not Button.CENTER in self.ev3.buttons.pressed():
            # print(self.LS_left.reflection(),self.LS_right.reflection(),self.LS_center.reflection(),self.LS_left.reflection()+self.LS_right.reflection()+self.LS_center.reflection())
            ...

        # while not self.Button1.pressed():
        #     ...
        


    def init_hardware(self):
        self.ev3 = EV3Brick()

        self.left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.B)

        self.mid_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)

        self.deg_enc = self.left_motor.angle()


        self.LS_right = ColorSensor(Port.S2)
        self.LS_left = ColorSensor(Port.S3)
    
        # self.LS_cube = ColorSensor(Port.S4)
        # self.LS_center = ColorSensor(Port.S4)

        # self.Button1 = TouchSensor(Port.S1)

        # self.LS_side_right = ColorSensor(Port.S4)
        # self.LS_side_left = ColorSensor(Port.S1)

        self.US = UltrasonicSensor(Port.S4)

        # if self.LS_right.color() == Color.GREEN

        # self.gyro_sensor = GyroSensor(Port.S4)



        self.robot = DriveBase(self.left_motor, self.right_motor, wheel_diameter=62.4, axle_track=185)
        self.robot.settings(straight_speed=self.default_speed)


    def init_vars(self):

        self.timer = StopWatch()

        self.S = 165
        self.d = 62.4

        self.min_sensor_data = 3
        self.max_sensor_data = 64

        self.gray = 45

        self.default_speed = 300

        self.number = 0

        self.last_error = 0
        self.error_i = 0

        self.n = 0

        self.filtered = [0, 0, 0]

        self.colors_int = {
            None: 0,
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

    
    def calibrate(self):
        """"""
        v=100
        degrees=720
        max_ = 0
        min_ = 100
        deg = self.left_motor.angle()
        while abs(self.left_motor.angle() - deg) < abs(degrees) * self.S / self.d:
            self.left_motor.run(v * degrees // abs(degrees))
            self.right_motor.run(-v * degrees // abs(degrees))
            if max(self.LS_right.reflection(), self.LS_left.reflection(), self.LS_center.reflection()) > max_:
                max_ = max(self.LS_right.reflection(), self.LS_left.reflection(), self.LS_center.reflection())
            if min(self.LS_right.reflection(), self.LS_left.reflection(), self.LS_center.reflection()) < min_:
                min_ = min(self.LS_right.reflection(), self.LS_left.reflection(), self.LS_center.reflection())
            wait(50)
        self.left_motor.stop()
        self.right_motor.stop()
        self.min_sensor_data = min_
        self.max_sensor_data = max_
        print(min_, max_)
    

    def smooth_run(self, v_max, v_min, a_max, deg_remains):
        speed=(self.left_motor.speed()+self.right_motor.speed())/2
        max_deg=a_max*10*(speed/(a_max*20))**2*1.4

        if abs(max_deg) >= abs(deg_remains):
            if abs(speed-a_max) > abs(v_min):
                self.left_motor.run(speed-a_max)
                self.right_motor.run(speed-a_max)
            else:
                self.left_motor.run(v_min)
                self.right_motor.run(v_min)
        else:
            if abs(speed + a_max) < abs(v_max):
                self.left_motor.run(speed + a_max)
                self.right_motor.run(speed + a_max)
            else:
                self.left_motor.run(v_max)
                self.right_motor.run(v_max)
        wait(50)
    
    def smooth_ride_mm(self, v_max, v_min, a_max, mm):
        deg = self.left_motor.angle()
        while abs(deg - self.left_motor.angle()) < self.mm_to_deg(mm):
            self.smooth_run(v_max, v_min, a_max, self.mm_to_deg(mm)-abs(deg - self.left_motor.angle()))
        self.left_motor.stop()
        self.right_motor.stop()


    def smooth_turn(self, v_max, v_min, a_max, deg_remains):
        speed=(-self.left_motor.speed()+self.right_motor.speed())/2
        max_deg=a_max*10*(speed/(a_max*20))**2*1.4

        if abs(max_deg) >= abs(deg_remains):
            if abs(speed-a_max) > abs(v_min):
                self.left_motor.run(-speed+a_max)
                self.right_motor.run(speed-a_max)
            else:
                self.left_motor.run(-v_min)
                self.right_motor.run(v_min)
        else:
            if abs(speed + a_max) < abs(v_max):
                self.left_motor.run(-speed - a_max)
                self.right_motor.run(speed + a_max)
            else:
                self.left_motor.run(-v_max)
                self.right_motor.run(v_max)
        wait(50)

    
    def smooth_tank_turn(self, v_max, v_min, a_max, degrees):
        deg = self.left_motor.angle()
        while abs(self.left_motor.angle() - deg) < abs(degrees) * self.S / self.d:
            self.smooth_turn(-v_max * degrees // abs(degrees), -v_min * degrees // abs(degrees), -a_max * degrees // abs(degrees), self.deg_to_mm(abs(degrees) * 180 / self.d - abs(self.left_motor.angle() - deg)))
        self.left_motor.stop()
        self.right_motor.stop()

    # def smooth_run_2(self, v_max_left, v_max_right, v_min, a_max, deg_remains):
        # speed=(self.left_motor.speed()+self.right_motor.speed())/2
        # max_deg=a_max*10*(speed/(a_max*20))**2*1.4

        # if max_deg >= deg_remains:
        #     if speed-a_max > v_min:
        #         self.left_motor.run(speed-a_max)
        #         self.right_motor.run(speed-a_max)
        #     else:
        #         self.left_motor.run(v_min)
        #         self.right_motor.run(v_min)
        # else:
        #     if speed + a_max < v_max:
        #         self.left_motor.run(speed + a_max)
        #         self.right_motor.run(speed + a_max)
        #     elif speed + a_max >= v_max:
        #         self.left_motor.run(v_max)
        #         self.right_motor.run(v_max)
        # wait(50)

    # def right_smooth_run(self, v_max, a_max, deg_remains, kp, ki, kd):

        
    #     error = deg_remains
    #     self.error_i = error + self.error_i
    #     error_d = error - self.last_error

    #     p = kp * error
    #     i = ki * self.error_i
    #     d = kd * error_d

    #     u = p + i + d
        
    #     self.last_error = error

    #     if u > v_max: u = v_max

    #     self.right_motor.run(u)
        
    #     wait(10)

        
    
    # def left_smooth_run(self, v_max, a_max, deg_remains, kp, ki, kd):

        
    #     error = deg_remains
    #     self.error_i = error + self.error_i
    #     error_d = error - self.last_error

    #     p = kp * error
    #     i = ki * self.error_i
    #     d = kd * error_d

    #     u = p + i + d
        
    #     self.last_error = error

    #     if u > v_max: u = v_max

    #     self.left_motor.run(u)
        
    #     wait(10)
        
        
    def change_range(self, value, start1, end1, start2, end2):
        k1 = end1 - start1
        k2 = end2 - start2
        return (value - start1) / k1 * k2 + start2

    
    def beep(self): # гудок
        self.ev3.speaker.beep(300)


    def mm_to_deg(self, mm): # перевод из миллиметров в градусы
        return mm * 360 / self.d / 3.1415

    def deg_to_mm(self, deg): # перевод из градусов в миллиметры
        return deg / 360 * self.d * 3.1415


    def motors_stop(self): # остановка моторов
        self.left_motor.stop()
        self.right_motor.stop()



    # езда
    def ride_mm(self, v, mm): # по миллиметрам
        # deg = self.left_motor.angle()
        # while abs(deg - self.left_motor.angle()) < self.mm_to_deg(mm):
        #     self.left_motor.run(v*abs(1-abs(((self.mm_to_deg(mm)/2)-abs(deg - self.left_motor.angle()))/(self.mm_to_deg(mm)/2)))**0.5+50)
        #     self.right_motor.run(v*abs(1-abs(((self.mm_to_deg(mm)/2)-abs(deg - self.left_motor.angle()))/(self.mm_to_deg(mm)/2)))**0.5+50)
        # self.left_motor.stop()
        # self.right_motor.stop()

        deg = self.left_motor.angle()
        while abs(deg - self.left_motor.angle()) < self.mm_to_deg(mm):
            self.left_motor.run(v)
            self.right_motor.run(v)
        self.left_motor.stop()
        self.right_motor.stop()

    def ride_deg(self, v, degrees): # по градусам
        deg = self.left_motor.angle()
        while abs(deg - self.left_motor.angle()) < degrees:
            self.left_motor.run(v)
            self.right_motor.run(v)
        self.left_motor.stop()
        self.right_motor.stop()

    def ride_S_higher(self, v, S): # по расстояния (пока нет стенки)
        while self.US.distance() > S:
            self.left_motor.run(v)
            self.right_motor.run(v)
        self.left_motor.stop()
        self.right_motor.stop()

    def ride_S_lower(self, v, S): # по расстояния (пока есть стенка)
        while self.US.distance() < S:
            self.left_motor.run(v)
            self.right_motor.run(v)
        self.left_motor.stop()
        self.right_motor.stop() 

    def ride_line(self, v, thereshold): # до линии
        while self.LS_left.reflection() > thereshold:
            self.left_motor.run(v)
            self.right_motor.run(v)
        self.left_motor.stop()
        self.right_motor.stop() 


    # езда по кривой
    def curve_v(self, v_left, v_right, mm): # по 2 скоростям
        deg_sum = self.left_motor.angle()+self.right_motor.angle()
        while abs(deg_sum - self.left_motor.angle() - self.right_motor.angle())/2 < self.mm_to_deg(mm):
            self.left_motor.run(v_left)
            self.right_motor.run(v_right)
        self.left_motor.stop()
        self.right_motor.stop() 

    def curve_r(self, v, r, mm): # по радиусу
        value = (r+self.S/2)/(r-self.S/2)
        v_right = v
        v_left = v_right*value
        deg_sum = self.left_motor.angle()+self.right_motor.angle()
        while abs(deg_sum - self.left_motor.angle() - self.right_motor.angle())/2 < self.mm_to_deg(mm):
            self.left_motor.run(v_left)
            self.right_motor.run(v_right)
        self.left_motor.stop()
        self.right_motor.stop() 

    def curve_angle(self, v, r, angle): # по углу
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


    # повороты
    def tank_turn(self, degrees): # танковый по углу
        deg = self.left_motor.angle()
        while abs(self.left_motor.angle() - deg) < abs(degrees) * self.S / self.d:
            self.left_motor.run(300 * degrees // abs(degrees))
            self.right_motor.run(-300 * degrees // abs(degrees))
        self.left_motor.stop()
        self.right_motor.stop()

    def turn(self, n, v, t, back=0): # по линиям (датчик по направлению поворота (ближний))
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

    def anti_turn(self, n, v, t):# по линиям (датчик дальний)
        for _ in range(n):
            self.left_motor.run(v)
            self.right_motor.run(-v)
            wait(t)
            while (self.LS_left.reflection() * (v > 0)) + (self.LS_right.reflection() * (v < 0)) > 20:
                self.left_motor.run(v)
                self.right_motor.run(-v)
            self.motors_stop()
    
    
    def both_turn(self, n, v, t, k=0.25):
        last_enc_between=0
        self.turn(n, v, t)
        last_enc_between=self.left_motor.angle()
        self.anti_turn(1, v, t)
        last_enc_between=last_enc_between-self.left_motor.angle()
        deg = self.left_motor.angle()
        while abs(self.left_motor.angle() - deg) < abs(last_enc_between)*k:
            self.left_motor.run(-v)
            self.right_motor.run(v)
        self.left_motor.stop()
        self.right_motor.stop()
        


    # пид регуляторы без цикла
    def empty_pid_reg_right(self,v,kp,ki,kd, offset = 0): # по правому датчику
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

    def empty_pid_reg_left(self,v,kp,ki,kd, offset = 0): # по левому датчику
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

    def empty_pid_reg(self,v,kp,ki,kd, offset = 0): # по 2 датчикам
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

    def empty_pid_reg_inversion(self,v,kp,ki,kd, offset = 0): # инверсия по 2 датчикам
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



    # пид регуляторы
    def pid_reg(self,v,kp,ki,kd,n,ride=20): # по перекрёсткам
        self.error_i = 0
        self.last_error = 0
        for _ in range(n):
            while self.LS_right.reflection() > 30 or self.LS_left.reflection() > 30:
                self.empty_pid_reg(v,kp,ki,kd)
            if ride:
                self.ride_mm(v, ride)
            self.motors_stop()

    def pid_reg_inversion(self,v,kp,ki,kd,n,ride=20): # инверсия
        self.error_i = 0
        self.last_error = 0
        for _ in range(n):
            while self.LS_right.reflection() < 30 or self.LS_left.reflection() < 30:
                self.empty_pid_reg_inversion(v,kp,ki,kd)
            if ride:
                self.ride_mm(v, ride)
            self.motors_stop()

    def pid_reg_deg(self,v,kp,ki,kd,deg): # по энкодеру
        old_deg = self.left_motor.angle()
        self.error_i = 0
        self.last_error = 0
        while self.left_motor.angle() - old_deg < deg:
            self.empty_pid_reg(v,kp,ki,kd)
        self.left_motor.stop()
        self.right_motor.stop()

    def pid_reg_deg_right(self,v,kp,ki,kd,deg): # по энкодеру правый датчик
        old_deg = self.left_motor.angle()
        self.error_i = 0
        self.last_error = 0
        while self.left_motor.angle() - old_deg < deg:
            self.empty_pid_reg_right(v,kp,ki,kd)
        self.left_motor.stop()
        self.right_motor.stop()

    def pid_reg_deg_left(self,v,kp,ki,kd,deg): # по энкодеру левый датчик
        old_deg = self.left_motor.angle()
        self.error_i = 0
        self.last_error = 0
        while self.left_motor.angle() - old_deg < deg:
            self.empty_pid_reg_left(v,kp,ki,kd)
        self.left_motor.stop()
        self.right_motor.stop()

    def pid_reg_4sensors(self,v,kp,ki,kd,n): # по 4 датчикам
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


    # работа с уз
    def filter(self, data, n): # медианный фильтр
        self.filtered.append(data)
        if len(self.filtered) >= 2*n:
            self.filtered = self.filtered[-1*n:]
            # ev3.screen.print(filtered[:n])
        return sorted(self.filtered[:n])[n//2]

    def border(self, val, min, max): # граница
        if min > val: val = min
        if max < val: val = max

        return val


     # пид регулятор по стене
    def empty_pid_reg_wall(self,v,kp,ki,kd,S): # без цикла
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


    def wall_ride_line(self,v,kp,ki,kd,S): # до линии
        self.error_i = 0
        self.last_error = 0
        while self.LS_left.reflection() > 30:
            self.empty_pid_reg_wall(v,kp,ki,kd,S)
        self.motors_stop()

    def wall_ride_S(self,v,kp,ki,kd,S,end): # до конца стены
        self.error_i = 0
        self.last_error = 0
        while self.US.distance() < end:
            self.empty_pid_reg_wall(v,kp,ki,kd,S)
        self.motors_stop()

    def wall_ride_wall(self,v,kp,ki,kd,S,end): # до стены перед роботом
        self.error_i = 0
        self.last_error = 0
        while self.US2.distance() > end:
            self.empty_pid_reg_wall(v,kp,ki,kd,S)
        self.motors_stop()


    # ввод и вывод данных
    def enter(self): # ввод числа
        while not Button.CENTER in self.ev3.buttons.pressed():
            if Button.UP in self.ev3.buttons.pressed():
                self.number += 1
                wait(200)
            if Button.DOWN in self.ev3.buttons.pressed():
                self.number -= 1
                wait(200)
            if Button.RIGHT in self.ev3.buttons.pressed():
                self.number += 10
            if Button.LEFT in self.ev3.buttons.pressed():
                self.number -= 10
                wait(50)
            if self.Button1.pressed():
                self.number = self.number * 10
                wait(400)
            self.ev3.screen.clear()
            self.ev3.screen.print(self.number)
            wait(10)

    # высота экрана 128 (0-127), ширина 178 (0-177), (0; 0) - в левом верхнем углу
    def print(self, data): # вывод текста на дисплей
        self.ev3.screen.print(data)

    def screen_draw_line(self, x1, x2, y1, y2, widht): # вывод линии
        self.ev3.screen.draw_line(x1, x2, y1, y2, widht)

    def screen_draw_box(self, x1, y1, x2, y2, r, fill): # вывод прямоугольника
        self.ev3.screen.draw_box(x1, y1, x2, y2, r, fill)

    def screen_draw_circle(self, x, y, r, fill): # вывод круга
        self.ev3.screen.draw_circle(x, y, r, fill)
        self.ev3.screen.draw_box(x1, y1, x2, y2, r, fill)

    def screen_draw_dot(self, x, y): # вывод точки
        self.ev3.screen.draw_line(x, y, x, y, 1)


    # открытие и закрытие захвата
    def open(self): # открытие захвата
        self.mid_motor.run_until_stalled(300, duty_limit=50)

    def close(self): # закрытие захвата
        self.mid_motor.run_until_stalled(-300, duty_limit=75)
        self.mid_motor.hold()

    def close_deg(self, degrees): # закрытие захвата по градусам
        self.mid_motor.run_angle(-300, degrees)
        self.mid_motor.hold()

    def close_color(self):
        self.sensor_motor.run_until_stalled(300, duty_limit=50)

    # работа с объектами
    def objects_count(self, v, S, mm): # подсчёт
        obj = False
        n=0
        deg = self.left_motor.angle()
        while abs(deg - self.left_motor.angle()) < self.mm_to_deg(mm):
            self.left_motor.run(v)
            self.right_motor.run(v)
            if not obj and self.US.distance() < S:
                n += 1
                obj = True
            
            if obj and self.US.distance() > S:
                obj = False
                wait(100)
        self.left_motor.stop()
        self.right_motor.stop()
        return n

    def to_object(self, v, S, n): # езда к n объекту
        obj = False
        n_current=0
        deg = self.left_motor.angle()
        while n_current < n:
            self.left_motor.run(v)
            self.right_motor.run(v)
            if not obj and self.US.distance() < S:
                n_current += 1
                obj = True
            
            if obj and self.US.distance() > S:
                obj = False
                wait(100)
        self.left_motor.stop()
        self.right_motor.stop()
        return self.deg_to_mm(self.left_motor.angle()-deg)

    def to_object_end(self, v, S): # к концу объекта
        obj = False
        deg = self.left_motor.angle()
        while self.US.distance() < S:
            self.left_motor.run(v)
            self.right_motor.run(v)
        wait(10)
        self.left_motor.stop()
        self.right_motor.stop()
        return self.deg_to_mm(self.left_motor.angle()-deg)

    def between_objects(self, v, S): # расстояние между объектами
        self.to_object(v, S, 1)
        self.ride_mm(v, 10)
        self.to_object_end(v, S)
        return self.to_object(v, S, 1)

    def object_width(self, v, S): # ширина объекта
        self.to_object(v, S, 1)
        self.ride_mm(v, 10)
        return self.to_object_end(v, S)

    def object_height(self, h_max): # высота
        return h_max - self.US.distance()

    def object_color(self): # цвет объекта
        return self.LS_cube.color()

    def object_light(self): # свет от объекта
        return self.LS_cube.ambient()
    
    def object_color_int(self): # цвет объекта по номеру
        return self.colors_int[self.LS_cube.color()]
    
    def object_density(self): # "плотность" объекта
        self.open()
        deg = self.mid_motor.angle()
        self.mid_motor.run_until_stalled(-300, duty_limit=70)
        self.mid_motor.hold()
        return self.mid_motor.angle()-deg

    def around_object_to_line(self, r, v): # объезд объекта по расстоянию до него по линии
        while self.LS_left.reflection() > 30:
            if self.US.distance() < r:
                self.left_motor.run(abs(v))
                self.right_motor.run(abs(v))
            if self.US.distance() > r:
                self.left_motor.run(v)
                self.right_motor.run(-1*v)
        self.left_motor.stop()
        self.right_motor.stop() 
    
    def around_object_mm(self, r, v, mm): # объезд объекта по расстоянию до него по миллиметрам
        deg_sum = self.left_motor.angle()+self.right_motor.angle()
        while abs(deg_sum - self.left_motor.angle() - self.right_motor.angle())/2 < self.mm_to_deg(mm):
            if self.US.distance() < r:
                self.left_motor.run(abs(v))
                self.right_motor.run(abs(v))
            if self.US.distance() > r:
                self.left_motor.run(v)
                self.right_motor.run(-1*v)
        self.left_motor.stop()
        self.right_motor.stop() 


    # выравнивание
    def line_align_backward(self, v, thereshold=30): # выравнивание "назад"
        while self.LS_left.reflection() > thereshold or self.LS_right.reflection() > thereshold:
            self.left_motor.run(-v)
            self.right_motor.run(-v)
            if self.LS_left.reflection() < thereshold:
                while self.LS_left.reflection() < thereshold:
                    self.left_motor.run(v)
                    self.right_motor.run(v/2)
                wait(100)
            if self.LS_right.reflection() < thereshold:
                while self.LS_right.reflection() < thereshold:
                    self.left_motor.run(v/2)
                    self.right_motor.run(v)
                wait(100)
            wait(10)
        self.motors_stop()
    
    def line_align_forward(self, v, thereshold=30): # выравнивание "вперёд"
        while self.LS_left.reflection() > thereshold or self.LS_right.reflection() > thereshold:
            self.left_motor.run(v)
            self.right_motor.run(v)
            if self.LS_left.reflection() < thereshold:
                while self.LS_left.reflection() < thereshold:
                    self.left_motor.run(-v)
                    self.right_motor.run(-v/2)
                wait(100)
            if self.LS_right.reflection() < thereshold:
                while self.LS_right.reflection() < thereshold:
                    self.left_motor.run(-v/2)
                    self.right_motor.run(-v)
                wait(100)
            wait(10)
        self.motors_stop()


    # штрихкоды
    def per_count(self,v,total_len,thereshold=30, round_=1, add=0): # считывание чб штрихкода по длинам
        pers_list_black=[]
        pers_list_white=[]
        self.line_align_forward(50, thereshold)
        start_deg = self.left_motor.angle()
        while self.left_motor.angle()-start_deg < self.mm_to_deg(total_len):
            old_deg = self.left_motor.angle()
            while self.LS_left.reflection() < thereshold:
                self.left_motor.run(v)
                self.right_motor.run(v)
            pers_list_black.append(floor(self.deg_to_mm(self.left_motor.angle()-old_deg)/round_+add)*round_)
            old_deg = self.left_motor.angle()
            if self.left_motor.angle()-start_deg < self.mm_to_deg(total_len):
                while self.LS_left.reflection() >= thereshold and self.left_motor.angle()-start_deg < self.mm_to_deg(total_len):
                    self.left_motor.run(v)
                    self.right_motor.run(v)
                pers_list_white.append(floor(self.deg_to_mm(self.left_motor.angle()-old_deg)/round_+add)*round_)
        self.motors_stop()
        return (pers_list_black, pers_list_white)
    

    
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

    def get_cube(self, dir, n, negative=0):
        if dir:
            # self.tank_turn(90)
            self.both_turn(1, 300*(0.5-negative)*2, 100)
        else:
            # self.tank_turn(-90)
            self.both_turn(1, -300*(0.5-negative)*2, 100)
        
        self.pid_reg_deg(300, 5, 0, 0, self.mm_to_deg(100*n+100))
        self.close()
        
        # self.tank_turn(180)
        self.both_turn(1, 300, 100)
        self.pid_reg(300, 5, 0, 0, 1, 30)
        
        if not dir:
            if self.mid_motor.angle()-self.closed_enc < 10:
                self.tank_turn(-90)
            else:  
                self.tank_turn(90)
            # self.both_turn(1, 300, 100)
        else:
            if self.mid_motor.angle()-self.closed_enc < 10:
                self.tank_turn(90)
            else:  
                self.tank_turn(-90)
            # self.both_turn(1, -300, 100)
        return self.mid_motor.angle()-self.closed_enc

    # def empty_finding(self):
    
    
    def task_cubes_scan(self):
        self.pid_reg(450, 5, 0, 0, 3)
        self.tank_turn(180)
        self.right_cubes = []
        self.left_cubes = []
        self.pid_reg(450, 5, 0, 0, 1, 0)
        self.ride_mm(-450, 35)
        print(self.US.distance())
        self.right_cubes.append(self.US.distance() < 250) # Б4
        self.ride_mm(450, 80)
        self.pid_reg(450, 5, 0, 0, 1, 0)
        self.ride_mm(-450, 35)
        print(self.US.distance())
        self.right_cubes.append(self.US.distance() < 250) # В4
        self.ride_mm(450, 80)
        self.pid_reg(450, 5, 0, 0, 1)
        self.pid_reg(450, 5, 0, 0, 1, 0)
        self.ride_mm(-450, 35)
        print(self.US.distance())
        self.right_cubes.append(self.US.distance() < 250) # Д4
        self.ride_mm(450, 80)
        self.pid_reg(450, 5, 0, 0, 1, 0)
        self.ride_mm(-450, 35)
        print(self.US.distance())
        self.right_cubes.append(self.US.distance() < 250) # Е4
        self.ride_mm(450, 80)
        self.pid_reg(450, 5, 0, 0, 1)
        self.tank_turn(180)
        self.pid_reg(450, 5, 0, 0, 1, 0)
        self.ride_mm(-450, 35)
        print(self.US.distance())
        self.left_cubes.insert(0, self.US.distance() < 250) # Е2
        self.ride_mm(450, 80)
        self.pid_reg(450, 5, 0, 0, 1, 0)
        self.ride_mm(-450, 35)
        print(self.US.distance())
        self.left_cubes.insert(0, self.US.distance() < 250) # Д2
        self.ride_mm(450, 80)
        self.pid_reg(450, 5, 0, 0, 1)
        self.pid_reg(450, 5, 0, 0, 1, 0)
        self.ride_mm(-450, 35)
        print(self.US.distance())
        self.left_cubes.insert(0, self.US.distance() < 250) # В2
        self.ride_mm(450, 80)
        self.pid_reg(450, 5, 0, 0, 1, 0)
        self.ride_mm(-450, 35)
        print(self.US.distance())
        self.left_cubes.insert(0, self.US.distance() < 250) # Б2
        self.ride_mm(450, 80)
        self.pid_reg(450, 5, 0, 0, 1)
        self.pid_reg_deg(450, 5, 0, 0, self.mm_to_deg(350))
        self.close()
        print(self.right_cubes)
        print(self.left_cubes)
        self.tank_turn(180)
        self.pid_reg(450, 5, 0, 0, 1)
        if not self.right_cubes[0]:
            self.tank_turn(90)
            self.pid_reg(450, 5, 0, 0, 1)
            self.ride_mm(450, 70)
            self.open()
            self.ride_mm(-450, 70)
            self.tank_turn(180)
            self.pid_reg(450, 5, 0, 0, 1)
            self.tank_turn(-90)
            self.pid_reg(450, 5, 0, 0, 1)
        elif not self.right_cubes[3]:
            self.pid_reg(450, 5, 0, 0, 6)
            self.tank_turn(90)
            self.pid_reg(450, 5, 0, 0, 1)
            self.ride_mm(450, 70)
            self.open()
            self.ride_mm(-450, 70)
            self.tank_turn(180)
            self.pid_reg(450, 5, 0, 0, 1)
            self.tank_turn(-90)
            self.pid_reg(450, 5, 0, 0, 7)
        else:
            self.pid_reg(450, 5, 0, 0, 3)
            self.tank_turn(90)
            self.pid_reg(450, 5, 0, 0, 1)
            self.ride_mm(450, 70)
            self.open()
            self.ride_mm(-450, 70)
            self.tank_turn(180)
            self.pid_reg(450, 5, 0, 0, 1)
            self.tank_turn(-90)
            self.pid_reg(450, 5, 0, 0, 4)
    
    def task_storage_cubes_arrangement(self):
        self.print("123")
        wait(10000)
        negative=0
        self.empty=[]
        for i in range(len(self.right_cubes)):
            if not self.right_cubes[i]:
                self.empty.append((i+int(i>2), 1))
        for i in range(len(self.left_cubes)):
            if not self.left_cubes[i]:
                self.empty.append((i+int(i>2), -1))
        self.print("".join(self.empty))
        wait(10000)
        self.print(str(self.empty))
        wait(10000)
        self.print(self.empty)
        wait(10000)
        # for d in range(2):
        #     for i in range(3):
        #         if self.get_cube(d, i, negative) < 10:
        #             # self.tank_turn(180)
        #             self.ride_mm(300, 10)
        #             self.pid_reg(350, 5, 0, 0, 1)
        #             self.open()
        #             self.ride_mm(-300, 20)
        #             self.tank_turn(180)
        #             self.pid_reg(350, 5, 0, 0, 1)
        #             # self.tank_turn(90)
        #             # self.both_turn(1, 300, 100)
        #             negative=1
        #         else:
        #             self.ride_mm(300, 10)
        #             self.pid_reg(350, 5, 0, 0, 1)
        #             self.empty_finding()
        #             # self.ride_mm(-300, 20)
        #             # self.tank_turn(180)
        #             # self.pid_reg(350, 5, 0, 0, 1)
        #             negative=1
        # if not negative:
        #     self.tank_turn(180)


    def main(self):
        # -----------------------------------------------КОД-----------------------------------------------
        # while True:
        #     self.close()
        #     print(self.mid_motor.angle()-self.closed_enc)
        #     wait(50)
        #     self.close_deg(-80)
        #     wait(50)
        
        
        
                
        
        
        wait(10000)












# конец класса



robot = Robot()
# robot.main()
robot.tank_turn(360)
# robot.task_cubes_scan()
# robot.task_storage_cubes_arrangement()

wait(10000)
