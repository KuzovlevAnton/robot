#!/usr/bin/env pybricks-micropython
from math import floor, ceil
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxServer, TextMailbox


# блок нужно переименовать в pult и запустить первым
# пример на управлении роботом с помощью кнопок

# здесь надо инициализировать все моторы и датчики
ev3 = EV3Brick()


# создание сервера и почтового ящика
server = BluetoothMailboxServer()
mbox = Mailbox('ev3_control', server)

# ожидание подключения
print('waiting for connection...')
server.wait_for_connection()
print('connected!')

control_list = [0,0,0,0,0] # сигналы с кнопок или других датчиков/моторов, используемых для управления


def control_update():
    global control_list
    # обновление всех сигналов, пример на кнопках на хабе
    control_list = [int(Button.UP in self.ev3.buttons.pressed()),
    int(Button.LEFT in self.ev3.buttons.pressed()),
    int(Button.CENTER in self.ev3.buttons.pressed()),
    int(Button.RIGHT in self.ev3.buttons.pressed()),
    int(Button.DOWN in self.ev3.buttons.pressed())]


while True:
    control_update()
    mbox.send(control_list)