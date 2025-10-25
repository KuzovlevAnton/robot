#!/usr/bin/env pybricks-micropython
from math import floor, ceil
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxClient, TextMailbox



# имя управляющего блока (пульта)
SERVER = 'pult'


# инициализация моторов
ev3 = EV3Brick()
left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)
v=300

# инициализация клиента
client = BluetoothMailboxClient()
mbox = Mailbox('ev3_control', client)

# подключение к серверу
print('establishing connection...')
client.connect(SERVER)
print('connected!')


control_list = [0,0,0,0,0] # получаемые сигналы с кнопок или других датчиков/моторов


while True:
    recived_list = [0,0,0,0,0] # нейтральный сигнал, при котором нет движения
    mbox.wait()
    recived_list = mbox.read()

    try:
        if recived_list[0]:
            left_motor.run(v)
            right_motor.run(v)
        elif recived_list[1]:
            left_motor.run(-v)
            right_motor.run(v)
        elif recived_list[2]:
            ev3.speaker.beep(300)
        elif recived_list[3]:
            left_motor.run(v)
            right_motor.run(-v)
        elif recived_list[4]:
            left_motor.run(-v)
            right_motor.run(-v)
        else:
            left_motor.stop()
            right_motor.stop()

    except exception as e:
        print(e)


