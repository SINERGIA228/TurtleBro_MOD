# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import threading
import serial  # Импортируем библиотеку для работы с последовательным портом

# Флаг для завершения работы программы
should_exit = False

# Настройка последовательного порта для связи с Arduino
arduino_port = '/dev/ttyUSB0'  # Укажите ваш последовательный порт
baud_rate = 9600  # Скорость передачи данных
arduino = serial.Serial(arduino_port, baud_rate)

def input_thread():
    global should_exit
    input("Нажмите Enter, чтобы остановить робота и завершить выполнение программы...\n")
    should_exit = True

# Инициализация узла ROS с именем "robot_control"
rospy.init_node(name="robot_control")

# Создаем Publisher для управления роботом
cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def move_robot(direction):
    twist = Twist()  # Создаем объект типа Twist для управления движением

    # Настраиваем линейную и угловую скорости в зависимости от направления
    if direction == '1':  # Вращение против часовой стрелки
        twist.linear.x = 0.25  # Линейная скорость
        twist.angular.z = 0.5  # Угловая скорость
    elif direction == '2':  # Вращение по часовой стрелке
        twist.linear.x = 0.25  # Линейная скорость
        twist.angular.z = -0.5  # Угловая скорость
    else:  # Остановка
        twist.linear.x = 0.0
        twist.angular.z = 0.0

    return twist

if __name__ == '__main__':
    try:
        # Запуск потока для ввода с клавиатуры
        thread = threading.Thread(target=input_thread)
        thread.start()

        while not rospy.is_shutdown() and not should_exit:
            if arduino.in_waiting > 0:  # Проверяем, есть ли данные от Arduino
                direction = arduino.readline().decode('utf-8').strip()  # Читаем данные и декодируем
                twist = move_robot(direction)  # Получаем команды движения в зависимости от направления
                cmd_pub.publish(twist)  # Публикуем команду движения
            rospy.sleep(0.1)  # Небольшая задержка для регулирования частоты публикации

    except rospy.ROSInterruptException:
        pass  # Завершение выполнения при исключении
    finally:
        # Остановка робота перед завершением
        cmd_pub.publish(Twist())  # Публикуем команду остановки
        print("Завершение программы.")