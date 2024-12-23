Основная страница для работы с TurtleBro -https://github.com/VOLTBRO-create/TurtleBro_Varification
# Управление Роботом через Arduino с Использованием Serial1

## Описание

В этом проекте мы интегрируем управление роботом с помощью кнопок на плате Arduino, используя библиотеку `pyserial` для связи через последовательный порт. Исходный код управляет движением робота по кругу, принимая команды от Arduino, которые будут отправляться в зависимости от нажатий кнопок.

## Цели проекта

- Использовать Arduino для управления маршрутом робота.
- Реализовать взаимодействие между Python и Arduino через последовательный порт.
- Обеспечить возможность остановки робота с помощью клавиатуры.

## Необходимые компоненты

- Плата Arduino (например, Arduino Uno)
- Робот с установленным ROS
- Библиотека `pyserial` для Python
- Кнопки для ввода

## Установка

1. Установите библиотеку `pyserial` с помощью pip:

   ```bash
   pip install pyserial
   ```

2. Подключите Arduino к компьютеру и загрузите соответствующий код на плату.

## Код на Python

Ниже представлен пример кода на Python, который управляет роботом на основе команд, получаемых от Arduino через последовательный порт `Serial1`.

```python
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
```

## Код на Arduino

На стороне Arduino необходимо написать код, который будет считывать нажатия кнопок и отправлять соответствующие команды через последовательный порт `Serial1`. Пример кода для Arduino:

```cpp
void setup() {
    Serial1.begin(9600); // Инициализация Serial1
    pinMode(2, INPUT_PULLUP); // Кнопка 1
    pinMode(3, INPUT_PULLUP); // Кнопка 2
}

void loop() {
    if (digitalRead(2) == LOW) { // Если кнопка 1 нажата
        Serial1.println("1");
        delay(500); // Задержка для предотвращения дребезга
    }
    else if (digitalRead(3) == LOW) { // Если кнопка 2 нажата
        Serial1.println("2");
        delay(500); // Задержка для предотвращения дребезга
    }
}
```

## Запуск проекта

1. Загрузите код на Arduino.
2. Запустите скрипт на Python.
3. Нажимайте кнопки на Arduino, чтобы управлять движением робота.

## Заключение

Теперь ваш робот может управляться с помощью кнопок на Arduino, что делает его более интерактивным и удобным в использовании. Убедитесь, что все компоненты правильно подключены и настроены, прежде чем запускать проект.
