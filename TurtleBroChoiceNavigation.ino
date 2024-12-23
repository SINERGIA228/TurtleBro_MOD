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