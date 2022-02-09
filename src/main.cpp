#include <Arduino.h>
#include <movingAvg.h>
/*
|-----------|   /-----\          |-----------|                    |-----------|
|  sygnał   |=>|   +   |=>uchyb=>|    PID    |=>sygnał sterujący=>|  silnik   |=>feedback (potencjometr/enkoder)
|-----------|   \-----/          |-----------|                    |-----------|
                  -feedback
*/
//definicje pinów
#define outpin 22
#define senspin 32
#define IN1 19
#define IN2 23

//wartości dla inicjalizacji PWM
const int freq = 10; //częstotliwość pwm
const int channel = 0;  //kanał pwm
const int resolution = 8; //rozdzielczość (0-255)
//wartości sensora (potencjometru)
int rawSensor = 0;  //czyste dane z sensora
int filteredSensor = 0; //dane przefiltrowane filtrem MA
int sensorScaled = 0; //dane przeskalowane do 0-255
//wartości czasowe
unsigned long t = 0;  //czas od początku programu (mikrosekundy)
unsigned int dt = 0;  //różnica czasu
unsigned long prevt = 0;  //poprzedni czas
//wartości do pętli PID
float kp = 1; //współczynnik KP
float ki = 0; //współczynnik KI
float kd = 0; //współczynnik KD
int inputPosition = 0;  //pozycja zadana
int pwm = 0;  //wyjście do elementu sterującego silnikiem
int error = 0;  //uchyb
int totalError = 0;  //kumulowany uchyb
int prevError = 0; //poprzedni uchyb
bool direction = 0; //kierunek obrotu silnika

//konstruktor filtru MA
movingAvg sensor(3); 

void setup() {

  //inicjalizacja pwm
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(outpin, channel);

  //inicjalizacja pinów wyjściowych
  pinMode(outpin, OUTPUT);
  pinMode(senspin, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  //inicjalizacja filtru MA
  sensor.begin();

  //inicjalizacja portu szeregowego
  Serial.begin(9600);
}


// funkcja wykonująca PID, zapisuje wartość w zmiennej pwm, jako wejście jest sygnał zadany - pozycja od 0 do 255
void PID (int inputPosition) {


  t = micros();  //czas od uruchomienia

  dt = t - prevt; //czas delta

  sensorScaled = map(filteredSensor, 0, 4095, 0, 255); //skalowanie sensora do wartości 8-bitowej

  error = inputPosition - sensorScaled; //liczenie uchybu (węzeł na wejściu pętli sterowania)
  totalError += error; //kumulacyjny uchyb (do całkowania)

  pwm = kp*error + ki*totalError*dt + kd*(error-prevError)/dt; //PID (k + całka + różniczka)
  
  if (error > 0){ //zmiana kierunku silnika w zależności od tego w którą stronę jest uchyb
    direction = 0;
  } else {
    direction = 1;
  }

  pwm = constrain(abs(pwm), 0, 255); // wyeliminowanie większych wartości niż jest w stanie podać pin (255 to wartość odpowiadająca 100% wypełnieniu sygnału) oraz wzięcie wartości bezwzględnej bo kierunek regulujemy przez direction

  prevError = error;  // zapisanie poprzednich wartości
  prevt = t;
}

void loop() { //główna pętla

//filtr wejścia z sensora i debugging przez port szeregowy
rawSensor = analogRead(senspin);
filteredSensor = sensor.reading(rawSensor);
Serial.print("Filtered sensor data: ");
Serial.println(filteredSensor);
Serial.print("direction: ");
Serial.println(direction);

//wykonanie cyklu PID
PID(120);

//wysłanie sygnału do elementu wykonawczego (sterownika silnika)
ledcWrite(channel, pwm); // wysłanie prędkości
digitalWrite(IN1, direction); // wysłanie kierunku
digitalWrite(IN2, !direction);

//debug
Serial.print("pwm: ");
Serial.println(pwm);
Serial.println();

delay(10);

}