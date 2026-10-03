
/*
LAB5 - Actividad 3
Control PI bidireccional de velocidad a 100 Hz

Referencias por Serial:
20, -20, 50, -50, 0

Serial Plotter:
Referencia, Velocidad y Error

Librerias necesarias (Gestor de Librerias):
TimerOne
MeanFilterLib - Luis Llamas
*/

#include <TimerOne.h>
#include "MeanFilterLib.h"

const uint8_t ENCODER_A = 2;
const uint8_t ENCODER_B = 3;
const uint8_t IN1 = 4;
const uint8_t IN2 = 5;
const uint8_t ENA = 6;

const float CUENTAS_REV = 4741.44;
const float Ts = 0.01;
const unsigned long Ts_us = 10000;

volatile float referencia_RPM = 0.0;

// Reemplazar con las ganancias obtenidas en MATLAB
float Kp = 1.2;
float Ki = 5.0;

/*
Filtro de media movil sobre la velocidad medida.

La medida de velocidad viene de contar pulsos del encoder en
10 ms, por lo que es ruidosa. El filtro entrega el promedio de
las ultimas VENTANA_FILTRO muestras.

Ventana mas grande -> medicion mas suave, pero mas retardo en
el lazo. El retardo es de (VENTANA_FILTRO - 1)/2 muestras,
es decir 3 muestras = 30 ms con VENTANA_FILTRO = 7.
Ese retardo resta margen de fase: si se aumenta la ventana,
hay que revisar la estabilidad del lazo.
*/
const uint8_t VENTANA_FILTRO = 7;
MeanFilter<float> filtro_velocidad(VENTANA_FILTRO);

const uint8_t DIVISOR_PLOTTER = 10;

volatile long cuentas = 0;
volatile float velocidad_RPM = 0.0;
volatile float error_RPM = 0.0;
volatile bool dato_nuevo = false;

long cuentas_anteriores = 0;
float integral = 0.0;
uint8_t contador_plotter = 0;

void encoder_A_ISR() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B))
    cuentas++;
  else
    cuentas--;
}

void encoder_B_ISR() {
  if (digitalRead(ENCODER_A) != digitalRead(ENCODER_B))
    cuentas++;
  else
    cuentas--;
}

void mover_motor(float control) {
  control = constrain(control, -255.0, 255.0);

  if (control > 0.0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, (int)control);
  }
  else if (control < 0.0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, (int)(-control));
  }
  else {
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

void control_PI() {
  long cuentas_actuales = cuentas;
  long delta_cuentas = cuentas_actuales - cuentas_anteriores;

  float velocidad_sin_filtro =
    delta_cuentas * 60.0 / (CUENTAS_REV * Ts);

  // Promedio de las ultimas VENTANA_FILTRO muestras
  float velocidad_filtrada =
    filtro_velocidad.AddValue(velocidad_sin_filtro);

  float referencia = referencia_RPM;
  float error = referencia - velocidad_filtrada;

  float integral_candidata = integral + error * Ts;

  float control_sin_saturar =
    Kp * error + Ki * integral_candidata;

  if ((control_sin_saturar >= -255.0 &&
       control_sin_saturar <= 255.0) ||
      (control_sin_saturar > 255.0 && error < 0.0) ||
      (control_sin_saturar < -255.0 && error > 0.0)) {
    integral = integral_candidata;
  }

  float control = Kp * error + Ki * integral;

  mover_motor(control);

  velocidad_RPM = velocidad_filtrada;
  error_RPM = error;
  dato_nuevo = true;
  cuentas_anteriores = cuentas_actuales;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(30);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  mover_motor(0.0);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoder_B_ISR, CHANGE);

  Timer1.initialize(Ts_us);
  Timer1.attachInterrupt(control_PI);
}

void loop() {
  if (Serial.available() > 0) {
    float nueva_referencia = Serial.parseFloat();

    noInterrupts();
    referencia_RPM = nueva_referencia;
    interrupts();

    while (Serial.available() > 0)
      Serial.read();
  }

  if (dato_nuevo) {
    noInterrupts();
    float referencia = referencia_RPM;
    float velocidad = velocidad_RPM;
    float error = error_RPM;
    dato_nuevo = false;
    interrupts();

    contador_plotter++;

    if (contador_plotter >= DIVISOR_PLOTTER) {
      contador_plotter = 0;

      Serial.print("Referencia:");
      Serial.print(referencia, 3);
      Serial.print('\t');
      Serial.print("Velocidad:");
      Serial.print(velocidad, 3);
      Serial.print('\t');
      Serial.print("Error:");
      Serial.println(error, 3);
    }
  }
}
