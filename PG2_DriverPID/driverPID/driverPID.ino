// ESP32 + HC-SR04 + L298N
// Ascensor con botones + comparador + PID robusto
// Sin delay()

const int trigPin = 32;
const int echoPin = 33;

const int ENA = 25;
const int IN1 = 26;
const int IN2 = 27;

// Botones
const int botonPiso1 = 36;
const int botonPiso2 = 39;
const int botonPiso3 = 34;
const int botonPiso4 = 35;
const int botonPiso5 = 14;

unsigned long ultimoTiempoBoton = 0;
const unsigned long debounceBoton = 250;

// Pisos
const int totalPisos = 5;

float pisos[totalPisos] = {
  5.0,
  10.0,
  15.0,
  20.0,
  25.0
};

int pisoActual = 0;
int pisoDestino = 0;

bool motorActivo = false;

// Sensor
float distanciaActual = -1;
float distanciaFiltrada = -1;
float ultimaDistanciaValida = -1;

const float velocidadSonido = 0.0343;
const float alphaFiltro = 0.65;

// Comparador
float referencia = 5.0;
float medicion = 0.0;
float errorComparador = 0.0;

float comparar(float referencia, float medicion) {
  return referencia - medicion;
}

// PID
float Kp = 18.0;
float Ki = 0.15;
float Kd = 5.0;

float error = 0.0;
float errorAnterior = 0.0;
float integral = 0.0;
float derivada = 0.0;
float derivadaFiltrada = 0.0;
float salidaPID = 0.0;

float tolerancia = 0.8;

int pwmMin = 85;
int pwmMax = 220;

float integralMax = 80.0;
float alphaDerivada = 0.7;

bool invertirDireccion = false;

// Tiempo sin delay
const unsigned long TsPID = 100;
unsigned long tiempoAnteriorPID = 0;

const unsigned long TsSensor = 70;
unsigned long tiempoAnteriorSensor = 0;

const unsigned long TsEstado = 500;
unsigned long tiempoAnteriorEstado = 0;

// Seguridad
unsigned long tiempoInicioMovimiento = 0;
const unsigned long tiempoMaxMovimiento = 15000;

int lecturasInvalidas = 0;
const int maxLecturasInvalidas = 8;

// =====================
// Identificación de piso
// =====================
int detectarPisoActual(float distancia) {
  int pisoMasCercano = 1;
  float menorError = abs(distancia - pisos[0]);

  for (int i = 1; i < totalPisos; i++) {
    float errorPiso = abs(distancia - pisos[i]);

    if (errorPiso < menorError) {
      menorError = errorPiso;
      pisoMasCercano = i + 1;
    }
  }

  return pisoMasCercano;
}

// =====================
// Sensor HC-SR04
// =====================
float medirDistancia() {
  float suma = 0;
  int validas = 0;

  for (int i = 0; i < 5; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(3);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duracion = pulseIn(echoPin, HIGH, 25000);

    if (duracion > 0) {
      float distancia = (duracion * velocidadSonido) / 2.0;

      if (distancia >= 2.0 && distancia <= 35.0) {
        suma += distancia;
        validas++;
      }
    }
  }

  if (validas == 0) {
    return -1;
  }

  return suma / validas;
}

void actualizarSensor() {
  unsigned long ahora = millis();

  if (ahora - tiempoAnteriorSensor >= TsSensor) {
    tiempoAnteriorSensor = ahora;

    float nuevaLectura = medirDistancia();

    if (nuevaLectura > 0) {
      distanciaActual = nuevaLectura;
      ultimaDistanciaValida = nuevaLectura;
      lecturasInvalidas = 0;

      if (distanciaFiltrada < 0) {
        distanciaFiltrada = nuevaLectura;
      } else {
        distanciaFiltrada = alphaFiltro * distanciaFiltrada + 
                            (1.0 - alphaFiltro) * nuevaLectura;
      }

      pisoActual = detectarPisoActual(distanciaFiltrada);
    } 
    else {
      lecturasInvalidas++;

      if (ultimaDistanciaValida > 0) {
        distanciaActual = ultimaDistanciaValida;
      }
    }
  }
}

// =====================
// Motor L298N
// =====================
void motorSubir(int pwm) {
  if (!invertirDireccion) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  analogWrite(ENA, pwm);
}

void motorBajar(int pwm) {
  if (!invertirDireccion) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, pwm);
}

void detenerMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

// =====================
// PID
// =====================
void resetPID() {
  error = 0.0;
  errorAnterior = 0.0;
  integral = 0.0;
  derivada = 0.0;
  derivadaFiltrada = 0.0;
  salidaPID = 0.0;
}

void seleccionarPiso(int piso) {
  if (piso < 1 || piso > totalPisos) {
    Serial.println("Piso invalido");
    return;
  }

  if (piso == pisoDestino && motorActivo) {
    Serial.println("Ese piso ya es el destino actual.");
    return;
  }

  pisoDestino = piso;
  referencia = pisos[pisoDestino - 1];

  resetPID();

  motorActivo = true;
  tiempoInicioMovimiento = millis();
  tiempoAnteriorPID = millis();

  Serial.println();
  Serial.print("Nuevo destino: piso ");
  Serial.println(pisoDestino);

  Serial.print("Referencia: ");
  Serial.print(referencia);
  Serial.println(" cm");

  Serial.print("Piso actual estimado: ");
  Serial.println(pisoActual);
}

// =====================
// Botones
// =====================
void leerBotones() {
  unsigned long ahora = millis();

  if (ahora - ultimoTiempoBoton < debounceBoton) {
    return;
  }

  if (digitalRead(botonPiso1) == HIGH) {
    seleccionarPiso(1);
    ultimoTiempoBoton = ahora;
  } 
  else if (digitalRead(botonPiso2) == HIGH) {
    seleccionarPiso(2);
    ultimoTiempoBoton = ahora;
  } 
  else if (digitalRead(botonPiso3) == HIGH) {
    seleccionarPiso(3);
    ultimoTiempoBoton = ahora;
  } 
  else if (digitalRead(botonPiso4) == HIGH) {
    seleccionarPiso(4);
    ultimoTiempoBoton = ahora;
  } 
  else if (digitalRead(botonPiso5) == HIGH) {
    seleccionarPiso(5);
    ultimoTiempoBoton = ahora;
  }
}

// =====================
// Control principal PID
// =====================
void controlarPID() {
  unsigned long ahora = millis();

  if (ahora - tiempoAnteriorPID < TsPID) {
    return;
  }

  float dt = (ahora - tiempoAnteriorPID) / 1000.0;
  tiempoAnteriorPID = ahora;

  if (lecturasInvalidas >= maxLecturasInvalidas) {
    detenerMotor();
    motorActivo = false;
    resetPID();

    Serial.println("Error: demasiadas lecturas invalidas. Motor detenido.");
    return;
  }

  if (distanciaFiltrada < 0) {
    detenerMotor();
    Serial.println("Esperando lectura valida del sensor...");
    return;
  }

  // Comparador
  medicion = distanciaFiltrada;
  errorComparador = comparar(referencia, medicion);

  // PID
  error = errorComparador;

  if (abs(error) <= tolerancia) {
    detenerMotor();
    motorActivo = false;
    pisoActual = detectarPisoActual(distanciaFiltrada);
    pisoDestino = pisoActual;
    referencia = pisos[pisoActual - 1];
    resetPID();

    Serial.print("Piso alcanzado: ");
    Serial.println(pisoActual);
    return;
  }

  if (ahora - tiempoInicioMovimiento > tiempoMaxMovimiento) {
    detenerMotor();
    motorActivo = false;
    resetPID();

    Serial.println("Error: tiempo maximo de movimiento excedido.");
    return;
  }

  integral += error * dt;

  if (integral > integralMax) integral = integralMax;
  if (integral < -integralMax) integral = -integralMax;

  derivada = (error - errorAnterior) / dt;

  derivadaFiltrada = alphaDerivada * derivadaFiltrada +
                     (1.0 - alphaDerivada) * derivada;

  salidaPID = Kp * error + Ki * integral + Kd * derivadaFiltrada;

  errorAnterior = error;

  int pwm = abs(salidaPID);

  if (pwm > pwmMax) pwm = pwmMax;
  if (pwm < pwmMin) pwm = pwmMin;

  if (error > 0) {
    motorSubir(pwm);
  } else {
    motorBajar(pwm);
  }

  Serial.print("Piso actual: ");
  Serial.print(pisoActual);
  Serial.print(" | Destino: ");
  Serial.print(pisoDestino);
  Serial.print(" | Ref: ");
  Serial.print(referencia, 2);
  Serial.print(" cm | Med: ");
  Serial.print(medicion, 2);
  Serial.print(" cm | Error: ");
  Serial.print(error, 2);
  Serial.print(" | PWM: ");
  Serial.println(pwm);
}

// =====================
// Estado
// =====================
void mostrarEstado() {
  unsigned long ahora = millis();

  if (ahora - tiempoAnteriorEstado >= TsEstado) {
    tiempoAnteriorEstado = ahora;

    Serial.print("Detenido | Medicion: ");
    Serial.print(distanciaFiltrada, 2);
    Serial.print(" cm | Piso actual estimado: ");
    Serial.print(pisoActual);
    Serial.println(" | Presione un boton de piso");
  }
}

// =====================
// Inicialización
// =====================
void inicializarPisoActual() {
  Serial.println("Inicializando posicion del ascensor...");

  float lecturaInicial = -1;

  for (int i = 0; i < 10; i++) {
    lecturaInicial = medirDistancia();

    if (lecturaInicial > 0) {
      break;
    }
  }

  if (lecturaInicial > 0) {
    distanciaActual = lecturaInicial;
    distanciaFiltrada = lecturaInicial;
    ultimaDistanciaValida = lecturaInicial;

    pisoActual = detectarPisoActual(distanciaFiltrada);
    pisoDestino = pisoActual;
    referencia = pisos[pisoActual - 1];

    Serial.print("Piso inicial detectado: ");
    Serial.println(pisoActual);

    Serial.print("Distancia inicial: ");
    Serial.print(distanciaFiltrada);
    Serial.println(" cm");
  } 
  else {
    pisoActual = 1;
    pisoDestino = 1;
    referencia = pisos[0];

    Serial.println("No se pudo detectar posicion inicial.");
    Serial.println("Se asigna piso 1 por defecto.");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(botonPiso1, INPUT);
  pinMode(botonPiso2, INPUT);
  pinMode(botonPiso3, INPUT);
  pinMode(botonPiso4, INPUT);
  pinMode(botonPiso5, INPUT);

  detenerMotor();

  Serial.println("Sistema de ascensor con PID robusto iniciado");

  inicializarPisoActual();

  Serial.println("Sistema listo. Presione un boton del piso 1 al 5.");
}

void loop() {
  leerBotones();
  actualizarSensor();

  if (motorActivo) {
    controlarPID();
  } else {
    mostrarEstado();
  }
}