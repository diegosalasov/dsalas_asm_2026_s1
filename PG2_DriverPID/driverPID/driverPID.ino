// ============================================================
// ESP32 + HC-SR04 + L298N
// Ascensor 5 pisos — Control PID con Kickstart
//
// ARQUITECTURA DE CONTROL
// ========================
// El motor pasa por tres estados bien definidos (máquina de estados):
//
//   DETENIDO ──[botón]──► KICKSTART ──[30 ms]──► MOVIENDO ──[llegó]──► DETENIDO
//
//   DETENIDO  : Motor apagado. Se muestra el estado por Serial cada 500 ms.
//   KICKSTART : Motor a PWM_KICKSTART (110) por DUR_KICKSTART (30 ms).
//               El PID NO corre en este estado. La dirección se fija antes
//               de arrancar y no cambia durante el impulso.
//   MOVIENDO  : PID activo. PWM limitado al rango [pwmMin, pwmMax] (45-55).
//               El PID corre cada TsPID (100 ms).
//
// POR QUÉ MÁQUINA DE ESTADOS Y NO BANDERAS
// ------------------------------------------
// Con la versión anterior (flag necesitaKickstart + controlarPID mezclados):
//   - El kickstart duraba 30 ms pero el PID corría cada 100 ms → el PID
//     sólo ejecutaba UNA vez dentro del kickstart, con pwmActual = 0.
//   - Cuando comprobarKickstart() disparaba a los 30 ms, aplicaba pwmActual=0
//     y detenía el motor involuntariamente.
//   - La separación de estados elimina cualquier carrera entre las dos funciones.
// ============================================================

// ── Pines ────────────────────────────────────────────────────────────────────
const int trigPin    = 32;
const int echoPin    = 33;

const int ENA = 25;
const int IN1 = 26;
const int IN2 = 27;

const int botonPiso1 = 36;
const int botonPiso2 = 39;
const int botonPiso3 = 34;
const int botonPiso4 = 35;
const int botonPiso5 = 14;

// ── Máquina de estados del motor ──────────────────────────────────────────────
// Tres estados posibles; nunca hay ambigüedad sobre qué debe hacer el loop.
const uint8_t ESTADO_DETENIDO  = 0;   // Motor apagado, esperando botón
const uint8_t ESTADO_KICKSTART = 1;   // Impulso inicial de alta corriente
const uint8_t ESTADO_MOVIENDO  = 2;   // Control PID activo (45-55 PWM)

uint8_t estadoMotor = ESTADO_DETENIDO;

// ── Kickstart ─────────────────────────────────────────────────────────────────
// Impulso inicial necesario para vencer la inercia estática del motor.
// Sin este impulso, el PID arranca con PWM 45-55 que puede no ser suficiente
// para que el motor comience a girar (zona muerta del L298N).
const int          PWM_KICKSTART  = 250;  // PWM durante el impulso
const unsigned long DUR_KICKSTART = 8;   // Duración en milisegundos

unsigned long tiempoInicioKickstart = 0;
bool          direccionSubir        = true; // Fijada al inicio, no cambia en kickstart

// ── Debounce de botones ───────────────────────────────────────────────────────
unsigned long ultimoTiempoBoton   = 0;
const unsigned long debounceBoton = 150;  // ms

// ── Pisos ─────────────────────────────────────────────────────────────────────
const int totalPisos = 5;

float pisos[totalPisos] = {
  5.0,   // Piso 1
  10.0,  // Piso 2
  15.0,  // Piso 3
  20.0,  // Piso 4
  25.0   // Piso 5
};

int pisoActual  = 0;
int pisoDestino = 0;

// ── Sensor HC-SR04 ────────────────────────────────────────────────────────────
float distanciaActual        = -1;
float distanciaFiltrada      = -1;
float ultimaDistanciaValida  = -1;

const float alphaFiltro = 0.65;  // Filtro exponencial: 0=sin filtro, 1=congelado

int lecturasInvalidas         = 0;
const int maxLecturasInvalidas = 5;

// ── PID ───────────────────────────────────────────────────────────────────────
float Kp = 2.5;
float Ki = 0.2;
float Kd = 5.0;

float error            = 0.0;
float errorAnterior    = 0.0;
float integral         = 0.0;
float derivada         = 0.0;
float derivadaFiltrada = 0.0;
float salidaPID        = 0.0;

// Comparador
float referencia        = 0.0;
float medicion          = 0.0;
float errorComparador   = 0.0;

// Tolerancia: si |error| < tolerancia → piso alcanzado
float tolerancia = 0.5;

// Rango dinámico real para el L298N
int pwmMin = 200;  // Tu velocidad lenta permanente
int pwmMax = 255; // Dejamos que el PID suba hasta 140 si el motor se queda pegado

float integralMax    = 25.0;
float alphaDerivada  = 0.85;

bool invertirDireccion = false;  // Invertir sentido físico del motor si es necesario

// ── Tiempos de muestreo (sin delay) ──────────────────────────────────────────
const unsigned long TsPID    = 80;   // Periodo del controlador PID (ms)
const unsigned long TsSensor =  80;   // Periodo de lectura del sensor (ms)
const unsigned long TsEstado = 500;   // Periodo de reporte en estado detenido (ms)

unsigned long tiempoAnteriorPID    = 0;
unsigned long tiempoAnteriorSensor = 0;
unsigned long tiempoAnteriorEstado = 0;

// Seguridad: si el motor lleva más de este tiempo sin llegar, se detiene
unsigned long tiempoInicioMovimiento = 0;
const unsigned long tiempoMaxMovimiento = 35000;  // 15 segundos


// =============================================================================
// FUNCIONES AUXILIARES
// =============================================================================

// ── Comparador (bloque previo al PID) ────────────────────────────────────────
float comparar(float ref, float med) {
  return ref - med;  // error = setpoint − medición
}

// ── Identificar piso por distancia ───────────────────────────────────────────
int detectarPisoActual(float distancia) {
  int   pisoMasCercano = 1;
  float menorError     = abs(distancia - pisos[0]);

  for (int i = 1; i < totalPisos; i++) {
    float errorPiso = abs(distancia - pisos[i]);
    if (errorPiso < menorError) {
      menorError     = errorPiso;
      pisoMasCercano = i + 1;
    }
  }
  return pisoMasCercano;
}

// ── Motor ─────────────────────────────────────────────────────────────────────
void motorSubir(int pwm) {
  if (!invertirDireccion) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else                    { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  analogWrite(ENA, pwm);
}

void motorBajar(int pwm) {
  if (!invertirDireccion) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else                    { digitalWrite(IN1, HIGH);  digitalWrite(IN2, LOW);  }
  analogWrite(ENA, pwm);
}

void detenerMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

// ── Reset del PID ─────────────────────────────────────────────────────────────
void resetPID() {
  error = errorAnterior = integral = derivada = derivadaFiltrada = salidaPID = 0.0;
}

float medirDistancia() {
  const unsigned long TIMEOUT_US = 6000;   // ← era 20000; suficiente para 100 cm

  float suma  = 0;
  int validas = 0;

  for (int i = 0; i < 3; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duracion = pulseIn(echoPin, HIGH, TIMEOUT_US);

    if (duracion > 0) {
      float d = (duracion * 0.0343f) / 2.0f;
      if (d >= 2.0f && d <= 100.0f) {
        suma += d;
        validas++;
      }
    }
    // delay(10) ELIMINADO — el tiempo entre disparos ya lo introduce
    // el propio pulseIn (bloquea hasta TIMEOUT_US si no hay eco).
  }

  return (validas > 0) ? (suma / validas) : -1.0f;
}

void actualizarSensor() {
  unsigned long ahora = millis();
  if (ahora - tiempoAnteriorSensor < TsSensor) return;
  tiempoAnteriorSensor = ahora;

  float nueva = medirDistancia();

  if (nueva > 0) {
    distanciaActual      = nueva;
    ultimaDistanciaValida = nueva;
    lecturasInvalidas    = 0;

    distanciaFiltrada = (distanciaFiltrada < 0)
                        ? nueva
                        : alphaFiltro * distanciaFiltrada + (1.0f - alphaFiltro) * nueva;

    pisoActual = detectarPisoActual(distanciaFiltrada);
  } else {
    lecturasInvalidas++;
    if (ultimaDistanciaValida > 0) distanciaActual = ultimaDistanciaValida;
  }
}

// =============================================================================
// SELECCIONAR PISO
// Valida el pedido, calcula la dirección y arranca el kickstart.
//
// Orden de operaciones:
//   1. Validar número de piso
//   2. Calcular error y dirección ANTES de tocar el motor
//      (dirección fijada aquí, no cambia durante kickstart)
//   3. Cambiar estado → KICKSTART
//   4. Arrancar motor a PWM_KICKSTART inmediatamente
//   5. Registrar tiempo de inicio
// =============================================================================
void seleccionarPiso(int piso) {
  if (piso < 1 || piso > totalPisos) {
    Serial.println("[ERROR] Piso invalido.");
    return;
  }
  if (piso == pisoDestino && estadoMotor != ESTADO_DETENIDO) {
    Serial.println("[INFO] Ese piso ya es el destino actual.");
    return;
  }

  pisoDestino = piso;
  referencia  = pisos[pisoDestino - 1];

  resetPID();

  // Calcular dirección usando la distancia filtrada actual
  // Si el sensor aún no leyó, usamos el piso detectado
  float posActual = (distanciaFiltrada > 0) ? distanciaFiltrada : pisos[pisoActual - 1];
  float errorInicial = referencia - posActual;
  direccionSubir = (errorInicial > 0);

  // ── Arrancar kickstart ────────────────────────────────────────────────────
  // El motor recibe el máximo impulso desde el primer instante.
  // El PID NO se involucra aquí; esperará hasta que termine el kickstart.
  if (direccionSubir) motorSubir(PWM_KICKSTART);
  else                motorBajar(PWM_KICKSTART);

  tiempoInicioKickstart    = millis();
  tiempoInicioMovimiento   = millis();
  estadoMotor              = ESTADO_KICKSTART;

  // El PID empezará su primer ciclo cuando se pase a ESTADO_MOVIENDO
  // (se inicializa tiempoAnteriorPID ahí para no saltarse el primer Ts)

  Serial.println();
  Serial.print("[DESTINO] Piso ");
  Serial.print(pisoDestino);
  Serial.print(" | Ref: ");
  Serial.print(referencia, 1);
  Serial.print(" cm | Dir: ");
  Serial.println(direccionSubir ? "SUBIR" : "BAJAR");
  Serial.print("[KICKSTART] PWM=");
  Serial.print(PWM_KICKSTART);
  Serial.print(" durante ");
  Serial.print(DUR_KICKSTART);
  Serial.println(" ms");
}

// =============================================================================
// BOTONES
// =============================================================================
void leerBotones() {
  unsigned long ahora = millis();
  if (ahora - ultimoTiempoBoton < debounceBoton) return;

  if      (digitalRead(botonPiso1) == HIGH) { seleccionarPiso(1); ultimoTiempoBoton = ahora; }
  else if (digitalRead(botonPiso2) == HIGH) { seleccionarPiso(2); ultimoTiempoBoton = ahora; }
  else if (digitalRead(botonPiso3) == HIGH) { seleccionarPiso(3); ultimoTiempoBoton = ahora; }
  else if (digitalRead(botonPiso4) == HIGH) { seleccionarPiso(4); ultimoTiempoBoton = ahora; }
  else if (digitalRead(botonPiso5) == HIGH) { seleccionarPiso(5); ultimoTiempoBoton = ahora; }
}

// =============================================================================
// ESTADO: KICKSTART
// Mantiene el impulso a PWM_KICKSTART hasta que pasan DUR_KICKSTART ms.
// No hace nada más: no lee error, no calcula PID.
// Cuando expira, transiciona a ESTADO_MOVIENDO con PID limpio.
// =============================================================================
void manejarKickstart() {
  unsigned long ahora = millis();

  if (ahora - tiempoInicioKickstart >= DUR_KICKSTART) {
    resetPID();
    tiempoAnteriorPID = ahora - TsPID;   // ← era "ahora"; ahora el PID corre ya
    estadoMotor       = ESTADO_MOVIENDO;

    Serial.print("[PID] Kickstart terminado. Pasando a control PID (PWM ");
    Serial.print(pwmMin);
    Serial.print("-");
    Serial.print(pwmMax);
    Serial.println(").");
  }
}

// =============================================================================
// ESTADO: MOVIENDO — Control PID
// Corre cada TsPID ms. PWM limitado a [pwmMin, pwmMax].
//
// Diagrama de bloques:
//   referencia ──► [Comparador] ──► error ──► [PID] ──► PWM ──► Motor
//                      ▲                                          │
//   distanciaFiltrada ─┘◄────────────────[Sensor]───────────────┘
// =============================================================================
// =============================================================================
// ESTADO: MOVIENDO — Control PID
// =============================================================================
void controlarPID() {
  unsigned long ahora = millis();
  if (ahora - tiempoAnteriorPID < TsPID) return;

  float dt          = (ahora - tiempoAnteriorPID) / 1000.0f;
  tiempoAnteriorPID = ahora;

  // ── Seguridades ───────────────────────────────────────────────────────────
  if (lecturasInvalidas >= maxLecturasInvalidas) {
    detenerMotor();
    estadoMotor = ESTADO_DETENIDO;
    resetPID();
    Serial.println("[ERROR] Demasiadas lecturas invalidas. Motor detenido.");
    return;
  }

  if (distanciaFiltrada < 0) {
    Serial.println("[ESPERA] Sin lectura valida del sensor...");
    return;  
  }

  if (ahora - tiempoInicioMovimiento > tiempoMaxMovimiento) {
    detenerMotor();
    estadoMotor = ESTADO_DETENIDO;
    resetPID();
    Serial.println("[ERROR] Tiempo maximo de movimiento excedido.");
    return;
  }

  // ── Comparador ────────────────────────────────────────────────────────────
  medicion       = distanciaFiltrada;
  errorComparador = comparar(referencia, medicion);
  error           = errorComparador;

  // ── Condición de llegada ──────────────────────────────────────────────────
  if (abs(error) <= tolerancia) {
    detenerMotor();
    estadoMotor = ESTADO_DETENIDO;
    pisoActual  = detectarPisoActual(distanciaFiltrada);
    pisoDestino = pisoActual;
    referencia  = pisos[pisoActual - 1];
    resetPID();

    Serial.print("[OK] Piso alcanzado: ");
    Serial.print(pisoActual);
    Serial.print(" | Distancia final: ");
    Serial.print(distanciaFiltrada, 2);
    Serial.println(" cm");
    return;
  }

  // ── DETECCIÓN DE CAMBIO DE DIRECCIÓN (NUEVO) ──────────────────────────────
  bool nuevaDireccionSubir = (error > 0);
  
  if (nuevaDireccionSubir != direccionSubir) {
    direccionSubir = nuevaDireccionSubir; // Actualizamos la bandera global
    
    // Disparamos el motor a máxima potencia en la nueva dirección
    if (direccionSubir) motorSubir(PWM_KICKSTART);
    else                motorBajar(PWM_KICKSTART);

    tiempoInicioKickstart = millis();
    estadoMotor = ESTADO_KICKSTART; // Devolvemos el control a la máquina de estados

    Serial.println("[PID] Overshoot. Cambio de direccion -> Nuevo Kickstart aplicado.");
    
    // Salimos inmediatamente de la función. El loop() ahora ejecutará manejarKickstart()
    return; 
  }

  // ── PID ───────────────────────────────────────────────────────────────────
  integral += error * dt;
  integral  = constrain(integral, -integralMax, integralMax);  // Anti-windup

  derivada         = (error - errorAnterior) / dt;
  derivadaFiltrada = alphaDerivada * derivadaFiltrada +
                     (1.0f - alphaDerivada) * derivada;  // Filtro derivada

  salidaPID = Kp * error + Ki * integral + Kd * derivadaFiltrada;
  errorAnterior = error;

  // ── PWM acotado al rango ──────────────────────────────────────────────────
  int pwm = constrain(pwmMin + (int)abs(salidaPID), pwmMin, pwmMax);

  // Ya sabemos la dirección gracias a direccionSubir, la usamos aquí
  if (direccionSubir) motorSubir(pwm);
  else                motorBajar(pwm);

  // ── Log Serial ────────────────────────────────────────────────────────────
  Serial.print("Piso:");
  Serial.print(pisoActual);
  Serial.print(" Dest:");
  Serial.print(pisoDestino);
  Serial.print(" | Ref:");
  Serial.print(referencia, 1);
  Serial.print(" Med:");
  Serial.print(medicion, 2);
  Serial.print(" Err:");
  Serial.print(error, 2);
  Serial.print(" | I:");
  Serial.print(integral, 2);
  Serial.print(" D:");
  Serial.print(derivadaFiltrada, 2);
  Serial.print(" | PWM:");
  Serial.println(pwm);
}

// =============================================================================
// ESTADO: DETENIDO — Reporte periódico
// =============================================================================
void mostrarEstado() {
  unsigned long ahora = millis();
  if (ahora - tiempoAnteriorEstado < TsEstado) return;
  tiempoAnteriorEstado = ahora;

  Serial.print("[DETENIDO] Dist: ");
  Serial.print(distanciaFiltrada, 2);
  Serial.print(" cm | Piso est.: ");
  Serial.print(pisoActual);
  Serial.println(" | Presione un boton");
}

// =============================================================================
// SETUP
// =============================================================================
void inicializarPisoActual() {
  Serial.println("[INIT] Buscando posicion inicial...");

  float lectura = -1;
  for (int i = 0; i < 10; i++) {
    lectura = medirDistancia();
    if (lectura > 0) break;
  }

  if (lectura > 0) {
    distanciaActual      = lectura;
    distanciaFiltrada    = lectura;
    ultimaDistanciaValida = lectura;
    pisoActual           = detectarPisoActual(lectura);
    pisoDestino          = pisoActual;
    referencia           = pisos[pisoActual - 1];

    Serial.print("[INIT] Piso inicial: ");
    Serial.print(pisoActual);
    Serial.print(" | Distancia: ");
    Serial.print(lectura, 2);
    Serial.println(" cm");
  } else {
    pisoActual  = 1;
    pisoDestino = 1;
    referencia  = pisos[0];
    Serial.println("[INIT] Sin lectura inicial. Se asume piso 1.");
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

  Serial.println("============================================");
  Serial.println(" Ascensor PID — ESP32 + HC-SR04 + L298N");
  Serial.println(" Kickstart: 110 PWM / 30 ms");
  Serial.println(" PID:       45-55 PWM");
  Serial.println("============================================");

  inicializarPisoActual();

  Serial.println("[LISTO] Presione un boton del piso 1 al 5.");
}

// =============================================================================
// LOOP PRINCIPAL
// Despacha al estado correcto. Nunca bloquea.
//
//   ┌─────────────────────────────────────────────┐
//   │  loop()                                     │
//   │   leerBotones()  ◄── siempre                │
//   │   actualizarSensor() ◄── siempre            │
//   │                                             │
//   │   switch(estadoMotor)                       │
//   │    DETENIDO  → mostrarEstado()              │
//   │    KICKSTART → manejarKickstart()           │
//   │    MOVIENDO  → controlarPID()               │
//   └─────────────────────────────────────────────┘
// =============================================================================
void loop() {
  leerBotones();
  actualizarSensor();

  switch (estadoMotor) {
    case ESTADO_DETENIDO:
      mostrarEstado();
      break;

    case ESTADO_KICKSTART:
      // El motor ya está girando a PWM_KICKSTART desde seleccionarPiso().
      // Aquí sólo esperamos a que pasen los 30 ms.
      manejarKickstart();
      break;

    case ESTADO_MOVIENDO:
      controlarPID();
      break;
  }
}
