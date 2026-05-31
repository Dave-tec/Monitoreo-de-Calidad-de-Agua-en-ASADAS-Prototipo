#include <WiFi.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <SD.h>
#include <time.h>
#include <esp_sleep.h>
#include <math.h>

// ======================================================
// CONFIGURACION GENERAL DEL SISTEMA
// ======================================================
//
// Este código realiza:
// 1. Captación de muestra de agua.
// 2. Limpieza básica del tubo de captación.
// 3. Medición de temperatura, pH, conductividad y turbidez.
// 4. Estimación indirecta de nitratos.
// 5. Almacenamiento local en MicroSD.
// 6. Envío de datos a ThingSpeak.
// 7. Lectura del intervalo de sleep desde ThingSpeak.
// 8. Evaluación automática de alertas.
// 9. Entrada en modo deep sleep.
//
// ======================================================


// ======================================================
// CONFIGURACION DE PINES
// ======================================================

// Sensor de temperatura DS18B20
#define PIN_TEMP 15

// Sensor de nivel tipo flotador
#define PIN_NIVEL 22

// Sensor de pH por salida analógica
// Se lee el voltaje directamente por ADC y se aplica la ecuación de ajuste.
#define PIN_PH 2

// Sensor de conductividad
// Pines DAC para excitar la celda
#define PIN_COND_DAC_A 25
#define PIN_COND_DAC_B 26

// Pines ADC para leer los nodos de la celda
#define PIN_COND_ADC_A 34
#define PIN_COND_ADC_B 35

// Sensor de turbidez
#define PIN_TURBIDEZ 13

// Bombas
// Bomba A = drenaje
// Bomba B = captación
#define PIN_BOMBA_A 32
#define PIN_BOMBA_B 33

// Pines SPI para MicroSD
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK  18
#define SD_CS   5


// ======================================================
// CONFIGURACION DE ADC Y REFERENCIAS
// ======================================================

// Referencia de voltaje usada para convertir ADC a voltaje.
// El ESP32 trabaja con ADC de 12 bits: 0 a 4095.
const float ADC_VREF = 3.3;


// ======================================================
// LOGICA DE NIVEL Y BOMBAS
// ======================================================

const int NIVEL_ACTIVO = LOW;
const int BOMBA_ON  = HIGH;
const int BOMBA_OFF = LOW;


// ======================================================
// TIEMPOS DE OPERACION
// ======================================================

// Tiempo de drenaje inicial. (20 segundos)
const unsigned long TIEMPO_DRENAJE_MS = 20000UL;

// Tiempo para captar agua durante limpieza del tubo. (10 segundos)
const unsigned long TIEMPO_LIMPIEZA_CAPTAR_MS = 10000UL;

// Tiempo para drenar durante limpieza del tubo.(10 segundos)
const unsigned long TIEMPO_LIMPIEZA_DRENAR_MS = 10000UL;

// Tiempo de espera antes del segundo intento WiFi. (1 minuto)
const unsigned long TIEMPO_REINTENTO_WIFI_MS = 60000UL;

// Tiempo extra que la bomba de captación queda encendida
// después de que el sensor de nivel detecta agua.(2 segundos)
const unsigned long TIEMPO_EXTRA_CAPTACION_MS = 2000UL;

// Tiempo máximo de captación por seguridad.
const unsigned long TIMEOUT_CAPTAR_MS = 120000UL;


// ======================================================
// CONFIGURACION WIFI Y THINGSPEAK
// ======================================================
// Nombre y contraseña de red WiFi 
const char* WIFI_SSID = "Dave";
const char* WIFI_PASS = "ladelwifi";

// Channel ID y los API KEY de escritura y lectura de la plataforma ThingSpeak
const char* TS_WRITE_API_KEY = "Q8MJFTI3GSVRTJL3";
const char* TS_READ_API_KEY  = "PW3B3BRDEV0D8GEV";
const char* TS_CHANNEL_ID    = "3374410";


// ======================================================
// CONFIGURACION NTP
// ======================================================

// Costa Rica = UTC -6
const long GMT_OFFSET_SEC = -6 * 3600;
const int DAYLIGHT_OFFSET_SEC = 0;


// ======================================================
// CONFIGURACION MICROSD
// ======================================================

SPIClass spiSD(VSPI);
const char* NOMBRE_ARCHIVO = "/datos.csv";


// ======================================================
// VARIABLE RTC PARA MANTENER EL TIEMPO DE SLEEP
// ENTRE REINICIOS POR DEEP SLEEP
// ======================================================
//
// Esta variable se conserva durante deep sleep.
// Se actualiza leyendo el field6 de ThingSpeak.
RTC_DATA_ATTR uint32_t minutosSleep = 1;


// ======================================================
// OBJETOS DE SENSORES
// ======================================================

OneWire oneWire(PIN_TEMP);
DallasTemperature sensorTemp(&oneWire);


// ======================================================
// CONSTANTES PARA CONDUCTIVIDAD
// ======================================================

// Resistencia de referencia en serie con la celda.
const float COND_R_REF = 2200.0;

// Niveles de excitación DAC.
// Se alternan para evitar polarización de la celda.
const uint8_t COND_DAC_BAJO = 64;
const uint8_t COND_DAC_ALTO = 192;

// Constante de celda.
float K_CELDA = 1.0;

// Coeficiente de compensación térmica.
// Aproximación común: 2% por °C.
float ALPHA_COND = 0.02;


// ======================================================
// ESTRUCTURA PARA RESULTADOS DE CONDUCTIVIDAD
// ======================================================
//
// Esta estructura almacena los valores calculados durante
// un semiciclo de medición de conductividad.
struct ResultadoConductividad {
  float VA;
  float VB;
  float Vcelda;
  float corriente;
  float resistencia;
  float conductancia;
  float conductividadSinComp;
  float conductividadCompensada;
};


// ======================================================
// ECUACIONES DE AJUSTE
// ======================================================

// ------------------------------------------------------
// pH ajustado por voltaje
// Ecuación:
// pH = 19.1805 - 10.4384(V)
// ------------------------------------------------------
const float PH_AJUSTE_A = 19.1805;
const float PH_AJUSTE_B = -10.4384;

// ------------------------------------------------------
// Conductividad ajustada por regresión lineal
// Ecuación:
// Conductividad = -157.27 + 6.8272(Sensor)
// Donde Sensor entra en uS/cm.
// Se usa el mejor ajuste para valores bajos de conductividad
// ------------------------------------------------------
const float COND_AJUSTE_A = -157.27;
const float COND_AJUSTE_B = 6.8272;

// ------------------------------------------------------
// Turbidez con ecuación DFRobot
// Ecuación:
// NTU = -1120.4(V^2) + 5742.3(V) - 4352.9
//
// Pero antes se adapta el voltaje leído por ESP32:
// V_adaptado = 4.1 * (V_ESP32 / 2.35)
// ------------------------------------------------------
const float TURB_VOLT_MAX_ESP32 = 2.35;
const float TURB_VOLT_AGUA_CLARA_DFROBOT = 4.1;
const float TURB_NTU_MAX = 3000.0;


// ======================================================
// RANGOS DE ALERTA
// ======================================================
//
// Turbidez > 5 NTU: alerta.
// Conductividad > 400 uS/cm: alerta.
// Temperatura < 18 o > 30 °C: alerta.
// pH < 6 o > 8: alerta.
// Nitratos > 25 mg/L: alarma de valor alerta.
// Nitratos > 50 mg/L: alerta por valor máximo admisible.
float TEMP_MIN = 18.0;
float TEMP_MAX = 30.0;

float PH_MIN = 6.0;
float PH_MAX = 8.0;

float COND_MAX = 400.0;

float TURB_MAX = 5.0;

float NITR_ALERTA = 25.0;
float NITR_VMA = 50.0;


// ======================================================
// VARIABLES GLOBALES DE PARAMETROS
// ======================================================
//
// Estos son los valores finales que se almacenan,
// se envían a ThingSpeak y se evalúan en alertas.
float parametroTemperatura      = NAN; // °C
float parametroPH               = NAN; // pH ajustado por voltaje
float parametroVoltajePH        = NAN; // voltaje promedio del sensor pH
float parametroConductividad    = NAN; // uS/cm ajustado
float parametroTurbidez         = NAN; // NTU estimado
float parametroVoltajeTurbidez  = NAN; // voltaje promedio del sensor turbidez
float parametroNitratos         = NAN; // mg/L estimado


// ======================================================
// FUNCION: convertir ADC a voltaje
// ======================================================
//
// Convierte una lectura ADC de 0 a 4095 en voltaje.
// Se usa para pH, turbidez y conductividad.
float adcToVoltage(float raw) {
  return (raw / 4095.0) * ADC_VREF;
}


// ======================================================
// FUNCION: convertir valor DAC a voltaje aproximado
// ======================================================
//
// Convierte el valor DAC de 0 a 255 en voltaje aproximado.
float dacToVoltage(uint8_t dacValue) {
  return (dacValue / 255.0) * ADC_VREF;
}


// ======================================================
// FUNCION: revisar si hay nivel de agua
// ======================================================
//
// Lee el sensor de nivel y determina si se detectó agua.
bool hayNivelAgua() {
  int lectura = digitalRead(PIN_NIVEL);

  Serial.print("[NIVEL] Estado crudo del pin ");
  Serial.print(PIN_NIVEL);
  Serial.print(": ");
  Serial.println(lectura);

  if (lectura == NIVEL_ACTIVO) {
    Serial.println("[NIVEL] Agua detectada.");
    return true;
  } else {
    Serial.println("[NIVEL] No hay agua detectada.");
    return false;
  }
}


// ======================================================
// FUNCIONES PARA CONTROLAR BOMBAS
// ======================================================

void setBombaA(bool estado) {
  digitalWrite(PIN_BOMBA_A, estado ? BOMBA_ON : BOMBA_OFF);

  Serial.print("[BOMBA A - DRENAR] ");
  Serial.println(estado ? "ENCENDIDA" : "APAGADA");
}

void setBombaB(bool estado) {
  digitalWrite(PIN_BOMBA_B, estado ? BOMBA_ON : BOMBA_OFF);

  Serial.print("[BOMBA B - CAPTAR] ");
  Serial.println(estado ? "ENCENDIDA" : "APAGADA");
}


// ======================================================
// FUNCION: limpieza del tubo de muestra
// ======================================================
//
// 1. Capta agua por un tiempo definido.
// 2. Drena el tubo.
// 3. Vuelve a captar hasta que el sensor de nivel detecte agua.
// 4. Mantiene la bomba B encendida 2 segundos extra.
// 5. Apaga la bomba B.
bool limpiarTuboMuestra() {
  Serial.println("======================================");
  Serial.println("[LIMPIEZA] Inicio de limpieza del tubo");
  Serial.println("======================================");

  Serial.println("[LIMPIEZA] Activando bomba B para captar agua.");
  setBombaB(true);
  delay(TIEMPO_LIMPIEZA_CAPTAR_MS);
  setBombaB(false);

  Serial.println("[LIMPIEZA] Activando bomba A para drenar.");
  setBombaA(true);
  delay(TIEMPO_LIMPIEZA_DRENAR_MS);
  setBombaA(false);

  Serial.println("[LIMPIEZA] Activando bomba B hasta detectar nivel.");
  unsigned long t0 = millis();
  setBombaB(true);

  while (!hayNivelAgua()) {
    if (millis() - t0 > TIMEOUT_CAPTAR_MS) {
      Serial.println("[LIMPIEZA] ERROR: tiempo maximo excedido.");
      setBombaB(false);
      return false;
    }
    delay(200);
  }

  Serial.println("[LIMPIEZA] Nivel detectado. Bomba B continua 2 segundos.");
  delay(TIEMPO_EXTRA_CAPTACION_MS);

  setBombaB(false);
  Serial.println("[LIMPIEZA] Limpieza completada correctamente.");
  return true;
}


// ======================================================
// FUNCION: etapa inicial de captacion
// ======================================================
//
// Si ya hay agua, se drena y se limpia el tubo.
// Si no hay agua, se activa la bomba B hasta detectar nivel.
bool etapaCaptacionInicial() {
  Serial.println("======================================");
  Serial.println("[ETAPA 1] Inicio de control de muestra");
  Serial.println("======================================");

  if (hayNivelAgua()) {
    Serial.println("[ETAPA 1] Ya existe agua. Se drenara primero.");

    setBombaA(true);
    delay(TIEMPO_DRENAJE_MS);
    setBombaA(false);

    Serial.println("[ETAPA 1] Drenaje finalizado. Se ejecuta limpieza.");

    bool okLimpieza = limpiarTuboMuestra();

    if (!okLimpieza) {
      Serial.println("[ETAPA 1] ERROR: fallo en limpieza.");
      return false;
    }

    Serial.println("[ETAPA 1] Captacion y limpieza completadas.");
    return true;
  } 
  else {
    Serial.println("[ETAPA 1] No hay agua. Se activa bomba B.");

    unsigned long t0 = millis();
    setBombaB(true);

    while (!hayNivelAgua()) {
      if (millis() - t0 > TIMEOUT_CAPTAR_MS) {
        Serial.println("[ETAPA 1] ERROR: tiempo maximo de captacion excedido.");
        setBombaB(false);
        return false;
      }
      delay(200);
    }

    Serial.println("[ETAPA 1] Nivel detectado. Bomba B continua 2 segundos.");
    delay(TIEMPO_EXTRA_CAPTACION_MS);

    setBombaB(false);
    Serial.println("[ETAPA 1] Captacion completada correctamente.");
    return true;
  }
}


// ======================================================
// FUNCION: medir temperatura con 30 muestras
// ======================================================
//
// Lee el sensor DS18B20 30 veces.
// Promedia solamente lecturas válidas.
float medirTemperaturaPromedio30() {
  Serial.println("======================================");
  Serial.println("[TEMPERATURA] Inicio de medicion");
  Serial.println("======================================");

  float suma = 0.0;
  int validas = 0;

  for (int i = 0; i < 30; i++) {
    sensorTemp.requestTemperatures();
    float t = sensorTemp.getTempCByIndex(0);

    Serial.print("[TEMPERATURA] Muestra ");
    Serial.print(i + 1);
    Serial.print(": ");

    if (t != DEVICE_DISCONNECTED_C) {
      Serial.print(t, 2);
      Serial.println(" °C");
      suma += t;
      validas++;
    } else {
      Serial.println("Lectura invalida");
    }

    delay(750);
  }

  if (validas == 0) {
    Serial.println("[TEMPERATURA] ERROR: no hubo lecturas validas.");
    return NAN;
  }

  float promedio = suma / validas;

  Serial.print("[TEMPERATURA] Promedio final: ");
  Serial.print(promedio, 2);
  Serial.println(" °C");

  return promedio;
}


// ======================================================
// FUNCION: medir pH usando voltaje
// ======================================================
//
// Se leen 30 muestras del pin analógico.
// Cada muestra se convierte a voltaje.
// Luego se aplica la ecuación:
// pH = 19.1805 - 10.4384(V)
//
// El promedio final es el valor usado por el sistema.
float medirPH() {
  Serial.println("======================================");
  Serial.println("[PH] Inicio de medicion por voltaje");
  Serial.println("======================================");

  const int muestras = 30;

  float sumaVoltaje = 0.0;
  float sumaPH = 0.0;
  int validas = 0;

  for (int i = 0; i < muestras; i++) {
    int rawADC = analogRead(PIN_PH);
    float voltaje = adcToVoltage(rawADC);

    float phCalculado = PH_AJUSTE_A + (PH_AJUSTE_B * voltaje);

    Serial.print("[PH] Muestra ");
    Serial.print(i + 1);

    Serial.print(" | ADC: ");
    Serial.print(rawADC);

    Serial.print(" | Voltaje: ");
    Serial.print(voltaje, 3);
    Serial.print(" V");

    Serial.print(" | pH ajustado: ");
    Serial.println(phCalculado, 2);

    if (!isnan(phCalculado) && phCalculado >= 0.0 && phCalculado <= 14.0) {
      sumaVoltaje += voltaje;
      sumaPH += phCalculado;
      validas++;
    } else {
      Serial.println("[PH] Lectura invalida");
    }

    delay(200);
  }

  if (validas == 0) {
    Serial.println("[PH] ERROR: no hubo lecturas validas.");
    parametroVoltajePH = NAN;
    return NAN;
  }

  float promedioVoltaje = sumaVoltaje / validas;
  float promedioPH = sumaPH / validas;

  parametroVoltajePH = promedioVoltaje;

  Serial.println("======================================");
  Serial.println("[PH] Resultado promedio");

  Serial.print("[PH] Voltaje promedio: ");
  Serial.print(promedioVoltaje, 3);
  Serial.println(" V");

  Serial.print("[PH] pH promedio ajustado: ");
  Serial.println(promedioPH, 2);

  Serial.println("======================================");

  return promedioPH;
}


// ======================================================
// FUNCION: leer voltaje promedio de conductividad
// ======================================================
//
// Promedia varias lecturas ADC para reducir ruido.
float leerVoltajePromedioConductividad(int pinADC, int muestras = 30) {
  long suma = 0;

  for (int i = 0; i < muestras; i++) {
    suma += analogRead(pinADC);
    delay(5);
  }

  float promedioADC = suma / (float)muestras;
  return adcToVoltage(promedioADC);
}


// ======================================================
// FUNCION: medir semiciclo de conductividad
// ======================================================
//
// Se aplica una excitación DAC en una dirección.
// Luego se leen los nodos de la celda.
// Se estima corriente, resistencia, conductancia y conductividad.
ResultadoConductividad medirSemicicloConductividad(uint8_t dacA, uint8_t dacB) {
  ResultadoConductividad r;

  dacWrite(PIN_COND_DAC_A, dacA);
  dacWrite(PIN_COND_DAC_B, dacB);

  delay(10);

  r.VA = leerVoltajePromedioConductividad(PIN_COND_ADC_A, 30);
  r.VB = leerVoltajePromedioConductividad(PIN_COND_ADC_B, 30);

  float VdriveA = dacToVoltage(dacA);
  float VdriveB = dacToVoltage(dacB);

  r.Vcelda = fabs(r.VA - r.VB);

  float IA = fabs(VdriveA - r.VA) / COND_R_REF;
  float IB = fabs(r.VB - VdriveB) / COND_R_REF;

  r.corriente = (IA + IB) / 2.0;

  if (r.corriente > 0.0000001) {
    r.resistencia = r.Vcelda / r.corriente;
    r.conductancia = 1.0 / r.resistencia;
    r.conductividadSinComp = K_CELDA * r.conductancia;
  } else {
    r.resistencia = NAN;
    r.conductancia = NAN;
    r.conductividadSinComp = NAN;
  }

  r.conductividadCompensada = NAN;

  return r;
}


// ======================================================
// FUNCION: compensar conductividad a 25 °C
// ======================================================
//
// Compensación térmica aproximada:
// C25 = C / (1 + alpha(T - 25))
float compensarConductividad25(float conductividad, float temperatura) {
  if (isnan(conductividad) || isnan(temperatura)) {
    return NAN;
  }

  return conductividad / (1.0 + ALPHA_COND * (temperatura - 25.0));
}


// ======================================================
// FUNCION: ajustar conductividad
// ======================================================
//
// La lectura compensada entra en uS/cm.
// Se aplica:
// Conductividad ajustada = -157.27 + 6.8272(Sensor)
float ajustarConductividad(float conductividadCompensada_uScm) {
  float conductividadAjustada = COND_AJUSTE_A + (COND_AJUSTE_B * conductividadCompensada_uScm);

  if (conductividadAjustada < 0.0) {
    conductividadAjustada = 0.0;
  }

  return conductividadAjustada;
}


// ======================================================
// FUNCION: medir conductividad
// ======================================================
//
// Se realizan 30 mediciones completas.
// Cada medición utiliza dos semiciclos para reducir polarización.
// Luego se compensa a 25 °C.
// Después se convierte a uS/cm y se aplica la ecuación de ajuste.
// El valor final queda en uS/cm.
float medirConductividad() {
  Serial.println("======================================");
  Serial.println("[CONDUCTIVIDAD] Inicio de medicion");
  Serial.println("======================================");

  float sumaConductividadAjustada = 0.0;
  int validas = 0;

  for (int i = 0; i < 30; i++) {
    Serial.print("[CONDUCTIVIDAD] Medicion completa ");
    Serial.println(i + 1);

    ResultadoConductividad r1 = medirSemicicloConductividad(COND_DAC_ALTO, COND_DAC_BAJO);
    ResultadoConductividad r2 = medirSemicicloConductividad(COND_DAC_BAJO, COND_DAC_ALTO);

    ResultadoConductividad rf;

    rf.VA = (r1.VA + r2.VA) / 2.0;
    rf.VB = (r1.VB + r2.VB) / 2.0;
    rf.Vcelda = (r1.Vcelda + r2.Vcelda) / 2.0;
    rf.corriente = (r1.corriente + r2.corriente) / 2.0;
    rf.resistencia = (r1.resistencia + r2.resistencia) / 2.0;
    rf.conductancia = (r1.conductancia + r2.conductancia) / 2.0;
    rf.conductividadSinComp = (r1.conductividadSinComp + r2.conductividadSinComp) / 2.0;

    rf.conductividadCompensada = compensarConductividad25(rf.conductividadSinComp, parametroTemperatura);

    Serial.print("[CONDUCTIVIDAD] VA: ");
    Serial.print(rf.VA, 4);
    Serial.println(" V");

    Serial.print("[CONDUCTIVIDAD] VB: ");
    Serial.print(rf.VB, 4);
    Serial.println(" V");

    Serial.print("[CONDUCTIVIDAD] Vcelda: ");
    Serial.print(rf.Vcelda, 4);
    Serial.println(" V");

    Serial.print("[CONDUCTIVIDAD] Corriente: ");
    Serial.print(rf.corriente * 1000.0, 4);
    Serial.println(" mA");

    Serial.print("[CONDUCTIVIDAD] Resistencia: ");
    Serial.print(rf.resistencia, 2);
    Serial.println(" ohm");

    Serial.print("[CONDUCTIVIDAD] Sin compensar en S/cm: ");
    Serial.println(rf.conductividadSinComp, 8);

    Serial.print("[CONDUCTIVIDAD] Compensada a 25 C en S/cm: ");
    Serial.println(rf.conductividadCompensada, 8);

    if (!isnan(rf.conductividadCompensada)) {
      float conductividadCompensada_uScm = rf.conductividadCompensada * 1000000.0;
      float conductividadAjustada_uScm = ajustarConductividad(conductividadCompensada_uScm);

      Serial.print("[CONDUCTIVIDAD] Compensada en uS/cm: ");
      Serial.println(conductividadCompensada_uScm, 2);

      Serial.print("[CONDUCTIVIDAD] Ajustada por regresion: ");
      Serial.print(conductividadAjustada_uScm, 2);
      Serial.println(" uS/cm");

      sumaConductividadAjustada += conductividadAjustada_uScm;
      validas++;
    } else {
      Serial.println("[CONDUCTIVIDAD] Lectura invalida.");
    }

    delay(100);
  }

  if (validas == 0) {
    Serial.println("[CONDUCTIVIDAD] ERROR: no hubo lecturas validas.");
    return NAN;
  }

  float promedioAjustado_uScm = sumaConductividadAjustada / validas;

  Serial.println("======================================");
  Serial.print("[CONDUCTIVIDAD] Promedio final ajustado: ");
  Serial.print(promedioAjustado_uScm, 2);
  Serial.println(" uS/cm");
  Serial.println("======================================");

  return promedioAjustado_uScm;
}


// ======================================================
// FUNCION: adaptar voltaje de turbidez al rango DFRobot
// ======================================================
//
// Se limita el voltaje leído para evitar extrapolaciones.
// Luego se aplica:
// V_adaptado = 4.1 * (V_ESP32 / 2.35)
float adaptarVoltajeTurbidezDFRobot(float voltajeESP32) {
  if (voltajeESP32 < 0.0) {
    voltajeESP32 = 0.0;
  }

  if (voltajeESP32 > TURB_VOLT_MAX_ESP32) {
    voltajeESP32 = TURB_VOLT_MAX_ESP32;
  }

  float voltajeAdaptado = TURB_VOLT_AGUA_CLARA_DFROBOT * (voltajeESP32 / TURB_VOLT_MAX_ESP32);

  return voltajeAdaptado;
}


// ======================================================
// FUNCION: calcular NTU con ecuacion DFRobot
// ======================================================
//
// NTU = -1120.4(V^2) + 5742.3(V) - 4352.9
// V debe ser el voltaje adaptado.
float calcularNTU_DFRobot(float voltajeAdaptado) {
  float ntu = (-1120.4 * voltajeAdaptado * voltajeAdaptado) +
              (5742.3 * voltajeAdaptado) -
              4352.9;

  if (ntu < 0.0) {
    ntu = 0.0;
  }

  if (ntu > TURB_NTU_MAX) {
    ntu = TURB_NTU_MAX;
  }

  return ntu;
}


// ======================================================
// FUNCION: medir turbidez
// ======================================================
//
// Se toman 30 muestras.
// En cada muestra:
// 1. Se lee ADC.
// 2. Se convierte a voltaje ESP32.
// 3. Se adapta a rango DFRobot.
// 4. Se calcula NTU.
// Al final se promedian los valores.
float medirTurbidez() {
  Serial.println("======================================");
  Serial.println("[TURBIDEZ] Inicio de medicion con 30 muestras");
  Serial.println("======================================");

  const int muestras = 30;

  float sumaVoltajeESP32 = 0.0;
  float sumaVoltajeAdaptado = 0.0;
  float sumaNTU = 0.0;
  int validas = 0;

  for (int i = 0; i < muestras; i++) {
    int rawADC = analogRead(PIN_TURBIDEZ);

    float voltajeESP32 = adcToVoltage(rawADC);
    float voltajeAdaptado = adaptarVoltajeTurbidezDFRobot(voltajeESP32);
    float ntu = calcularNTU_DFRobot(voltajeAdaptado);

    Serial.print("[TURBIDEZ] Muestra ");
    Serial.print(i + 1);

    Serial.print(" | ADC: ");
    Serial.print(rawADC);

    Serial.print(" | Voltaje ESP32: ");
    Serial.print(voltajeESP32, 3);
    Serial.print(" V");

    Serial.print(" | Voltaje adaptado: ");
    Serial.print(voltajeAdaptado, 3);
    Serial.print(" V");

    Serial.print(" | Turbidez: ");
    Serial.print(ntu, 2);
    Serial.println(" NTU");

    if (!isnan(voltajeESP32) && !isnan(ntu)) {
      sumaVoltajeESP32 += voltajeESP32;
      sumaVoltajeAdaptado += voltajeAdaptado;
      sumaNTU += ntu;
      validas++;
    }

    delay(100);
  }

  if (validas == 0) {
    Serial.println("[TURBIDEZ] ERROR: no hubo lecturas validas.");
    parametroVoltajeTurbidez = NAN;
    return NAN;
  }

  float promedioVoltajeESP32 = sumaVoltajeESP32 / validas;
  float promedioVoltajeAdaptado = sumaVoltajeAdaptado / validas;
  float promedioNTU = sumaNTU / validas;

  parametroVoltajeTurbidez = promedioVoltajeESP32;

  Serial.println("======================================");
  Serial.println("[TURBIDEZ] Resultado promedio");

  Serial.print("[TURBIDEZ] Voltaje promedio ESP32: ");
  Serial.print(promedioVoltajeESP32, 3);
  Serial.println(" V");

  Serial.print("[TURBIDEZ] Voltaje promedio adaptado DFRobot: ");
  Serial.print(promedioVoltajeAdaptado, 3);
  Serial.println(" V");

  Serial.print("[TURBIDEZ] Turbidez promedio estimada: ");
  Serial.print(promedioNTU, 2);
  Serial.println(" NTU");

  Serial.println("[TURBIDEZ] Nota: valor estimado mediante ecuacion DFRobot adaptada al ESP32.");
  Serial.println("======================================");

  return promedioNTU;
}


// ======================================================
// FUNCION: medir nitratos por ecuacion indirecta
// ======================================================
//
// Ecuación:
// Nitratos = 146.5209 - 15.3177(pH) - 4.0287(Turbidez) - 0.1030(Conductividad)
//
// pH: valor ajustado por voltaje.
// Turbidez: NTU estimado.
// Conductividad: uS/cm ajustado.
//
// Esta es una estimación indirecta, no una medición directa.
float medirNitratos() {
  Serial.println("======================================");
  Serial.println("[NITRATOS] Inicio de calculo por ecuacion");
  Serial.println("======================================");

  if (isnan(parametroPH)) {
    Serial.println("[NITRATOS] ERROR: pH no disponible.");
    return NAN;
  }

  if (isnan(parametroTurbidez)) {
    Serial.println("[NITRATOS] ERROR: turbidez no disponible.");
    return NAN;
  }

  if (isnan(parametroConductividad)) {
    Serial.println("[NITRATOS] ERROR: conductividad no disponible.");
    return NAN;
  }

  float nitratos = 146.5209
                   - (15.3177 * parametroPH)
                   - (4.0287 * parametroTurbidez)
                   - (0.1030 * parametroConductividad);

  if (nitratos < 0.0) {
    nitratos = 0.0;
  }

  Serial.print("[NITRATOS] pH usado: ");
  Serial.println(parametroPH, 2);

  Serial.print("[NITRATOS] Turbidez usada: ");
  Serial.print(parametroTurbidez, 2);
  Serial.println(" NTU");

  Serial.print("[NITRATOS] Conductividad usada: ");
  Serial.print(parametroConductividad, 2);
  Serial.println(" uS/cm");

  Serial.print("[NITRATOS] Resultado estimado: ");
  Serial.print(nitratos, 2);
  Serial.println(" mg/L");

  Serial.println("[NITRATOS] Nota: valor calculado mediante modelo matematico indirecto.");
  Serial.println("======================================");

  return nitratos;
}


// ======================================================
// FUNCION: medir todos los parametros
// ======================================================
//
// Ejecuta las mediciones en orden.
// La temperatura se mide primero porque se usa para compensar conductividad.
void medirTodosLosParametros() {
  Serial.println("======================================");
  Serial.println("[MEDICION] Inicio de medicion general");
  Serial.println("======================================");

  parametroTemperatura   = medirTemperaturaPromedio30();
  parametroPH            = medirPH();
  parametroConductividad = medirConductividad();
  parametroTurbidez      = medirTurbidez();
  parametroNitratos      = medirNitratos();

  Serial.println("======================================");
  Serial.println("[MEDICION] Resumen de parametros");
  Serial.println("======================================");

  Serial.print("Temperatura: ");
  Serial.print(parametroTemperatura, 2);
  Serial.println(" °C");

  Serial.print("Voltaje pH: ");
  Serial.print(parametroVoltajePH, 3);
  Serial.println(" V");

  Serial.print("pH ajustado: ");
  Serial.println(parametroPH, 2);

  Serial.print("Conductividad ajustada: ");
  Serial.print(parametroConductividad, 2);
  Serial.println(" uS/cm");

  Serial.print("Voltaje turbidez: ");
  Serial.print(parametroVoltajeTurbidez, 3);
  Serial.println(" V");

  Serial.print("Turbidez estimada: ");
  Serial.print(parametroTurbidez, 2);
  Serial.println(" NTU");

  Serial.print("Nitratos estimados: ");
  Serial.print(parametroNitratos, 2);
  Serial.println(" mg/L");

  Serial.println("======================================");
}


// ======================================================
// FUNCION: intentar conexion WiFi
// ======================================================
//
// Realiza un intento de conexión con timeout.
bool intentarConexionWiFi(unsigned long timeoutMs) {
  WiFi.disconnect(true);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);

    if (millis() - t0 > timeoutMs) {
      Serial.println();
      Serial.println("[WIFI] ERROR: tiempo maximo de conexion excedido.");
      return false;
    }
  }

  Serial.println();
  Serial.println("[WIFI] Conexion exitosa.");
  Serial.print("[WIFI] IP local: ");
  Serial.println(WiFi.localIP());

  return true;
}


// ======================================================
// FUNCION: conectar WiFi con reintento
// ======================================================
//
// Primer intento: 20 segundos.
// Si falla, espera 1 minuto.
// Segundo intento: 20 segundos.
bool conectarWiFi() {
  Serial.println("======================================");
  Serial.println("[WIFI] Iniciando conexion WiFi");
  Serial.println("======================================");

  Serial.println("[WIFI] Primer intento de conexion...");
  bool conectado = intentarConexionWiFi(20000UL);

  if (conectado) {
    return true;
  }

  Serial.println("[WIFI] No se pudo conectar en el primer intento.");
  Serial.println("[WIFI] Esperando 1 minuto antes de reintentar...");
  delay(TIEMPO_REINTENTO_WIFI_MS);

  Serial.println("[WIFI] Segundo intento de conexion...");
  conectado = intentarConexionWiFi(20000UL);

  if (conectado) {
    return true;
  }

  Serial.println("[WIFI] ERROR: no se pudo conectar despues del segundo intento.");
  return false;
}


// ======================================================
// FUNCION: sincronizar hora por NTP
// ======================================================
//
// Obtiene fecha y hora para almacenar datos con timestamp.
bool sincronizarHoraNTP() {
  Serial.println("======================================");
  Serial.println("[NTP] Sincronizando fecha y hora");
  Serial.println("======================================");

  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov");

  struct tm timeinfo;

  for (int i = 0; i < 20; i++) {
    if (getLocalTime(&timeinfo)) {
      Serial.println("[NTP] Hora sincronizada correctamente.");
      return true;
    }

    Serial.println("[NTP] Esperando respuesta NTP...");
    delay(500);
  }

  Serial.println("[NTP] ERROR: no se pudo sincronizar la hora.");
  return false;
}


// ======================================================
// FUNCION: obtener fecha y hora actual
// ======================================================
//
// Si no hay hora disponible, retorna SIN_HORA.
String obtenerFechaHora() {
  struct tm timeinfo;

  if (!getLocalTime(&timeinfo)) {
    Serial.println("[FECHA/HORA] No disponible.");
    return "SIN_HORA";
  }

  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);

  Serial.print("[FECHA/HORA] Valor actual: ");
  Serial.println(buffer);

  return String(buffer);
}


// ======================================================
// FUNCION: inicializar MicroSD
// ======================================================

bool iniciarSD() {
  Serial.println("======================================");
  Serial.println("[MICROSD] Inicializando MicroSD");
  Serial.println("======================================");

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, spiSD, 1000000)) {
    Serial.println("[MICROSD] ERROR: no se pudo inicializar.");
    return false;
  }

  Serial.println("[MICROSD] Inicializacion correcta.");
  return true;
}


// ======================================================
// FUNCION: guardar CSV en MicroSD
// ======================================================
//
// Guarda los valores finales:
// temperatura, voltaje pH, pH ajustado,
// conductividad ajustada, voltaje turbidez,
// turbidez NTU y nitratos estimados.
void guardarCSVEnSD() {
  Serial.println("======================================");
  Serial.println("[MICROSD] Guardando datos");
  Serial.println("======================================");

  if (!iniciarSD()) {
    Serial.println("[MICROSD] No se guardaron los datos.");
    return;
  }

  bool existe = SD.exists(NOMBRE_ARCHIVO);
  File archivo = SD.open(NOMBRE_ARCHIVO, FILE_APPEND);

  if (!archivo) {
    Serial.println("[MICROSD] ERROR: no se pudo abrir el archivo.");
    return;
  }

  if (!existe) {
    Serial.println("[MICROSD] Archivo no existia. Se escribira encabezado.");
    archivo.println("fecha_hora,temperatura,voltaje_ph,ph,conductividad_uscm,voltaje_turbidez,turbidez_ntu,nitratos_mgl");
  }

  String fechaHora = obtenerFechaHora();

  archivo.print(fechaHora);
  archivo.print(",");
  archivo.print(parametroTemperatura, 2);
  archivo.print(",");
  archivo.print(parametroVoltajePH, 3);
  archivo.print(",");
  archivo.print(parametroPH, 2);
  archivo.print(",");
  archivo.print(parametroConductividad, 2);
  archivo.print(",");
  archivo.print(parametroVoltajeTurbidez, 3);
  archivo.print(",");
  archivo.print(parametroTurbidez, 2);
  archivo.print(",");
  archivo.println(parametroNitratos, 2);

  archivo.close();

  Serial.println("[MICROSD] Datos guardados correctamente en datos.csv");
}


// ======================================================
// FUNCION: enviar datos a ThingSpeak
// ======================================================
//
// field1 = pH
// field2 = temperatura
// field3 = conductividad ajustada uS/cm
// field4 = turbidez NTU
// field5 = nitratos mg/L
// field6 se reserva para leer tiempo de sleep.
bool enviarDatosThingSpeak() {
  Serial.println("======================================");
  Serial.println("[THINGSPEAK] Enviando datos");
  Serial.println("======================================");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[THINGSPEAK] ERROR: WiFi no conectado.");
    return false;
  }

  HTTPClient http;

  String url = "http://api.thingspeak.com/update?api_key=" + String(TS_WRITE_API_KEY) +
               "&field1=" + String(parametroPH, 2) +
               "&field2=" + String(parametroTemperatura, 2) +
               "&field3=" + String(parametroConductividad, 2) +
               "&field4=" + String(parametroTurbidez, 2) +
               "&field5=" + String(parametroNitratos, 2);

  Serial.print("[THINGSPEAK] URL: ");
  Serial.println(url);

  http.begin(url);
  int code = http.GET();

  Serial.print("[THINGSPEAK] Codigo HTTP: ");
  Serial.println(code);

  if (code > 0) {
    String respuesta = http.getString();
    Serial.print("[THINGSPEAK] Respuesta: ");
    Serial.println(respuesta);
    http.end();
    return true;
  }

  http.end();

  Serial.println("[THINGSPEAK] ERROR en envio.");
  return false;
}


// ======================================================
// FUNCION: leer field6 de ThingSpeak
// ======================================================
//
// field6 define los minutos de deep sleep.
// El valor se conserva en RTC_DATA_ATTR.
void leerMinutosSleepDesdeThingSpeak() {
  Serial.println("======================================");
  Serial.println("[THINGSPEAK] Leyendo field6 para tiempo de sleep");
  Serial.println("======================================");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[THINGSPEAK] ERROR: WiFi no conectado.");
    return;
  }

  HTTPClient http;

  String url = "http://api.thingspeak.com/channels/" + String(TS_CHANNEL_ID) +
               "/fields/6/last.txt?api_key=" + String(TS_READ_API_KEY);

  Serial.print("[THINGSPEAK] URL lectura: ");
  Serial.println(url);

  http.begin(url);
  int code = http.GET();

  Serial.print("[THINGSPEAK] Codigo HTTP lectura: ");
  Serial.println(code);

  if (code > 0) {
    String payload = http.getString();
    payload.trim();

    Serial.print("[THINGSPEAK] Valor recibido en field6: ");
    Serial.println(payload);

    int valor = payload.toInt();

    if (valor > 0 && valor <= 1440) {
      minutosSleep = valor;

      Serial.print("[RTC] Nuevo tiempo de sleep guardado: ");
      Serial.print(minutosSleep);
      Serial.println(" minutos");
    } else {
      Serial.println("[RTC] Valor invalido. Se mantiene el anterior.");
    }
  } else {
    Serial.println("[THINGSPEAK] ERROR al leer field6.");
  }

  http.end();
}


// ======================================================
// FUNCION: evaluar rangos y emitir alertas
// ======================================================
//
// Criterios:
// Temperatura < 18 o > 30 °C.
// pH < 6 o > 8.
// Conductividad > 400 uS/cm.
// Turbidez > 5 NTU.
// Nitratos > 25 mg/L: alarma de valor alerta.
// Nitratos > 50 mg/L: alerta de valor máximo admisible.
void evaluarYEmitirAlertas() {
  Serial.println("======================================");
  Serial.println("[ALERTAS] Evaluando parametros");
  Serial.println("======================================");

  bool existeAlerta = false;

  if (!isnan(parametroTemperatura)) {
    if (parametroTemperatura < TEMP_MIN || parametroTemperatura > TEMP_MAX) {
      existeAlerta = true;
      Serial.print("[ALERTA] Temperatura fuera de rango: ");
      Serial.print(parametroTemperatura, 2);
      Serial.println(" °C");
    }
  }

  if (!isnan(parametroPH)) {
    if (parametroPH < PH_MIN || parametroPH > PH_MAX) {
      existeAlerta = true;
      Serial.print("[ALERTA] pH fuera de rango: ");
      Serial.println(parametroPH, 2);
    }
  }

  if (!isnan(parametroConductividad)) {
    if (parametroConductividad > COND_MAX) {
      existeAlerta = true;
      Serial.print("[ALERTA] Conductividad mayor a 400 uS/cm: ");
      Serial.print(parametroConductividad, 2);
      Serial.println(" uS/cm");
    }
  }

  if (!isnan(parametroTurbidez)) {
    if (parametroTurbidez > TURB_MAX) {
      existeAlerta = true;
      Serial.print("[ALERTA] Turbidez mayor a 5 NTU: ");
      Serial.print(parametroTurbidez, 2);
      Serial.println(" NTU");
    }
  }

  if (!isnan(parametroNitratos)) {
    if (parametroNitratos > NITR_VMA) {
      existeAlerta = true;
      Serial.print("[ALERTA VMA] Nitratos superan valor maximo admisible de 50 mg/L: ");
      Serial.print(parametroNitratos, 2);
      Serial.println(" mg/L");
    } 
    else if (parametroNitratos > NITR_ALERTA) {
      existeAlerta = true;
      Serial.print("[ALARMA VALOR ALERTA] Nitratos superan 25 mg/L: ");
      Serial.print(parametroNitratos, 2);
      Serial.println(" mg/L");
    }
  }

  if (!existeAlerta) {
    Serial.println("[ALERTAS] Todos los parametros dentro de los rangos configurados.");
  }

  Serial.println("======================================");
}


// ======================================================
// FUNCION: dormir ESP32
// ======================================================
//
// Apaga WiFi y entra en deep sleep.
// El tiempo se define en minutosSleep.
void dormirESP32() {
  Serial.println("======================================");
  Serial.println("[SLEEP] Preparando deep sleep");
  Serial.println("======================================");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  Serial.print("[SLEEP] Tiempo programado: ");
  Serial.print(minutosSleep);
  Serial.println(" minutos");

  esp_sleep_enable_timer_wakeup((uint64_t)minutosSleep * 60ULL * 1000000ULL);

  Serial.println("[SLEEP] Entrando en deep sleep...");
  delay(500);

  esp_deep_sleep_start();
}


// ======================================================
// SETUP
// ======================================================
//
// Configura pines, sensores, ADC, DAC y estado inicial.
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("======================================");
  Serial.println("[SETUP] Inicio del sistema");
  Serial.println("======================================");

  pinMode(PIN_NIVEL, INPUT_PULLUP);

  pinMode(PIN_BOMBA_A, OUTPUT);
  pinMode(PIN_BOMBA_B, OUTPUT);

  setBombaA(false);
  setBombaB(false);

  sensorTemp.begin();

  // Configuración ADC general.
  analogReadResolution(12);

  // ADC_11db permite ampliar el rango de lectura del ADC.
  analogSetPinAttenuation(PIN_PH, ADC_11db);
  analogSetPinAttenuation(PIN_TURBIDEZ, ADC_11db);
  analogSetPinAttenuation(PIN_COND_ADC_A, ADC_11db);
  analogSetPinAttenuation(PIN_COND_ADC_B, ADC_11db);

  // Inicializar DAC de conductividad en nivel bajo.
  dacWrite(PIN_COND_DAC_A, COND_DAC_BAJO);
  dacWrite(PIN_COND_DAC_B, COND_DAC_BAJO);

  Serial.println("[SETUP] Sensor de temperatura iniciado.");
  Serial.println("[SETUP] Medicion de pH configurada por voltaje.");
  Serial.println("[SETUP] Conductividad configurada con ajuste por regresion.");
  Serial.println("[SETUP] Turbidez configurada con ecuacion DFRobot adaptada.");
  Serial.println("[SETUP] Sistema listo.");
}


// ======================================================
// LOOP PRINCIPAL
// ======================================================
//
// Flujo principal:
// 1. Captar muestra.
// 2. Medir parámetros.
// 3. Conectar WiFi.
// 4. Sincronizar hora.
// 5. Guardar en MicroSD.
// 6. Enviar a ThingSpeak.
// 7. Leer tiempo de sleep.
// 8. Evaluar alertas.
// 9. Dormir ESP32.
void loop() {
  Serial.println("======================================");
  Serial.println("[LOOP] Nueva iteracion del sistema");
  Serial.println("======================================");

  // 1. Control de captación de muestra.
  bool okCaptacion = etapaCaptacionInicial();

  if (!okCaptacion) {
    Serial.println("[LOOP] ERROR en etapa de captacion.");
    minutosSleep = 5;
    dormirESP32();
  }

  // 2. Medición de todos los parámetros.
  medirTodosLosParametros();

  // 3. Conexión WiFi.
  bool wifiOK = conectarWiFi();

  // 4. Sincronización de hora.
  if (wifiOK) {
    sincronizarHoraNTP();
  } else {
    Serial.println("[LOOP] Se continua sin WiFi. La fecha podria quedar SIN_HORA.");
  }

  // 5. Guardar datos localmente.
  guardarCSVEnSD();

  // 6. Enviar datos y leer nuevo tiempo de sleep.
  if (wifiOK) {
    enviarDatosThingSpeak();
    leerMinutosSleepDesdeThingSpeak();
  } else {
    Serial.println("[LOOP] No se enviaron datos a ThingSpeak por falta de WiFi.");
  }

  // 7. Evaluar alertas.
  evaluarYEmitirAlertas();

  // 8. Entrar en deep sleep.
  dormirESP32();
}
