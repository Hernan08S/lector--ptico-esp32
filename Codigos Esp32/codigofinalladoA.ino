/*////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   Lado A: PC/FT232 <-> ESP32 <-> Bluetooth (cliente SPP) 
   - Puentea PC <--> FT232 <-> BT.
   - IMPRIME solo lo que recibe por Bluetooth (respuestas del medidor).
   - Salida: ASCII + HEX, timestamp y dirección.
   - Mapea caracteres de control: <SOH><STX><ETX><ACK><NAK><ENQ><EOT><CR><LF> estos identificados solo por la normativa IC 62056-21
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
#include <Arduino.h>
#include "BluetoothSerial.h"

// ----- Opciones de log ------
#define LOG_BITS   1  // 0 = no imprimir bits; 1 = imprimir bits por byte sin los de paridad
#define LOG_CSV    0   // 0 = formato legible; 1 = CSV (para Excel)

// ---------------- Configuración ----------------
#define USB_BAUD     115200

// UART hacia FT232 (pines del ESP32)
#define FT232_UART   Serial1
#define FT232_BAUD   9600
#define FT232_CFG    SERIAL_8N1
#define FT232_RX_PIN 16   // ESP32 RX  <- FT232 TX
#define FT232_TX_PIN 17   // ESP32 TX  -> FT232 RX

//Bloque Bluetooth SPP (A es cliente) y definicion de parametros
BluetoothSerial BT;
#define BT_LOCAL_NAME  "ESP32_A_FT232" //nombre del usuario (esclavo) 
#define BT_REMOTE_NAME "ESP32_B_METER" //nombre del usuario (servidor)
#define BT_RETRY_MS    3000            //Tiempo de espera para reconexión

// ---------------- Buffers ----------------
static uint8_t bufFT[256];
static uint8_t bufBT[256];

// ---------------- Log ASCII/HEX/BITS ----------------
static String  acc_A_BT_FT232;          // ASCII acumulada (A:BT->FT232) muestra texto legible 
static uint8_t raw_A_BT_FT232[512];     // arreglo de bytes crudos de la línea, el tamaño es 512 por una trama máxima extendida IEC 62056-21 
                                        //puede llegar a: varias decenas de bytes (modo E) y más de 200 bytes si el medidor envía perfiles completos con esto se cumple un margen de tamaño 
static size_t  raw_A_BT_FT232_n = 0;    //Un contador que dice cuántos bytes hay almacenados en el arreglo, sirve para para saber hasta dónde imprimir el HEX verdadero,
                                        //para el cálculo del BCC, para vaciar el buffer cuando se termina la trama cuando llega CR/LF.
// Estado para BCC (A:BT->FT232)
static bool   a_in_block = false;      // estamos dentro de STX..ETX el bloque de control de datos
static bool   a_wait_bcc = false;      // el próximo byte es el BCC para poder enviar un ok o un error
static uint8_t a_bcc_run = 0;          // XOR acumulado para la comprobacion 

//---------identificar rápidamente qué parte de la comunicación------------------
static inline const char* tagOf(const String& s) {
  if (s.startsWith("/?!")) return "Solicitud";
  if (s.indexOf(')') != -1) return "Trama";
  return "ASCII";
}

static String hexOf(const uint8_t* data, size_t n) {
  static const char* H = "0123456789ABCDEF";
  String out; out.reserve(3*n);
  for (size_t i = 0; i < n; ++i) {
    uint8_t b = data[i];
    out += H[(b>>4)&0xF]; 
    out += H[b&0xF];
    if (i+1<n) out += ' ';
  }
  return out;
}

static String bitsOf(uint8_t b) {
  String s; s.reserve(8);
  for (int i = 7; i >= 0; --i) s += ((b>>i)&1)?'1':'0';
  return s;
}

// Mapea bytes a visible ASCII con etiquetas para control
static inline void appendVisible(String &acc, uint8_t b) {
  switch (b) {
    case 0x01: acc += "<SOH>"; break;
    case 0x02: acc += "<STX>"; break;
    case 0x03: acc += "<ETX>"; break;
    case 0x04: acc += "<EOT>"; break;
    case 0x05: acc += "<ENQ>"; break;
    case 0x06: acc += "<ACK>"; break;
    case 0x15: acc += "<NAK>"; break;
    case 0x0D: acc += "<CR>";  break;
    case 0x0A: acc += "<LF>";  break;
    default:
      if (b >= 32 && b <= 126) acc += (char)b; else acc += '.';
      break;
  }
}

// --- Cierra e imprime una línea (ASCII + HEX [+BITS])
static void flushLine(const char* dir, String &acc) {
  if (acc.length() == 0) return;
  unsigned long now = millis();
  const char* tag = tagOf(acc);
  String hex = hexOf(raw_A_BT_FT232, raw_A_BT_FT232_n);

  if (LOG_CSV) {
    // CSV: time_ms,dir,tag,len,ascii,hex
    Serial.print(now); Serial.print(",");
    Serial.print(dir); Serial.print(",");
    Serial.print(tag); Serial.print(",");
    Serial.print(raw_A_BT_FT232_n); Serial.print(",\"");
    Serial.print(acc); Serial.print("\",\"");
    Serial.print(hex); Serial.println("\"");
    if (LOG_BITS) {
      Serial.print(now); Serial.print(",");
      Serial.print(dir); Serial.print(",");
      Serial.print("BITS,");
      Serial.print(raw_A_BT_FT232_n); Serial.print(",\"");
      for (size_t i=0;i<raw_A_BT_FT232_n;i++){
        Serial.print(bitsOf(raw_A_BT_FT232[i]));
        if (i+1<raw_A_BT_FT232_n) Serial.print(" | ");
      }
      Serial.println("\"");
    }
  } else {
    String line; line.reserve(acc.length()+64);
    line += '['; line += String(now); line += " ms] [";
    line += dir; line += "] ";
    line += tag; line += " len="; line += String(raw_A_BT_FT232_n);
    line += "\nASCII: "; line += acc;
    line += "\nHEX:   "; line += hex;
    Serial.println(line);
    if (LOG_BITS) {
      Serial.print("BITS:  ");
      for (size_t i=0;i<raw_A_BT_FT232_n;i++){
        Serial.print(bitsOf(raw_A_BT_FT232[i]));
        if (i+1<raw_A_BT_FT232_n) Serial.print(" | ");
      }
      Serial.println();
    }
    Serial.println("----");
  }

  acc = "";
  raw_A_BT_FT232_n = 0;
}

// --- Acumula bytes hasta CR o LF (incluyéndolos visiblemente)
static void logFrameA(const char* dir, String &acc, const uint8_t* buf, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    uint8_t b = buf[i];
    // Registrar el byte crudo
    if (raw_A_BT_FT232_n < sizeof(raw_A_BT_FT232)) raw_A_BT_FT232[raw_A_BT_FT232_n++] = b;
    // ¿Estamos esperando el BCC (inmediatamente después de ETX)?
    if (a_wait_bcc) {
      // Mostrar BCC y validar
      char tag[20];
      sprintf(tag, "<BCC=%02X ", b);
      acc += tag;
      acc += (b == a_bcc_run) ? "OK>" : "BAD>";
      a_wait_bcc = false;
      a_in_block = false;   // terminó el bloque STX..ETX+BCC
      // No hacemos flush aquí; dejamos que CR/LF cierre la línea
      continue;
    }
    // Manejo de control para STX/ETX
    if (b == 0x02) {                     // STX
      a_in_block = true;
      a_bcc_run = 0;                     // BCC comienza DESPUÉS de STX
      appendVisible(acc, b);
      continue;
    }
    if (b == 0x03) {                     // ETX
      // ETX se INCLUYE en el XOR
      if (a_in_block) a_bcc_run ^= b;
      appendVisible(acc, b);
      a_wait_bcc = true;                 // el siguiente byte es el BCC recibido
      continue;
    }
    // Si estamos dentro del bloque, acumular XOR desde el PRIMER byte tras STX
    if (a_in_block) a_bcc_run ^= b;
    // Mostrar visible el byte (con etiquetas si es control)
    appendVisible(acc, b);
    // Cierre por CR/LF
    if (b == 0x0D || b == 0x0A) {
      flushLine(dir, acc);
    }
  }
}


// ---------------- Setup ----------------
unsigned long lastTry = 0;

void setup() {
  Serial.begin(USB_BAUD);
  delay(200);
  Serial.println();
  Serial.println("=== Lado A: FT232 <-> ESP32 <-> Bluetooth (cliente) ===");

  FT232_UART.begin(FT232_BAUD, FT232_CFG, FT232_RX_PIN, FT232_TX_PIN);
  Serial.print("FT232 UART @ "); Serial.print(FT232_BAUD);
  Serial.print(", cfg=8N1, RX="); Serial.print(FT232_RX_PIN);
  Serial.print(", TX="); Serial.println(FT232_TX_PIN);

  // ESP32 como cliente (master)
  BT.begin(BT_LOCAL_NAME, true);
  Serial.print("BT local: "); Serial.print(BT_LOCAL_NAME);
  Serial.print(" (cliente). Buscando: "); Serial.println(BT_REMOTE_NAME);
}

// ---------------- Loop ----------------
void loop() {
  // Conexión al servidor SPP remoto
  if (!BT.hasClient()) {
    if (millis() - lastTry > BT_RETRY_MS) {
      Serial.println("Conectando a servidor SPP remoto...");
      if (BT.connect(BT_REMOTE_NAME)) {
        Serial.println("-> Conectado a ESP32_B_METER por SPP");
      } else {
        Serial.println("-> No conectó. Reintentará.");
      }
      lastTry = millis();
    }
  }

  // 1) FT232 -> BT (SE TRANSMITE, PERO NO SE IMPRIME EN A)
  int n1 = FT232_UART.available();
  if (n1 > 0) {
    if (n1 > (int)sizeof(bufFT)) n1 = sizeof(bufFT);
    n1 = FT232_UART.readBytes(bufFT, n1);
    if (n1 > 0 && BT.hasClient()) {
      BT.write(bufFT, n1);
      // (silenciado) log de A:FT232->BT deshabilitado para no mezclar
    }}
  // 2) BT -> FT232 (ESTE SÍ IMPRIME: RESPUESTAS DEL MEDIDOR)
  int n2 = BT.available();
  if (n2 > 0) {
    if (n2 > (int)sizeof(bufBT)) n2 = sizeof(bufBT);
    n2 = BT.readBytes(bufBT, n2);
    if (n2 > 0) {
      FT232_UART.write(bufBT, n2);
      logFrameA("A:BT->FT232", acc_A_BT_FT232, bufBT, n2);
    }
  }
}
