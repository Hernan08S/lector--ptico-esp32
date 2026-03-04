
/* ////////////////////////////////////////////////////////////////////////////////////
  Lado B: Bluetooth (servidor SPP) <-> ESP32 <-> Medidor (UART)
  - Puentea BT<->MEDIDOR.
  - IMPRIME SOLO lo que RECIBE por Bluetooth (solicitudes del FT232).
  - Salida: ASCII + HEX (+ BITS opcional), timestamp y dirección.
  - Mapea caracteres de control: <STX><ETX><ACK><NAK><ENQ><EOT><CR><LF>
  <SOH><STX><ETX><ACK><NAK><ENQ><EOT><CR><LF>
  ////////////////////////////////////////////////////////////////////////////////////// */

#include <Arduino.h>
#include "BluetoothSerial.h"

// ===== Opciones de log (para tesis) =====
#define LOG_BITS_B   0   // 0 = no imprimir bits; 1 = imprimir bits por byte
#define LOG_CSV_B    0   // 0 = formato legible; 1 = CSV (para Excel)

// ---------------- Configuración ----------------
#define USB_BAUD_B     115200

// UART hacia MEDIDOR (pines del ESP32)
#define METER_UART          Serial1
#define METER_BAUD          9600
#define METER_CFG           SERIAL_8N1
#define METER_RX_PIN        16
#define METER_TX_PIN        17

// Bluetooth SPP (B es servidor)
BluetoothSerial BT_B;
#define BT_SERVER_NAME      "ESP32_B_METER"

// ---------------- Buffers ----------------
static uint8_t bufM[256];
static uint8_t bufBTb[256];

// Estado para BCC (B:BT->MEDIDOR)
static bool    b_in_block = false;
static bool    b_wait_bcc = false;
static uint8_t b_bcc_run  = 0;


// ---------------- Log ASCII/HEX/BITS ----------------
static String  acc_B_BT_MEDIDOR;            // ASCII acumulada (B:BT->MEDIDOR)
static uint8_t raw_B_BT_MEDIDOR[512];       // bytes crudos de la línea
static size_t  raw_B_BT_MEDIDOR_n = 0;

static inline const char* tagOfB(const String& s) {
  if (s.startsWith("/?!")) return "Solicitud";
  if (s.indexOf(')') != -1) return "Trama";
  return "ASCII";
}

static String hexOfB(const uint8_t* data, size_t n) {
  static const char* H = "0123456789ABCDEF";
  String out; out.reserve(3*n);
  for (size_t i = 0; i < n; ++i) {
    uint8_t b = data[i];
    out += H[(b>>4)&0xF]; out += H[b&0xF];
    if (i+1<n) out += ' ';
  }
  return out;
}

static String bitsOfB(uint8_t b) {
  String s; s.reserve(8);
  for (int i = 7; i >= 0; --i) s += ((b>>i)&1)?'1':'0';
  return s;
}

static inline void appendVisibleB(String &acc, uint8_t b) {
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
static void flushLineB(const char* dir, String &acc) {
  if (acc.length() == 0) return;
  unsigned long now = millis();
  const char* tag = tagOfB(acc);
  String hex = hexOfB(raw_B_BT_MEDIDOR, raw_B_BT_MEDIDOR_n);

  if (LOG_CSV_B) {
    Serial.print(now); Serial.print(",");
    Serial.print(dir); Serial.print(",");
    Serial.print(tag); Serial.print(",");
    Serial.print(raw_B_BT_MEDIDOR_n); Serial.print(",\"");
    Serial.print(acc); Serial.print("\",\"");
    Serial.print(hex); Serial.println("\"");
    if (LOG_BITS_B) {
      Serial.print(now); Serial.print(",");
      Serial.print(dir); Serial.print(",");
      Serial.print("BITS,");
      Serial.print(raw_B_BT_MEDIDOR_n); Serial.print(",\"");
      for (size_t i=0;i<raw_B_BT_MEDIDOR_n;i++){
        Serial.print(bitsOfB(raw_B_BT_MEDIDOR[i]));
        if (i+1<raw_B_BT_MEDIDOR_n) Serial.print(" | ");
      }
      Serial.println("\"");
    }
  } else {
    String line; line.reserve(acc.length()+64);
    line += '['; line += String(now); line += " ms] [";
    line += dir; line += "] ";
    line += tag; line += " len="; line += String(raw_B_BT_MEDIDOR_n);
    line += "\nASCII: "; line += acc;
    line += "\nHEX:   "; line += hex;
    Serial.println(line);
    if (LOG_BITS_B) {
      Serial.print("BITS:  ");
      for (size_t i=0;i<raw_B_BT_MEDIDOR_n;i++){
        Serial.print(bitsOfB(raw_B_BT_MEDIDOR[i]));
        if (i+1<raw_B_BT_MEDIDOR_n) Serial.print(" | ");
      }
      Serial.println();
    }
    Serial.println("----");
  }

  acc = "";
  raw_B_BT_MEDIDOR_n = 0;
}

// --- Acumula bytes hasta CR o LF (incluyéndolos visiblemente)
static void logFrameB(const char* dir, String &acc, const uint8_t* buf, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    uint8_t b = buf[i];

    if (raw_B_BT_MEDIDOR_n < sizeof(raw_B_BT_MEDIDOR)) raw_B_BT_MEDIDOR[raw_B_BT_MEDIDOR_n++] = b;

    if (b_wait_bcc) {
      char tag[20];
      sprintf(tag, "<BCC=%02X ", b);
      acc += tag;
      acc += (b == b_bcc_run) ? "OK>" : "BAD>";
      b_wait_bcc = false;
      b_in_block = false;
      continue;
    }

    if (b == 0x02) { // STX
      b_in_block = true;
      b_bcc_run = 0;
      appendVisibleB(acc, b);
      continue;
    }
    if (b == 0x03) { // ETX
      if (b_in_block) b_bcc_run ^= b; // ETX se incluye
      appendVisibleB(acc, b);
      b_wait_bcc = true;              // próximo byte es el BCC
      continue;
    }

    if (b_in_block) b_bcc_run ^= b;

    appendVisibleB(acc, b);

    if ( b == 0x0A) {
      flushLineB(dir, acc);
    }
  }
}
   

// ---------------- Setup ----------------
void setup() {
  Serial.begin(USB_BAUD_B);
  delay(200);
  Serial.println();
  Serial.println("=== Lado B: Bluetooth <-> ESP32 <-> MEDIDOR (servidor) ===");

  METER_UART.begin(METER_BAUD, METER_CFG, METER_RX_PIN, METER_TX_PIN);
  Serial.print("METER UART @ "); Serial.print(METER_BAUD);
  Serial.print(", cfg=8N1, RX="); Serial.print(METER_RX_PIN);
  Serial.print(", TX="); Serial.println(METER_TX_PIN);

  BT_B.begin(BT_SERVER_NAME); // servidor SPP
  Serial.print("BT servidor listo: "); Serial.println(BT_SERVER_NAME);
}

// ---------------- Loop ----------------
void loop() {
  // 1) MEDIDOR -> BT (se transmite, pero NO se imprime en B)
  int n1 = METER_UART.available();
  if (n1 > 0) {
    if (n1 > (int)sizeof(bufM)) n1 = sizeof(bufM);
    n1 = METER_UART.readBytes(bufM, n1);
    if (n1 > 0 && BT_B.hasClient()) {
      BT_B.write(bufM, n1);
      // (silenciado) log de B:MEDIDOR->BT deshabilitado para no mezclar
    }
  }

  // 2) BT -> MEDIDOR (ESTE SÍ IMPRIME: SOLICITUDES DEL FT232)
  int n2 = BT_B.available();
  if (n2 > 0) {
    if (n2 > (int)sizeof(bufBTb)) n2 = sizeof(bufBTb);
    n2 = BT_B.readBytes(bufBTb, n2);
    if (n2 > 0) {
      METER_UART.write(bufBTb, n2);
      logFrameB("B:BT->MEDIDOR", acc_B_BT_MEDIDOR, bufBTb, n2);
    }
  }
}
