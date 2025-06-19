/*
 * @Description: Gateway Modbus RTU Slave - CAN J1939 con WiFi e Web Interface
 * Legge dati motore da CAN J1939, li espone tramite Modbus RTU su RS485
 * e fornisce interfaccia web per configurazione e monitoraggio
 * @Author: Your Name
 * @Date: 2024
 */

#include <Arduino.h>
#include <ModbusSlave.h>
#include "driver/twai.h"

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <DNSServer.h>


// RS485
#define RS485_TX 22
#define RS485_RX 21
#define RS485_CALLBACK 17
#define RS485_EN 19

// WS2812B
#define WS2812B_DATA 4

// CAN
#define CAN_TX 27
#define CAN_RX 26
#define CAN_SPEED_MODE 23

// RS485 and CAN Boost power supply
#define ME2107_EN 16 

//SD
#define SD_MISO 2
#define SD_MOSI 15
#define SD_SCLK 14
#define SD_CS 13


// Configurazione Modbus
#define MODBUS_SLAVE_ID 1
#define MODBUS_BAUDRATE 19200
#define MODBUS_SERIAL_MODE SERIAL_8N1

// Configurazione Access Point per provisioning
#define AP_SSID "Gateway_Setup"
#define AP_PASSWORD "12345678"

// Registri Modbus (Holding Registers)
#define MB_REG_ENGINE_RPM           0   // 2 registri (32-bit)
#define MB_REG_ENGINE_TEMP          2   // 1 registro (16-bit) 
#define MB_REG_OIL_PRESSURE         3   // 1 registro (16-bit)
#define MB_REG_FUEL_RATE            4   // 2 registri (32-bit)
#define MB_REG_ENGINE_HOURS         6   // 2 registri (32-bit)
#define MB_REG_COOLANT_TEMP         8   // 1 registro (16-bit)
#define MB_REG_INTAKE_TEMP          9   // 1 registro (16-bit)
#define MB_REG_EXHAUST_TEMP         10  // 1 registro (16-bit)
#define MB_REG_ENGINE_LOAD          11  // 1 registro (16-bit)
#define MB_REG_THROTTLE_POS         12  // 1 registro (16-bit)
#define MB_REG_ENGINE_TORQUE        13  // 2 registri (32-bit)
#define MB_REG_BATTERY_VOLTAGE      15  // 1 registro (16-bit)
#define MB_REG_STATUS_FLAGS         16  // 1 registro (16-bit)
#define MB_REG_ERROR_FLAGS          17  // 1 registro (16-bit)
#define MB_REG_DTC_COUNT            18  // 1 registro (16-bit)
#define MB_REG_LAST_UPDATE          19  // 2 registri (32-bit timestamp)

#define MODBUS_REGISTERS_COUNT      21

// J1939 PGN comuni per motori
#define PGN_ENGINE_SPEED            0xF004  // 61444 - Engine Speed
#define PGN_ENGINE_TEMP             0xFEEE  // 65262 - Engine Temperature
#define PGN_ENGINE_FLUID_LEVEL      0xFEFC  // 65276 - Engine Fluid Level/Pressure
#define PGN_ENGINE_HOURS            0xFEE5  // 65253 - Engine Hours
#define PGN_FUEL_ECONOMY            0xFEF2  // 65266 - Fuel Economy
#define PGN_INTAKE_EXHAUST_COND     0xFEB4  // 65204 - Intake/Exhaust Conditions
#define PGN_ELECTRONIC_ENGINE_1     0xF003  // 61443 - Electronic Engine Controller 1
#define PGN_ELECTRONIC_ENGINE_2     0xF004  // 61444 - Electronic Engine Controller 2
#define PGN_VEHICLE_ELECTRICAL      0xFEF7  // 65271 - Vehicle Electrical Power
#define PGN_DIAGNOSTIC_MESSAGE_1    0xFECA  // 65226 - DM1 Active Diagnostic Trouble Codes

// Struttura dati motore
struct EngineData {
    uint32_t rpm;              // Giri motore
    uint16_t engineTemp;       // Temperatura motore (°C * 10)
    uint16_t oilPressure;      // Pressione olio (kPa)
    uint32_t fuelRate;         // Consumo carburante (L/h * 100)
    uint32_t engineHours;      // Ore di funzionamento
    uint16_t coolantTemp;      // Temperatura liquido raffreddamento (°C * 10)
    uint16_t intakeTemp;       // Temperatura aria aspirazione (°C * 10)
    uint16_t exhaustTemp;      // Temperatura gas scarico (°C * 10)
    uint16_t engineLoad;       // Carico motore (%)
    uint16_t throttlePos;      // Posizione acceleratore (%)
    uint32_t engineTorque;     // Coppia motore (Nm)
    uint16_t batteryVoltage;   // Tensione batteria (V * 10)
    uint16_t statusFlags;      // Flag di stato
    uint16_t errorFlags;       // Flag errori
    uint16_t dtcCount;         // Numero codici errore attivi
    uint32_t lastUpdate;       // Timestamp ultimo aggiornamento
} engineData;

// Oggetti globali
Modbus modbus(Serial1, MODBUS_SLAVE_ID);
AsyncWebServer server(80);
DNSServer dnsServer;
Preferences preferences;

// Buffer registri Modbus
uint16_t modbusRegisters[MODBUS_REGISTERS_COUNT];

// Variabili WiFi
bool wifiConfigured = false;
bool apMode = false;
String ssid = "";
String password = "";

// HTML per pagina web
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <title>Gateway Modbus-CAN J1939</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { 
            font-family: Arial; 
            margin: 20px;
            background-color: #f0f0f0;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
            background-color: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
        }
        h1 { 
            color: #333;
            text-align: center;
        }
        .data-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 15px;
            margin-top: 20px;
        }
        .data-item {
            background-color: #f8f8f8;
            padding: 15px;
            border-radius: 5px;
            border: 1px solid #ddd;
        }
        .data-label {
            font-weight: bold;
            color: #555;
            margin-bottom: 5px;
        }
        .data-value {
            font-size: 1.2em;
            color: #333;
        }
        .status {
            padding: 5px 10px;
            border-radius: 3px;
            display: inline-block;
            margin-top: 10px;
        }
        .status-ok { background-color: #4CAF50; color: white; }
        .status-warning { background-color: #ff9800; color: white; }
        .status-error { background-color: #f44336; color: white; }
        .config-section {
            margin-top: 30px;
            padding: 20px;
            background-color: #f0f0f0;
            border-radius: 5px;
        }
        input[type="text"], input[type="password"], input[type="number"] {
            width: 100%;
            padding: 8px;
            margin: 5px 0;
            box-sizing: border-box;
        }
        button {
            background-color: #4CAF50;
            color: white;
            padding: 10px 20px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            margin-top: 10px;
        }
        button:hover {
            background-color: #45a049;
        }
        .register-map {
            margin-top: 20px;
            font-size: 0.9em;
        }
        table {
            width: 100%;
            border-collapse: collapse;
        }
        th, td {
            border: 1px solid #ddd;
            padding: 8px;
            text-align: left;
        }
        th {
            background-color: #f2f2f2;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Gateway Modbus-CAN J1939</h1>
        
        <div id="connectionStatus"></div>
        
        <h2>Dati Motore in Tempo Reale</h2>
        <div class="data-grid" id="engineData">
            <div class="data-item">
                <div class="data-label">RPM Motore</div>
                <div class="data-value" id="rpm">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Temperatura Motore</div>
                <div class="data-value" id="engineTemp">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Pressione Olio</div>
                <div class="data-value" id="oilPressure">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Consumo Carburante</div>
                <div class="data-value" id="fuelRate">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Ore di Funzionamento</div>
                <div class="data-value" id="engineHours">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Temp. Liquido Raff.</div>
                <div class="data-value" id="coolantTemp">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Temp. Aria Aspirazione</div>
                <div class="data-value" id="intakeTemp">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Temp. Gas Scarico</div>
                <div class="data-value" id="exhaustTemp">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Carico Motore</div>
                <div class="data-value" id="engineLoad">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Posizione Acceleratore</div>
                <div class="data-value" id="throttlePos">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Coppia Motore</div>
                <div class="data-value" id="engineTorque">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Tensione Batteria</div>
                <div class="data-value" id="batteryVoltage">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Codici Errore Attivi</div>
                <div class="data-value" id="dtcCount">-</div>
            </div>
            <div class="data-item">
                <div class="data-label">Ultimo Aggiornamento</div>
                <div class="data-value" id="lastUpdate">-</div>
            </div>
        </div>
        
        <div class="config-section">
            <h3>Configurazione WiFi</h3>
            <form id="wifiForm">
                <label>SSID:</label>
                <input type="text" id="ssid" name="ssid" required>
                
                <label>Password:</label>
                <input type="password" id="password" name="password">
                
                <button type="submit">Salva Configurazione WiFi</button>
            </form>
        </div>
        
        <div class="config-section">
            <h3>Configurazione Modbus</h3>
            <form id="modbusForm">
                <label>Slave ID:</label>
                <input type="number" id="slaveId" name="slaveId" min="1" max="247" value="1">
                
                <label>Baudrate:</label>
                <select id="baudrate" name="baudrate">
                    <option value="9600">9600</option>
                    <option value="19200" selected>19200</option>
                    <option value="38400">38400</option>
                    <option value="57600">57600</option>
                    <option value="115200">115200</option>
                </select>
                
                <button type="submit">Salva Configurazione Modbus</button>
            </form>
        </div>
        
        <div class="register-map">
            <h3>Mappa Registri Modbus</h3>
            <table>
                <tr>
                    <th>Registro</th>
                    <th>Descrizione</th>
                    <th>Tipo</th>
                    <th>Unità</th>
                </tr>
                <tr><td>0-1</td><td>RPM Motore</td><td>32-bit</td><td>RPM</td></tr>
                <tr><td>2</td><td>Temperatura Motore</td><td>16-bit</td><td>°C × 10</td></tr>
                <tr><td>3</td><td>Pressione Olio</td><td>16-bit</td><td>kPa</td></tr>
                <tr><td>4-5</td><td>Consumo Carburante</td><td>32-bit</td><td>L/h × 100</td></tr>
                <tr><td>6-7</td><td>Ore di Funzionamento</td><td>32-bit</td><td>ore</td></tr>
                <tr><td>8</td><td>Temp. Liquido Raffreddamento</td><td>16-bit</td><td>°C × 10</td></tr>
                <tr><td>9</td><td>Temp. Aria Aspirazione</td><td>16-bit</td><td>°C × 10</td></tr>
                <tr><td>10</td><td>Temp. Gas Scarico</td><td>16-bit</td><td>°C × 10</td></tr>
                <tr><td>11</td><td>Carico Motore</td><td>16-bit</td><td>%</td></tr>
                <tr><td>12</td><td>Posizione Acceleratore</td><td>16-bit</td><td>%</td></tr>
                <tr><td>13-14</td><td>Coppia Motore</td><td>32-bit</td><td>Nm</td></tr>
                <tr><td>15</td><td>Tensione Batteria</td><td>16-bit</td><td>V × 10</td></tr>
                <tr><td>16</td><td>Flag di Stato</td><td>16-bit</td><td>-</td></tr>
                <tr><td>17</td><td>Flag Errori</td><td>16-bit</td><td>-</td></tr>
                <tr><td>18</td><td>Numero Codici Errore</td><td>16-bit</td><td>-</td></tr>
                <tr><td>19-20</td><td>Timestamp Ultimo Aggiornamento</td><td>32-bit</td><td>ms</td></tr>
            </table>
        </div>
    </div>
    
    <script>
        // Aggiorna dati ogni 2 secondi
        setInterval(updateData, 2000);
        
        function updateData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('rpm').textContent = data.rpm + ' RPM';
                    document.getElementById('engineTemp').textContent = (data.engineTemp / 10).toFixed(1) + ' °C';
                    document.getElementById('oilPressure').textContent = data.oilPressure + ' kPa';
                    document.getElementById('fuelRate').textContent = (data.fuelRate / 100).toFixed(2) + ' L/h';
                    document.getElementById('engineHours').textContent = data.engineHours + ' h';
                    document.getElementById('coolantTemp').textContent = (data.coolantTemp / 10).toFixed(1) + ' °C';
                    document.getElementById('intakeTemp').textContent = (data.intakeTemp / 10).toFixed(1) + ' °C';
                    document.getElementById('exhaustTemp').textContent = (data.exhaustTemp / 10).toFixed(1) + ' °C';
                    document.getElementById('engineLoad').textContent = data.engineLoad + ' %';
                    document.getElementById('throttlePos').textContent = data.throttlePos + ' %';
                    document.getElementById('engineTorque').textContent = data.engineTorque + ' Nm';
                    document.getElementById('batteryVoltage').textContent = (data.batteryVoltage / 10).toFixed(1) + ' V';
                    document.getElementById('dtcCount').textContent = data.dtcCount;
                    
                    let statusText = '';
                    let statusClass = '';
                    
                    if (data.statusFlags & 0x8000) {
                        statusText = 'Errore Comunicazione CAN';
                        statusClass = 'status-error';
                    } else {
                        let timeSinceUpdate = Date.now() - data.lastUpdate;
                        if (timeSinceUpdate < 1000) {
                            statusText = 'Connesso';
                            statusClass = 'status-ok';
                        } else if (timeSinceUpdate < 5000) {
                            statusText = 'Connesso (ritardo)';
                            statusClass = 'status-warning';
                        } else {
                            statusText = 'Timeout Comunicazione';
                            statusClass = 'status-error';
                        }
                    }
                    
                    document.getElementById('connectionStatus').innerHTML = 
                        '<span class="status ' + statusClass + '">' + statusText + '</span>';
                    
                    document.getElementById('lastUpdate').textContent = new Date(data.lastUpdate).toLocaleTimeString();
                })
                .catch(error => console.error('Error:', error));
        }
        
        // Gestione form WiFi
        document.getElementById('wifiForm').addEventListener('submit', function(e) {
            e.preventDefault();
            const formData = new FormData(e.target);
            fetch('/wifi', {
                method: 'POST',
                body: formData
            })
            .then(response => response.text())
            .then(data => {
                alert('Configurazione WiFi salvata. Il dispositivo si riavvierà.');
                setTimeout(() => { location.reload(); }, 3000);
            });
        });
        
        // Gestione form Modbus
        document.getElementById('modbusForm').addEventListener('submit', function(e) {
            e.preventDefault();
            const formData = new FormData(e.target);
            fetch('/modbus', {
                method: 'POST',
                body: formData
            })
            .then(response => response.text())
            .then(data => {
                alert('Configurazione Modbus salvata. Il dispositivo si riavvierà.');
                setTimeout(() => { location.reload(); }, 3000);
            });
        });
        
        // Carica dati iniziali
        updateData();
    </script>
</body>
</html>
)rawliteral";

// Funzione per gestire richieste Modbus
uint8_t readHoldingRegisters(uint8_t fc, uint16_t address, uint16_t length) {
    // Aggiorna i registri con i dati correnti del motore
    updateModbusRegisters();
    
    // Controlla limiti
    if (address + length > MODBUS_REGISTERS_COUNT) {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }
    
    // Restituisci i dati richiesti
    for (uint16_t i = 0; i < length; i++) {
        modbus.writeRegisterToBuffer(i, modbusRegisters[address + i]);
    }
    
    return STATUS_OK;
}

// Aggiorna registri Modbus con dati motore
void updateModbusRegisters() {
    // RPM motore (32-bit)
    modbusRegisters[MB_REG_ENGINE_RPM] = (engineData.rpm >> 16) & 0xFFFF;
    modbusRegisters[MB_REG_ENGINE_RPM + 1] = engineData.rpm & 0xFFFF;
    
    // Temperature e pressioni (16-bit)
    modbusRegisters[MB_REG_ENGINE_TEMP] = engineData.engineTemp;
    modbusRegisters[MB_REG_OIL_PRESSURE] = engineData.oilPressure;
    
    // Consumo carburante (32-bit)
    modbusRegisters[MB_REG_FUEL_RATE] = (engineData.fuelRate >> 16) & 0xFFFF;
    modbusRegisters[MB_REG_FUEL_RATE + 1] = engineData.fuelRate & 0xFFFF;
    
    // Ore motore (32-bit)
    modbusRegisters[MB_REG_ENGINE_HOURS] = (engineData.engineHours >> 16) & 0xFFFF;
    modbusRegisters[MB_REG_ENGINE_HOURS + 1] = engineData.engineHours & 0xFFFF;
    
    // Altri parametri
    modbusRegisters[MB_REG_COOLANT_TEMP] = engineData.coolantTemp;
    modbusRegisters[MB_REG_INTAKE_TEMP] = engineData.intakeTemp;
    modbusRegisters[MB_REG_EXHAUST_TEMP] = engineData.exhaustTemp;
    modbusRegisters[MB_REG_ENGINE_LOAD] = engineData.engineLoad;
    modbusRegisters[MB_REG_THROTTLE_POS] = engineData.throttlePos;
    
    // Coppia motore (32-bit)
    modbusRegisters[MB_REG_ENGINE_TORQUE] = (engineData.engineTorque >> 16) & 0xFFFF;
    modbusRegisters[MB_REG_ENGINE_TORQUE + 1] = engineData.engineTorque & 0xFFFF;
    
    // Tensione e flag
    modbusRegisters[MB_REG_BATTERY_VOLTAGE] = engineData.batteryVoltage;
    modbusRegisters[MB_REG_STATUS_FLAGS] = engineData.statusFlags;
    modbusRegisters[MB_REG_ERROR_FLAGS] = engineData.errorFlags;
    modbusRegisters[MB_REG_DTC_COUNT] = engineData.dtcCount;
    
    // Timestamp ultimo aggiornamento (32-bit)
    modbusRegisters[MB_REG_LAST_UPDATE] = (engineData.lastUpdate >> 16) & 0xFFFF;
    modbusRegisters[MB_REG_LAST_UPDATE + 1] = engineData.lastUpdate & 0xFFFF;
}

// Inizializza CAN bus per J1939
void CAN_J1939_Init() {
    // Configura per J1939 (250 kbps)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    
    // Filtra solo PGN di interesse
    twai_filter_config_t f_config = {
        .acceptance_code = 0x00000000,
        .acceptance_mask = 0x00000000,  // Accetta tutti per ora
        .single_filter = true
    };
    
    // Installa driver TWAI
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("CAN driver installed");
    } else {
        Serial.println("Failed to install CAN driver");
        return;
    }
    
    // Avvia driver
    if (twai_start() == ESP_OK) {
        Serial.println("CAN driver started");
    } else {
        Serial.println("Failed to start CAN driver");
    }
    
    // Configura alert
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_BUS_ERROR | 
                                TWAI_ALERT_ERR_PASS | TWAI_ALERT_TX_FAILED;
    twai_reconfigure_alerts(alerts_to_enable, NULL);
}

// Estrai PGN dal CAN ID (formato J1939)
uint32_t getPGN(uint32_t canId) {
    // J1939 usa extended ID (29-bit)
    // PGN è nei bit 8-25 del CAN ID
    uint32_t pf = (canId >> 16) & 0xFF;  // PDU Format
    uint32_t ps = (canId >> 8) & 0xFF;   // PDU Specific
    
    if (pf < 240) {
        // PDU1 format - PS è destination address
        return (pf << 8);
    } else {
        // PDU2 format - PS fa parte del PGN
        return (pf << 8) | ps;
    }
}

// Processa messaggio J1939
void processJ1939Message(twai_message_t &message) {
    if (!message.extd) return;  // J1939 usa solo extended frame
    
    uint32_t pgn = getPGN(message.identifier);
    uint8_t sa = message.identifier & 0xFF;  // Source Address
    
    switch (pgn) {
        case PGN_ENGINE_SPEED:
            // Byte 3-4: Engine Speed (0.125 rpm/bit)
            if (message.data_length_code >= 4) {
                engineData.rpm = ((message.data[3] << 8) | message.data[2]) * 0.125;
                engineData.lastUpdate = millis();
            }
            break;
            
        case PGN_ENGINE_TEMP:
            // Byte 0: Engine Coolant Temperature (1°C/bit, -40°C offset)
            if (message.data_length_code >= 1) {
                int16_t temp = message.data[0] - 40;
                engineData.coolantTemp = (temp * 10);  // Memorizza in °C * 10
                engineData.lastUpdate = millis();
            }
            break;
            
        case PGN_ENGINE_FLUID_LEVEL:
            // Byte 3: Engine Oil Pressure (4 kPa/bit)
            if (message.data_length_code >= 4) {
                engineData.oilPressure = message.data[3] * 4;
                engineData.lastUpdate = millis();
            }
            break;
            
        case PGN_ENGINE_HOURS:
            // Byte 0-3: Engine Total Hours of Operation (0.05 hr/bit)
            if (message.data_length_code >= 4) {
                uint32_t hours = (message.data[3] << 24) | (message.data[2] << 16) | 
                                (message.data[1] << 8) | message.data[0];
                engineData.engineHours = hours * 0.05;
                engineData.lastUpdate = millis();
            }
            break;
            
        case PGN_FUEL_ECONOMY:
            // Byte 0-1: Fuel Rate (0.05 L/h per bit)
            if (message.data_length_code >= 2) {
                uint16_t rate = (message.data[1] << 8) | message.data[0];
                engineData.fuelRate = rate * 5;  // Memorizza in L/h * 100
                engineData.lastUpdate = millis();
            }
            break;
            
        case PGN_VEHICLE_ELECTRICAL:
            // Byte 4-5: Battery Potential (0.05 V/bit)
            if (message.data_length_code >= 6) {
                uint16_t voltage = (message.data[5] << 8) | message.data[4];
                engineData.batteryVoltage = voltage * 0.5;  // Memorizza in V * 10
                engineData.lastUpdate = millis();
            }
            break;
            
        case PGN_ELECTRONIC_ENGINE_1:
            // Byte 2: Engine Percent Load At Current Speed (1%/bit)
            // Byte 1: Driver's Demand Engine - Percent Torque (1%/bit, -125 offset)
            if (message.data_length_code >= 3) {
                engineData.engineLoad = message.data[2];
                int16_t torque = message.data[1] - 125;
                engineData.throttlePos = max(0, torque);  // Usa come indicazione acceleratore
                engineData.lastUpdate = millis();
            }
            break;
            
        case PGN_DIAGNOSTIC_MESSAGE_1:
            // Conta DTC attivi
            if (message.data_length_code >= 2) {
                // Byte 0-1: Lamp status e flash codes
                engineData.errorFlags = (message.data[0] << 8) | message.data[1];
                // I DTC seguono dal byte 2 in poi (ogni DTC è 4 byte)
                engineData.dtcCount = (message.data_length_code - 2) / 4;
                engineData.lastUpdate = millis();
            }
            break;
    }
}

// Task per gestione CAN
void CAN_Task() {
    // Controlla alert
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, 0);
    
    // Se ci sono dati disponibili
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
        twai_message_t message;
        while (twai_receive(&message, 0) == ESP_OK) {
            processJ1939Message(message);
        }
    }
    
    // Gestione errori bus
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
        twai_status_info_t status;
        twai_get_status_info(&status);
        Serial.printf("CAN Bus Error! Error count: %d\n", status.bus_error_count);
    }
}

// Setup server web
void setupWebServer() {
    // Pagina principale
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });
    
    // API per dati in tempo reale
    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
        DynamicJsonDocument doc(1024);
        doc["rpm"] = engineData.rpm;
        doc["engineTemp"] = engineData.engineTemp;
        doc["oilPressure"] = engineData.oilPressure;
        doc["fuelRate"] = engineData.fuelRate;
        doc["engineHours"] = engineData.engineHours;
        doc["coolantTemp"] = engineData.coolantTemp;
        doc["intakeTemp"] = engineData.intakeTemp;
        doc["exhaustTemp"] = engineData.exhaustTemp;
        doc["engineLoad"] = engineData.engineLoad;
        doc["throttlePos"] = engineData.throttlePos;
        doc["engineTorque"] = engineData.engineTorque;
        doc["batteryVoltage"] = engineData.batteryVoltage;
        doc["statusFlags"] = engineData.statusFlags;
        doc["errorFlags"] = engineData.errorFlags;
        doc["dtcCount"] = engineData.dtcCount;
        doc["lastUpdate"] = engineData.lastUpdate;
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
    
    // Configurazione WiFi
    server.on("/wifi", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("ssid", true)) {
            ssid = request->getParam("ssid", true)->value();
            password = request->hasParam("password", true) ? 
                      request->getParam("password", true)->value() : "";
            
            preferences.putString("ssid", ssid);
            preferences.putString("password", password);
            
            request->send(200, "text/plain", "OK");
            delay(1000);
            ESP.restart();
        } else {
            request->send(400, "text/plain", "Missing parameters");
        }
    });
    
    // Configurazione Modbus
    server.on("/modbus", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("slaveId", true) && request->hasParam("baudrate", true)) {
            int slaveId = request->getParam("slaveId", true)->value().toInt();
            int baudrate = request->getParam("baudrate", true)->value().toInt();
            
            preferences.putInt("slaveId", slaveId);
            preferences.putInt("baudrate", baudrate);
            
            request->send(200, "text/plain", "OK");
            delay(1000);
            ESP.restart();
        } else {
            request->send(400, "text/plain", "Missing parameters");
        }
    });
    
    server.begin();
}

// Setup WiFi
void setupWiFi() {
    // Leggi configurazione salvata
    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");
    
    if (ssid.length() > 0) {
        // Prova a connettersi
        Serial.printf("Connecting to WiFi: %s\n", ssid.c_str());
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), password.c_str());
        
        // Timeout 20 secondi
        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
            delay(500);
            Serial.print(".");
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            wifiConfigured = true;
            Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
            return;
        }
    }
    
    // Se non configurato o fallito, avvia AP
    Serial.println("\nStarting Access Point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    apMode = true;
    
    // Avvia DNS server per captive portal
    dnsServer.start(53, "*", WiFi.softAPIP());
    
    Serial.printf("AP Started. Connect to %s\n", AP_SSID);
    Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());
}

void setup() {
    Serial.begin(115200);
    Serial.println("Gateway Modbus-CAN J1939 with WiFi Starting...");
    
    // Inizializza preferences
    preferences.begin("gateway", false);
    
    // Abilita alimentazione periferiche
    pinMode(ME2107_EN, OUTPUT);
    digitalWrite(ME2107_EN, HIGH);
    
    // Configura RS485
    pinMode(RS485_EN, OUTPUT);
    digitalWrite(RS485_EN, HIGH);  // Abilita RS485
    pinMode(RS485_CALLBACK, OUTPUT);
    digitalWrite(RS485_CALLBACK, HIGH);  // Disabilita callback
    
    // Configura velocità CAN (J1939 usa tipicamente high-speed)
    pinMode(CAN_SPEED_MODE, OUTPUT);
    digitalWrite(CAN_SPEED_MODE, LOW);  // High speed mode
    
    // Inizializza struttura dati motore
    memset(&engineData, 0, sizeof(engineData));
    
    // Inizializza CAN per J1939
    CAN_J1939_Init();
    
    // Leggi configurazione Modbus
    int savedSlaveId = preferences.getInt("slaveId", MODBUS_SLAVE_ID);
    int savedBaudrate = preferences.getInt("baudrate", MODBUS_BAUDRATE);
    
    // Inizializza Modbus RTU slave su Serial1 (RS485)
    Serial1.begin(savedBaudrate, MODBUS_SERIAL_MODE, RS485_RX, RS485_TX);
    modbus.begin(&Serial1);
    modbus.slave(savedSlaveId);
    
    // Registra callback per lettura holding registers
    modbus.cbVector[CB_READ_HOLDING_REGISTERS] = readHoldingRegisters;
    
    // Setup WiFi e Web Server
    setupWiFi();
    setupWebServer();
    
    Serial.println("Gateway ready!");
    Serial.printf("Modbus Slave ID: %d\n", savedSlaveId);
    Serial.printf("Modbus Baudrate: %d\n", savedBaudrate);
    Serial.println("CAN J1939: 250 kbps");
    
    if (wifiConfigured) {
        Serial.printf("Web interface: http://%s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.printf("Configure WiFi at: http://%s\n", WiFi.softAPIP().toString().c_str());
    }
}

void loop() {
    // Gestisci comunicazione CAN
    CAN_Task();
    
    // Gestisci richieste Modbus
    modbus.poll();
    
    // Gestisci DNS server se in AP mode
    if (apMode) {
        dnsServer.processNextRequest();
    }
    
    // Timeout dati - marca come non validi se troppo vecchi
    if (millis() - engineData.lastUpdate > 5000) {
        engineData.statusFlags |= 0x8000;  // Set bit errore comunicazione
    } else {
        engineData.statusFlags &= ~0x8000;  // Clear bit errore
    }
    
    // Debug periodico (opzionale)
    static uint32_t lastDebug = 0;
    if (millis() - lastDebug > 5000) {
        lastDebug = millis();
        Serial.printf("RPM: %d, Temp: %.1f°C, Oil: %d kPa, Load: %d%%\n", 
            engineData.rpm, 
            engineData.coolantTemp / 10.0, 
            engineData.oilPressure,
            engineData.engineLoad);
    }
}