
#include <WiFi.h>
#include <WebServer.h>

// STM32 USART2 <-> ESP32-C3 UART
#define STM32_RX 4
#define STM32_TX 5

// Enter your mobile hotspot credentials
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

WebServer server(80);

String latestTelemetry = "Waiting for STM32 data...";
String incomingTelemetry = "";

const size_t MAX_LINE_LENGTH = 250;

// Dashboard webpage
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SENTRY Robot Telemetry</title>

  <style>
    body {
      background: #101820;
      color: white;
      font-family: Arial, sans-serif;
      text-align: center;
      padding: 25px;
    }

    .card {
      background: #1d2b38;
      padding: 25px;
      margin: 20px auto;
      border-radius: 12px;
      max-width: 650px;
    }

    h1 { color: #00d4ff; }

    #data {
      font-size: 18px;
      line-height: 2;
      overflow-wrap: anywhere;
    }

    .status { color: #00ff99; }
  </style>
</head>

<body>
  <h1>SENTRY Autonomous Robot</h1>
  <p class="status">Wireless Telemetry Dashboard</p>

  <div class="card">
    <h2>Live Sensor Data</h2>
    <div id="data">Waiting for STM32 data...</div>
  </div>

  <p>ESP32-C3 Telemetry Gateway</p>

  <script>
    async function updateData() {
      try {
        const response = await fetch('/data');
        const data = await response.text();
        document.getElementById('data').innerText = data;
      }
      catch (error) {
        document.getElementById('data').innerText =
          'Unable to receive telemetry';
      }
    }

    setInterval(updateData, 500);
    updateData();
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);

  // UART connection to STM32
  Serial1.begin(115200, SERIAL_8N1, STM32_RX, STM32_TX);

  Serial.println("SENTRY ESP32-C3 Starting...");

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Wi-Fi connected!");

  Serial.print("Dashboard IP: ");
  Serial.println(WiFi.localIP());

  // Main dashboard
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", webpage);
  });

  // Latest telemetry endpoint
  server.on("/data", HTTP_GET, []() {
    server.send(200, "text/plain", latestTelemetry);
  });

  server.begin();

  Serial.println("SENTRY dashboard server started.");
}

void loop() {
  // Read incoming STM32 UART data
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      incomingTelemetry.trim();

      if (incomingTelemetry.length() > 0) {
        latestTelemetry = incomingTelemetry;
        Serial.println(latestTelemetry);
      }

      incomingTelemetry = "";
    }
    else if (c != '\r') {
      if (incomingTelemetry.length() < MAX_LINE_LENGTH) {
        incomingTelemetry += c;
      }
      else {
        // Discard an oversized/incomplete line
        incomingTelemetry = "";
      }
    }
  }

  // Handle browser requests
  server.handleClient();
}
