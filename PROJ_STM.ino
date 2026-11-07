
#include <WiFi.h>
#include <WebServer.h>

#define STM32_RX 4
#define STM32_TX 5

const char* ssid = "POCO M7 Plus 5G";
const char* password = "";

WebServer server(80);

String latestTelemetry = "Waiting for STM32 data...";

const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Smart Robot Dashboard</title>
  <style>
    body {
      font-family: Arial;
      background: #101827;
      color: white;
      text-align: center;
      padding: 25px;
    }
    .card {
      background: #1e293b;
      padding: 22px;
      border-radius: 12px;
      margin: 20px auto;
      max-width: 600px;
    }
    h1 { color: #38bdf8; }
    #data {
      font-size: 17px;
      line-height: 2;
      overflow-wrap: anywhere;
    }
  </style>
</head>
<body>
  <h1>SMART ROBOT DASHBOARD</h1>
  <div class="card">
    <h2>Live STM32 Telemetry</h2>
    <p id="status">Connecting...</p>
    <div id="data">Waiting for data...</div>
  </div>

  <script>
    async function updateData() {
      try {
        const response = await fetch('/data');
        const data = await response.text();
        document.getElementById('data').textContent = data;
        document.getElementById('status').textContent =
          'Live connection active';
      } catch (error) {
        document.getElementById('status').textContent =
          'Connection lost';
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
  Serial1.begin(115200, SERIAL_8N1, STM32_RX, STM32_TX);

  WiFi.begin(ssid, password);

  Serial.print("Connecting to hotspot");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("Dashboard IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", []() {
    server.send(200, "text/html", webpage);
  });

  server.on("/data", []() {
    server.send(200, "text/plain", latestTelemetry);
  });

  server.begin();
  Serial.println("Web dashboard server started");
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      latestTelemetry.trim();
    } else if (c != '\r') {
      latestTelemetry += c;

      if (latestTelemetry.length() > 250) {
        latestTelemetry = "";
      }
    }
  }

  server.handleClient();
}
