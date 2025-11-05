#include "WiFiCommunication.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include "SharedData.h"
#include "CANCommunication.h"
#include "I2CCommunication.h"
#include <ESPmDNS.h>
#include <WebSocketsServer.h>

// WiFi credentials - replace with your network details
const char *ssid = "labilley_base";
const char *password = "Manit0ba";

// Server on port 80
WiFiServer server(80);

// WebSocket server on port 81 for low-latency commands
WebSocketsServer webSocket(81);

// Throttle metrics broadcast over WebSocket
static unsigned long lastWsMetricsMs = 0;

// When true, dedicated FreeRTOS tasks service WS/HTTP and the legacy
// handleWiFiClients() should become a no-op to avoid double-processing.
static volatile bool g_wifiTasksRunning = false;

// Forward declarations for FreeRTOS task functions
static void WebSocketTask(void *parameter);
static void HTTPTask(void *parameter);

static void writeCorsHeaders(WiFiClient &client)
{
    client.println("Access-Control-Allow-Origin: *");
    client.println("Access-Control-Allow-Methods: GET, POST, OPTIONS");
    client.println("Access-Control-Allow-Headers: Content-Type, Authorization, X-Requested-With");
}

static bool readHttpRequest(WiFiClient &client, String &header, String &body, int &contentLength)
{
    const unsigned long timeoutMs = 3000;
    unsigned long start = millis();
    bool headerComplete = false;
    contentLength = 0;

    while (client.connected() && (millis() - start) < timeoutMs)
    {
        while (client.available())
        {
            char c = client.read();
            if (!headerComplete)
            {
                header += c;
                if (header.endsWith("\r\n\r\n"))
                {
                    headerComplete = true;
                    int lenIdx = header.indexOf("Content-Length:");
                    if (lenIdx >= 0)
                    {
                        int endIdx = header.indexOf("\r\n", lenIdx);
                        String value = header.substring(lenIdx + 15, endIdx);
                        value.trim();
                        contentLength = value.toInt();
                    }
                    if (contentLength == 0)
                    {
                        return true;
                    }
                }
            }
            else
            {
                body += c;
                if ((int)body.length() >= contentLength)
                {
                    return true;
                }
            }
            // Yield to other tasks
            delay(0);
        }
        // Yield between availability checks
        delay(0);
    }

    return headerComplete && ((int)body.length() >= contentLength);
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
            break;
        }
        case WStype_TEXT: {
            Serial.printf("[%u] Received: %s\n", num, payload);
            // Record when the WS command arrived; reuse HTTP timing fields for unified metrics
            sharedData.t_http_received_ms = millis();
            sharedData.t_cmd_set_ms = 0;
            sharedData.t_ctrl_apply_ms = 0;
            StaticJsonDocument<512> doc;
            DeserializationError error = deserializeJson(doc, payload, length);
            if (error) {
                webSocket.sendTXT(num, "{\"error\":\"Invalid JSON\"}");
                return;
            }
            String cmd = doc["command"];
            if (cmd == "pulse") {
                String type = doc["type"];
                if (type == "buzz") type = "on";
                int duration = doc["duration"];
                int strength = doc["strength"];
                int frequency = doc["frequency"];
                updatePulseData(type, duration, strength, frequency);
                StaticJsonDocument<128> resp;
                resp["result"] = "pulse updated";
                if (doc.containsKey("client_ts")) resp["client_ts"] = doc["client_ts"];
                String out; serializeJson(resp, out);
                webSocket.sendTXT(num, out);
            } else if (cmd == "idle") {
                if (xSemaphoreTake(mutex, 0) == pdTRUE) {
                    controlState.mode = 0;
                    pendingApplyIdle = true;
                    xSemaphoreGive(mutex);
                }
                sharedData.t_cmd_set_ms = millis();
                updateModeData("off");
                updatePulseData("off", 0, 0, 0);
                updateRowData("off", 0, 0, 0);
                StaticJsonDocument<128> resp;
                resp["result"] = "idle";
                if (doc.containsKey("client_ts")) resp["client_ts"] = doc["client_ts"];
                String out; serializeJson(resp, out);
                webSocket.sendTXT(num, out);
            } else if (cmd == "strength") {
                float weight_lbs = doc.containsKey("weight_lbs") ? (float)doc["weight_lbs"] : 0.0f;
                float weight_kg = doc.containsKey("weight_kg") ? (float)doc["weight_kg"] : (weight_lbs * 0.45359237f);
                float target_force_n = weight_kg * 9.81f;
                if (xSemaphoreTake(mutex, 0) == pdTRUE) {
                    sharedCfgData.target_force = target_force_n;
                    controlState.mode = 1;
                    pendingApplyStrength = true;
                    xSemaphoreGive(mutex);
                }
                sharedData.t_cmd_set_ms = millis();
                updateModeData("strength");
                StaticJsonDocument<128> resp;
                resp["result"] = "strength";
                resp["weight_kg"] = weight_kg;
                resp["target_force_n"] = target_force_n;
                if (doc.containsKey("client_ts")) resp["client_ts"] = doc["client_ts"];
                String response;
                serializeJson(resp, response);
                webSocket.sendTXT(num, response);
            } else if (cmd == "detent") {
                String type = doc["type"];
                int strength = doc["strength"];
                float start_position = doc["start_position"];
                float step_position = doc["step_position"];
                int total_steps = doc["total_steps"];
                updateDetentData(type, strength, start_position, step_position, total_steps);
                StaticJsonDocument<128> resp;
                resp["result"] = "detent updated";
                if (doc.containsKey("client_ts")) resp["client_ts"] = doc["client_ts"];
                String out; serializeJson(resp, out);
                webSocket.sendTXT(num, out);
            } else if (cmd == "force") {
                String type = doc["type"];
                int strength = doc["strength"];
                int start_strength = doc["start_strength"];
                float start_position = doc["start_position"];
                float saturation_position = doc["saturation_position"];
                updateForceData(type, strength, start_strength, start_position, saturation_position);
                StaticJsonDocument<128> resp;
                resp["result"] = "force updated";
                if (doc.containsKey("client_ts")) resp["client_ts"] = doc["client_ts"];
                String out; serializeJson(resp, out);
                webSocket.sendTXT(num, out);
            } else if (cmd == "mode") {
                String type = doc["type"];
                updateModeData(type);
                StaticJsonDocument<128> resp;
                resp["result"] = "mode updated";
                if (doc.containsKey("client_ts")) resp["client_ts"] = doc["client_ts"];
                String out; serializeJson(resp, out);
                webSocket.sendTXT(num, out);
            } else if (cmd == "row") {
                String type = doc["type"];
                int damping = doc["damping"];
                int gear_ratio = doc["gear_ratio"];
                int inertia = doc["inertia"];
                updateRowData(type, damping, gear_ratio, inertia);
                StaticJsonDocument<128> resp;
                resp["result"] = "row updated";
                if (doc.containsKey("client_ts")) resp["client_ts"] = doc["client_ts"];
                String out; serializeJson(resp, out);
                webSocket.sendTXT(num, out);
            } else {
                webSocket.sendTXT(num, "{\"error\":\"Unknown command\"}");
            }
            break;
        }
        default:
            break;
    }
}

void initWiFi()
{
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    // Disable WiFi modem sleep to reduce latency spikes
    WiFi.setSleep(false);
    // Boost TX power for better link margin
    #ifdef WIFI_POWER_19_5dBm
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    #endif
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    server.begin();
    Serial.println("Server started.");
    // Disable Nagle to avoid 200ms delayed ACK behavior on small packets
    server.setNoDelay(true);

    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);
    Serial.println("WebSocket server started on port 81.");

    WiFi.setHostname("digital-weight");
    if (MDNS.begin("digital-weight"))
    {
        MDNS.addService("http", "tcp", 80);
        Serial.println("mDNS responder started: http://digital-weight.local/");
    }
    else
    {
        Serial.println("mDNS start failed");
    }

    // Start dedicated tasks for WS and HTTP to avoid mutual starvation.
    // WS on core 0 at higher priority along with control task; HTTP on core 1.
    BaseType_t wsOk = xTaskCreatePinnedToCore(
        WebSocketTask,
        "WebSocketTask",
        8192,
        nullptr,
        2,
        nullptr,
        0);
    BaseType_t httpOk = xTaskCreatePinnedToCore(
        HTTPTask,
        "HTTPTask",
        6144,
        nullptr,
        1,
        nullptr,
        1);
    if (wsOk == pdPASS && httpOk == pdPASS) {
        g_wifiTasksRunning = true;
        Serial.println("WiFi tasks started: WS(core0,p2), HTTP(core1,p1)");
    } else {
        Serial.println("ERROR: Failed to start WiFi tasks, falling back to loop()-based handling.");
    }
}

void handleWiFiClients()
{
    // If dedicated tasks are running, avoid double-processing here.
    if (g_wifiTasksRunning) {
        return;
    }

    webSocket.loop();

    // Broadcast metrics periodically over WS to avoid HTTP polling overhead
    unsigned long nowMs = millis();
    if (nowMs - lastWsMetricsMs >= 250) {
        lastWsMetricsMs = nowMs;
        StaticJsonDocument<512> doc;
        doc["type"] = "metrics";
        doc["force"] = sharedData.force;
        doc["position"] = sharedData.position;
        doc["velocity"] = sharedData.velocity;
        doc["accelerometer_x"] = sharedData.accelerometer_x;
        doc["accelerometer_y"] = sharedData.accelerometer_y;
        doc["accelerometer_z"] = sharedData.accelerometer_z;
        doc["status"] = sharedData.status;
        doc["virtual_velocity"] = sharedData.virtual_velocity;
        doc["t_http_received_ms"] = sharedData.t_http_received_ms;
        doc["t_cmd_set_ms"] = sharedData.t_cmd_set_ms;
        doc["t_ctrl_apply_ms"] = sharedData.t_ctrl_apply_ms;
        String out; serializeJson(doc, out);
        webSocket.broadcastTXT(out);
    }

    WiFiClient client = server.available();
    if (!client)
    {
        return;
    }

    Serial.println("New client connected.");

    String header;
    String body;
    int contentLength = 0;
    if (!readHttpRequest(client, header, body, contentLength))
    {
        client.stop();
        Serial.println("Client disconnected (timeout/incomplete request).");
        return;
    }

    // Do not record HTTP timing here; we only record for POST /command to avoid metrics GET overwriting it.

    String requestLine;
    int lineEnd = header.indexOf("\r\n");
    if (lineEnd > 0)
    {
        requestLine = header.substring(0, lineEnd);
    }

    if (requestLine.startsWith("OPTIONS "))
    {
        client.println("HTTP/1.1 204 No Content");
        writeCorsHeaders(client);
        client.println("Connection: close");
        client.println();
        client.stop();
        Serial.println("Client disconnected.");
        return;
    }

    if (requestLine.indexOf("GET /health") == 0)
    {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        writeCorsHeaders(client);
        client.println("Connection: close");
        client.println();
        StaticJsonDocument<200> doc;
        doc["status"] = "OK";
        doc["uptime"] = millis();
        serializeJson(doc, client);
    }
    else if (requestLine.indexOf("GET /metrics") == 0)
    {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        writeCorsHeaders(client);
        client.println("Connection: close");
        client.println();
        StaticJsonDocument<1024> doc;
        doc["accelerometer_x"] = sharedData.accelerometer_x;
        doc["accelerometer_y"] = sharedData.accelerometer_y;
        doc["accelerometer_z"] = sharedData.accelerometer_z;
        doc["gyro_x"] = sharedData.gyro_x;
        doc["gyro_y"] = sharedData.gyro_y;
        doc["gyro_z"] = sharedData.gyro_z;
        doc["force"] = sharedData.force;
        doc["position"] = sharedData.position;
        doc["velocity"] = sharedData.velocity;
        doc["virtual_velocity"] = sharedData.virtual_velocity;
        doc["status"] = sharedData.status;
        // Debug timestamps for latency tracing
        doc["t_http_received_ms"] = sharedData.t_http_received_ms;
        doc["t_cmd_set_ms"] = sharedData.t_cmd_set_ms;
        doc["t_ctrl_apply_ms"] = sharedData.t_ctrl_apply_ms;
        serializeJson(doc, client);
    }
    else if (requestLine.indexOf("POST /command") == 0)
    {
        // Record when the command HTTP request arrived; reset subsequent timestamps
        sharedData.t_http_received_ms = millis();
        sharedData.t_cmd_set_ms = 0;
        sharedData.t_ctrl_apply_ms = 0;
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, body);
        if (error)
        {
            client.println("HTTP/1.1 400 Bad Request");
            client.println("Content-Type: application/json");
            writeCorsHeaders(client);
            client.println("Connection: close");
            client.println();
            StaticJsonDocument<100> errDoc;
            errDoc["error"] = "Invalid JSON";
            serializeJson(errDoc, client);
        }
        else
        {
            String cmd = doc["command"];
            if (cmd == "pulse")
            {
                String type = doc["type"];
                if (type == "buzz")
                {
                    type = "on";
                }
                int duration = doc["duration"];
                int strength = doc["strength"];
                int frequency = doc["frequency"];
                updatePulseData(type, duration, strength, frequency);
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: application/json");
                writeCorsHeaders(client);
                client.println("Connection: close");
                client.println();
                StaticJsonDocument<50> resp;
                resp["result"] = "pulse updated";
                serializeJson(resp, client);
            }
            else if (cmd == "idle")
            {
                // Request idle mode to be applied by the control task (centralized CAN TX)
                if (xSemaphoreTake(mutex, 0) == pdTRUE)
                {
                    controlState.mode = 0;
                    pendingApplyIdle = true;
                    xSemaphoreGive(mutex);
                }
                sharedData.t_cmd_set_ms = millis();
                updateModeData("off");
                updatePulseData("off", 0, 0, 0);
                updateRowData("off", 0, 0, 0);
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: application/json");
                writeCorsHeaders(client);
                client.println("Connection: close");
                client.println();
                StaticJsonDocument<50> resp;
                resp["result"] = "idle";
                serializeJson(resp, client);
            }
            else if (cmd == "strength")
            {
                float weight_lbs = doc.containsKey("weight_lbs") ? (float)doc["weight_lbs"] : 0.0f;
                float weight_kg = doc.containsKey("weight_kg") ? (float)doc["weight_kg"] : (weight_lbs * 0.45359237f);
                float target_force_n = weight_kg * 9.81f;
                if (xSemaphoreTake(mutex, 0) == pdTRUE)
                {
                    sharedCfgData.target_force = target_force_n;
                    controlState.mode = 1;
                    pendingApplyStrength = true;
                    xSemaphoreGive(mutex);
                }
                sharedData.t_cmd_set_ms = millis();
                updateModeData("strength");
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: application/json");
                writeCorsHeaders(client);
                client.println("Connection: close");
                client.println();
                StaticJsonDocument<80> resp;
                resp["result"] = "strength";
                resp["weight_kg"] = weight_kg;
                resp["target_force_n"] = target_force_n;
                serializeJson(resp, client);
            }
            else if (cmd == "detent")
            {
                String type = doc["type"];
                int strength = doc["strength"];
                float start_position = doc["start_position"];
                float step_position = doc["step_position"];
                int total_steps = doc["total_steps"];
                updateDetentData(type, strength, start_position, step_position, total_steps);
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: application/json");
                writeCorsHeaders(client);
                client.println("Connection: close");
                client.println();
                StaticJsonDocument<50> resp;
                resp["result"] = "detent updated";
                serializeJson(resp, client);
            }
            else if (cmd == "force")
            {
                String type = doc["type"];
                int strength = doc["strength"];
                int start_strength = doc["start_strength"];
                float start_position = doc["start_position"];
                float saturation_position = doc["saturation_position"];
                updateForceData(type, strength, start_strength, start_position, saturation_position);
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: application/json");
                writeCorsHeaders(client);
                client.println("Connection: close");
                client.println();
                StaticJsonDocument<50> resp;
                resp["result"] = "force updated";
                serializeJson(resp, client);
            }
            else if (cmd == "mode")
            {
                String type = doc["type"];
                updateModeData(type);
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: application/json");
                writeCorsHeaders(client);
                client.println("Connection: close");
                client.println();
                StaticJsonDocument<50> resp;
                resp["result"] = "mode updated";
                serializeJson(resp, client);
            }
            else if (cmd == "row")
            {
                String type = doc["type"];
                int damping = doc["damping"];
                int gear_ratio = doc["gear_ratio"];
                int inertia = doc["inertia"];
                updateRowData(type, damping, gear_ratio, inertia);
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: application/json");
                writeCorsHeaders(client);
                client.println("Connection: close");
                client.println();
                StaticJsonDocument<50> resp;
                resp["result"] = "row updated";
                serializeJson(resp, client);
            }
            else
            {
                client.println("HTTP/1.1 400 Bad Request");
                client.println("Content-Type: application/json");
                writeCorsHeaders(client);
                client.println("Connection: close");
                client.println();
                StaticJsonDocument<100> errDoc;
                errDoc["error"] = "Unknown command";
                serializeJson(errDoc, client);
            }
        }
    }
    else
    {
        client.println("HTTP/1.1 404 Not Found");
        client.println("Content-Type: application/json");
        writeCorsHeaders(client);
        client.println("Connection: close");
        client.println();
        StaticJsonDocument<50> doc;
        doc["error"] = "Endpoint not found";
        serializeJson(doc, client);
    }

    client.stop();
    Serial.println("Client disconnected.");
}

// ===== Task implementations =====

static void WebSocketTask(void *parameter)
{
    (void)parameter;
    for(;;) {
        // Service WebSocket events
        webSocket.loop();

        // Periodic metrics broadcast (same cadence as legacy path)
        unsigned long nowMs = millis();
        if (nowMs - lastWsMetricsMs >= 250) {
            lastWsMetricsMs = nowMs;
            StaticJsonDocument<512> doc;
            doc["type"] = "metrics";
            doc["force"] = sharedData.force;
            doc["position"] = sharedData.position;
            doc["velocity"] = sharedData.velocity;
            doc["accelerometer_x"] = sharedData.accelerometer_x;
            doc["accelerometer_y"] = sharedData.accelerometer_y;
            doc["accelerometer_z"] = sharedData.accelerometer_z;
            doc["status"] = sharedData.status;
            doc["virtual_velocity"] = sharedData.virtual_velocity;
            doc["t_http_received_ms"] = sharedData.t_http_received_ms;
            doc["t_cmd_set_ms"] = sharedData.t_cmd_set_ms;
            doc["t_ctrl_apply_ms"] = sharedData.t_ctrl_apply_ms;
            String out; serializeJson(doc, out);
            webSocket.broadcastTXT(out);
        }

        // Yield to other tasks
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

static void HTTPTask(void *parameter)
{
    (void)parameter;
    for(;;) {
        WiFiClient client = server.available();
        if (!client) {
            // Nothing to do, yield
            vTaskDelay(1 / portTICK_PERIOD_MS);
            continue;
        }

        Serial.println("New client connected.");

        String header;
        String body;
        int contentLength = 0;
        if (!readHttpRequest(client, header, body, contentLength))
        {
            client.stop();
            Serial.println("Client disconnected (timeout/incomplete request).");
            continue;
        }

        String requestLine;
        int lineEnd = header.indexOf("\r\n");
        if (lineEnd > 0) {
            requestLine = header.substring(0, lineEnd);
        }

        if (requestLine.startsWith("OPTIONS "))
        {
            client.println("HTTP/1.1 204 No Content");
            writeCorsHeaders(client);
            client.println("Connection: close");
            client.println();
            client.stop();
            Serial.println("Client disconnected.");
            continue;
        }

        if (requestLine.indexOf("GET /health") == 0)
        {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: application/json");
            writeCorsHeaders(client);
            client.println("Connection: close");
            client.println();
            StaticJsonDocument<200> doc;
            doc["status"] = "OK";
            doc["uptime"] = millis();
            serializeJson(doc, client);
        }
        else if (requestLine.indexOf("GET /metrics") == 0)
        {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: application/json");
            writeCorsHeaders(client);
            client.println("Connection: close");
            client.println();
            StaticJsonDocument<1024> doc;
            doc["accelerometer_x"] = sharedData.accelerometer_x;
            doc["accelerometer_y"] = sharedData.accelerometer_y;
            doc["accelerometer_z"] = sharedData.accelerometer_z;
            doc["gyro_x"] = sharedData.gyro_x;
            doc["gyro_y"] = sharedData.gyro_y;
            doc["gyro_z"] = sharedData.gyro_z;
            doc["force"] = sharedData.force;
            doc["position"] = sharedData.position;
            doc["velocity"] = sharedData.velocity;
            doc["virtual_velocity"] = sharedData.virtual_velocity;
            doc["status"] = sharedData.status;
            doc["t_http_received_ms"] = sharedData.t_http_received_ms;
            doc["t_cmd_set_ms"] = sharedData.t_cmd_set_ms;
            doc["t_ctrl_apply_ms"] = sharedData.t_ctrl_apply_ms;
            serializeJson(doc, client);
        }
        else if (requestLine.indexOf("POST /command") == 0)
        {
            // Record when the command HTTP request arrived; reset subsequent timestamps
            sharedData.t_http_received_ms = millis();
            sharedData.t_cmd_set_ms = 0;
            sharedData.t_ctrl_apply_ms = 0;
            StaticJsonDocument<512> doc;
            DeserializationError error = deserializeJson(doc, body);
            if (error)
            {
                client.println("HTTP/1.1 400 Bad Request");
                client.println("Content-Type: application/json");
                writeCorsHeaders(client);
                client.println("Connection: close");
                client.println();
                StaticJsonDocument<100> errDoc;
                errDoc["error"] = "Invalid JSON";
                serializeJson(errDoc, client);
            }
            else
            {
                String cmd = doc["command"];
                if (cmd == "pulse")
                {
                    String type = doc["type"];
                    if (type == "buzz") { type = "on"; }
                    int duration = doc["duration"];
                    int strength = doc["strength"];
                    int frequency = doc["frequency"];
                    updatePulseData(type, duration, strength, frequency);
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: application/json");
                    writeCorsHeaders(client);
                    client.println("Connection: close");
                    client.println();
                    StaticJsonDocument<50> resp;
                    resp["result"] = "pulse updated";
                    serializeJson(resp, client);
                }
                else if (cmd == "idle")
                {
                    if (xSemaphoreTake(mutex, 0) == pdTRUE)
                    {
                        controlState.mode = 0;
                        pendingApplyIdle = true;
                        xSemaphoreGive(mutex);
                    }
                    sharedData.t_cmd_set_ms = millis();
                    updateModeData("off");
                    updatePulseData("off", 0, 0, 0);
                    updateRowData("off", 0, 0, 0);
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: application/json");
                    writeCorsHeaders(client);
                    client.println("Connection: close");
                    client.println();
                    StaticJsonDocument<50> resp;
                    resp["result"] = "idle";
                    serializeJson(resp, client);
                }
                else if (cmd == "strength")
                {
                    float weight_lbs = doc.containsKey("weight_lbs") ? (float)doc["weight_lbs"] : 0.0f;
                    float weight_kg = doc.containsKey("weight_kg") ? (float)doc["weight_kg"] : (weight_lbs * 0.45359237f);
                    float target_force_n = weight_kg * 9.81f;
                    if (xSemaphoreTake(mutex, 0) == pdTRUE)
                    {
                        sharedCfgData.target_force = target_force_n;
                        controlState.mode = 1;
                        pendingApplyStrength = true;
                        xSemaphoreGive(mutex);
                    }
                    sharedData.t_cmd_set_ms = millis();
                    updateModeData("strength");
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: application/json");
                    writeCorsHeaders(client);
                    client.println("Connection: close");
                    client.println();
                    StaticJsonDocument<80> resp;
                    resp["result"] = "strength";
                    resp["weight_kg"] = weight_kg;
                    resp["target_force_n"] = target_force_n;
                    serializeJson(resp, client);
                }
                else if (cmd == "detent")
                {
                    String type = doc["type"];
                    int strength = doc["strength"];
                    float start_position = doc["start_position"];
                    float step_position = doc["step_position"];
                    int total_steps = doc["total_steps"];
                    updateDetentData(type, strength, start_position, step_position, total_steps);
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: application/json");
                    writeCorsHeaders(client);
                    client.println("Connection: close");
                    client.println();
                    StaticJsonDocument<50> resp;
                    resp["result"] = "detent updated";
                    serializeJson(resp, client);
                }
                else if (cmd == "force")
                {
                    String type = doc["type"];
                    int strength = doc["strength"];
                    int start_strength = doc["start_strength"];
                    float start_position = doc["start_position"];
                    float saturation_position = doc["saturation_position"];
                    updateForceData(type, strength, start_strength, start_position, saturation_position);
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: application/json");
                    writeCorsHeaders(client);
                    client.println("Connection: close");
                    client.println();
                    StaticJsonDocument<50> resp;
                    resp["result"] = "force updated";
                    serializeJson(resp, client);
                }
                else if (cmd == "mode")
                {
                    String type = doc["type"];
                    updateModeData(type);
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: application/json");
                    writeCorsHeaders(client);
                    client.println("Connection: close");
                    client.println();
                    StaticJsonDocument<50> resp;
                    resp["result"] = "mode updated";
                    serializeJson(resp, client);
                }
                else if (cmd == "row")
                {
                    String type = doc["type"];
                    int damping = doc["damping"];
                    int gear_ratio = doc["gear_ratio"];
                    int inertia = doc["inertia"];
                    updateRowData(type, damping, gear_ratio, inertia);
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: application/json");
                    writeCorsHeaders(client);
                    client.println("Connection: close");
                    client.println();
                    StaticJsonDocument<50> resp;
                    resp["result"] = "row updated";
                    serializeJson(resp, client);
                }
                else
                {
                    client.println("HTTP/1.1 400 Bad Request");
                    client.println("Content-Type: application/json");
                    writeCorsHeaders(client);
                    client.println("Connection: close");
                    client.println();
                    StaticJsonDocument<100> errDoc;
                    errDoc["error"] = "Unknown command";
                    serializeJson(errDoc, client);
                }
            }
        }
        else
        {
            client.println("HTTP/1.1 404 Not Found");
            client.println("Content-Type: application/json");
            writeCorsHeaders(client);
            client.println("Connection: close");
            client.println();
            StaticJsonDocument<50> doc;
            doc["error"] = "Endpoint not found";
            serializeJson(doc, client);
        }

        client.stop();
        Serial.println("Client disconnected.");
    }
}