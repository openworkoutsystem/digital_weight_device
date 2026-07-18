#include "WiFiCommunication.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include "SharedData.h"
#include "CANCommunication.h"
#include "I2CCommunication.h"
#include "MotorControl.h"
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
static const unsigned long WS_METRICS_INTERVAL_MS = 400; // relaxed from 250ms to reduce burst contention
static unsigned long lastWsCmdMs = 0; // Track when commands arrive to avoid broadcast blocking

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
            // Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            // Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
            break;
        }
        case WStype_TEXT: {
            // Avoid synchronous serial printing in WS hot path to prevent latency spikes
            // Record when the WS command arrived; reuse HTTP timing fields for unified metrics
            lastWsCmdMs = millis();
            sharedData.t_http_received_ms = lastWsCmdMs;
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
                // Enqueue non-blocking idle command; control task will apply.
                CommandMsg msg; msg.type = CMD_IDLE; msg.weight_kg = 0;
                if (g_cmdQueue) { xQueueSend(g_cmdQueue, &msg, 0); }
                // Only update mode data locally if transitioning
                if (controlState.mode != 0) {
                    updateModeData("off");
                    updatePulseData("off", 0, 0, 0);
                    updateRowData("off", 0, 0, 0);
                }
                // Echo RTT
                if (doc.containsKey("client_ts")) {
                    String out = "{\"result\":\"idle\",\"client_ts\":";
                    out += doc["client_ts"].as<long long>();
                    out += "}";
                    webSocket.sendTXT(num, out);
                } else {
                    webSocket.sendTXT(num, "{\"result\":\"idle\"}");
                }
            } else if (cmd == "strength") {
                float weight_lbs = doc.containsKey("weight_lbs") ? (float)doc["weight_lbs"] : 0.0f;
                float weight_kg = doc.containsKey("weight_kg") ? (float)doc["weight_kg"] : (weight_lbs * 0.45359237f);
                // Enqueue non-blocking strength command
                CommandMsg msg; msg.type = CMD_STRENGTH; msg.weight_kg = weight_kg; if (g_cmdQueue) { xQueueSend(g_cmdQueue, &msg, 0); }
                updateModeData("strength");
                if (doc.containsKey("client_ts")) {
                    String out = "{\"result\":\"strength\",\"client_ts\":";
                    out += doc["client_ts"].as<long long>();
                    out += "}";
                    webSocket.sendTXT(num, out);
                } else {
                    webSocket.sendTXT(num, "{\"result\":\"strength\"}");
                }
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
            } else if (cmd == "cal") {
                // Live force calibration; omitted params stay unchanged.
                // RAM only — resets to compiled defaults on reboot.
                float force_scale = doc.containsKey("force_scale") ? (float)doc["force_scale"] : -1.0f;
                float zero_m1 = doc.containsKey("zero_m1") ? (float)doc["zero_m1"] : -1.0f;
                setForceCal(force_scale, zero_m1);
                if (doc.containsKey("debug")) setForceCalDebug((bool)doc["debug"]);
                if (doc.containsKey("row") && doc.containsKey("values")) {
                    JsonArray arr = doc["values"];
                    float vals[6]; int n = 0;
                    for (JsonVariant v : arr) { if (n < 6) vals[n++] = v.as<float>(); }
                    setForceCalRow((int)doc["row"], vals, n);
                }
                setWeightGateDistances(
                    doc.containsKey("delta_x") ? (float)doc["delta_x"] : -1.0f,
                    doc.containsKey("delta_y") ? (float)doc["delta_y"] : -1.0f,
                    doc.containsKey("deadband") ? (float)doc["deadband"] : -1.0f,
                    doc.containsKey("cancel") ? (float)doc["cancel"] : -1.0f);
                if (doc.containsKey("pull_sign")) setWeightGatePullSign((float)doc["pull_sign"]);
                if (doc.containsKey("test_torque")) setTestTorque((float)doc["test_torque"]);
                setDriveParams(
                    doc.containsKey("amps_per_nm") ? (float)doc["amps_per_nm"] : -1.0f,
                    doc.containsKey("vel_limit") ? (float)doc["vel_limit"] : -1.0f,
                    doc.containsKey("max_current") ? (float)doc["max_current"] : -1.0f);
                setSoftMinEndpoint(
                    doc.containsKey("ep_min") ? (int)doc["ep_min"] : -1,
                    doc.containsKey("use_ep") ? ((bool)doc["use_ep"] ? 1 : 0) : -1);
                setHomeBehavior(
                    doc.containsKey("zone") ? (float)doc["zone"] : -1.0f,
                    doc.containsKey("recoil_tq") ? (float)doc["recoil_tq"] : -1.0f,
                    doc.containsKey("recoil_vel") ? (float)doc["recoil_vel"] : -1.0f,
                    doc.containsKey("soft_max") ? (float)doc["soft_max"] : -1.0f,
                    doc.containsKey("sleep_s") ? (float)doc["sleep_s"] : -1.0f,
                    doc.containsKey("wake_d") ? (float)doc["wake_d"] : -1.0f);
                setDropCatch(
                    doc.containsKey("drop_vmin") ? (float)doc["drop_vmin"] : -1.0f,
                    doc.containsKey("drop_vmax") ? (float)doc["drop_vmax"] : -1.0f,
                    doc.containsKey("drop_restore") ? (float)doc["drop_restore"] : -1.0f);
                getForceCal(&force_scale, &zero_m1);
                StaticJsonDocument<128> resp;
                resp["result"] = "cal updated";
                resp["force_scale"] = force_scale;
                resp["zero_m1"] = zero_m1;
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
    // webSocket.setNoDelay(true); // Not supported by WebSocketsServer lib; rely on server.setNoDelay(true) for TCP
    webSocket.onEvent(onWebSocketEvent);
    Serial.println("WebSocket server started on port 81.");

    WiFi.setHostname("digital-weight");
    delay(500); // give network stack a moment before starting mDNS
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
        2, // match control task priority to allow fair scheduling
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
    // Skip broadcast if command received recently (within 100ms) to avoid blocking during rapid toggling
    if (nowMs - lastWsMetricsMs >= WS_METRICS_INTERVAL_MS && (nowMs - lastWsCmdMs) > 250) {
        lastWsMetricsMs = nowMs;
        StaticJsonDocument<512> doc;
        doc["type"] = "metrics";
        doc["force"] = sharedData.force;
        doc["position"] = sharedData.position;
        doc["velocity"] = sharedData.velocity;
        doc["accelerometer_x"] = sharedData.accelerometer_x;
        doc["accelerometer_y"] = sharedData.accelerometer_y;
        doc["accelerometer_z"] = sharedData.accelerometer_z;
        doc["voltage"] = sharedStateData.voltage;
        doc["current"] = sharedStateData.current;
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
        doc["voltage"] = sharedStateData.voltage;
        doc["current"] = sharedStateData.current;
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
                // Queue non-blocking idle command
                CommandMsg msg; msg.type = CMD_IDLE; msg.weight_kg = 0; if (g_cmdQueue) { xQueueSend(g_cmdQueue, &msg, 0); }
                if (controlState.mode != 0) {
                    updateModeData("off");
                    updatePulseData("off", 0, 0, 0);
                    updateRowData("off", 0, 0, 0);
                }
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: application/json");
                writeCorsHeaders(client);
                client.println("Connection: close");
                client.println();
                if (doc.containsKey("client_ts")) {
                    client.print("{\"result\":\"idle\",\"client_ts\":");
                    client.print((long long)doc["client_ts"].as<long long>());
                    client.print("}");
                } else {
                    client.print("{\"result\":\"idle\"}");
                }
            }
            else if (cmd == "strength")
            {
                float weight_lbs = doc.containsKey("weight_lbs") ? (float)doc["weight_lbs"] : 0.0f;
                float weight_kg = doc.containsKey("weight_kg") ? (float)doc["weight_kg"] : (weight_lbs * 0.45359237f);
                CommandMsg msg; msg.type = CMD_STRENGTH; msg.weight_kg = weight_kg; if (g_cmdQueue) { xQueueSend(g_cmdQueue, &msg, 0); }
                updateModeData("strength");
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: application/json");
                writeCorsHeaders(client);
                client.println("Connection: close");
                client.println();
                if (doc.containsKey("client_ts")) {
                    client.print("{\"result\":\"strength\",\"client_ts\":");
                    client.print((long long)doc["client_ts"].as<long long>());
                    client.print("}");
                } else {
                    client.print("{\"result\":\"strength\"}");
                }
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
        // Skip broadcast if command received recently (within 100ms) to avoid blocking during rapid toggling
        if (nowMs - lastWsMetricsMs >= WS_METRICS_INTERVAL_MS && (nowMs - lastWsCmdMs) > 250) {
            lastWsMetricsMs = nowMs;
            StaticJsonDocument<512> doc;
            doc["type"] = "metrics";
            doc["force"] = sharedData.force;
            doc["position"] = sharedData.position;
            doc["velocity"] = sharedData.velocity;
            doc["accelerometer_x"] = sharedData.accelerometer_x;
            doc["accelerometer_y"] = sharedData.accelerometer_y;
            doc["accelerometer_z"] = sharedData.accelerometer_z;
            doc["voltage"] = sharedStateData.voltage;
            doc["current"] = sharedStateData.current;
            doc["status"] = sharedData.status;
            doc["virtual_velocity"] = sharedData.virtual_velocity;
            doc["t_http_received_ms"] = sharedData.t_http_received_ms;
            doc["t_cmd_set_ms"] = sharedData.t_cmd_set_ms;
            doc["t_ctrl_apply_ms"] = sharedData.t_ctrl_apply_ms;
            doc["mode_latency_ms"] = sharedData.mode_latency_ms;
            doc["mode_latency_ms"] = sharedData.mode_latency_ms;
            String out; serializeJson(doc, out);
            webSocket.broadcastTXT(out);
        }

        // Small sleep to let IDLE0 run and feed WDT; prevents starvation
        vTaskDelay(1);
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
            doc["voltage"] = sharedStateData.voltage;
            doc["current"] = sharedStateData.current;
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
                    // Route through the command queue like the newer handlers
                    // so the motor task owns all mode/gate transitions
                    CommandMsg msg; msg.type = CMD_IDLE; msg.weight_kg = 0;
                    if (g_cmdQueue) { xQueueSend(g_cmdQueue, &msg, 0); }
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
                    // SAFETY: must go through the command queue so the
                    // weight-update gate applies; writing target_force
                    // directly here would slam the new weight on instantly.
                    CommandMsg msg; msg.type = CMD_STRENGTH; msg.weight_kg = weight_kg;
                    if (g_cmdQueue) { xQueueSend(g_cmdQueue, &msg, 0); }
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
                else if (cmd == "cal")
                {
                    // Live force calibration; omitted params stay unchanged.
                    // RAM only — resets to compiled defaults on reboot.
                    float force_scale = doc.containsKey("force_scale") ? (float)doc["force_scale"] : -1.0f;
                    float zero_m1 = doc.containsKey("zero_m1") ? (float)doc["zero_m1"] : -1.0f;
                    setForceCal(force_scale, zero_m1);
                    if (doc.containsKey("debug")) setForceCalDebug((bool)doc["debug"]);
                    if (doc.containsKey("row") && doc.containsKey("values"))
                    {
                        JsonArray arr = doc["values"];
                        float vals[6]; int n = 0;
                        for (JsonVariant v : arr) { if (n < 6) vals[n++] = v.as<float>(); }
                        setForceCalRow((int)doc["row"], vals, n);
                    }
                    setWeightGateDistances(
                        doc.containsKey("delta_x") ? (float)doc["delta_x"] : -1.0f,
                        doc.containsKey("delta_y") ? (float)doc["delta_y"] : -1.0f,
                        doc.containsKey("deadband") ? (float)doc["deadband"] : -1.0f,
                        doc.containsKey("cancel") ? (float)doc["cancel"] : -1.0f);
                    if (doc.containsKey("pull_sign")) setWeightGatePullSign((float)doc["pull_sign"]);
                    if (doc.containsKey("test_torque")) setTestTorque((float)doc["test_torque"]);
                    setDriveParams(
                        doc.containsKey("amps_per_nm") ? (float)doc["amps_per_nm"] : -1.0f,
                        doc.containsKey("vel_limit") ? (float)doc["vel_limit"] : -1.0f,
                        doc.containsKey("max_current") ? (float)doc["max_current"] : -1.0f);
                    setSoftMinEndpoint(
                        doc.containsKey("ep_min") ? (int)doc["ep_min"] : -1,
                        doc.containsKey("use_ep") ? ((bool)doc["use_ep"] ? 1 : 0) : -1);
                    setHomeBehavior(
                        doc.containsKey("zone") ? (float)doc["zone"] : -1.0f,
                        doc.containsKey("recoil_tq") ? (float)doc["recoil_tq"] : -1.0f,
                        doc.containsKey("recoil_vel") ? (float)doc["recoil_vel"] : -1.0f,
                        doc.containsKey("soft_max") ? (float)doc["soft_max"] : -1.0f,
                        doc.containsKey("sleep_s") ? (float)doc["sleep_s"] : -1.0f,
                        doc.containsKey("wake_d") ? (float)doc["wake_d"] : -1.0f);
                    setDropCatch(
                        doc.containsKey("drop_vmin") ? (float)doc["drop_vmin"] : -1.0f,
                        doc.containsKey("drop_vmax") ? (float)doc["drop_vmax"] : -1.0f,
                        doc.containsKey("drop_restore") ? (float)doc["drop_restore"] : -1.0f);
                    getForceCal(&force_scale, &zero_m1);
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: application/json");
                    writeCorsHeaders(client);
                    client.println("Connection: close");
                    client.println();
                    StaticJsonDocument<128> resp;
                    resp["result"] = "cal updated";
                    resp["force_scale"] = force_scale;
                    resp["zero_m1"] = zero_m1;
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