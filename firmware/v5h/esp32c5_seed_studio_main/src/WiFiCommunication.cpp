#include "WiFiCommunication.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
#include "SharedData.h"
#include "CANCommunication.h"
#include "I2CCommunication.h"
#include "MotorControl.h"
#include <ESPmDNS.h>
#include <WebSocketsServer.h>

// Known networks, tried best-signal-first (WiFiMulti). The phone hotspot
// currently impersonates the home SSID/password; add a distinct entry here
// if that ever changes — one line, nothing else to touch.
// Credentials live in include/secrets.h (gitignored; copy secrets.h.example).
#include "secrets.h"
static const struct
{
    const char *ssid;
    const char *pass;
} KNOWN_NETWORKS[] = {
    {WIFI_SSID_HOME, WIFI_PASS_HOME},
};

static WiFiMulti wifiMulti;
// True once the servers/mDNS/tasks are up (they start on first connect;
// the machine is fully functional offline before/without that).
static volatile bool g_netServicesUp = false;
static void startNetworkServices();
static void WiFiManagerTask(void *parameter);

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
    // Chrome Private Network Access: lets pages on more-public origins pass
    // their preflight when reaching this LAN device
    client.println("Access-Control-Allow-Private-Network: true");
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

// Shared "cal" command body for the WS and HTTP handlers: live tuning of
// the force pipeline; omitted params stay unchanged. RAM only — resets to
// compiled defaults on reboot. ODrive-era knobs (amps_per_nm, max_current,
// ep_min/use_ep) are gone with the moteus move; new here: "reel_in_vel"
// (chase speed) and "brs" (CAN-FD bit-rate-switch on/off — the official
// moteus fallback for marginal 5 Mbps buses is BRS off = 1 Mbps).
static void applyCalCommand(JsonDocument &doc)
{
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
        doc.containsKey("reel_in_vel") ? (float)doc["reel_in_vel"] : -1.0f,
        doc.containsKey("vel_limit") ? (float)doc["vel_limit"] : -1.0f);
    if (doc.containsKey("brs")) setCANUseBRS((bool)doc["brs"]);
    if (doc.containsKey("can_quiet")) setCANQuiet((bool)doc["can_quiet"]);
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
    setFrictionComp(
        doc.containsKey("fric_k") ? (float)doc["fric_k"] : -1.0f,
        doc.containsKey("fric_band") ? (float)doc["fric_band"] : -1.0f);
    setConcentric(
        doc.containsKey("con_pct") ? (float)doc["con_pct"] : -1.0f,
        doc.containsKey("con_lo") ? (float)doc["con_lo"] : -1.0f,
        doc.containsKey("con_hi") ? (float)doc["con_hi"] : -1.0f);
    setRowTuning(
        doc.containsKey("row_kc") ? (float)doc["row_kc"] : -1.0f,
        doc.containsKey("row_inertia") ? (float)doc["row_inertia"] : -1.0f,
        doc.containsKey("row_drag_base") ? (float)doc["row_drag_base"] : -1.0f,
        doc.containsKey("row_return") ? (float)doc["row_return"] : -1.0f,
        doc.containsKey("row_max") ? (float)doc["row_max"] : -1.0f,
        doc.containsKey("row_zone") ? (float)doc["row_zone"] : -1.0f);
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
                CommandMsg msg; msg.type = CMD_IDLE; msg.weight_lb = 0;
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
                // Canonical unit is POUNDS (screen units); weight_kg input
                // is converted TO lb, not the other way around
                float weight_lb = doc.containsKey("weight_lbs") ? (float)doc["weight_lbs"]
                                 : (doc.containsKey("weight_kg") ? (float)doc["weight_kg"] * 2.20462262f : 0.0f);
                // Enqueue non-blocking strength command
                CommandMsg msg; msg.type = CMD_STRENGTH; msg.weight_lb = weight_lb; if (g_cmdQueue) { xQueueSend(g_cmdQueue, &msg, 0); }
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
                // Row mode: {"command":"row","type":"on"|"off","gear":1-10,"drag":1-10}
                // Routed through the command queue like weight commands; the
                // motor task owns the flywheel model and mode transitions.
                // Legacy keys gear_ratio/damping are accepted as aliases.
                String type = doc["type"];
                CommandMsg msg = {};
                msg.type = CMD_ROW;
                msg.row_enable = (type == "off") ? 0 : 1;
                int gear = doc.containsKey("gear") ? (int)doc["gear"]
                          : (doc.containsKey("gear_ratio") ? (int)doc["gear_ratio"] : 0);
                int drag = doc.containsKey("drag") ? (int)doc["drag"]
                          : (doc.containsKey("damping") ? (int)doc["damping"] : 0);
                msg.row_gear = (uint8_t)constrain(gear, 0, 10);
                msg.row_drag = (uint8_t)constrain(drag, 0, 10);
                if (g_cmdQueue) { xQueueSend(g_cmdQueue, &msg, 0); }
                StaticJsonDocument<128> resp;
                resp["result"] = "row updated";
                if (doc.containsKey("client_ts")) resp["client_ts"] = doc["client_ts"];
                String out; serializeJson(resp, out);
                webSocket.sendTXT(num, out);
            } else if (cmd == "cal") {
                applyCalCommand(doc);
                float force_scale, zero_m1;
                getForceCal(&force_scale, &zero_m1);
                StaticJsonDocument<192> resp;
                resp["result"] = "cal updated";
                resp["force_scale"] = force_scale;
                resp["zero_m1"] = zero_m1;
                resp["brs"] = getCANUseBRS();
                resp["can_quiet"] = getCANQuiet();
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
    for (auto &n : KNOWN_NETWORKS)
    {
        wifiMulti.addAP(n.ssid, n.pass);
    }
    Serial.printf("Connecting to WiFi (%u known network(s))...\n",
                  (unsigned)(sizeof(KNOWN_NETWORKS) / sizeof(KNOWN_NETWORKS[0])));
    // One bounded attempt inline so a normal boot comes up connected before
    // setup() ends. If nothing is in range the machine runs OFFLINE (screen
    // and motor fully functional) and the manager task keeps retrying —
    // the old code blocked setup() forever here when away from home.
    if (wifiMulti.run(15000) == WL_CONNECTED)
    {
        startNetworkServices();
    }
    else
    {
        Serial.println("No known WiFi in range — running offline, retrying in background.");
    }
    xTaskCreate(WiFiManagerTask, "WiFiManagerTask", 4096, nullptr, 1, nullptr);
}

// Background connect/reconnect: scans for the best known network whenever
// disconnected and brings the servers up on the first successful join.
// POLICY: never scan while force is being rendered. On the single-core C5
// a dual-band scan preempts the 500 Hz motor stream long enough to trip
// the moteus watchdog — offline garage sessions felt like periodic force
// dropouts every ~13 s (2026-07-17). Attempts run only while the machine
// is idle/asleep, with a 30 s backoff between offline attempts.
static void WiFiManagerTask(void *parameter)
{
    (void)parameter;
    for (;;)
    {
        if (WiFi.status() != WL_CONNECTED && controlState.mode == 0)
        {
            if (wifiMulti.run(8000) == WL_CONNECTED && !g_netServicesUp)
            {
                startNetworkServices();
            }
            if (WiFi.status() != WL_CONNECTED)
            {
                vTaskDelay(pdMS_TO_TICKS(30000)); // offline backoff
                continue;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// Everything that needs a live connection, started exactly once on the
// first join. A later switch to a different known network keeps the same
// servers (they bind to all interfaces); mDNS may go stale across a network
// change — use the IP from the [hb] line in that case.
static void startNetworkServices()
{
    // Disable WiFi modem sleep to reduce latency spikes
    WiFi.setSleep(false);
    // Boost TX power for better link margin
    #ifdef WIFI_POWER_19_5dBm
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    #endif
    Serial.printf("WiFi connected: %s\n", WiFi.SSID().c_str());
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
    // Single-core C5 (the S3 spread these across two cores): priorities do
    // the separation instead — control task p3 > WS p2 > HTTP p1.
    BaseType_t wsOk = xTaskCreate(
        WebSocketTask,
        "WebSocketTask",
        8192,
        nullptr,
        2,
        nullptr);
    BaseType_t httpOk = xTaskCreate(
        HTTPTask,
        "HTTPTask",
        6144,
        nullptr,
        1,
        nullptr);
    if (wsOk == pdPASS && httpOk == pdPASS) {
        g_wifiTasksRunning = true;
        Serial.println("WiFi tasks started: WS(p2), HTTP(p1)");
    } else {
        Serial.println("ERROR: Failed to start WiFi tasks, falling back to loop()-based handling.");
    }
    g_netServicesUp = true;
}

void handleWiFiClients()
{
    // Nothing to service until the first WiFi join brings the servers up
    if (!g_netServicesUp) {
        return;
    }
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
        doc["row_watts"] = sharedStateData.row_watts;
        doc["row_spm"] = sharedStateData.row_spm;
        doc["active_weight"] = sharedStateData.active_weight;
        doc["pending_weight"] = sharedStateData.pending_weight;
        doc["weight_pending"] = sharedStateData.weight_pending;
        doc["t_http_received_ms"] = sharedData.t_http_received_ms;
        doc["t_cmd_set_ms"] = sharedData.t_cmd_set_ms;
        doc["t_ctrl_apply_ms"] = sharedData.t_ctrl_apply_ms;
        String out; serializeJson(doc, out);
        webSocket.broadcastTXT(out);
    }

    WiFiClient client = server.accept();
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
        doc["row_watts"] = sharedStateData.row_watts;
        doc["row_spm"] = sharedStateData.row_spm;
        doc["active_weight"] = sharedStateData.active_weight;
        doc["pending_weight"] = sharedStateData.pending_weight;
        doc["weight_pending"] = sharedStateData.weight_pending;
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
                CommandMsg msg; msg.type = CMD_IDLE; msg.weight_lb = 0; if (g_cmdQueue) { xQueueSend(g_cmdQueue, &msg, 0); }
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
                // Canonical unit is POUNDS (screen units)
                float weight_lb = doc.containsKey("weight_lbs") ? (float)doc["weight_lbs"]
                                 : (doc.containsKey("weight_kg") ? (float)doc["weight_kg"] * 2.20462262f : 0.0f);
                CommandMsg msg; msg.type = CMD_STRENGTH; msg.weight_lb = weight_lb; if (g_cmdQueue) { xQueueSend(g_cmdQueue, &msg, 0); }
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
            doc["row_watts"] = sharedStateData.row_watts;
            doc["row_spm"] = sharedStateData.row_spm;
            doc["active_weight"] = sharedStateData.active_weight;
            doc["pending_weight"] = sharedStateData.pending_weight;
            doc["weight_pending"] = sharedStateData.weight_pending;
            doc["t_http_received_ms"] = sharedData.t_http_received_ms;
            doc["t_cmd_set_ms"] = sharedData.t_cmd_set_ms;
            doc["t_ctrl_apply_ms"] = sharedData.t_ctrl_apply_ms;
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
        WiFiClient client = server.accept();
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
            doc["row_watts"] = sharedStateData.row_watts;
            doc["row_spm"] = sharedStateData.row_spm;
            doc["active_weight"] = sharedStateData.active_weight;
            doc["pending_weight"] = sharedStateData.pending_weight;
            doc["weight_pending"] = sharedStateData.weight_pending;
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
                    CommandMsg msg; msg.type = CMD_IDLE; msg.weight_lb = 0;
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
                    // Canonical unit is POUNDS (screen units)
                    float weight_lb = doc.containsKey("weight_lbs") ? (float)doc["weight_lbs"]
                                     : (doc.containsKey("weight_kg") ? (float)doc["weight_kg"] * 2.20462262f : 0.0f);
                    // SAFETY: must go through the command queue so the
                    // weight-update gate applies; writing target_force
                    // directly here would slam the new weight on instantly.
                    CommandMsg msg; msg.type = CMD_STRENGTH; msg.weight_lb = weight_lb;
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
                    resp["weight_lbs"] = weight_lb;
                    serializeJson(resp, client);
                }
                else if (cmd == "cal")
                {
                    applyCalCommand(doc);
                    float force_scale, zero_m1;
                    getForceCal(&force_scale, &zero_m1);
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: application/json");
                    writeCorsHeaders(client);
                    client.println("Connection: close");
                    client.println();
                    StaticJsonDocument<192> resp;
                    resp["result"] = "cal updated";
                    resp["force_scale"] = force_scale;
                    resp["zero_m1"] = zero_m1;
                    resp["brs"] = getCANUseBRS();
                    resp["can_quiet"] = getCANQuiet();
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
                    // Row mode via the command queue (see WS handler); legacy
                    // keys gear_ratio/damping accepted as gear/drag aliases
                    String type = doc["type"];
                    CommandMsg rowMsg = {};
                    rowMsg.type = CMD_ROW;
                    rowMsg.row_enable = (type == "off") ? 0 : 1;
                    int gear = doc.containsKey("gear") ? (int)doc["gear"]
                              : (doc.containsKey("gear_ratio") ? (int)doc["gear_ratio"] : 0);
                    int drag = doc.containsKey("drag") ? (int)doc["drag"]
                              : (doc.containsKey("damping") ? (int)doc["damping"] : 0);
                    rowMsg.row_gear = (uint8_t)constrain(gear, 0, 10);
                    rowMsg.row_drag = (uint8_t)constrain(drag, 0, 10);
                    if (g_cmdQueue) { xQueueSend(g_cmdQueue, &rowMsg, 0); }
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
