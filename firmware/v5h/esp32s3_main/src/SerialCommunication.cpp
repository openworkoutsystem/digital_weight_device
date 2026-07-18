#include "SerialCommunication.h"
#include "SharedData.h"
#include <ArduinoJson.h>

void initSerial()
{
    Serial.begin(115200);
}

// declare processCommand function
void processCommand(const String &data);

void readSerialData()
{
    if (Serial.available())
    {
        String data = Serial.readStringUntil('\n');
        processCommand(data);
        // now print the data that was just processed. ie the respective struct
    }
}

void sendSerialData(const String &data)
{
    Serial.println("DATA:" + data);
}

void sendAggregatedData()
{
    noInterrupts();
    String dataPacket = "";
    dataPacket += "accelerometer_x:" + String(sharedData.accelerometer_x) + "|";
    dataPacket += "accelerometer_y:" + String(sharedData.accelerometer_y) + "|";
    dataPacket += "accelerometer_z:" + String(sharedData.accelerometer_z) + "|";
    dataPacket += "gyro_x:" + String(sharedData.gyro_x) + "|";
    dataPacket += "gyro_y:" + String(sharedData.gyro_y) + "|";
    dataPacket += "gyro_z:" + String(sharedData.gyro_z) + "|";
    dataPacket += "force:" + String(sharedData.force) + "|";
    dataPacket += "position:" + String(sharedData.position) + "|";
    dataPacket += "velocity:" + String(sharedData.velocity) + "|";
    dataPacket += "virtual_velocity:" + String(sharedData.virtual_velocity) + "|";
    dataPacket += "status:" + sharedData.status + "\n";
    interrupts();
    Serial.print("DATA:" + dataPacket);
}

void processCommand(const String &data)
{
    // Split the command and value
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, data);
    if (error)
    {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
    }
    const char *command = doc["command"];

    int separatorIndex = data.indexOf(':');
    if (separatorIndex == -1)
        return;

    if (strcmp(command, "SET_FORCE") == 0)
    {
        const char *type = doc["type"];
        int strength = doc["strength"];
        int start_strength = doc["start_strength"];
        float start_position = doc["start_position"];
        float saturation_position = doc["saturation_position"];

        updateForceData(type, strength, start_strength, start_position, saturation_position);
    }
    else if (strcmp(command, "SET_ROW") == 0)
    {
        const char *type = doc["type"];
        int damping = doc["damping"];
        int gear_ratio = doc["gear_ratio"];
        int inertia = doc["inertia"];

        updateRowData(type, damping, gear_ratio, inertia);
    }
    else if (strcmp(command, "SET_PULSE") == 0)
    {
        const char *type = doc["type"];
        int duration = doc["duration"];
        int strength = doc["strength"];
        int frequency = doc["frequency"];

        updatePulseData(type, duration, strength, frequency);
    }
    else if (strcmp(command, "SET_DETENTS") == 0)
    {
        const char *type = doc["type"];
        int strength = doc["strength"];
        float start_position = doc["start_position"];
        float step_position = doc["step_position"];
        int total_steps = doc["total_steps"];

        updateDetentData(type, strength, start_position, step_position, total_steps);
    }
    else if (strcmp(command, "SET_MODE") == 0)
    {
        const char *type = doc["type"];

        updateModeData(type);
    }
}
