API Reference
=============

This section covers the HTTP API hooks in the ESP32 firmware.

Base URL
---------
All endpoints are at ``http://<device-ip>:80/``.

GET /weight
-----------
**Description**: Retrieves the current weight reading in grams.

**Response**:
- 200 OK: JSON ``{"weight": 123.45, "unit": "g"}``
- Errors: 500 Internal Server Error.

POST /calibrate
----------------
**Description**: Calibrates the scale with a known weight.

**Request Body**: JSON ``{"known_weight": 100.0}``

**Response**:
- 200 OK: ``{"status": "calibrated"}``
- Errors: 400 Bad Request if invalid input.

.. note::
   Add more endpoints here as you implement them. Use tables for complex params:

+------------+----------+----------+
| Parameter  | Type     | Required |
+============+==========+==========+
| known_weight | float  | Yes      |
+------------+----------+----------+