---
name: iot-solutions
description: IoT architecture, solutions design, and deployment patterns. Hardware selection, communication protocols, cloud platforms, security, and IoT diagram design. Use when designing IoT systems, selecting IoT hardware, planning IoT architecture, or creating IoT solution diagrams.
---

# IoT Solutions

> Principles for designing and deploying IoT systems from edge to cloud.

---

## 1. IoT Architecture Layers

### Four-Layer Architecture

| Layer           | Components                            | Responsibilities                                  |
| --------------- | ------------------------------------- | ------------------------------------------------- |
| **Perception**  | Sensors, actuators, RFID, cameras     | Physical data collection and interaction          |
| **Network**     | Gateways, routers, protocols          | Data transmission and connectivity                |
| **Processing**  | Edge servers, fog nodes, middleware   | Data aggregation, filtering, local processing     |
| **Application** | Cloud services, analytics, dashboards | Storage, intelligence, visualization, user access |

### Architecture Patterns

| Pattern           | Characteristics           | Use Cases                                              |
| ----------------- | ------------------------- | ------------------------------------------------------ |
| **Edge-First**    | Process at device level   | Real-time requirements, privacy, bandwidth constraints |
| **Fog Computing** | Regional processing nodes | Latency-sensitive, distributed systems                 |
| **Cloud-Centric** | All processing in cloud   | Analytics-heavy, unlimited storage needed              |
| **Hybrid**        | Distributed processing    | Complex systems, varying requirements                  |

---

## 2. Hardware Selection

### Microcontrollers

| Platform        | CPU           | RAM        | Connectivity | Best For                     |
| --------------- | ------------- | ---------- | ------------ | ---------------------------- |
| **Arduino Uno** | 16 MHz        | 2 KB       | None         | Learning, simple projects    |
| **ESP32**       | 240 MHz       | 520 KB     | WiFi, BLE    | IoT prototyping, wireless    |
| **ESP8266**     | 80 MHz        | 80 KB      | WiFi         | Low-cost WiFi projects       |
| **STM32**       | Up to 480 MHz | Up to 1 MB | Various      | Industrial, high-performance |
| **nRF52**       | 64 MHz        | 256 KB     | BLE 5.0      | Wearables, low-power         |

### Single-Board Computers

| Platform               | CPU               | RAM    | Storage   | Use Cases                |
| ---------------------- | ----------------- | ------ | --------- | ------------------------ |
| **Raspberry Pi 4**     | Quad-core 1.5 GHz | 1-8 GB | SD card   | Edge computing, gateway  |
| **NVIDIA Jetson Nano** | Quad-core ARM     | 4 GB   | SD card   | Edge AI, computer vision |
| **BeagleBone Black**   | ARM Cortex-A8     | 512 MB | 4 GB eMMC | Industrial I/O           |
| **Rock Pi**            | Hexa-core         | 2-4 GB | eMMC/SD   | Edge computing           |

### Selection Decision Tree

```
Need WiFi/BLE?
├─ Yes → ESP32, nRF52
└─ No → Arduino, STM32

Need Linux/AI?
├─ Yes → Raspberry Pi, Jetson
└─ No → Microcontroller

Real-time critical?
├─ Yes → STM32, Teensy
└─ No → ESP32, Arduino

Battery-powered?
├─ Yes → ESP32 (deep sleep), nRF52
└─ No → Any platform
```

---

## 3. Sensor Types & Selection

### Environmental Sensors

| Type            | Models                 | Range         | Accuracy | Use Cases            |
| --------------- | ---------------------- | ------------- | -------- | -------------------- |
| **Temperature** | DHT22, DS18B20, BME280 | -40 to 125°C  | ±0.5°C   | HVAC, monitoring     |
| **Humidity**    | DHT22, BME280, SHT31   | 0-100%        | ±2%      | Agriculture, storage |
| **Pressure**    | BMP280, BME280         | 300-1100 hPa  | ±1 hPa   | Weather, altitude    |
| **Air Quality** | CCS811, MQ-135, BME680 | Various gases | Varies   | IAQ monitoring       |

### Motion & Position

| Type           | Models           | Characteristics     | Applications         |
| -------------- | ---------------- | ------------------- | -------------------- |
| **PIR**        | HC-SR501         | Passive infrared    | Motion detection     |
| **Ultrasonic** | HC-SR04          | 2-400 cm            | Distance measurement |
| **LIDAR**      | VL53L0X, TF-Luna | 0-8m, high accuracy | Robotics, automation |
| **IMU**        | MPU6050, BNO055  | 6/9-axis            | Orientation, motion  |
| **GPS**        | NEO-6M, NEO-M8N  | Location tracking   | Asset tracking       |

---

## 4. Communication Protocols

### Wired Protocols

| Protocol     | Speed      | Range  | Topology       | Use Cases              |
| ------------ | ---------- | ------ | -------------- | ---------------------- |
| **I2C**      | 400 kbps   | <1m    | Multi-drop     | Sensor networks        |
| **SPI**      | 10+ Mbps   | <1m    | Star           | High-speed peripherals |
| **UART**     | 115200 bps | <15m   | Point-to-point | Serial communication   |
| **RS485**    | 10 Mbps    | 1200m  | Multi-drop     | Industrial networks    |
| **Modbus**   | Varies     | Varies | Various        | Industrial automation  |
| **Ethernet** | 1 Gbps     | 100m   | Star/mesh      | High-speed, reliable   |

### Wireless Protocols

| Protocol    | Range     | Data Rate   | Power    | Topology  | Use Cases             |
| ----------- | --------- | ----------- | -------- | --------- | --------------------- |
| **WiFi**    | 50-100m   | 54-600 Mbps | High     | Star      | Internet connectivity |
| **BLE**     | 10-100m   | 1-2 Mbps    | Very low | Star/mesh | Wearables, beacons    |
| **Zigbee**  | 10-100m   | 250 kbps    | Low      | Mesh      | Home automation       |
| **LoRaWAN** | 2-15 km   | 0.3-50 kbps | Very low | Star      | Long-range sensors    |
| **NB-IoT**  | Wide area | 100 kbps    | Low      | Cellular  | Asset tracking        |
| **LTE-M**   | Wide area | 1 Mbps      | Medium   | Cellular  | Mobile IoT            |

### Protocol Selection Matrix

| Requirement            | Recommended Protocol |
| ---------------------- | -------------------- |
| Short range, low power | BLE, Zigbee          |
| Long range, low power  | LoRaWAN, NB-IoT      |
| High bandwidth         | WiFi, Ethernet       |
| Mesh networking        | Zigbee, BLE Mesh     |
| Industrial             | RS485, Modbus, CAN   |

---

## 5. Cloud Platforms

### Major Platforms

| Platform             | Strengths                     | Pricing Model    | Best For               |
| -------------------- | ----------------------------- | ---------------- | ---------------------- |
| **AWS IoT Core**     | Scalable, enterprise features | Pay-per-message  | Large deployments      |
| **Azure IoT Hub**    | Microsoft integration         | Per-unit pricing | Enterprise, hybrid     |
| **Google Cloud IoT** | ML/AI integration             | Pay-per-GB       | Analytics-heavy        |
| **ThingSpeak**       | Free tier, MATLAB             | Free/paid tiers  | Education, prototyping |
| **Adafruit IO**      | Hobbyist-friendly             | Free tier        | DIY projects           |
| **Particle Cloud**   | Hardware + cloud              | Device-based     | Cellular IoT           |

### Key Features Comparison

| Feature             | AWS       | Azure     | Google  | ThingSpeak |
| ------------------- | --------- | --------- | ------- | ---------- |
| **Device Registry** | ✓         | ✓         | ✓       | Limited    |
| **Rule Engine**     | ✓         | ✓         | ✓       | ✗          |
| **Analytics**       | Advanced  | Good      | Best    | Basic      |
| **ML Integration**  | SageMaker | ML Studio | AutoML  | MATLAB     |
| **Free Tier**       | Limited   | Limited   | Limited | Yes        |

---

## 6. Message Protocols

### MQTT

| Feature        | Description                                           |
| -------------- | ----------------------------------------------------- |
| **Type**       | Pub/Sub messaging                                     |
| **QoS Levels** | 0 (at most once), 1 (at least once), 2 (exactly once) |
| **Use Cases**  | IoT standard, low bandwidth                           |
| **Brokers**    | Mosquitto, HiveMQ, AWS IoT Core                       |

### HTTP/HTTPS

| Feature       | Description            |
| ------------- | ---------------------- |
| **Type**      | Request/Response       |
| **Security**  | TLS/SSL encryption     |
| **Use Cases** | RESTful APIs, webhooks |
| **Overhead**  | Higher than MQTT       |

### CoAP

| Feature       | Description                  |
| ------------- | ---------------------------- |
| **Type**      | Request/Response (UDP-based) |
| **Overhead**  | Very low                     |
| **Use Cases** | Constrained devices          |
| **Security**  | DTLS                         |

---

## 7. IoT Security

### Security Layers

| Layer           | Threats                              | Mitigation                        |
| --------------- | ------------------------------------ | --------------------------------- |
| **Device**      | Physical tampering, firmware attacks | Secure boot, hardware encryption  |
| **Network**     | Eavesdropping, MITM                  | TLS/DTLS, VPN                     |
| **Application** | Unauthorized access                  | OAuth, API keys, JWT              |
| **Data**        | Data breaches                        | Encryption at rest and in transit |

### Security Best Practices

- [ ] Use strong authentication (certificates, not passwords)
- [ ] Encrypt all data transmission (TLS 1.2+)
- [ ] Implement OTA firmware updates
- [ ] Use principle of least privilege
- [ ] Network segmentation (separate IoT VLAN)
- [ ] Regular security audits
- [ ] Monitor for anomalies
- [ ] Disable unnecessary services/ports

---

## 8. IoT Diagram Design

### Diagram Types

| Type                    | Purpose               | Tools                      |
| ----------------------- | --------------------- | -------------------------- |
| **System Architecture** | High-level components | Draw.io, Lucidchart, Figma |
| **Network Topology**    | Device connections    | Visio, PlantUML            |
| **Data Flow**           | Information movement  | Draw.io, Mermaid           |
| **Deployment**          | Physical layout       | Visio, Draw.io             |
| **Sequence**            | Message flow          | PlantUML, Mermaid          |

### Essential Elements

| Element            | Representation           | When to Include              |
| ------------------ | ------------------------ | ---------------------------- |
| **IoT Devices**    | Icons (sensors, cameras) | Always                       |
| **Gateways**       | Router/bridge icons      | When multiple protocols      |
| **Cloud Services** | Cloud icons              | Always                       |
| **Communication**  | Arrows with labels       | Always (label with protocol) |
| **Data Storage**   | Database cylinders       | When storing data            |
| **Analytics**      | Graph/chart icons        | When processing data         |
| **User Interface** | Mobile/web icons         | Always                       |

### Design Checklist

- [ ] All devices labeled with type
- [ ] Communication protocols clearly marked
- [ ] Data flow direction shown
- [ ] Security boundaries indicated
- [ ] Power sources noted
- [ ] Network segments defined
- [ ] Scalability considered

---

## 9. Power Management

### Power Sources

| Source                 | Capacity      | Lifetime    | Use Cases            |
| ---------------------- | ------------- | ----------- | -------------------- |
| **Alkaline Batteries** | 2000-3000 mAh | Months      | Disposable sensors   |
| **LiPo/Li-ion**        | 500-10000 mAh | 500+ cycles | Rechargeable devices |
| **Solar**              | Varies        | Years       | Remote installations |
| **PoE**                | Unlimited     | Unlimited   | Fixed installations  |
| **Energy Harvesting**  | μW-mW         | Unlimited   | Ultra-low power      |

### Low-Power Techniques

| Technique                   | Power Saving | Implementation                |
| --------------------------- | ------------ | ----------------------------- |
| **Deep Sleep**              | 90-99%       | ESP32: sleep between readings |
| **Sensor Duty Cycling**     | 50-80%       | Turn off when not needed      |
| **Dynamic Voltage Scaling** | 20-40%       | Lower voltage when possible   |
| **Efficient Protocols**     | 30-50%       | BLE vs WiFi                   |
| **Local Processing**        | Varies       | Reduce transmission           |

---

## 10. Edge Computing

### Edge vs Cloud Processing

| Factor          | Edge             | Cloud             |
| --------------- | ---------------- | ----------------- |
| **Latency**     | <100ms           | 100-500ms         |
| **Bandwidth**   | Minimal          | High              |
| **Privacy**     | Data stays local | Data transmitted  |
| **Cost**        | Higher upfront   | Ongoing fees      |
| **Scalability** | Limited          | Unlimited         |
| **Reliability** | Offline capable  | Internet required |

### Edge Processing Use Cases

- Real-time control and automation
- Video analytics and face detection
- Predictive maintenance
- Privacy-sensitive applications
- Bandwidth-constrained environments

---

## 11. Data Management

### Data Pipeline

1. **Collection** - Sensors gather data
2. **Validation** - Check for errors, outliers
3. **Transformation** - Format, aggregate, filter
4. **Storage** - Time-series DB, data lake
5. **Analysis** - Real-time and batch analytics
6. **Visualization** - Dashboards, alerts
7. **Action** - Automated responses

### Storage Options

| Type               | Use Cases       | Examples              |
| ------------------ | --------------- | --------------------- |
| **Time-Series DB** | Sensor data     | InfluxDB, TimescaleDB |
| **NoSQL**          | Flexible schema | MongoDB, DynamoDB     |
| **Data Lake**      | Raw data        | S3, Azure Data Lake   |
| **SQL**            | Structured data | PostgreSQL, MySQL     |

---

## 12. Common Pitfalls

| Problem                      | Solution                                     |
| ---------------------------- | -------------------------------------------- |
| **Underestimating power**    | Measure actual consumption, oversize battery |
| **Poor connectivity**        | Add redundancy, fallback protocols           |
| **Security as afterthought** | Design security from start                   |
| **Scalability issues**       | Plan for 10x growth from day one             |
| **Data loss**                | Implement local buffering, retry logic       |
| **Network congestion**       | Local processing, data aggregation           |
| **Vendor lock-in**           | Use open standards (MQTT, HTTP)              |

---

> **Remember:** Start with clear requirements, choose appropriate protocols, design for security and scale. Test in real conditions before deployment.
