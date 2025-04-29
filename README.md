# BladeOfGrass-NTRIP-client

## Overview

The BladeOfGrass NTRIP Client is an ESP32-based solution designed to achieve RTK fixed positioning by interfacing with the UM980 GNSS module. This client establishes connectivity between NTRIP correction services and GNSS hardware to enable high-precision positioning capabilities.

## Functionality

The system operates by establishing two primary connections:
- Wi-Fi connection to an NTRIP server for correction data
- UART connection to the UM980 GNSS module for location reporting

The client is specifically optimized for the LitPOS network infrastructure, which requires regular location updates. The system follows a precise operational workflow:

1. Every 10 seconds, the NTRIP client requests GGA data from the GNSS module
2. This location data is forwarded to the NTRIP server
3. The server responds by continuously transmitting correction data to the client
4. The client delivers these corrections to the GNSS module

This bidirectional data exchange enables and maintains RTK fixed positioning. Additional modules can simultaneously access location data by connecting to the second UART port of the UM980 module, allowing for independent location data retrieval without interfering with the correction stream.

## License
This project is licensed under the terms of the [LICENSE](./LICENSE) file.