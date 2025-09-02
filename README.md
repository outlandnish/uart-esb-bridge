# nRF52840 USB-UART-Gazelle Bridge

This project implements firmware for the nRF52840 that acts as a USB UART bridge to another identical board using the Nordic Gazelle protocol.

## Features

- **USB CDC ACM Interface**: Appears as a virtual COM port on the host computer
- **High-Speed UART**: Configured for 1Mbps baud rate communication
- **Gazelle Wireless Protocol**: Nordic proprietary protocol for reliable wireless communication
- **LED Status Indicators**: Visual feedback for USB, Gazelle, and activity status
- **Ring Buffer Flow Control**: Prevents data loss during high throughput
- **Error Handling & Recovery**: Automatic reconnection and error reporting
- **Real-time Statistics**: Monitoring of packet transmission statistics

## Hardware Requirements

- nRF52840 Development Kit (or compatible board)
- USB cable for programming and power
- Another nRF52840 board for wireless communication testing

## Software Requirements

- PlatformIO IDE or CLI
- NRF Connect SDK / Zephyr RTOS (managed by PlatformIO)
- Nordic Gazelle Protocol Library (included with NRF Connect SDK)
- J-Link debugger tools (for flashing)

## Build Instructions

1. **Clone/Download the project**
   ```bash
   cd /path/to/gazelle-link
   ```

2. **Build firmware for both environments**
   ```bash
   # Build host firmware
   pio build -e gazelle_host
   
   # Build device firmware
   pio build -e gazelle_device
   
   # Or build both
   pio build
   ```

3. **Upload to boards**
   ```bash
   # Upload to first board (Host)
   pio upload -e gazelle_host
   
   # Upload to second board (Device) 
   pio upload -e gazelle_device
   ```

4. **Monitor serial output**
   ```bash
   # Monitor host board
   pio device monitor -e gazelle_host
   
   # Monitor device board (in separate terminal)
   pio device monitor -e gazelle_device
   ```

## Configuration

### Gazelle Protocol Settings
- **Channel**: 2 (configurable in `include/gazelle_link.h`)
- **Max Payload**: 32 bytes per packet
- **Timeslot Period**: 600µs
- **Modes**: 
  - Host mode (`gazelle_host` environment)
  - Device mode (`gazelle_device` environment)

### UART Settings
- **Baud Rate**: 1,000,000 bps (1Mbps)
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

### USB CDC Settings
- **VID**: 0x2FE3 (common for both)
- **PID**: 
  - Host mode: 0x0100 ("USB-UART Bridge Host")
  - Device mode: 0x0101 ("USB-UART Bridge Device")

## LED Indicators

- **LED 0** (Green): USB connection status
- **LED 1** (Blue): Gazelle connection status  
- **LED 2** (Red): Data activity indicator

## Usage

1. **Flash two nRF52840 boards** with different firmware:
   - Board 1: `gazelle_host` environment 
   - Board 2: `gazelle_device` environment
2. **Connect both boards** to your computer via USB
3. **Open serial terminals** for debugging:
   - Host board: 115200 baud on `/dev/ttyACM0`
   - Device board: 115200 baud on `/dev/ttyACM1`
4. **Open the USB CDC ports** (virtual COM ports) for data transmission
5. **Send data** through either USB CDC port - it will be wirelessly transmitted to the other board
6. **Data received wirelessly** will appear on the corresponding USB CDC port

### Build Environment Details
- **gazelle_host**: Acts as Gazelle protocol host, manages communication timing
- **gazelle_device**: Acts as Gazelle protocol device, responds to host timing

## File Structure

```
gazelle-link/
├── platformio.ini          # PlatformIO dual-environment configuration
├── prj.conf                # Base Zephyr project configuration (legacy)  
├── prj_host.conf           # Host-specific Zephyr configuration (PID 0x0100)
├── prj_device.conf         # Device-specific Zephyr configuration (PID 0x0101)
├── CMakeLists.txt          # CMake build configuration
├── include/
│   └── gazelle_link.h      # Gazelle protocol interface
├── src/
│   ├── main.c              # Main application code
│   └── gazelle.c           # Nordic Gazelle protocol implementation
└── README.md               # This file
```

## Implementation Details

### Data Flow
1. **USB → Gazelle**: Data received on USB CDC is buffered and transmitted wirelessly
2. **Gazelle → USB**: Wireless data is buffered and sent to USB CDC port
3. **Ring Buffers**: Prevent data loss during speed mismatches
4. **Packet Chunking**: Large data is split into 32-byte Gazelle packets

### Threading
- **Main Thread**: Initialization, status monitoring, error handling
- **Bridge Thread**: Data forwarding between USB and Gazelle
- **Gazelle TX Thread**: Wireless packet transmission simulation
- **Interrupt Handlers**: USB and UART receive processing

### Error Handling
- **Buffer Overflow Protection**: Ring buffers with overflow detection
- **Connection Recovery**: Automatic Gazelle reconnection
- **Statistics Logging**: Periodic packet transmission statistics
- **LED Error Indication**: Visual feedback for error conditions

## Troubleshooting

### Build Issues
- Ensure PlatformIO and Zephyr are properly installed
- Check that the correct board is selected in `platformio.ini`
- Verify J-Link drivers are installed for upload

### Runtime Issues
- Check LED indicators for status information
- Monitor serial debug output at 115200 baud
- Verify both boards are flashed and powered
- Check for interference on the 2.4GHz band

### Performance Issues
- Monitor buffer usage in debug logs
- Adjust ring buffer sizes if necessary
- Consider reducing UART baud rate if packet loss occurs
- Verify Gazelle channel is clear of interference

## Development Notes

### NRF Connect SDK Implementation
This version uses the official Nordic Gazelle implementation from the NRF Connect SDK:

1. Uses `gazelle.c` with official Nordic SDK headers (`nrf_gzll.h`, `nrf_gzp.h`)
2. Includes proper Gazelle libraries via NRF Connect SDK platform
3. Configured with appropriate `prj.conf` settings (`CONFIG_NRF_GZLL=y`)
4. Supports both Host and Device modes via compile-time flags

### Future Enhancements
- Add support for multiple pipe communication
- Implement pairing and security features
- Add configurable baud rate support
- Implement hardware flow control
- Add bootloader for firmware updates

## License

This project is provided as-is for educational and development purposes.