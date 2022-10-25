# DecentralizedLV-LPDRV
A repository for the SolarPack Decentralized Low Voltage Low-Power Driver CAN Boards. The low-power driver boards are a modular implementation of a power system where pins can be controlled by data in a can message

## Hardware Capabilities

- CANBus control with 250K, 500K, and 1Mbps speeds
- 4 Low-power binary outputs (5-Amp max)
- 4 Low-power PWM outputs (5-Amp max)
- 3 High-power outputs (16-Amp, normally closed and normally open)
- 4 Multipurpose outputs
    + Digital input from a switch
    + Digital output to addressable LED strips
    + Analog 12-bit voltage sensing
- Fused 12V supply breakout

## Software Capabilities

- CANBus transmit and receive for reporting data to other microcontrollers in the system
- 10 millisecond response time
- Object-based pins for easy configuration
    + Every pin configurable to automatically respond to data in a CAN message based on type (LP Output, LP PWM Output, HP Output)
    + Once set-up with a CAN Message ID, target data byte and data width, one function will update pin state
    + High modularity with easy disabling of outputs. Not all output hardware must be populated.
- Low-overhead timer to enable animations of neopixel strips and PWM outputs
- Multi-CANBus Message subscription capability to enable controlling different pins with different CAN IDs

## PCB Design

![IMG_2095](https://user-images.githubusercontent.com/47908040/197671351-ad8b1cf1-9c58-4582-a6aa-be047470b1a8.JPG)

## Animation Capability

![headlight animation](https://user-images.githubusercontent.com/47908040/197675022-6a4fffce-e2d0-4149-b49e-6b1ef7047027.gif)
