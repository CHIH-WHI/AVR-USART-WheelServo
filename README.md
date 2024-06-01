# AVR-USART-WheelServo

This project is for AVR microcontrollers, specifically using the USART (Universal Synchronous/Asynchronous Receiver-Transmitter) and WheelServo. The project utilizes various advanced techniques for controlling servos and wheels, and for serial communication.

## Features and Technologies Used

- **USART Communication**: Initialization and handling of USART for serial communication.
- **Servo Control**: Functions for starting and controlling servos with precise timing.
- **Wheel Control**: Functions for controlling wheel speeds and applying brakes.
- **Interrupts**: Usage of interrupt service routines (ISR) for handling real-time events.

## Prerequisites

To build this project, you need the following tools installed on your system:

- Docker
- Git

## Getting Started

### Clone the Repository

First, clone the repository to your local machine:

```sh
git clone https://github.com/CHIH-WHI/AVR-USART-WheelServo.git
cd AVR-USART-WheelServo
```

### Build the Docker Image

Build the Docker image that contains the AVR-GCC toolchain and CMake:

```sh
docker build -t avr .
```

### Build the Project

Create a build directory and use CMake to configure the project. Then, compile the code using Make:

```sh
mkdir -p build
cd build
docker run --rm -v $(pwd):/AVR-USART-WheelServo -w /AVR-USART-WheelServo/build avr cmake ..
docker run --rm -v $(pwd):/AVR-USART-WheelServo -w /AVR-USART-WheelServo/build avr make
```

### Flash the Microcontroller

After building the project, you will get a `.hex` file that you can flash to your AVR microcontroller using `avrdude`. Here is an example command to flash the hex file:

```sh
avrdude -c usbasp -p m328p -U flash:w:main.hex
```

Make sure to adjust the `-c` and `-p` options to match your programmer and microcontroller.

## Directory Structure

```
.
├── CMakeLists.txt
├── Dockerfile
├── README.md
├── src
│   └── main.c
└── build
```

- `CMakeLists.txt`: CMake configuration file.
- `Dockerfile`: Dockerfile to set up the build environment.
- `src`: Directory containing the source code.
- `build`: Directory where build artifacts are stored.

## Acknowledgments

- [AVR-GCC](https://gcc.gnu.org/wiki/avr-gcc)
- [CMake](https://cmake.org/)
- [Docker](https://www.docker.com/)
