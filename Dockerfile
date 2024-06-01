# Use an official Ubuntu as a parent image
FROM ubuntu:24.04

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    gcc-avr \
    avr-libc \
    avrdude \
    cmake \
    make \
    git \
    && apt-get clean

# Set the working directory inside the container
WORKDIR /AVR-USART-WheelServo

# Specify the command to run when the container starts
CMD ["bash"]
