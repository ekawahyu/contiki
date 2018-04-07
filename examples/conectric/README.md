![Conectric Logo](conectric.png)

# Conectric Internet of Things Network Protocol


## Introduction
We are pleased to announce the launch of our Internet of Things Network Protocol stack. It is based on [Contiki](http://www.contiki-os.org/), the ubiqitous open source operating system for the Internet of Things. It connects tiny low-cost, low-power microcontrollers to the Internet and is a powerful toolbox for building complex wireless systems.

We have worked hard to develop an easy-to-use mesh networking stack accompanied with concise documentation for you to get started quickly. We have promised to deliver an end-to-end wireless device messaging requiring painless network configuration and a little to no knowledge of mesh networking.

## Minimum Hardware Requirements
In order to send/receive messages over Conectric network, you need to have at least one Conectric USB Gateway/Router and one of Conectric Sensors (Temperature, Humidity, Motion, Switch, RS485 Hub, etc). We have published an article on [Medium](https://medium.com/conectric-networks/announcing-conectrics-usb-iot-gateway-sensor-product-86087af7ae57) that allows you to go from unboxing to a functional mesh network with in a matter of minutes. Please leave us an email at [solutions@conectric.com](mailto:solutions@conectric.com) for questions about our low-cost IoT Sensors and Device Solutions.

## Wireless Network Messaging
Conectric USB Gateway/Router is equipped with a built-in serial communication peripheral, that allows any computer with at least one USB port to send and receive messages over the mesh network. Any computer can simply dump bytes onto serial console and gets responses at the same inteface.

### Serial Communication
A brief explanation on how the serial communication looks like:

    <0d36000001010600000001480a
    >06790100dfbc09010600000001480a
    >06790100dfbc09010600000001480a
    >06790100dfbc09010600000001480a
    >06790100dfbc09010600000001480a
    >06790100dfbc09010600000001480a
    <0d3600000101060000000089ca
    >067a0100dfbc0901060000000089ca
    >067a0100dfbc0901060000000089ca
    >067a0100dfbc0901060000000089ca
    >067a0100dfbc0901060000000089ca
    >067a0100dfbc0901060000000089ca

All of outgoing network messages start with `<` and all of incoming network messages start with `>`. Let's assume that we wanted to send 'Hello' from one computer through Conectric G3 Serial 


### motion

### switch

### tempHumidity

## For Developers

## Licensing

This project is licensed under the terms of the [FreeBSD license](https://opensource.org/licenses/BSD-2-Clause). In layman's term, TLDR Legal provides an explanation of [FreeBSD license in plain English](https://tldrlegal.com/license/bsd-2-clause-license-(freebsd)).