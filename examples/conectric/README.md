![Conectric Logo](conectric.png)

# Conectric Internet of Things Network Protocol


## Introduction
We are pleased to announce the launch of our Internet of Things Network Protocol stack. It is based on [Contiki](http://www.contiki-os.org/), the ubiqitous open source operating system for the Internet of Things. It connects tiny low-cost, low-power microcontrollers to the Internet and is a powerful toolbox for building complex wireless systems.

We have worked hard to develop an easy-to-use mesh networking stack accompanied with concise documentation for you to get started quickly. We have promised to deliver an end-to-end wireless device messaging requiring painless network configuration and a little to no knowledge of mesh networking.

## Minimum Hardware Requirements
In order to send/receive messages over Conectric network, you need to have at least one Conectric USB Router and one of Conectric Sensors (Temperature, Humidity, Motion, Switch, RS485 Hub, etc). We have published an article on [Medium](https://medium.com/conectric-networks/announcing-conectrics-usb-iot-gateway-sensor-product-86087af7ae57) that allows you to go from unboxing to a functional mesh network with in a matter of minutes. Please leave us an email at [solutions@conectric.com](mailto:solutions@conectric.com) for questions about our low-cost IoT Sensors and Device Solutions.

## Wireless Network Messaging
Conectric USB Router is equipped with a built-in serial communication peripheral, that allows any computer with at least one USB port to send and receive messages over the mesh network. Any computer can simply dump bytes onto serial console and gets responses at the same inteface.

### Serial Communication

All of outgoing network messages start with `<` and all of incoming network messages start with `>`. Messages are in hexstring format and every 2 hexstring digits represent 1 byte. Let's assume that there are two computers and two Conectric USB Routers, and you want to send a message `HELLO` (= `48 45 4c 4c 4f` in hexstring) from one end to the other. Please Refer to this [ASCII Table](http://asciitable.com) to get the hexadecimal value for each letter you wanted to send.

The outgoing messaging protocol is as follow:

`<``LEN``REQ``DESTH``DESTL``01``DATA0``DATA1` ... `DATAn`

* `<` an outgoing message starts with this character
* `LEN = 0x0a` message length including this length byte itself is 10 hextrings
* `REQ = 0x61` request/message type of `CONECTRIC_TEXT_MESSAGE`
* `DESTH = 0xdf` destination address high byte (of 16-bit short address `0xdfbc`)
* `DESTL = 0xbc` destination address low byte (of  16-bit short address `0xdfbc`)
* `01 = 0x01` this byte is reserved, always 0x01
* `DATA0 = 0x48` letter `H`
* `DATA1 = 0x45` letter `E`
* `DATA2 = 0x4c` letter `L`
* `DATA3 = 0x4c` letter `L`
* `DATA4 = 0x4f` letter `O`

So sending out `HELLO` message would look like this:

    <0a6100000148454c4c4f

You can type those digits and letters in, manually by hand, on the serial console, then press `RETURN` key to send this message out in the air. When the wireless device with 16-bit short address of `0xdfbc` is listening and within the radio coverage. It will receive an incoming message that looks like the following:

    >06010100dfbc0d6168656a6a6f207468657265

The incoming message protocol is:





### motion

### switch

### tempHumidity

## For Developers

## Licensing

This project is licensed under the terms of the [FreeBSD license](https://opensource.org/licenses/BSD-2-Clause). In layman's term, TLDR Legal provides an explanation of [FreeBSD license in plain English](https://tldrlegal.com/license/bsd-2-clause-license-(freebsd)).
