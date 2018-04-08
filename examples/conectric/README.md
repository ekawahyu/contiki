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
* `LEN = 0x0a` message length including this length byte itself is 10 bytes
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

    >060101001e200706148454c4c4f

The incoming message protocol is:

`>``HDRLEN``SEQ``HOPS``HOPMAX``SRCH``SRCL``DLEN``DATA0``DATA1` ... `DATAn`

* `>` an incoming message starts with this character
* `HDRLEN = 0x06` header length including this length byte itself is 6 bytes
* `SEQ = 0x01` message sequence number
* `HOP = 0x01` message has been passed through 1 hop (direct message)
* `HOPMAX = 0x00` zero value means that no hop limit being implemented
* `SRCH = 0x1e` destination address high byte (of 16-bit short address `0x1e20`)
* `SRCL = 0x20` destination address low byte (of  16-bit short address `0x1e20`)
* `DLEN = 0x07` data length is 7 bytes long, including the length byte itself
* `DATA0 = 0x61` request/message type of `CONECTRIC_TEXT_MESSAGE`
* `DATA1 = 0x48` letter `H`
* `DATA2 = 0x45` letter `E`
* `DATA3 = 0x4c` letter `L`
* `DATA4 = 0x4c` letter `L`
* `DATA5 = 0x4f` letter `O`

### RS485 Hub

## Executable Serial Command
In this section we list all executable commands through serial console:

### MAC Address Read (`MR` - MAC Read)
This command read the full 64-bit MAC Address. The 16-bit short address are extracted from the last two bytes. In this example, it is `0xdfbc`. Example of use:

    MR
    MR:00124b000514dfbc

### Configure USB Router as a data sink (`SS` - Sink Set)
When USB Router is set as a data sink, it becomes the gateway between the mesh network to the outside world. The mesh network can have single or multiple data sinks. All of sensor broadcasts will be sent to a sink with the lowest cost. Example of use:

    SS
    SS:Ok

### Disable USB Router as a data sink (`SR` - Sink Reset)
By default, USB Router is not a data sink. But if it was configured as one, then this command will turn if off.

    SR
    SR:Ok

### Show data sinks table (`ST` - Sink Table)
By default, USB Router keeps a table of data sink broadcasts. If no data sink is seen, the command returns no table.

    ST
    ST:0:df.bc(C:2:LT:10)
    |  |  |  |   |    |
    |  |  |  |   |    +-> data sink life time
    |  |  |  |   +-> cost to reach the data sink
    |  |  |  +-> data sink address L
    |  |  +-> data sink address H
    |  +-> list index
    +-> Sink Table indicator

### Show routing table (`RT` - Routing Table)
By default, USB Router keeps a routing table everytime it receives a route discovery request. If no routing table is available, this command returns nothing.

    RT
    RT:0:df.bc->1e.20(C:2:LT:10)
    |  |  |  |   |  |   |    |
    |  |  |  |   |  |   |    +-> routing life time
    |  |  |  |   |  |   +-> routing cost
    |  |  |  |   |  +-> next hop address L
    |  |  |  |   +-> next hop address H
    |  |  |  +-> destination address L
    |  |  +-> destination address H
    |  +-> list index
    +-> Routing Table indicator


## For Developers

## Licensing

This project is licensed under the terms of the [FreeBSD license](https://opensource.org/licenses/BSD-2-Clause). In layman's term, TLDR Legal provides an explanation of [FreeBSD license in plain English](https://tldrlegal.com/license/bsd-2-clause-license-(freebsd)).
