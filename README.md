# Zephyr Bornhack 2023 Badge

## What's this?

This is a Zephyr based project to play around with the Bornhack 2023 badge.
The badge consists of two parts, an I2C-progammable NFC chip and a microcontroller based "reader".
The latter one is particularly interesting and I'm currently only working with this and not the other part.

This "reader" has an [RP2040](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
(the microcontroller made for/used on Raspberry Pi Picos) and the NFC Controller [PN7150](https://www.nxp.com/products/rfid-nfc/nfc-hf/nfc-readers/high-performance-nfc-controller-with-integrated-firmware-for-smart-devices:PN7150) made by NXP.
It also features an in-built antenna and is made in a credid card shape. Here's the [official GitHub repo](https://github.com/bornack/badge2023) of the badge. 

I'm putting quotes around "reader", because, while how it was being sold it only could really read,
it technically is a full on NFC Forum Device (with some light limitations) which can also do things
like Card Emulation and Peer-to-Peer communication.

Currently I'm trying to get NFC-B ISO-DEP (thus T4T) Card Emulation to run. 

## Why reinvent the wheel? Aren't there other NFC implementations out there?

Yes, there are. The reason I'm doing this more or less from scratch is because I want to learn about the technology on a very deep level.
I've already gathered (and read big portions of) lots of standards and documentation about the chips, protocols and whatnot involved
and I think it's a good exercise to also get better at programming in C++ and expanding my skill- and toolset for embedded programming.

It's fun. I'm having fun. Let me do my thing. Thanks :3

## Why Zephyr?

First of all, starting off I didn't (and still don't) know a lot about Zephyr, but I really like the versatility of it.
I'm not doing a deep dive here but it is very easy to port Zephyr based projects to different hardware
which might be useful for projects in the future involving NFC or other people who -for whatever reason- would want to use my dreadful code. 
