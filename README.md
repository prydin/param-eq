# Paremetric Equalizer Experiments

## Introduction
This is (at least so far) just a playground for my experiments with digital parametric equalizers. 

### The DSP module
This repo contains a (mostly) working prototype of a three band digital equalizer based on a Teensy 4.1
microcontroller board, which features a ARM Cortex M7 chip. This little processor has some impressive
floating point performance for a chip costing a few dollars. Anecdotally, you can squeeze out hundreds
of MFLOPS from it. Because of this, I decided to do all my filter calculations using single precision
floats, since they're about as fast as 32-bit integers. And it makes everything SO much easier!

I currently offer three types of filters: Peak, low shelf and high shelf. In addition to this, there's
an identity filter that just passes the signal straight through that can be used to disable some of
the filters.

The audio pipeline is based on the Teensy Audio Library. It works great, but it's limited to CD quality.
At some point, I'm going to build my own audio pipeline and I'll probably make it floating point all
the way through. Because I can and it makes it a lot easier to avoid overflows and loss of precision.

### The UI module
So far, the project features three rotary encoders. They are used to vary the frequency, gain and Q of
the filters. The user can also push them to cycle the filter being manipulated, the filter type and
the display mode. 

The display is currently a CYD (Cheap Yellow Display) I found on Amazon for close to no money at all.
It's based on an ESP32 microcontroller driving a TFT display. The communication between the DSP and 
the UI is based on I2C using a simple packet based protocol.

# The future
My goal with this project is to incorporate it into a DAC/DSP/preamplifier that I'm (slowly) designing.
Once I have a decent working version of the equalizer, I will create a PCB housing the DSP hardware.
