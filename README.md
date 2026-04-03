# Parametric EQ to DSP Playground

This project started as a focused experiment: build a practical, hardware-controlled parametric equalizer.
Along the way it grew into something broader: a general-purpose DSP platform for real-time audio processing,
instrumentation, and UI experiments.

What began as "just three EQ bands" now includes filter control, combined response visualization,
VU metering, spectrum visualization, and a register-based control/data protocol between two MCUs.

![UI](images/ui.png)
![Hardware](images/ratsnest.png)

## Project Story

The original goal was simple:

1. Build a hands-on parametric EQ with physical controls.
2. Show the selected filter behavior and resulting response in real time.
3. Keep the software and hardware open enough to keep evolving.

That third point changed everything. Once the DSP core and UI link were stable, it became obvious that
the architecture could do far more than EQ alone. The codebase now behaves more like a modular DSP
workbench than a single-purpose effect.

## Architecture

The system uses two processors, each doing the job it is best at.

### Teensy 4.1: Audio DSP Engine

- Handles real-time signal processing.
- Computes filter behavior and response data.
- Publishes metering and spectrum data.
- Maintains the parameter/register state that the UI consumes.

Why Teensy 4.1:

- Cortex-M7 performance is excellent for this class of project.
- Floating-point math is fast enough to keep filter code readable and robust.
- Great ecosystem for embedded audio prototyping.

### ESP32 CYD: UI Processor

The UI runs on a CYD (Cheap Yellow Display), which is an ESP32-based board with an integrated TFT.

- Renders LVGL-based UI modes.
- Reads user input from rotary encoders/buttons.
- Displays EQ response, VU, and spectrum information.
- Exchanges register updates with the Teensy via I2C.

### Interconnect: Register-Based I2C Protocol

Communication between DSP and UI uses compact register messages over I2C.

- Control parameters and mode changes flow from UI-side interactions.
- DSP publishes status, levels, and spectrum bins back to UI.
- Commit-style register updates keep state transitions predictable.

## UI Modes

The UI currently provides three main modes, each optimized for a different use case.

### 1) Detailed EQ Mode

Purpose: precise tuning and analysis.

- Displays selected filter response and optional combined response.
- Shows active filter type and selected filter band.
- Shows numeric parameter values including frequency, gain, Q, sample rate, and level settings.
- Intended for surgical parameter work.

### 2) Simple Meter Mode

Purpose: quick operating view.

- Large, easy-to-read core values.
- Stereo VU metering with ballistic behavior and peak hold.
- Designed for "use while listening" rather than deep editing.

### 3) Spectrum Analyzer Mode

Purpose: visualize real-time spectral behavior.

- Real-time spectrum bars with smoothing and peak hold.
- Log-style frequency scale and supplemental operating info.
- Useful for spotting trends, resonances, and level distribution.

### Filter Parameter Popup Overlay

When operating in Simple or Spectrum mode, parameter changes can trigger a framed popup showing key
filter settings (band, type, frequency, gain, and Q). The popup auto-hides after inactivity, and can
be gated by user-input state so remote/background updates do not constantly interrupt the view.

## Current Scope

- Multi-band parametric EQ control.
- Low shelf, high shelf, and peak filter workflows.
- Dual-MCU DSP/UI split with robust register transport.
- Multiple operator-focused UI modes.
- Real-time VU and spectrum visualization.

## Where It Is Heading

Long term, the plan is to fold this work into a DAC/DSP/preamplifier build.
This repository is the proving ground for that hardware/software stack.

## Acknowledgements

Big thanks to the diyAudio.com community.

Many design choices, implementation refinements, and troubleshooting ideas were inspired by the
knowledge shared by diyAudio members. The project benefited directly from their practical guidance,
experience, and generosity in public discussion threads.
