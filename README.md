# Master-slave robot arm demo (MATLAB)

This project simulates a 4-servo arm and optional 3-axis gyroscope over an
acoustic HFM/FSK link. The master and slave each have one GUI; run them in
separate MATLAB sessions or on separate computers.

## Run

In the receiving MATLAB session, connected to the hydrophone/audio input:

```matlab
robot_arm_control("slave")
```

In the transmitting MATLAB session, connected to the underwater speaker/audio
output:

```matlab
robot_arm_control("master")
```

Drag a servo slider with the mouse to move the master arm in the 3-D view.
While a servo or gyro value is changing, the master checks for a new pose once
per waveform interval (about 0.334 seconds) and transmits the latest changed
state. When all values stop changing, transmission stops; an unchanged pose is
not resent. MATLAB uses the operating system's default audio output and input
devices.

## Fixed frame and modem

The protected frame is always 38 bytes:

| Field | Bytes |
| --- | ---: |
| Sync word | 4 |
| Version | 1 |
| Master session ID | 2 |
| Sequence number | 2 |
| Four servo angles (0..180 degrees) | 4 |
| Gyroscope X/Y/Z (signed, 0.01 degree/s resolution) | 6 |
| CRC-16/CCITT | 2 |
| SECDED Hamming encoding for the 17 bytes above | 34 |

The waveform layer adds a 25 ms Hyperbolic FM (HFM) synchronization preamble,
a short guard interval, then 1000 bit/s binary FSK at 48 kHz. The slave
correlates against HFM, demodulates FSK, corrects single-bit errors with
SECDED, checks CRC, and updates its 3-D view only for a new sequence number.
Incoming waveform samples are buffered in a circular ring buffer.

The master plays the modulated waveform using MATLAB audio output. The slave
continuously records from the default audio input, stores quantized samples in
a circular ring buffer, and updates its 3-D arm when a valid frame is decoded.
This is a real acoustic audio path, not a UDP or network transport. For an
underwater link, connect a suitable underwater projector/transducer to the
output chain and a hydrophone/pre-amplifier to the input chain. The specified
14.5–19.5 kHz passband and 48 kHz audio sampling rate must be supported by the
sound interfaces and transducers. Use appropriately rated, waterproof
equipment; ordinary speakers and microphones are not underwater transducers.

Servo sliders and gyro fields are still simulated inputs. The slave rotates
the displayed arm; the program does not yet drive physical servos or sample a
physical gyro. Audio capture is periodically restarted to bound MATLAB's
recorder memory use, so a small capture gap can occur at each restart.

## Tests

```matlab
runtests("test_robot_packet.m")
```
