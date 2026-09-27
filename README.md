# OpenK4A

OpenK4A is a C implementation of the Azure Kinect DK Sensor SDK API for
Windows. It communicates with the camera directly over USB, reads calibration
and requests infrared, color and depth from device, decoding depth on the CPU.

It does not require `k4a.dll`, `depthengine_2_0.dll`, a GPU depth path, or an
installed Azure Kinect Sensor SDK runtime.

## limitations that OpenK4A has removed (compared to AzureKinectSDK)

The Azure Kinect offers more than the MS SDK allows! new features i expose:

- **Infrared at 60 fps.** `k4a_fps_t` holds three rates - 5, 15 and 30 -
  and the SDK refuses anything else, `PASSIVE_IR` at 1024x1024 runs at **
  over twice the rate the normal SDK will allow for.

- **A wider field of view, in depth.** The same decode runs the wide-field
  modes as well as the narrow ones: **120x120 degrees** from `WFOV_UNBINNED` at
  1024x1024 and `WFOV_2X2BINNED` at 512x512, against the narrow modes' 75x65.
  The binned wide mode also runs at **30** where the SDK's table stops at 15 -
  so the widest depth the camera makes is also faster here than the SDK will
  ask for.

- **A little more range, both ends.** The depth comes off the device's own
  radial table, and the pass keeps about **12% more pixels** than the engine
  does - it fills the halo the engine's gates leave around lit surfaces - so
  the near and far ends of the range read a little further than the engine's
  masked output.

- **Latency with the round trip taken out.** The closed SDK sends every raw
  frame up to the GPU, runs `depthengine_2_0.dll` there, and pulls the depth
  back down. This tree never makes that trip: the bytes come off USB straight
  into the C decode, in the same buffer the transfer landed in. The next
  frame's 5.3 MB read is posted before the decode begins, so it runs underneath
  it - a 5.3 MB transfer and a 24 ms decode come out as 33 ms together, not
  57. The upload, the download and the engine dispatch are simply not there.

- **About 270 fps depth frame.** A depth frame is actually just nine taps -
  three frequencies of three phases - exposed one after another, so the sensor
  is genuinely taking roughly 270 exposures a second and this repos exposes it.
 `mkview --show taps` unpacks them one per tick and puts them on the screen at
  **271 fps** (3.69 ms a step, 2.6% of a core): 270
  samples a second of IR time resolution inside a stream the SDK calls thirty.
  It is not 270 frames a second of throughput - the MCU moves the whole 5.3 MB
  frame as one unit and there is no command for a single tap - but it is
  sub-frame timing the camera will not otherwise hand over, and it is a joy to
  watch.


## Depth processing

Depth is decoded from the sensor's raw phase frames.
The depth model was derived from measurements of Microsoft's closed `depthengine_2_0.dll` results.

## Returned Data Types

| Area | Support |
| --- | --- |
| Device | Enumeration, open/close, serial number, hardware version, sync-jack state, and raw or parsed calibration |
| Depth and IR | `NFOV_2X2BINNED` at 320x288, `NFOV_UNBINNED` at 640x576, `WFOV_2X2BINNED` at 512x512, `WFOV_UNBINNED` at 1024x1024, and `PASSIVE_IR` at 1024x1024 |
| Colour | 720p through 3072p, MJPG, NV12, YUY2, and BGRA32 output |
| Camera controls | Exposure, white balance, ISO, brightness, contrast, saturation, sharpness, gain, backlight compensation, and powerline frequency |
| IMU | Accelerometer and gyroscope samples with per-device calibration |
| Metadata | Device and system timestamps, exposure, white balance, ISO, and measured temperatures |
| Processing | Calibration conversion, depth/colour transforms, and depth-to-point-cloud conversion |

## License

OpenK4A is available under the MIT License. See `LICENSE` for details and for
the Microsoft SDK components reproduced in this tree.
