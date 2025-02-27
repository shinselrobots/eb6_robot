# Streaming Recognition
This package has two main functions:

## Streaming Speech Recogniton
Robot Speech recognition using realtime streaming google speech recognition.
This returns speech as it's decoded, resulting in much lower latency.

## Keyword spotting and non-streaming 
Waits until Keyword heard (robot's name), then sends further sounds to Google for recognizing.
This is mostly used to tell robot to wake up, or to start listening after telling it to stop listening.
Keyword detection utilizes PicoVoice Porcupine keyword spotter
https://picovoice.ai/platform/porcupine/


## Installation
Google Cloud Speech:  Requires account and creation of keys (key.json file)
https://cloud.google.com/sdk/docs/install

Picovoice: Requires (currently free) account, which allows training of your own custom wakeword(s)
see https://picovoice.ai/platform/porcupine/



## Setting default Ubuntu mic and speaker (not sure this is required)

pactl list short sinks
pactl set-default-sink <paste desired device from above> # Example: alsa_output.usb-Plugable_Plugable_USB_Audio_Device_000000000000-00.analog-stereo
Open Startup Applications Preferences
Add:
Name:      Speaker
Command:   pactl set-default-sink <paste desired device from above>

Do the same for microphone:
pactl list short sources
pactl set-default-source <paste desired device from above>  # Example: alsa_input.usb-Cyber_Acoustics_CVL-2005_IM50000001-00.analog-stereo

