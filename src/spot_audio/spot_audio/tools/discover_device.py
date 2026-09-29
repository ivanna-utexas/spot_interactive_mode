"""Discover and probe ALSA audio input devices.

Enumerates all audio input devices visible to PortAudio, logs capabilities
(sample rates, channels, latency), and optionally highlights a device by
substring match.

Usage:
    ros2 run spot_audio discover_device
    ros2 run spot_audio discover_device -- --name Anker
    python3 -m spot_audio.tools.discover_device --name Anker
"""

import argparse
import sys

import sounddevice as sd


def main(args=None):
    parser = argparse.ArgumentParser(description='Discover audio input devices')
    parser.add_argument(
        '--name', type=str, default='',
        help='Substring to highlight/match (case-insensitive)',
    )
    parsed = parser.parse_args(args=args)
    search = parsed.name.lower()

    print('=' * 60)
    print('Audio Device Discovery')
    print('=' * 60)

    devices = sd.query_devices()
    input_devices = []

    for i, dev in enumerate(devices):
        if dev['max_input_channels'] > 0:
            input_devices.append((i, dev))

    if not input_devices:
        print('\nERROR: No audio input devices found!')
        print('Check that:')
        print('  - /dev/snd is accessible in the container')
        print('  - Your microphone is connected via USB')
        print('  - Host PulseAudio is not holding the device')
        print('    (check: pactl list short sources)')
        print('  - ALSA/PortAudio libraries are installed (libportaudio2)')
        sys.exit(1)

    print(f'\nFound {len(input_devices)} input device(s):\n')

    match_found = False

    for idx, dev in input_devices:
        is_match = search and search in dev['name'].lower()
        marker = ' <<<< MATCH' if is_match else ''
        if is_match:
            match_found = True

        print(f'  [{idx}] {dev["name"]}{marker}')
        print(f'       Max input channels : {dev["max_input_channels"]}')
        print(f'       Default sample rate : {dev["default_samplerate"]:.0f} Hz')
        print(f'       Default low latency : {dev["default_low_input_latency"]:.4f}s')
        print(f'       Default high latency: {dev["default_high_input_latency"]:.4f}s')

        # Probe supported capture sample rates
        test_rates = [8000, 16000, 22050, 44100, 48000, 96000]
        supported = []
        for rate in test_rates:
            try:
                sd.check_input_settings(
                    device=idx,
                    channels=1,
                    samplerate=rate,
                    dtype='float32',
                )
                supported.append(rate)
            except sd.PortAudioError:
                pass

        print(f'       Supported rates     : {supported}')

        # Probe channel counts
        test_channels = [1, 2, 4, 7, 8]
        supported_ch = []
        for ch in test_channels:
            if ch > dev['max_input_channels']:
                continue
            try:
                sd.check_input_settings(
                    device=idx,
                    channels=ch,
                    samplerate=supported[0] if supported else dev['default_samplerate'],
                    dtype='float32',
                )
                supported_ch.append(ch)
            except sd.PortAudioError:
                pass

        print(f'       Supported channels  : {supported_ch}')
        print()

    print('=' * 60)
    if search:
        if match_found:
            print(f'RESULT: Device matching "{parsed.name}" found.')
        else:
            print(f'WARNING: No device matching "{parsed.name}" found.')
            print('Available device names:')
            for idx, dev in input_devices:
                print(f'  [{idx}] {dev["name"]}')
    print()
    print('TIP: Use `device_id: N` in audio.yaml for deterministic selection.')
    print('=' * 60)

    if search:
        sys.exit(0 if match_found else 1)
    sys.exit(0)


if __name__ == '__main__':
    main()
