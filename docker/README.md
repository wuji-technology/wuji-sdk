# Wuji SDK Docker Example

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](../LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-sdk)](https://github.com/wuji-technology/wuji-sdk/releases)

Run the Python SDK in an Ubuntu container with a non-root `wuji` user. The image installs the latest stable SDK from PyPI when built. It provides a runtime for applications mounted from the host.

For SDK usage, see the [Python examples](../examples/python/README.md) and the [Wuji Docs Center](https://docs.wuji.tech/).

## Tested Platforms

| Host Platform | Architecture | Host OS | Docker Engine |
| --- | --- | --- | --- |
| NVIDIA Jetson Orin NX | ARM64 | Ubuntu 20.04.6 LTS, L4T 35.6.1 | 26.1.3 |
| Linux PC | x86_64 | Ubuntu 24.04.4 LTS | 29.1.3 |

## Quick Start

### Prerequisites

- A Linux host with Docker Engine installed.
- For Ethernet devices, a host network interface on the device's subnet that permits UDP broadcasts and SDK traffic.
- For Wuji Hand, host USB access configured as described in [USB Devices](#usb-devices).

The runtime uses Ubuntu 22.04 and Python 3.10.

### Build

Run these commands from this `docker/` directory:

```bash
docker build --pull --no-cache -t wuji-sdk:local .
```

The build uses the host's CPU architecture by default and installs the SDK from PyPI with `pip install wuji-sdk`.

Use `--pull --no-cache` when updating the SDK. A cached build can reuse an earlier installation. Existing images keep their installed version until rebuilt. Keep the image ID of a working deployment before replacing its tag.

### Run Ethernet Devices

```bash
mkdir -p "$HOME/.wuji"
wuji_examples="$(cd ../examples/python && pwd)"
docker run --rm -it --name wuji-sdk --network host \
  --mount "type=bind,src=$wuji_examples,dst=/examples,readonly" \
  --mount "type=bind,src=$HOME/.wuji,dst=/home/wuji/.wuji" \
  wuji-sdk:local
```

This opens a shell as `wuji` (UID/GID 1000), with `/workspace` as the working directory. It does not enable a hand or send movement commands. The bind mount shares the host's `~/.wuji` profiles, calibration data, and logs with the container. Changes made in the container remain in that host directory after the container exits.

The mounted directory and its existing files must be accessible to UID/GID 1000. Check the host account's numeric IDs with `id`. If they differ, grant the container user read/write access through the host directory's permissions or ACLs.

Use `--network host` on Linux so the SDK can discover devices on the host's network. Stop other applications using the target device when checking control operations or comparing host and container performance.

From another host terminal, scan without sending commands:

```bash
docker exec -i wuji-sdk python - <<'PY'
from wuji_sdk import SdkManager

manager = SdkManager.instance()
try:
    for device in manager.scan():
        print(device.sn, device.device_type, device.address)
finally:
    manager.disconnect_all()
PY
```

Run a read-only example in the container shell:

```bash
python /examples/wuji_hand_2/0.subscribe_callback.py
# Or, for a Wuji Glove:
python /examples/wuji_glove/0.subscribe_callback.py
```

To run an application without a shell, append its command to the same `docker run` options, for example `wuji-sdk:local python /examples/wuji_hand_2/0.subscribe_callback.py`. Mount application code read-only, and store output in a separate writable mount.

## USB Devices

Connect Wuji Hand to the host by USB, and mount `/dev/bus/usb` in the container as shown below. If a tactile glove is attached, also pass its serial port, such as `/dev/ttyACM0`, to the container.

### Host Permissions

Create host rules for the device nodes:

```bash
sudo tee /etc/udev/rules.d/70-wuji-container.rules > /dev/null <<'RULES'
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="2000", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5700", MODE="0666"
RULES
sudo udevadm control --reload-rules
```

Reconnect the devices after loading the rules. Check their permissions with `ls -l /dev/bus/usb/*/*` and, when an attachment is present, `ls -l /dev/ttyACM*`. These host rules grant read/write access without adding a user group.

### Run Wuji Hand

Stop the earlier container before reusing its name. From this `docker/` directory:

```bash
mkdir -p "$HOME/.wuji"
wuji_examples="$(cd ../examples/python && pwd)"
docker run --rm -it --name wuji-sdk --network host \
  --mount "type=bind,src=$wuji_examples,dst=/examples,readonly" \
  --mount "type=bind,src=$HOME/.wuji,dst=/home/wuji/.wuji" \
  --mount type=bind,src=/dev/bus/usb,dst=/dev/bus/usb \
  --device-cgroup-rule='c 189:* rw' \
  wuji-sdk:local
```

Keep both the USB mount and the `--device-cgroup-rule` option so the container can access devices after they are reconnected. The host permission rules above must also be applied.

For the tactile attachment, identify its current serial port and add this option to the command above:

```bash
--device=/dev/ttyACM0
```

Replace `/dev/ttyACM0` with the actual serial port. Attach the tactile glove before connecting the hand.

Check the user and run the read-only hand example in the container shell:

```bash
id
python /examples/wuji_hand/0.subscribe.py
# With a tactile glove attached:
python /examples/wuji_hand/3.tactile_status.py
```

## Troubleshooting

1. **The SDK version did not change after a build**

   Rebuild with `--pull --no-cache`, and read the installed version from package metadata. Restart the application using the new image.

2. **No Ethernet device is discovered**

   Use `--network host`, check the network interface, subnet, routing, and firewall, and compare with a scan outside the container. Stop other applications using the device before checking control access.

3. **USB scan succeeds but connection fails**

   Check the USB mount, `--device-cgroup-rule` option, host permission rules, and other applications using the device. Verify that the `wuji` user has read/write access to the device.

4. **An attachment stops working after reconnecting**

   Recreate the container with the current serial port in `--device`, then reconnect the hand through the SDK.

5. **The SDK state directory is not writable**

   Check the permissions of the host's `~/.wuji` directory and its existing files. Grant UID/GID 1000 read/write access before mounting the directory at `/home/wuji/.wuji`.

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
