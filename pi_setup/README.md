# Pi setup (Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy)

System changes this rover needs outside the ROS workspace.

## I2C at 50 kHz
In `/boot/firmware/config.txt` (under `[all]`):

    dtparam=i2c_arm=on,i2c_arm_baudrate=50000

At 100 kHz the bus logged "lost arbitration" errors and the PCA9685 dropped off;
at 10 kHz the INA260 stopped answering. This bus also drops the first transaction
after any idle period, so the servo and INA260 nodes retry immediately.

I2C devices: PCA9685 servo driver at 0x40 (0x70 is its all-call address), INA260 at 0x45.

## Serial port for the RoboClaws
Ubuntu has no `/dev/serial0` alias, so `roboclaw_params.yaml` uses `/dev/ttyAMA0`.

- Add `dtparam=uart0=on` to `/boot/firmware/config.txt`
- Remove `console=serial0,115200` from `/boot/firmware/cmdline.txt`
- `sudo systemctl disable --now serial-getty@ttyAMA0.service && sudo systemctl mask serial-getty@ttyAMA0.service`

## Start on boot
    cp start_rover.sh ~/osr_ws/ && chmod +x ~/osr_ws/start_rover.sh
    sudo cp osr.service /etc/systemd/system/
    sudo systemctl daemon-reload && sudo systemctl enable --now osr

`WorkingDirectory`/`LG_WD` must point somewhere writable: lgpio (used by
adafruit-blinka) creates pipe files in the working directory, and the servo node
crashes if the service runs from `/`.

Stop the service (`sudo systemctl stop osr`) before running scripts or a manual launch.
Logs: `journalctl -u osr -f`

## Controller
DualSense over Bluetooth: pair, then `bluetoothctl trust <MAC>` so it reconnects on PS-button press.
