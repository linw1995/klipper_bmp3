# klipper_bmp3
Support BMP3 Sensors for Klipper 

## Setup

```sh
cd ~ && git clone https://github.com/linw1995/klipper_bmp3
cd klipper_bmp3
ln -sf ~/klipper_bmp3/bmp3.py ~/klipper/klippy/extras/
```

## Config

```yaml
#####################################################################
# 	Temperature Sensors
#####################################################################

[bmp3] # Load custom plugin for BMP388 Sensor Type

[temperature_sensor 打印仓-Corner]
sensor_type: BMP388 # Custom Plugin
i2c_software_scl_pin: PA8
i2c_software_sda_pin: PC9
```

## mainsail display

only support the below sensors to display pressure.

```ts
export const additionalSensors = ['bme280', 'aht10', 'htu21d']
```