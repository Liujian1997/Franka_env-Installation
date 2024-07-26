# ESP32 MicroPython环境搭建

- [ESP32 MicroPython环境搭建](#esp32-micropython环境搭建)
  - [下载MicroPython固件](#下载micropython固件)
  - [烧录固件](#烧录固件)

## 下载MicroPython固件

本次使用的`ESP32 S3`，固件下载地址：【[链接](https://micropython.org/download/ESP32_GENERIC_S3/)】

淘宝网店以提供本次的固件以及全部资料，均可在本仓库下载

## 烧录固件

烧录之前需要擦除整个内存：

```bash
pip install esptool
#  从1.3开始的版本支持Python 2.7和Python 3.4(或更新版本)。旧版本(至少需要1.2.1)工作正常，但需要Python 2.7。 
```
