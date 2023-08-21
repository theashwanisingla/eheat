# Envi Eheat Project

This Repo Contains Firmware for Eheat Hardware.  

```text
    Project Support : Bluetooth BluFI Protocol For Provisioning.
                      OTA with AWS IOT Core.
                      Connects with users WiFi network
                      SH1106 Display- able to show text and bitmaps.
```

## How to use this repo

Clone the repo and build the code with esp-idf commands. ex.: ```idf.py build```

## folder contents

The project **SX5001A-FIRMWARE** contains one source file in C language [eheat.c](main/eheat.c).
The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both).

Below is short explanation of remaining files in the project folder.

```text
├── Components                                         This is components folder.
│   ├── BluFi
|   |   └── Include
│   |   |   └── blufi_init.h
│   |   ├── blufi_init.c
│   |   ├── blufi_security.c
|   |   └── CMakeLists.txt
│   ├── u8g2
|   |   └── csrc
│   |       └── **.c                                    All files are the Display files.
|   └── CMakeLists.txt
|
├── main
│   ├── certs
|   |   └── aws_codesign.crt
|   |   └── client.crt
|   |   └── client.key
|   |   └── root_cert_auth.pem
│   ├── blufi_main.c                                    This file is used to establish the BluFi Provisioning.
│   ├── eheat_ota_config.h
│   ├── FreeRTOSIPConfig.h
│   ├── idf_component.yml
│   ├── Kconfig.projbuild
│   ├── mqtt_subscription_manager.c
│   ├── mqtt_subscription_manager.h
│   ├── ota_core_mqtt.c                                 This file used to establish ota support.
│   ├── shadow_helpers.c
│   ├── shadow_helpers.h
│   ├── shadow_main.c
│   ├── icons.h
│   ├── u8g2_esp32_hal.h                                This file is used to initialise Display.
│   ├── u8g2_esp32_hal.c
│   ├── CMakeLists.txt
│   ├── eheat.h
│   └── eheat.c                                         This is the main file. 
├── CMakeLists.txt
├── sdkconfig
├── partitions_ota_mqtt.csv
└── README.md                                           This is the file you are currently reading
```

Need to Add U8g2 Lib in esp-idf/components folder. -> <https://github.com/olikraus/u8g2> for Display.

Above file structure represents the complete repo.
