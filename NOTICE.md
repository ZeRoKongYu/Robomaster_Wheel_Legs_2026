# Third-Party License Notice

This repository contains original RoboMaster embedded-control code together
with vendor and middleware code generated or distributed for STM32 projects.

## Project code

Original application, task, control, motor, board-communication, and
documentation files are released under the root `LICENSE` unless a file says
otherwise.

## Bundled third-party components

- `Drivers/CMSIS/`: Arm CMSIS, Apache-2.0. See `Drivers/CMSIS/LICENSE.txt`.
- `Drivers/CMSIS/Device/ST/STM32F4xx/`: ST device support files. See the
  license notice inside that directory.
- `Drivers/STM32F4xx_HAL_Driver/`: STM32 HAL driver files generated or
  distributed by STMicroelectronics. Keep the ST file headers and package
  license terms.
- `Middlewares/Third_Party/FreeRTOS/`: FreeRTOS kernel/CMSIS-RTOS wrapper,
  MIT license. See `Middlewares/Third_Party/FreeRTOS/Source/LICENSE`.
- `Middlewares/ST/STM32_USB_Device_Library/`: ST USB Device Library. See
  `Middlewares/ST/STM32_USB_Device_Library/LICENSE.txt`; this component has
  ST-specific redistribution and device-use terms.

The root MIT license does not relicense those third-party components. When
porting or redistributing the project, retain all original license files and
headers.
