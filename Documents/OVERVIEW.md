# STM32H7S78-DK_BSP

The **STMicroelectronics STM32H7S78-DK Board Support Pack (BSP)**:

- Contains examples in *csolution format* for usage with the [CMSIS-Toolbox](https://github.com/Open-CMSIS-Pack/cmsis-toolbox/blob/main/docs/README.md) and the  [VS Code CMSIS Solution](https://marketplace.visualstudio.com/items?itemName=Arm.cmsis-csolution) extension.
- Requires the [Device Family Pack (DFP) for the STM32H7RS series](https://www.keil.arm.com/packs/stm32h7rsxx_dfp-keil).
- Is configured with [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) for the Arm Compiler 6 (MDK).

## Content in *csolution format*

- [Examples/Blinky](https://github.com/Open-CMSIS-Pack/STM32H7S78-DK_BSP/tree/main/Examples/Blinky) shows the basic usage of this board.

> **Notes:**
>
> - Required STM32CubeMX 6.14.0 or higher.
> - Limitation of STM32CubeMX 6.14.0: Peripheral SDMMC must be disabled, otherwise STM32CubeMX hangs!
