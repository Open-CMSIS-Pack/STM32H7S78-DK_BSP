Board: STMicroelectronics [STM32H7S78-DK](https://www.st.com/en/evaluation-tools/stm32h7s78-dk.html)
------------------------------------------

Device: **STM32H7S7L8H6H**  
System Core Clock: **600 MHz**

This setup is configured using **STM32CubeMX**, an interactive tool provided by STMicroelectronics for device configuration.
Refer to ["Configure STM32 Devices with CubeMX"](https://github.com/Open-CMSIS-Pack/cmsis-toolbox/blob/main/docs/CubeMX.md) for additional information.

### STM32CubeMX configuration
Start STM32CubeMX with default STM32H7S78-DK board setup and make the following changes:
1. Under **Pinout & Configuration** select **Categories - System Core - SYS**:
    - De-select **Boot** and select **Application** for **Runtime context**
    - Select **TIM17** for **Timebase Source**
2. Under **Pinout & Configuration** select **Categories - System Core - NVIC_APPLI**:
    - Under **NVIC** tab change:
      - **UART4 global interrupt - Enabled** to **checked**
      - **SDMMC1 global interrupt - Enabled** to **checked**
    - Under **Code Generation** tab change:
      - **System service call via SWI instruction - Generate IRQ handler** to **unchecked**
      - **Pendable request for system service - Generate IRQ handler** to **unchecked**
      - **System tick timer - Generate IRQ handler** to **unchecked**
3. Under **Pinout & Configuration** select **Categories - Connectivity - USB_OTG_FS**:
    - Under **Mode** section change **Mode** to **Device_Only**
4. Under **Pinout & Configuration** select **Categories - Connectivity - USB_OTG_FS**:
    - Under **Mode** section change **Internal HS Phy** to **Host_Only**
5. Under **Project Manager** select **Project**:
    - Under **Project Settings** section change:
      - **Project Structure - Boot** to **unchecked**
    - Under **Linker Settings** section change:
      - **Minimum Heap Size** set **0x400**
      - **Minimum Stack Size** set **0x400**
6. Under **Project Manager** select **Advanced Settings**:
    - Under **Generated Function Calls** section change:
      - **Do Not Generate Function Calls** to **checked** for **Peripheral Instance Name - SDMMC1** 
7. Click on **GENERATE**

### System Configuration

| System resource         | Setting
|:------------------------|:--------------------------------------------
| Heap                    | 1 kB (configured in the STM32CubeMX)
| Stack (MSP)             | 1 kB (configured in the STM32CubeMX)

### STDIO mapping

**STDIO** is routed to Virtual COM port on the ST-Link (using UART4 peripheral)

### CMSIS-Driver mapping

| CMSIS-Driver  | Peripheral
|:--------------|:----------
| USBD0         | USB_OTG_FS

### CMSIS-Driver Virtual I/O mapping

| CMSIS-Driver VIO  | Physical resource
|:------------------|:-----------------------
| vioBUTTON0        | Button USER (GPIO C.13)
| vioLED0           | LED3 RED    (GPIO M.2)
| vioLED1           | LED1 GREEN  (GPIO O.1)
| vioLED2           | LED4 BLUE   (GPIO M.3)
| vioLED3           | LED2 ORANGE (GPIO O.5)

## Board configuration
**Board setup**
  - Insure that the **JP1** is in the **STLK** position
  - Check that the Boot selection switch **SW1** is in the **0** / default position
  - Connect a **USB C cable** between the **STLK** connector and your **Personal Computer**
