################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"C:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang" -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/Debug" -I"C:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_10_00_04/source" -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/hardware/IMU660RB/Fusion" -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/hardware/IMU660RB" -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/hardware" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1542680794: ../gpio_toggle_output.syscfg
	@echo 'SysConfig - building file: "$<"'
	"C:/ti/ccs2050/ccs/utils/sysconfig_1.27.0/sysconfig_cli.bat" -s "C:/ti/mspm0_sdk_2_10_00_04/.metadata/product.json" --script "C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/gpio_toggle_output.syscfg" -o "." --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1542680794 ../gpio_toggle_output.syscfg
device.opt: build-1542680794
device.cmd.genlibs: build-1542680794
ti_msp_dl_config.c: build-1542680794
ti_msp_dl_config.h: build-1542680794
Event.dot: build-1542680794

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"C:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang" -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/Debug" -I"C:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_10_00_04/source" -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/hardware/IMU660RB/Fusion" -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/hardware/IMU660RB" -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/hardware" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: C:/ti/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"C:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang" -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/Debug" -I"C:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_10_00_04/source" -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/hardware/IMU660RB/Fusion" -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/hardware/IMU660RB" -I"C:/Users/26447/workspace_ccstheia/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang/hardware" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


