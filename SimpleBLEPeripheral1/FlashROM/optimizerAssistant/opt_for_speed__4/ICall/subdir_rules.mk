################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
ICall/ICall.obj: C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/ports/tirtos/ICall.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.5/bin/armcl" --cmd_file="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/CCS/SimpleBLEPeripheral/../../IAR/Application/CC2640/../../../CCS/Config/ccsCompilerDefines.bcfg"  -mv7M3 --code_state=16 --abi=eabi -me -O0 --opt_for_speed=4 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.5/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/Source/Application" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/ICall/Include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/DevInfo" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/applib/heap" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/hci" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/controller/CC26xx/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/host" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/CC2650TIRTOS" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/_common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/osal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/sdata" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/saddr" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/include" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/products/cc26xxware_2_21_01_15600" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/packages/ti/boards/SRF06EB/CC2650EM_7ID" -g --gcc --define=USE_ICALL --define=POWER_SAVING --define=SBP_TASK_STACK_SIZE=700 --define=GAPROLE_TASK_STACK_SIZE=520 --define=HEAPMGR_SIZE=2672 --define=TI_DRIVERS_SPI_DMA_INCLUDED --define=TI_DRIVERS_LCD_INCLUDED --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=6 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=MAX_NUM_BLE_CONNS=1 --define=CC26XXWARE --define=CC26XX --diag_wrap=off --diag_suppress=48 --diag_warning=225 --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="ICall/ICall.d" --obj_directory="ICall" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

ICall/ICallCC2650.obj: C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/ports/tirtos/ICallCC2650.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.5/bin/armcl" --cmd_file="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/CCS/SimpleBLEPeripheral/../../IAR/Application/CC2640/../../../CCS/Config/ccsCompilerDefines.bcfg"  -mv7M3 --code_state=16 --abi=eabi -me -O0 --opt_for_speed=4 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.5/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/Source/Application" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/ICall/Include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/DevInfo" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/applib/heap" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/hci" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/controller/CC26xx/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/host" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/CC2650TIRTOS" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/_common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/osal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/sdata" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/saddr" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/include" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/products/cc26xxware_2_21_01_15600" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/packages/ti/boards/SRF06EB/CC2650EM_7ID" -g --gcc --define=USE_ICALL --define=POWER_SAVING --define=SBP_TASK_STACK_SIZE=700 --define=GAPROLE_TASK_STACK_SIZE=520 --define=HEAPMGR_SIZE=2672 --define=TI_DRIVERS_SPI_DMA_INCLUDED --define=TI_DRIVERS_LCD_INCLUDED --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=6 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=MAX_NUM_BLE_CONNS=1 --define=CC26XXWARE --define=CC26XX --diag_wrap=off --diag_suppress=48 --diag_warning=225 --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="ICall/ICallCC2650.d" --obj_directory="ICall" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


