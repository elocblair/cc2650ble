#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#
#  target compatibility key = ti.targets.arm.elf.M3{1,0,5.2,5
#
ifeq (,$(MK_NOGENDEPS))
-include package/cfg/appBLE_pem3.oem3.dep
package/cfg/appBLE_pem3.oem3.dep: ;
endif

package/cfg/appBLE_pem3.oem3: | .interfaces
package/cfg/appBLE_pem3.oem3: package/cfg/appBLE_pem3.c package/cfg/appBLE_pem3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clem3 $< ...
	$(ti.targets.arm.elf.M3.rootDir)/bin/armcl -c  -mv7M3 --code_state=16 --abi=eabi -me -O0 --opt_for_speed=2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.5/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/Source/Application" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/ICall/Include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/DevInfo" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/applib/heap" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/hci" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/controller/CC26xx/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/host" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/CC2650TIRTOS" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/_common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/osal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/sdata" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/saddr" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/include" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/products/cc26xxware_2_21_01_15600" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/packages/ti/boards/SRF06EB/CC2650EM_7ID" -g --gcc --define=USE_ICALL --define=POWER_SAVING --define=SBP_TASK_STACK_SIZE=700 --define=GAPROLE_TASK_STACK_SIZE=520 --define=HEAPMGR_SIZE=2672 --define=TI_DRIVERS_SPI_DMA_INCLUDED --define=TI_DRIVERS_LCD_INCLUDED --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=6 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=MAX_NUM_BLE_CONNS=1 --define=CC26XXWARE --define=CC26XX --diag_wrap=off --diag_suppress=48 --diag_warning=225 --display_error_number --gen_func_subsections=on  -qq -pdsw225 -ms --fp_mode=strict --endian=little -mv7M3 --abi=eabi -eo.oem3 -ea.sem3  --embed_inline_assembly  -Dxdc_cfg__xheader__='"configPkg/package/cfg/appBLE_pem3.h"'  -Dxdc_target_name__=M3 -Dxdc_target_types__=ti/targets/arm/elf/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_5_2_5 -O2  $(XDCINCS) -I$(ti.targets.arm.elf.M3.rootDir)/include/rts -I$(ti.targets.arm.elf.M3.rootDir)/include  -fs=./package/cfg -fr=./package/cfg -fc $<
	$(MKDEP) -a $@.dep -p package/cfg -s oem3 $< -C   -mv7M3 --code_state=16 --abi=eabi -me -O0 --opt_for_speed=2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.5/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/Source/Application" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/ICall/Include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/DevInfo" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/applib/heap" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/hci" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/controller/CC26xx/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/host" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/CC2650TIRTOS" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/_common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/osal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/sdata" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/saddr" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/include" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/products/cc26xxware_2_21_01_15600" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/packages/ti/boards/SRF06EB/CC2650EM_7ID" -g --gcc --define=USE_ICALL --define=POWER_SAVING --define=SBP_TASK_STACK_SIZE=700 --define=GAPROLE_TASK_STACK_SIZE=520 --define=HEAPMGR_SIZE=2672 --define=TI_DRIVERS_SPI_DMA_INCLUDED --define=TI_DRIVERS_LCD_INCLUDED --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=6 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=MAX_NUM_BLE_CONNS=1 --define=CC26XXWARE --define=CC26XX --diag_wrap=off --diag_suppress=48 --diag_warning=225 --display_error_number --gen_func_subsections=on  -qq -pdsw225 -ms --fp_mode=strict --endian=little -mv7M3 --abi=eabi -eo.oem3 -ea.sem3  --embed_inline_assembly  -Dxdc_cfg__xheader__='"configPkg/package/cfg/appBLE_pem3.h"'  -Dxdc_target_name__=M3 -Dxdc_target_types__=ti/targets/arm/elf/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_5_2_5 -O2  $(XDCINCS) -I$(ti.targets.arm.elf.M3.rootDir)/include/rts -I$(ti.targets.arm.elf.M3.rootDir)/include  -fs=./package/cfg -fr=./package/cfg
	-@$(FIXDEP) $@.dep $@.dep
	
package/cfg/appBLE_pem3.oem3: export C_DIR=
package/cfg/appBLE_pem3.oem3: PATH:=$(ti.targets.arm.elf.M3.rootDir)/bin/;$(PATH)
package/cfg/appBLE_pem3.oem3: Path:=$(ti.targets.arm.elf.M3.rootDir)/bin/;$(PATH)

package/cfg/appBLE_pem3.sem3: | .interfaces
package/cfg/appBLE_pem3.sem3: package/cfg/appBLE_pem3.c package/cfg/appBLE_pem3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clem3 -n $< ...
	$(ti.targets.arm.elf.M3.rootDir)/bin/armcl -c -n -s --symdebug:none -mv7M3 --code_state=16 --abi=eabi -me -O0 --opt_for_speed=2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.5/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/Source/Application" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/ICall/Include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/DevInfo" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/applib/heap" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/hci" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/controller/CC26xx/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/host" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/CC2650TIRTOS" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/_common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/osal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/sdata" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/saddr" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/include" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/products/cc26xxware_2_21_01_15600" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/packages/ti/boards/SRF06EB/CC2650EM_7ID" -g --gcc --define=USE_ICALL --define=POWER_SAVING --define=SBP_TASK_STACK_SIZE=700 --define=GAPROLE_TASK_STACK_SIZE=520 --define=HEAPMGR_SIZE=2672 --define=TI_DRIVERS_SPI_DMA_INCLUDED --define=TI_DRIVERS_LCD_INCLUDED --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=6 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=MAX_NUM_BLE_CONNS=1 --define=CC26XXWARE --define=CC26XX --diag_wrap=off --diag_suppress=48 --diag_warning=225 --display_error_number --gen_func_subsections=on  -qq -pdsw225 --endian=little -mv7M3 --abi=eabi -eo.oem3 -ea.sem3   -Dxdc_cfg__xheader__='"configPkg/package/cfg/appBLE_pem3.h"'  -Dxdc_target_name__=M3 -Dxdc_target_types__=ti/targets/arm/elf/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_5_2_5 -O2  $(XDCINCS) -I$(ti.targets.arm.elf.M3.rootDir)/include/rts -I$(ti.targets.arm.elf.M3.rootDir)/include  -fs=./package/cfg -fr=./package/cfg -fc $<
	$(MKDEP) -a $@.dep -p package/cfg -s oem3 $< -C  -n -s --symdebug:none -mv7M3 --code_state=16 --abi=eabi -me -O0 --opt_for_speed=2 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.5/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/SimpleBLEPeripheral/CC26xx/Source/Application" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/ICall/Include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/Roles" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/DevInfo" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile/CC26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/Profiles/SimpleProfile" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Projects/ble/common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/applib/heap" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/hci" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/controller/CC26xx/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/host" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/CC2650TIRTOS" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/target/_common/cc26xx" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/hal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/osal/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/sdata" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/services/saddr" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/icall/include" --include_path="C:/ti/simplelink/ble_cc26xx_2_01_01_44627/Components/ble/include" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/products/cc26xxware_2_21_01_15600" --include_path="C:/ti/tirtos_simplelink_2_13_00_06/packages/ti/boards/SRF06EB/CC2650EM_7ID" -g --gcc --define=USE_ICALL --define=POWER_SAVING --define=SBP_TASK_STACK_SIZE=700 --define=GAPROLE_TASK_STACK_SIZE=520 --define=HEAPMGR_SIZE=2672 --define=TI_DRIVERS_SPI_DMA_INCLUDED --define=TI_DRIVERS_LCD_INCLUDED --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=6 --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=MAX_NUM_BLE_CONNS=1 --define=CC26XXWARE --define=CC26XX --diag_wrap=off --diag_suppress=48 --diag_warning=225 --display_error_number --gen_func_subsections=on  -qq -pdsw225 --endian=little -mv7M3 --abi=eabi -eo.oem3 -ea.sem3   -Dxdc_cfg__xheader__='"configPkg/package/cfg/appBLE_pem3.h"'  -Dxdc_target_name__=M3 -Dxdc_target_types__=ti/targets/arm/elf/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_5_2_5 -O2  $(XDCINCS) -I$(ti.targets.arm.elf.M3.rootDir)/include/rts -I$(ti.targets.arm.elf.M3.rootDir)/include  -fs=./package/cfg -fr=./package/cfg
	-@$(FIXDEP) $@.dep $@.dep
	
package/cfg/appBLE_pem3.sem3: export C_DIR=
package/cfg/appBLE_pem3.sem3: PATH:=$(ti.targets.arm.elf.M3.rootDir)/bin/;$(PATH)
package/cfg/appBLE_pem3.sem3: Path:=$(ti.targets.arm.elf.M3.rootDir)/bin/;$(PATH)

clean,em3 ::
	-$(RM) package/cfg/appBLE_pem3.oem3
	-$(RM) package/cfg/appBLE_pem3.sem3

appBLE.pem3: package/cfg/appBLE_pem3.oem3 package/cfg/appBLE_pem3.mak

clean::
	-$(RM) package/cfg/appBLE_pem3.mak
