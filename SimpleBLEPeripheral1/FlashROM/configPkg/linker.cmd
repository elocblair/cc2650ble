/*
 * Do not modify this file; it is automatically generated from the template
 * linkcmd.xdt in the ti.platforms.simplelink package and will be overwritten.
 */

/*
 * put '"'s around paths because, without this, the linker
 * considers '-' as minus operator, not a file name character.
 */


-l"C:\ti\simplelink\ble_cc26xx_2_01_01_44627\Projects\ble\SimpleBLEPeripheral\CC26xx\CCS\SimpleBLEPeripheral\FlashROM\configPkg\package\cfg\appBLE_pem3.oem3"
-l"C:\ti\simplelink\ble_cc26xx_2_01_01_44627\Projects\ble\SimpleBLEPeripheral\CC26xx\CCS\Config\src\sysbios\sysbios.aem3"
-l"C:\ti\tirtos_simplelink_2_13_00_06\products\bios_6_42_00_08\packages\ti\targets\arm\rtsarm\lib\boot.aem3"
-l"C:\ti\tirtos_simplelink_2_13_00_06\products\bios_6_42_00_08\packages\ti\targets\arm\rtsarm\lib\auto_init.aem3"

--retain="*(xdc.meta)"

/* C6x Elf symbols */
--symbol_map __TI_STACK_SIZE=__STACK_SIZE
--symbol_map __TI_STACK_BASE=__stack
--symbol_map _stack=__stack


--args 0x0
-heap  0x0
-stack 0x400

/*
 * Linker command file contributions from all loaded packages:
 */

/* Content from xdc.services.global (null): */

/* Content from xdc (null): */

/* Content from xdc.corevers (null): */

/* Content from xdc.shelf (null): */

/* Content from xdc.services.spec (null): */

/* Content from xdc.services.intern.xsr (null): */

/* Content from xdc.services.intern.gen (null): */

/* Content from xdc.services.intern.cmd (null): */

/* Content from xdc.bld (null): */

/* Content from ti.targets (null): */

/* Content from ti.targets.arm.elf (null): */

/* Content from xdc.rov (null): */

/* Content from xdc.runtime (null): */

/* Content from ti.targets.arm.rtsarm (null): */

/* Content from ti.sysbios.rom (null): */

/* Content from ti.sysbios.interfaces (null): */

/* Content from ti.sysbios.family (null): */

/* Content from ti.sysbios.family.arm (ti/sysbios/family/arm/linkcmd.xdt): */
--retain "*(.vecs)"

/* Content from xdc.services.getset (null): */

/* Content from ti.sysbios.rts (ti/sysbios/rts/linkcmd.xdt): */

/* Content from xdc.runtime.knl (null): */

/* Content from ti.catalog.arm.cortexm3 (null): */

/* Content from ti.catalog.peripherals.hdvicp2 (null): */

/* Content from ti.catalog (null): */

/* Content from ti.catalog.arm.peripherals.timers (null): */

/* Content from xdc.platform (null): */

/* Content from xdc.cfg (null): */

/* Content from ti.catalog.arm.cortexm4 (null): */

/* Content from ti.platforms.simplelink (null): */

/* Content from ti.sysbios (null): */

/* Content from ti.sysbios.hal (null): */

/* Content from ti.sysbios.family.arm.cc26xx (null): */

/* Content from ti.sysbios.family.arm.m3 (ti/sysbios/family/arm/m3/linkcmd.xdt): */
-u _c_int00
--retain "*(.resetVecs)"
ti_sysbios_family_arm_m3_Hwi_nvic = 0xe000e000;

/* Content from ti.sysbios.knl (null): */

/* Content from ti.sysbios.gates (null): */

/* Content from ti.sysbios.heaps (null): */

/* Content from ti.sysbios.rom.cortexm.cc26xx (C:/ti/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/sysbios/rom/cortexm/cc26xx/golden/CC26xx/CC26xx_link_ti.xdt): */

-u xdc_runtime_Error_policy__C
-u xdc_runtime_IModule_Interface__BASE__C
-u xdc_runtime_Startup_lastFxns__C
-u ti_sysbios_gates_GateMutex_Object__DESC__C
-u ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_initDevice__I
-u xdc_runtime_Startup_execImpl__C
-u ti_sysbios_gates_GateMutex_Instance_State_sem__O
-u ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_getMaxTicks__E
-u ti_sysbios_knl_Swi_Object__count__C
-u ti_sysbios_knl_Idle_funcList__C
-u ti_sysbios_family_arm_m3_Hwi_Object__PARAMS__C
-u xdc_runtime_Text_isLoaded__C
-u ti_sysbios_knl_Clock_Object__DESC__C
-u ti_sysbios_knl_Mailbox_Instance_State_dataQue__O
-u ti_sysbios_gates_GateMutex_Module__FXNS__C
-u ti_sysbios_knl_Task_Module_State_inactiveQ__O
-u ti_sysbios_family_arm_m3_Hwi_Module__id__C
-u ti_sysbios_family_arm_cc26xx_Timer_Module__id__C
-u ti_sysbios_knl_Mailbox_Object__table__C
-u ti_sysbios_family_arm_m3_Hwi_Object__table__C
-u ti_sysbios_knl_Swi_Object__DESC__C
-u xdc_runtime_Text_charCnt__C
-u ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_start__E
-u ti_sysbios_heaps_HeapMem_Object__table__C
-u xdc_runtime_Error_policyFxn__C
-u ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_getCount64__E
-u xdc_runtime_Startup_firstFxns__C
-u ti_sysbios_knl_Swi_Object__PARAMS__C
-u ti_sysbios_knl_Clock_serviceMargin__C
-u xdc_runtime_Text_charTab__C
-u ti_sysbios_rom_ROM_AONRTCCurrentCompareValueGet
-u ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_TimestampProvider_get32__E
-u ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_getCurrentTick__E
-u ti_sysbios_family_arm_m3_TaskSupport_stackAlignment__C
-u ti_sysbios_family_arm_m3_Hwi_NUM_INTERRUPTS__C
-u xdc_runtime_Main_Module__diagsMask__C
-u ti_sysbios_knl_Swi_Object__table__C
-u xdc_runtime_Memory_Module__id__C
-u ti_sysbios_knl_Task_Object__PARAMS__C
-u ti_sysbios_gates_GateMutex_Object__PARAMS__C
-u ti_sysbios_heaps_HeapMem_Module__gateObj__C
-u ti_sysbios_family_arm_cc26xx_Timer_startupNeeded__C
-u ti_sysbios_knl_Queue_Object__DESC__C
-u ti_sysbios_knl_Task_Object__DESC__C
-u xdc_runtime_Assert_E_assertFailed__C
-u ti_sysbios_heaps_HeapMem_Object__PARAMS__C
-u ti_sysbios_gates_GateHwi_Module__id__C
-u ti_sysbios_gates_GateHwi_Object__PARAMS__C
-u xdc_runtime_IHeap_Interface__BASE__C
-u xdc_runtime_SysCallback_exitFxn__C
-u ti_sysbios_heaps_HeapMem_Module__id__C
-u ti_sysbios_family_arm_m3_Hwi_excHandlerFunc__C
-u ti_sysbios_heaps_HeapMem_Module__FXNS__C
-u xdc_runtime_System_maxAtexitHandlers__C
-u ti_sysbios_knl_Queue_Object__count__C
-u ti_sysbios_knl_Task_Object__table__C
-u ti_sysbios_knl_Mailbox_Object__DESC__C
-u ti_sysbios_family_arm_m3_Hwi_nullIsrFunc__C
-u ti_sysbios_knl_Clock_tickMode__C
-u ti_sysbios_gates_GateMutex_Module__id__C
-u ti_sysbios_knl_Swi_numPriorities__C
-u ti_sysbios_knl_Task_numConstructedTasks__C
-u xdc_runtime_Startup_maxPasses__C
-u ti_sysbios_rom_ROM_AONRTCEventClear
-u ti_sysbios_knl_Task_initStackFlag__C
-u xdc_runtime_Main_Module__diagsEnabled__C
-u xdc_runtime_Main_Module__diagsIncluded__C
-u xdc_runtime_System_abortFxn__C
-u ti_sysbios_knl_Mailbox_Instance_State_dataSem__O
-u ti_sysbios_gates_GateHwi_Module__FXNS__C
-u ti_sysbios_hal_Hwi_Object__DESC__C
-u ti_sysbios_family_arm_m3_Hwi_priGroup__C
-u xdc_runtime_Error_E_memory__C
-u ti_sysbios_family_arm_m3_Hwi_E_alreadyDefined__C
-u ti_sysbios_knl_Mailbox_Instance_State_freeSem__O
-u ti_sysbios_knl_Queue_Object__table__C
-u ti_sysbios_knl_Semaphore_Object__PARAMS__C
-u xdc_runtime_System_exitFxn__C
-u ti_sysbios_knl_Clock_Object__PARAMS__C
-u ti_sysbios_rom_ROM_AONRTCCompareValueSet
-u ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_setNextTick__E
-u ti_sysbios_heaps_HeapMem_reqAlign__C
-u xdc_runtime_Main_Module__id__C
-u xdc_runtime_Startup_sfxnRts__C
-u ti_sysbios_knl_Semaphore_Object__DESC__C
-u ti_sysbios_gates_GateHwi_Object__DESC__C
-u ti_sysbios_heaps_HeapMem_Object__count__C
-u ti_sysbios_family_arm_m3_Hwi_numSparseInterrupts__C
-u ti_sysbios_family_arm_cc26xx_TimestampProvider_useClockTimer__C
-u ti_sysbios_rom_ROM_xdc_runtime_System_SupportProxy_exit__E
-u ti_sysbios_knl_Queue_Object__PARAMS__C
-u ti_sysbios_knl_Task_allBlockedFunc__C
-u ti_sysbios_rom_ROM_xdc_runtime_System_SupportProxy_abort__E
-u ti_sysbios_knl_Mailbox_Object__count__C
-u xdc_runtime_Text_nameStatic__C
-u ti_sysbios_rom_ROM_xdc_runtime_Startup_getState__I
-u ti_sysbios_knl_Clock_Module_State_clockQ__O
-u ti_sysbios_knl_Task_defaultStackSize__C
-u xdc_runtime_IGateProvider_Interface__BASE__C
-u ti_sysbios_family_arm_m3_Hwi_E_hwiLimitExceeded__C
-u xdc_runtime_Startup_startModsFxn__C
-u ti_sysbios_knl_Semaphore_Instance_State_pendQ__O
-u ti_sysbios_family_arm_m3_Hwi_Object__DESC__C
-u xdc_runtime_Text_nameEmpty__C
-u ti_sysbios_family_arm_m3_Hwi_Object__count__C
-u xdc_runtime_SysCallback_abortFxn__C
-u ti_sysbios_knl_Task_defaultStackHeap__C
-u ti_sysbios_family_arm_m3_Hwi_ccr__C
-u ti_sysbios_knl_Mailbox_Object__PARAMS__C
-u ti_sysbios_hal_Hwi_Object__PARAMS__C
-u ti_sysbios_heaps_HeapMem_E_memory__C
-u ti_sysbios_knl_Task_Object__count__C
-u ti_sysbios_rom_ROM_AONRTCChannelEnable
-u ti_sysbios_heaps_HeapMem_Object__DESC__C
-u xdc_runtime_Text_nameUnknown__C
-u xdc_runtime_Memory_defaultHeapInstance__C
-u ti_sysbios_knl_Mailbox_Instance_State_freeQue__O
-u ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_setThreshold__I
-u xdc_runtime_Startup_sfxnTab__C
-u ti_sysbios_knl_Clock_Module__state__V
-u ti_sysbios_family_arm_cc26xx_TimestampProvider_Module__state__V
-u xdc_runtime_Startup_Module__state__V
-u ti_sysbios_BIOS_Module__state__V
-u ti_sysbios_knl_Swi_Module__state__V
-u ti_sysbios_knl_Task_Module__state__V
-u xdc_runtime_Memory_Module__state__V
-u xdc_runtime_System_Module__state__V
-u ti_sysbios_family_arm_m3_Hwi_Module__state__V
-u ti_sysbios_family_arm_cc26xx_Timer_Module__state__V

ti_sysbios_rom_cortexm_cc26xx_CC26xx_getRevision__E = 0x1001ca9b;
ti_sysbios_knl_Queue_get__E = 0x1001bf11;
ti_sysbios_knl_Swi_enabled__E = 0x1001c0e5;
ti_sysbios_knl_Clock_scheduleNextTick__E = 0x1001bebd;
ti_sysbios_knl_Swi_runLoop__I = 0x1001b0d5;
ti_sysbios_knl_Clock_getTicks__E = 0x1001b6d9;
ti_sysbios_gates_GateMutex_Object__destruct__S = 0x1001bc49;
ti_sysbios_knl_Queue_enqueue__E = 0x1001c1b5;
ti_sysbios_knl_Queue_put__E = 0x1001bf2d;
ti_sysbios_family_arm_m3_Hwi_Object__create__S = 0x1001aab1;
ti_sysbios_gates_GateHwi_Instance_init__E = 0x1001b463;
ti_sysbios_hal_Hwi_Instance_finalize__E = 0x1001c1f1;
ti_sysbios_BIOS_RtsGateProxy_leave__E = 0x1001c24d;
ti_sysbios_heaps_HeapMem_Object__create__S = 0x1001b845;
xdc_runtime_Error_raiseX__E = 0x1001c769;
ti_sysbios_knl_Semaphore_construct = 0x1001b709;
ti_sysbios_knl_Clock_Object__destruct__S = 0x1001bce9;
ti_sysbios_knl_Clock_TimerProxy_getMaxTicks__E = 0x1001c0a5;
ti_sysbios_knl_Swi_Object__destruct__S = 0x1001bf49;
ti_sysbios_family_arm_cc26xx_TimestampProvider_getFreq__E = 0x1001c12f;
ti_sysbios_gates_GateMutex_Handle__label__S = 0x1001bc29;
ti_sysbios_knl_Mailbox_delete = 0x1001c2c5;
ti_sysbios_knl_Semaphore_destruct = 0x1001c2e1;
ti_sysbios_BIOS_RtsGateProxy_enter__E = 0x1001c249;
ti_sysbios_knl_Task_processVitalTaskFlag__I = 0x1001b739;
ti_sysbios_knl_Mailbox_create = 0x1001abdd;
xdc_runtime_Core_deleteObject__I = 0x1001c4f9;
ti_sysbios_knl_Queue_delete = 0x1001c2cd;
ti_sysbios_family_arm_m3_Hwi_doSwiRestore__I = 0x1001c1fb;
xdc_runtime_System_atexit__E = 0x1001c681;
ti_sysbios_gates_GateMutex_Params__init__S = 0x1001c179;
ti_sysbios_knl_Clock_getTimerHandle__E = 0x1001c191;
ti_sysbios_knl_Task_enable__E = 0x1001c243;
ti_sysbios_knl_Clock_TimerProxy_getExpiredTicks__E = 0x1001c2ad;
ti_sysbios_knl_Queue_Object__destruct__S = 0x1001bef5;
ti_sysbios_knl_Clock_Object__delete__S = 0x1001badd;
ti_sysbios_gates_GateMutex_delete = 0x1001c26d;
ti_sysbios_heaps_HeapMem_restore__E = 0x1001c105;
ti_sysbios_knl_Swi_create = 0x1001ad05;
ti_sysbios_heaps_HeapMem_Module_GateProxy_leave__E = 0x1001c2a1;
ti_sysbios_knl_Semaphore_pend__E = 0x1001a0cd;
ti_sysbios_knl_Mailbox_Instance_finalize__E = 0x1001a9dd;
xdc_runtime_Startup_startMods__I = 0x1001c30d;
ti_sysbios_heaps_HeapMem_init__I = 0x1001b545;
ti_sysbios_knl_Swi_Object__delete__S = 0x1001bda9;
ti_sysbios_hal_Hwi_HwiProxy_enableInterrupt__E = 0x1001c285;
ti_sysbios_knl_Clock_removeI__E = 0x1001c2bd;
xdc_runtime_System_abort__E = 0x1001c749;
ti_sysbios_family_arm_m3_Hwi_dispatchC__I = 0x1001aa49;
ti_sysbios_knl_Swi_construct = 0x1001b085;
ti_sysbios_knl_Task_sleepTimeout__I = 0x1001bf81;
ti_sysbios_knl_Queue_remove__E = 0x1001c121;
ti_sysbios_knl_Semaphore_Instance_finalize__E = 0x1001c0c5;
ti_sysbios_gates_GateMutex_destruct = 0x1001c271;
ti_sysbios_knl_Task_SupportProxy_Module__startupDone__S = 0x1001c2f1;
ti_sysbios_knl_Queue_Object__delete__S = 0x1001bd49;
ti_sysbios_knl_Mailbox_Object__get__S = 0x1001b89d;
ti_sysbios_family_arm_m3_Hwi_Instance_init__E = 0x1001a3c9;
ti_sysbios_knl_Clock_delete = 0x1001c2b5;
ti_sysbios_knl_Clock_walkQueueDynamic__E = 0x1001a80d;
ti_sysbios_knl_Mailbox_Object__destruct__S = 0x1001bed9;
ti_sysbios_knl_Mailbox_post__E = 0x1001a591;
ti_sysbios_knl_Clock_Instance_init__E = 0x1001b50d;
ti_sysbios_knl_Task_allBlockedFunction__I = 0x1001b365;
ti_sysbios_knl_Task_postInit__I = 0x1001a615;
ti_sysbios_knl_Task_enter__I = 0x1001be09;
ti_sysbios_hal_Hwi_switchFromBootStack__E = 0x1001c299;
ti_sysbios_knl_Semaphore_Object__destruct__S = 0x1001bd89;
ti_sysbios_hal_Hwi_HwiProxy_Object__create__S = 0x1001c085;
ti_sysbios_family_arm_cc26xx_Timer_postInit__I = 0x1001b769;
ti_sysbios_knl_Swi_Module_startup__E = 0x1001c237;
ti_sysbios_gates_GateMutex_Instance_finalize__E = 0x1001c075;
xdc_runtime_Core_assignParams__I = 0x1001c5f1;
ti_sysbios_hal_Hwi_HwiProxy_switchFromBootStack__E = 0x1001c291;
ti_sysbios_knl_Swi_post__E = 0x1001b321;
ti_sysbios_hal_Hwi_initStack = 0x1001b819;
xdc_runtime_Memory_alloc__E = 0x1001c489;
ti_sysbios_knl_Queue_next__E = 0x1001c2d9;
ti_sysbios_knl_Clock_Instance_finalize__E = 0x1001bfcd;
ti_sysbios_knl_Queue_elemClear__E = 0x1001c231;
ti_sysbios_knl_Clock_Params__init__S = 0x1001c185;
ti_sysbios_knl_Task_Instance_init__E = 0x1001a25d;
ti_sysbios_hal_Hwi_HwiProxy_startup__E = 0x1001c28d;
ti_sysbios_knl_Task_self__E = 0x1001c1e5;
ti_sysbios_knl_Task_startup__E = 0x1001c309;
ti_sysbios_gates_GateHwi_Object__delete__S = 0x1001bc09;
ti_sysbios_family_arm_cc26xx_Timer_startup__E = 0x1001b795;
xdc_runtime_Memory_free__E = 0x1001c801;
ti_sysbios_hal_Hwi_delete = 0x1001c295;
ti_sysbios_knl_Queue_Instance_init__E = 0x1001c22b;
ti_sysbios_family_arm_cc26xx_Timer_Module_startup__E = 0x1001ba71;
xdc_runtime_Assert_raise__I = 0x1001c5a9;
ti_sysbios_hal_Hwi_create = 0x1001ab79;
ti_sysbios_knl_Task_destruct = 0x1001c305;
ti_sysbios_hal_Hwi_Module_startup__E = 0x1001c021;
ti_sysbios_family_arm_m3_Hwi_excHandler__I = 0x1001bbc9;
xdc_runtime_Core_destructObject__I = 0x1001c7a1;
ti_sysbios_knl_Swi_disable__E = 0x1001c0d5;
ti_sysbios_BIOS_setThreadType__E = 0x1001c045;
ti_sysbios_knl_Task_disable__E = 0x1001c0f5;
ti_sysbios_knl_Swi_Instance_init__E = 0x1001aca5;
ti_sysbios_knl_Semaphore_pendTimeout__I = 0x1001b94d;
ti_sysbios_knl_Clock_create = 0x1001b3ed;
ti_sysbios_knl_Idle_loop__E = 0x1001c20b;
ti_sysbios_gates_GateHwi_leave__E = 0x1001c21f;
ti_sysbios_family_arm_m3_Hwi_enableInterrupt__E = 0x1001b2dd;
ti_sysbios_knl_Semaphore_Params__init__S = 0x1001c1c1;
ti_sysbios_knl_Task_unblock__E = 0x1001bfb5;
ti_sysbios_knl_Swi_destruct = 0x1001c2e9;
ti_sysbios_BIOS_getCpuFreq__E = 0x1001bff9;
xdc_runtime_Memory_calloc__E = 0x1001c84d;
ti_sysbios_family_arm_m3_Hwi_startup__E = 0x1001c203;
xdc_runtime_SysCallback_exit__E = 0x1001c821;
ti_sysbios_knl_Queue_empty__E = 0x1001c113;
ti_sysbios_knl_Clock_logTick__E = 0x1001be67;
ti_sysbios_knl_Task_yield__E = 0x1001b3a9;
ti_sysbios_knl_Task_SupportProxy_getStackAlignment__E = 0x1001c2f5;
ti_sysbios_family_arm_m3_Hwi_create = 0x1001ab15;
xdc_runtime_Timestamp_SupportProxy_get32__E = 0x1001c831;
ti_sysbios_family_arm_m3_Hwi_destruct = 0x1001c261;
ti_sysbios_knl_Clock_stop__E = 0x1001c225;
ti_sysbios_family_arm_m3_Hwi_doTaskRestore__I = 0x1001c213;
ti_sysbios_knl_Swi_run__I = 0x1001afd9;
ti_sysbios_family_arm_cc26xx_Timer_Module__startupDone__S = 0x1001bb49;
xdc_runtime_Core_createObject__I = 0x1001c405;
ti_sysbios_knl_Queue_create = 0x1001b921;
ti_sysbios_hal_Hwi_Object__delete__S = 0x1001bc69;
ti_sysbios_knl_Clock_construct = 0x1001b579;
xdc_runtime_System_abortSpin__E = 0x1001c899;
ti_sysbios_family_arm_m3_Hwi_Object__destruct__S = 0x1001be69;
ti_sysbios_hal_Hwi_HwiProxy_delete = 0x1001c27d;
ti_sysbios_gates_GateMutex_Object__create__S = 0x1001b649;
ti_sysbios_family_arm_m3_Hwi_getStackInfo__E = 0x1001b465;
ti_sysbios_heaps_HeapMem_Module_GateProxy_enter__E = 0x1001c29d;
ti_sysbios_knl_Semaphore_post__E = 0x1001ac41;
ti_sysbios_knl_Task_exit__E = 0x1001b429;
ti_sysbios_heaps_HeapMem_Instance_init__E = 0x1001b4d5;
ti_sysbios_knl_Swi_restore__E = 0x1001b5e1;
ti_sysbios_knl_Task_startCore__E = 0x1001ae75;
ti_sysbios_knl_Semaphore_create = 0x1001b5ad;
ti_sysbios_gates_GateHwi_enter__E = 0x1001c16d;
ti_sysbios_knl_Task_blockI__E = 0x1001b615;
ti_sysbios_heaps_HeapMem_free__E = 0x1001a795;
ti_sysbios_knl_Task_Object__destruct__S = 0x1001bf65;
ti_sysbios_family_arm_cc26xx_Timer_periodicStub__E = 0x1001a469;
ti_sysbios_hal_Hwi_Instance_init__E = 0x1001b9f9;
ti_sysbios_gates_GateHwi_query__E = 0x1001c269;
xdc_runtime_System_processAtExit__E = 0x1001c6b9;
ti_sysbios_family_arm_cc26xx_Timer_setPeriod__E = 0x1001c259;
xdc_runtime_Error_init__E = 0x1001c841;
ti_sysbios_knl_Semaphore_Instance_init__E = 0x1001bb01;
ti_sysbios_knl_Queue_head__E = 0x1001c2d5;
xdc_runtime_Error_check__E = 0x1001c7b9;
xdc_runtime_Error_policySpin__E = 0x1001c487;
ti_sysbios_gates_GateMutex_create = 0x1001b679;
xdc_runtime_Gate_leaveSystem__E = 0x1001c879;
ti_sysbios_knl_Swi_restoreHwi__E = 0x1001ad61;
ti_sysbios_knl_Task_sleep__E = 0x1001a96d;
ti_sysbios_knl_Task_create = 0x1001ae19;
ti_sysbios_knl_Mailbox_Params__init__S = 0x1001c19d;
ti_sysbios_knl_Task_restoreHwi__E = 0x1001be29;
ti_sysbios_knl_Mailbox_postInit__I = 0x1001b8c9;
ti_sysbios_knl_Task_delete = 0x1001c301;
ti_sysbios_heaps_HeapMem_isBlocking__E = 0x1001c2a5;
ti_sysbios_knl_Clock_startI__E = 0x1001a719;
ti_sysbios_knl_Clock_start__E = 0x1001bfe3;
ti_sysbios_family_arm_m3_Hwi_Object__delete__S = 0x1001bba9;
ti_sysbios_knl_Clock_TimerProxy_getPeriod__E = 0x1001c2b1;
ti_sysbios_knl_Task_SupportProxy_start__E = 0x1001c2f9;
ti_sysbios_heaps_HeapMem_Handle__label__S = 0x1001bc89;
ti_sysbios_family_arm_m3_Hwi_delete = 0x1001c25d;
ti_sysbios_knl_Semaphore_Object__delete__S = 0x1001bb25;
ti_sysbios_hal_Hwi_HwiProxy_getStackInfo__E = 0x1001c289;
ti_sysbios_knl_Idle_run__E = 0x1001bd09;
ti_sysbios_knl_Swi_delete = 0x1001c2e5;
xdc_runtime_Memory_valloc__E = 0x1001c785;
ti_sysbios_knl_Mailbox_Object__delete__S = 0x1001bd29;
ti_sysbios_family_arm_m3_TaskSupport_start__E = 0x1001b49d;
ti_sysbios_family_arm_m3_Hwi_Module__startupDone__S = 0x1001bb89;
ti_sysbios_knl_Swi_startup__E = 0x1001c23d;
ti_sysbios_knl_Task_schedule__I = 0x1001b175;
ti_sysbios_gates_GateMutex_leave__E = 0x1001bf9d;
ti_sysbios_heaps_HeapMem_Object__delete__S = 0x1001bca9;
ti_sysbios_knl_Clock_TimerProxy_setNextTick__E = 0x1001c0b5;
ti_sysbios_knl_Swi_Object__get__S = 0x1001b979;
ti_sysbios_knl_Task_restore__E = 0x1001ba49;
xdc_runtime_Memory_HeapProxy_alloc__E = 0x1001c889;
ti_sysbios_gates_GateHwi_Object__create__S = 0x1001ba95;
ti_sysbios_hal_Hwi_HwiProxy_disableInterrupt__E = 0x1001c281;
ti_sysbios_BIOS_start__E = 0x1001c055;
ti_sysbios_BIOS_exit__E = 0x1001c035;
ti_sysbios_family_arm_m3_TaskSupport_getStackAlignment__E = 0x1001c161;
xdc_runtime_SysCallback_abort__E = 0x1001c811;
ti_sysbios_knl_Queue_destruct = 0x1001c2d1;
ti_sysbios_family_arm_m3_Hwi_postInit__I = 0x1001a319;
ti_sysbios_gates_GateMutex_Instance_init__E = 0x1001bea1;
ti_sysbios_knl_Task_Instance_finalize__E = 0x1001a4fd;
ti_sysbios_knl_Clock_TimerProxy_getCurrentTick__E = 0x1001c095;
ti_sysbios_family_arm_m3_Hwi_disableFxn__E = 0x1001c149;
xdc_runtime_Memory_HeapProxy_free__E = 0x1001c88d;
ti_sysbios_knl_Mailbox_Module_startup__E = 0x1001b251;
ti_sysbios_knl_Task_Object__delete__S = 0x1001bdc9;
ti_sysbios_gates_GateHwi_Handle__label__S = 0x1001bbe9;
xdc_runtime_Text_ropeText__E = 0x1001c7e9;
ti_sysbios_knl_Clock_destruct = 0x1001c2b9;
ti_sysbios_knl_Queue_construct = 0x1001b8f5;
ti_sysbios_family_arm_m3_Hwi_switchFromBootStack__E = 0x1001c00d;
ti_sysbios_heaps_HeapMem_Object__get__S = 0x1001b871;
ti_sysbios_hal_Hwi_HwiProxy_create = 0x1001be49;
ti_sysbios_gates_GateMutex_query__E = 0x1001c275;
ti_sysbios_knl_Swi_schedule__I = 0x1001b031;
ti_sysbios_knl_Task_Params__init__S = 0x1001c1d9;
ti_sysbios_family_arm_m3_Hwi_Params__init__S = 0x1001c13d;
ti_sysbios_family_arm_m3_Hwi_plug__E = 0x1001c065;
xdc_runtime_System_exitSpin__E = 0x1001c89b;
ti_sysbios_gates_GateMutex_construct = 0x1001b7ed;
xdc_runtime_System_Module_GateProxy_leave__E = 0x1001c895;
ti_sysbios_knl_Mailbox_pend__E = 0x1001a8fd;
ti_sysbios_family_arm_m3_TaskSupport_Module__startupDone__S = 0x1001c265;
xdc_runtime_Core_assignLabel__I = 0x1001c6ed;
xdc_runtime_System_Module_GateProxy_enter__E = 0x1001c891;
xdc_runtime_System_exit__E = 0x1001c7d1;
ti_sysbios_knl_Swi_Params__init__S = 0x1001c1cd;
ti_sysbios_knl_Clock_workFunc__E = 0x1001a885;
ti_sysbios_family_arm_m3_Hwi_restoreFxn__E = 0x1001c219;
ti_sysbios_family_arm_cc26xx_TimestampProvider_Module_startup__E = 0x1001bb69;
ti_sysbios_knl_Semaphore_delete = 0x1001c2dd;
ti_sysbios_family_arm_cc26xx_Timer_getPeriod__E = 0x1001c255;
ti_sysbios_family_arm_m3_Hwi_initNVIC__E = 0x1001a699;
ti_sysbios_knl_Clock_addI__E = 0x1001ba21;
ti_sysbios_family_arm_m3_Hwi_Instance_finalize__E = 0x1001aed1;
ti_sysbios_heaps_HeapMem_alloc__E = 0x1001a199;
ti_sysbios_knl_Task_unblockI__E = 0x1001b9a5;
ti_sysbios_knl_Swi_Instance_finalize__E = 0x1001bf9b;
ti_sysbios_family_arm_m3_Hwi_disableInterrupt__E = 0x1001b299;
ti_sysbios_family_arm_m3_Hwi_enableFxn__E = 0x1001c155;
xdc_runtime_Gate_enterSystem__E = 0x1001c885;
ti_sysbios_gates_GateMutex_Object__delete__S = 0x1001bab9;
ti_sysbios_family_arm_cc26xx_TimestampProvider_get64__E = 0x1001b9d1;
ti_sysbios_knl_Mailbox_Instance_init__E = 0x1001a001;
xdc_runtime_Text_cordText__E = 0x1001c71d;
xdc_runtime_Startup_exec__E = 0x1001c559;
ti_sysbios_hal_Hwi_HwiProxy_Module__startupDone__S = 0x1001c279;
ti_sysbios_heaps_HeapMem_getStats__E = 0x1001b209;
ti_sysbios_knl_Task_SupportProxy_swap__E = 0x1001c2fd;
xdc_runtime_Memory_getMaxDefaultTypeAlign__E = 0x1001c859;
ti_sysbios_knl_Task_Object__get__S = 0x1001bde9;
ti_sysbios_family_arm_m3_Hwi_construct = 0x1001af29;
ti_sysbios_knl_Clock_TimerProxy_Module__startupDone__S = 0x1001c2a9;
ti_sysbios_knl_Clock_Module_startup__E = 0x1001bcc9;
ti_sysbios_knl_Mailbox_construct = 0x1001af81;
ti_sysbios_knl_Task_construct = 0x1001b125;
xdc_runtime_Core_constructObject__I = 0x1001c639;
ti_sysbios_knl_Queue_dequeue__E = 0x1001c1a9;
ti_sysbios_knl_Task_Module_startup__E = 0x1001adbd;
ti_sysbios_family_arm_cc26xx_Timer_getExpiredTicks__E = 0x1001c251;
ti_sysbios_family_arm_m3_Hwi_Object__get__S = 0x1001b7c1;
ti_sysbios_knl_Mailbox_destruct = 0x1001c2c9;
xdc_runtime_System_Module_startup__E = 0x1001c87f;
ti_sysbios_knl_Swi_postInit__I = 0x1001c2ed;
ti_sysbios_family_arm_m3_Hwi_Module_startup__E = 0x1001b1c1;
ti_sysbios_gates_GateMutex_enter__E = 0x1001b6a9;
ti_sysbios_family_arm_m3_Hwi_setPriority__E = 0x1001be85;
ti_sysbios_knl_Queue_Object__get__S = 0x1001bd69;
ti_sysbios_knl_Clock_setTimeout__E = 0x1001c2c1;
ti_sysbios_knl_Task_swapReturn = 0x1001ca71;
ti_sysbios_family_arm_m3_TaskSupport_glue = 0x1001ca59;
ti_sysbios_family_arm_m3_TaskSupport_buildTaskStack = 0x1001c96d;
ti_sysbios_family_arm_m3_TaskSupport_swap__E = 0x1001ca69;
ti_sysbios_family_arm_m3_Hwi_excHandlerAsm__I = 0x1001ca01;
ti_sysbios_family_arm_m3_Hwi_return = 0x1001ca57;
ti_sysbios_family_arm_m3_Hwi_pendSV__I = 0x1001ca3f;
ti_sysbios_family_arm_m3_Hwi_dispatch__I = 0x1001c89d;
ti_sysbios_family_xxx_Hwi_switchAndRunFunc = 0x1001ca21;
ti_sysbios_family_arm_m3_Hwi_initStacks__E = 0x1001c9b9;
ti_sysbios_BIOS_RtsGateProxy_Object__delete__S = 0x1001bab9;
ti_sysbios_BIOS_RtsGateProxy_Params__init__S = 0x1001c179;
ti_sysbios_BIOS_RtsGateProxy_Handle__label__S = 0x1001bc29;
ti_sysbios_BIOS_RtsGateProxy_query__E = 0x1001c275;
ti_sysbios_knl_Clock_TimerProxy_startup__E = 0x1001b795;
ti_sysbios_hal_Hwi_disableInterrupt__E = 0x1001c281;
ti_sysbios_hal_Hwi_enableInterrupt__E = 0x1001c285;
ti_sysbios_hal_Hwi_getStackInfo__E = 0x1001c289;
ti_sysbios_hal_Hwi_startup__E = 0x1001c28d;
ti_sysbios_hal_Hwi_HwiProxy_Object__delete__S = 0x1001bba9;
ti_sysbios_hal_Hwi_HwiProxy_Params__init__S = 0x1001c13d;
ti_sysbios_heaps_HeapMem_Module_GateProxy_query__E = 0x1001c275;
ti_sysbios_heaps_HeapMem_Module_GateProxy_Object__delete__S = 0x1001bab9;
ti_sysbios_heaps_HeapMem_Module_GateProxy_Params__init__S = 0x1001c179;
ti_sysbios_heaps_HeapMem_Module_GateProxy_Handle__label__S = 0x1001bc29;
xdc_runtime_Timestamp_SupportProxy_get64__E = 0x1001b9d1;
xdc_runtime_Timestamp_SupportProxy_getFreq__E = 0x1001c12f;
xdc_runtime_Timestamp_get32__E = 0x1001c831;
xdc_runtime_Timestamp_get64__E = 0x1001b9d1;
xdc_runtime_Timestamp_getFreq__E = 0x1001c12f;
xdc_runtime_Memory_HeapProxy_Object__delete__S = 0x1001bca9;
xdc_runtime_Memory_HeapProxy_Handle__label__S = 0x1001bc89;
xdc_runtime_System_Module_GateProxy_Object__delete__S = 0x1001bc09;
xdc_runtime_System_Module_GateProxy_Handle__label__S = 0x1001bbe9;
xdc_runtime_System_Module_GateProxy_query__E = 0x1001c269;

SECTIONS
{
    .const:xdc_runtime_Error_policy__C: LOAD > 0x0000058c
    .const:xdc_runtime_IModule_Interface__BASE__C: LOAD > 0x00000538
    .const:xdc_runtime_Startup_lastFxns__C: LOAD > 0x0000051c
    .const:ti_sysbios_gates_GateMutex_Object__DESC__C: LOAD > 0x00000310
    .const:ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_initDevice__I: LOAD > 0x000005ac
    .const:xdc_runtime_Startup_execImpl__C: LOAD > 0x0000054c
    .const:ti_sysbios_gates_GateMutex_Instance_State_sem__O: LOAD > 0x000005cc
    .const:ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_getMaxTicks__E: LOAD > 0x000005a8
    .const:ti_sysbios_knl_Swi_Object__count__C: LOAD > 0x000004dc
    .const:ti_sysbios_knl_Idle_funcList__C: LOAD > 0x00000478
    .const:ti_sysbios_family_arm_m3_Hwi_Object__PARAMS__C: LOAD > 0x00000170
    .const:xdc_runtime_Text_isLoaded__C: LOAD > 0x0000058a
    .const:ti_sysbios_knl_Clock_Object__DESC__C: LOAD > 0x00000370
    .const:ti_sysbios_knl_Mailbox_Instance_State_dataQue__O: LOAD > 0x000005d4
    .const:ti_sysbios_gates_GateMutex_Module__FXNS__C: LOAD > 0x00000240
    .const:ti_sysbios_knl_Task_Module_State_inactiveQ__O: LOAD > 0x000005e8
    .const:ti_sysbios_family_arm_m3_Hwi_Module__id__C: LOAD > 0x00000506
    .const:ti_sysbios_family_arm_cc26xx_Timer_Module__id__C: LOAD > 0x00000502
    .const:ti_sysbios_knl_Mailbox_Object__table__C: LOAD > 0x000004d0
    .const:ti_sysbios_family_arm_m3_Hwi_Object__table__C: LOAD > 0x00000498
    .const:ti_sysbios_knl_Swi_Object__DESC__C: LOAD > 0x000003f0
    .const:xdc_runtime_Text_charCnt__C: LOAD > 0x00000588
    .const:ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_start__E: LOAD > 0x000005b8
    .const:ti_sysbios_heaps_HeapMem_Object__table__C: LOAD > 0x000004c0
    .const:xdc_runtime_Error_policyFxn__C: LOAD > 0x0000052c
    .const:ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_getCount64__E: LOAD > 0x000005a0
    .const:xdc_runtime_Startup_firstFxns__C: LOAD > 0x00000514
    .const:ti_sysbios_knl_Swi_Object__PARAMS__C: LOAD > 0x000001f4
    .const:ti_sysbios_knl_Clock_serviceMargin__C: LOAD > 0x000004c8
    .const:xdc_runtime_Text_charTab__C: LOAD > 0x00000574
    .const:ti_sysbios_rom_ROM_AONRTCCurrentCompareValueGet: LOAD > 0x00000598
    .const:ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_TimestampProvider_get32__E: LOAD > 0x000005bc
    .const:ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_getCurrentTick__E: LOAD > 0x000005a4
    .const:ti_sysbios_family_arm_m3_TaskSupport_stackAlignment__C: LOAD > 0x000004b0
    .const:ti_sysbios_family_arm_m3_Hwi_NUM_INTERRUPTS__C: LOAD > 0x00000490
    .const:xdc_runtime_Main_Module__diagsMask__C: LOAD > 0x00000544
    .const:ti_sysbios_knl_Swi_Object__table__C: LOAD > 0x000004e0
    .const:xdc_runtime_Memory_Module__id__C: LOAD > 0x00000586
    .const:ti_sysbios_knl_Task_Object__PARAMS__C: LOAD > 0x00000100
    .const:ti_sysbios_gates_GateMutex_Object__PARAMS__C: LOAD > 0x00000448
    .const:ti_sysbios_heaps_HeapMem_Module__gateObj__C: LOAD > 0x000004b8
    .const:ti_sysbios_family_arm_cc26xx_Timer_startupNeeded__C: LOAD > 0x00000484
    .const:ti_sysbios_knl_Queue_Object__DESC__C: LOAD > 0x000003b0
    .const:ti_sysbios_knl_Task_Object__DESC__C: LOAD > 0x00000410
    .const:xdc_runtime_Assert_E_assertFailed__C: LOAD > 0x00000524
    .const:ti_sysbios_heaps_HeapMem_Object__PARAMS__C: LOAD > 0x00000264
    .const:ti_sysbios_gates_GateHwi_Module__id__C: LOAD > 0x00000508
    .const:ti_sysbios_gates_GateHwi_Object__PARAMS__C: LOAD > 0x00000430
    .const:xdc_runtime_IHeap_Interface__BASE__C: LOAD > 0x00000534
    .const:xdc_runtime_SysCallback_exitFxn__C: LOAD > 0x00000564
    .const:ti_sysbios_heaps_HeapMem_Module__id__C: LOAD > 0x0000050c
    .const:ti_sysbios_family_arm_m3_Hwi_excHandlerFunc__C: LOAD > 0x000004a0
    .const:ti_sysbios_heaps_HeapMem_Module__FXNS__C: LOAD > 0x000001cc
    .const:xdc_runtime_System_maxAtexitHandlers__C: LOAD > 0x00000570
    .const:ti_sysbios_knl_Queue_Object__count__C: LOAD > 0x000004d4
    .const:ti_sysbios_knl_Task_Object__table__C: LOAD > 0x000004ec
    .const:ti_sysbios_knl_Mailbox_Object__DESC__C: LOAD > 0x00000390
    .const:ti_sysbios_family_arm_m3_Hwi_nullIsrFunc__C: LOAD > 0x000004a4
    .const:ti_sysbios_knl_Clock_tickMode__C: LOAD > 0x00000510
    .const:ti_sysbios_gates_GateMutex_Module__id__C: LOAD > 0x0000050a
    .const:ti_sysbios_knl_Swi_numPriorities__C: LOAD > 0x000004e4
    .const:ti_sysbios_knl_Task_numConstructedTasks__C: LOAD > 0x000004fc
    .const:xdc_runtime_Startup_maxPasses__C: LOAD > 0x00000550
    .const:ti_sysbios_rom_ROM_AONRTCEventClear: LOAD > 0x0000059c
    .const:ti_sysbios_knl_Task_initStackFlag__C: LOAD > 0x0000050e
    .const:xdc_runtime_Main_Module__diagsEnabled__C: LOAD > 0x0000053c
    .const:xdc_runtime_Main_Module__diagsIncluded__C: LOAD > 0x00000540
    .const:xdc_runtime_System_abortFxn__C: LOAD > 0x00000568
    .const:ti_sysbios_knl_Mailbox_Instance_State_dataSem__O: LOAD > 0x000005d8
    .const:ti_sysbios_gates_GateHwi_Module__FXNS__C: LOAD > 0x0000021c
    .const:ti_sysbios_hal_Hwi_Object__DESC__C: LOAD > 0x00000330
    .const:ti_sysbios_family_arm_m3_Hwi_priGroup__C: LOAD > 0x000004ac
    .const:xdc_runtime_Error_E_memory__C: LOAD > 0x00000528
    .const:ti_sysbios_family_arm_m3_Hwi_E_alreadyDefined__C: LOAD > 0x00000488
    .const:ti_sysbios_knl_Mailbox_Instance_State_freeSem__O: LOAD > 0x000005e0
    .const:ti_sysbios_knl_Queue_Object__table__C: LOAD > 0x000004d8
    .const:ti_sysbios_knl_Semaphore_Object__PARAMS__C: LOAD > 0x000002ac
    .const:xdc_runtime_System_exitFxn__C: LOAD > 0x0000056c
    .const:ti_sysbios_knl_Clock_Object__PARAMS__C: LOAD > 0x00000288
    .const:ti_sysbios_rom_ROM_AONRTCCompareValueSet: LOAD > 0x00000594
    .const:ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_setNextTick__E: LOAD > 0x000005b0
    .const:ti_sysbios_heaps_HeapMem_reqAlign__C: LOAD > 0x000004c4
    .const:xdc_runtime_Main_Module__id__C: LOAD > 0x00000584
    .const:xdc_runtime_Startup_sfxnRts__C: LOAD > 0x00000554
    .const:ti_sysbios_knl_Semaphore_Object__DESC__C: LOAD > 0x000003d0
    .const:ti_sysbios_gates_GateHwi_Object__DESC__C: LOAD > 0x000002f0
    .const:ti_sysbios_heaps_HeapMem_Object__count__C: LOAD > 0x000004bc
    .const:ti_sysbios_family_arm_m3_Hwi_numSparseInterrupts__C: LOAD > 0x000004a8
    .const:ti_sysbios_family_arm_cc26xx_TimestampProvider_useClockTimer__C: LOAD > 0x00000504
    .const:ti_sysbios_rom_ROM_xdc_runtime_System_SupportProxy_exit__E: LOAD > 0x000005c8
    .const:ti_sysbios_knl_Queue_Object__PARAMS__C: LOAD > 0x00000460
    .const:ti_sysbios_knl_Task_allBlockedFunc__C: LOAD > 0x000004f0
    .const:ti_sysbios_rom_ROM_xdc_runtime_System_SupportProxy_abort__E: LOAD > 0x000005c4
    .const:ti_sysbios_knl_Mailbox_Object__count__C: LOAD > 0x000004cc
    .const:xdc_runtime_Text_nameStatic__C: LOAD > 0x0000057c
    .const:ti_sysbios_rom_ROM_xdc_runtime_Startup_getState__I: LOAD > 0x000005c0
    .const:ti_sysbios_knl_Clock_Module_State_clockQ__O: LOAD > 0x000005d0
    .const:ti_sysbios_knl_Task_defaultStackSize__C: LOAD > 0x000004f8
    .const:xdc_runtime_IGateProvider_Interface__BASE__C: LOAD > 0x00000530
    .const:ti_sysbios_family_arm_m3_Hwi_E_hwiLimitExceeded__C: LOAD > 0x0000048c
    .const:xdc_runtime_Startup_startModsFxn__C: LOAD > 0x0000055c
    .const:ti_sysbios_knl_Semaphore_Instance_State_pendQ__O: LOAD > 0x000005e4
    .const:ti_sysbios_family_arm_m3_Hwi_Object__DESC__C: LOAD > 0x000002d0
    .const:xdc_runtime_Text_nameEmpty__C: LOAD > 0x00000578
    .const:ti_sysbios_family_arm_m3_Hwi_Object__count__C: LOAD > 0x00000494
    .const:xdc_runtime_SysCallback_abortFxn__C: LOAD > 0x00000560
    .const:ti_sysbios_knl_Task_defaultStackHeap__C: LOAD > 0x000004f4
    .const:ti_sysbios_family_arm_m3_Hwi_ccr__C: LOAD > 0x0000049c
    .const:ti_sysbios_knl_Mailbox_Object__PARAMS__C: LOAD > 0x0000013c
    .const:ti_sysbios_hal_Hwi_Object__PARAMS__C: LOAD > 0x000001a0
    .const:ti_sysbios_heaps_HeapMem_E_memory__C: LOAD > 0x000004b4
    .const:ti_sysbios_knl_Task_Object__count__C: LOAD > 0x000004e8
    .const:ti_sysbios_rom_ROM_AONRTCChannelEnable: LOAD > 0x00000590
    .const:ti_sysbios_heaps_HeapMem_Object__DESC__C: LOAD > 0x00000350
    .const:xdc_runtime_Text_nameUnknown__C: LOAD > 0x00000580
    .const:xdc_runtime_Memory_defaultHeapInstance__C: LOAD > 0x00000548
    .const:ti_sysbios_knl_Mailbox_Instance_State_freeQue__O: LOAD > 0x000005dc
    .const:ti_sysbios_rom_ROM_ti_sysbios_family_arm_cc26xx_Timer_setThreshold__I: LOAD > 0x000005b4
    .const:xdc_runtime_Startup_sfxnTab__C: LOAD > 0x00000558
    .data:ti_sysbios_knl_Clock_Module__state__V: LOAD > 0x20000178
    .data:ti_sysbios_family_arm_cc26xx_TimestampProvider_Module__state__V: LOAD > 0x200001ec
    .data:xdc_runtime_Startup_Module__state__V: LOAD > 0x200001f0
    .data:ti_sysbios_BIOS_Module__state__V: LOAD > 0x200001a4
    .data:ti_sysbios_knl_Swi_Module__state__V: LOAD > 0x200001c8
    .data:ti_sysbios_knl_Task_Module__state__V: LOAD > 0x20000100
    .data:xdc_runtime_Memory_Module__state__V: LOAD > 0x20000200
    .data:xdc_runtime_System_Module__state__V: LOAD > 0x200001f8
    .data:ti_sysbios_family_arm_m3_Hwi_Module__state__V: LOAD > 0x20000144
    .data:ti_sysbios_family_arm_cc26xx_Timer_Module__state__V: LOAD > 0x200001e4
}

/* Content from ti.sysbios.xdcruntime (null): */

/* Content from ti.sysbios.utils (null): */

/* Content from configPkg (null): */

/* Content from xdc.services.io (null): */



/*
 * symbolic aliases for static instance objects
 */
xdc_runtime_Startup__EXECFXN__C = 1;
xdc_runtime_Startup__RESETFXN__C = 1;
TSK_idle = ti_sysbios_knl_Task_Object__table__V + 0;


SECTIONS
{
    .bootVecs:  type = DSECT
    .vecs: load > 0x20000000
    .resetVecs: load > 0x0



    xdc.meta: type = COPY
    xdc.noload: type = COPY
}
