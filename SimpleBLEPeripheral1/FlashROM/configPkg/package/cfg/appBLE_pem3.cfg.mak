# invoke SourceDir generated makefile for appBLE.pem3
appBLE.pem3: .libraries,appBLE.pem3
.libraries,appBLE.pem3: package/cfg/appBLE_pem3.xdl
	$(MAKE) -f C:\ti\simplelink\ble_cc26xx_2_01_01_44627\Projects\ble\SimpleBLEPeripheral\CC26xx\CCS\Config/src/makefile.libs

clean::
	$(MAKE) -f C:\ti\simplelink\ble_cc26xx_2_01_01_44627\Projects\ble\SimpleBLEPeripheral\CC26xx\CCS\Config/src/makefile.libs clean

