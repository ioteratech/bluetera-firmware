nrfutil settings generate --family NRF52840 --application _build/bluetera.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 _build/bootloader_settings.hex
nrfjprog --program _build/bluetera.hex --sectorerase
nrfjprog --program _build/bootloader_settings.hex --sectoranduicrerase
nrfjprog -r