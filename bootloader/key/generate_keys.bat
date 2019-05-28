@rem see: http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.2.0%2Flib_bootloader_dfu_keys.html

@echo Make sure you run this script from Anaconda 2 prompt, so the correct nrfutil is used

@rem Generate a private key, and create a c-file public key
@nrfutil keys generate private_key.pem
@nrfutil keys display --key pk --format code private_key.pem --out_file public_key.c