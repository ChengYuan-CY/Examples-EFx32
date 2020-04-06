
Please note that the LZMA compression is not supported on EFR32xG13/xGM13x devices. It's documented in the UG266, the bootloader sizes are listed there, and LZMA always goes over the 16KB limit.
The option for EFR32xG13/xGM13x is to use LZ4 instead as that fits into the 16KB space.

Compressed GBL files can only be decompressed by the bootloader when running standalone, not through the Application Interface. This means that upgrade image verification performed by the application prior to reboot will not attempt to decompress the application upgrade, it will only verify the signature of the compressed payload. After rebooting into the bootloader, it will decompress the image as part of the upgrade process.
The above means that Bluetooth in-place application upgrades cannot be compressed, as they are processed by the Bluetooth Supervisor or AppLoader using functionality in the bootloader through the Application Interface. Supervisor/stack and AppLoader updates can be compressed, but the user application cannot.

So if you use the standard Bluetooth OTA DFU, then you cannot use lzma compression. This is because the GBL files are parsed by the apploader on-the-fly (during the upload process), and lzma could be decompressed only offline (after the whole image is uploaded), see section 6.2 in UG266: 
https://www.silabs.com/documents/public/user-guides/ug266-gecko-bootloader-user-guide.pdf

If you need lzma compression, you can use an application level uploader, which uploads the whole image first, then uses the bootloader to decompress and load it:  https://www.silabs.com/community/wireless/bluetooth/knowledge-base.entry.html/2017/09/20/uploading_imagesto-DXxD
