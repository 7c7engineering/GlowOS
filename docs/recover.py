import esptool

# try in a loop to get ESP into bootloader mode and then erase flash
while True:
    try:
        esptool.main([
            '--chip', 'esp32-c3',
            '--port', '/dev/tty.usbmodem1101',
            '--baud', '115200',
            'erase-flash'
        ])
        print("Flash erased successfully!")
        break
    except Exception as e:
        # if exception is CTRLC, exit the loop
        if isinstance(e, KeyboardInterrupt):
            print("Exiting...")
            break
        print("retrying...")