# import asyncio
# import sys
# from bleak import BleakClient

# async def main(address):
#   async with BleakClient(address) as client:
#     if (not client.is_connected):
#       raise "client not connected"

#     services = await client.get_services()

#     for service in services:
#       print('\nservice', service.handle, service.uuid, service.description)

#       characteristics = service.characteristics

#       for char in characteristics:
#         print('  characteristic', char.handle, char.uuid, char.description, char.properties)

#         descriptors = char.descriptors

#         for desc in descriptors:
#           print('    descriptor', desc)

# if __name__ == "__main__":
#     print(sys.argv)
#     address = sys.argv[1]
#     # address = 'Arduino'

#     print('address:', address)
#     loop = asyncio.get_event_loop()
#     loop.run_until_complete(main(address))



from bluepy.btle import Scanner, DefaultDelegate, Peripheral



arduinoAddress = '58:BF:25:3B:32:E2'
arduinoDeviceName = 'ArduinoBLE'
arduinoServiceId = '0000000-0000-0000-0000-000000111111'
arduinoCharsId1 = '00000000-0000-0000-0000-000000000001'
arduinoCharsId2 = '00000000-0000-0000-0000-000000000100'
arduinoCharsId3 = '00000000-0000-0000-0000-000000010000'
arduinoCharsIdList = [arduinoCharsId1, arduinoCharsId2, arduinoCharsId3]


class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        # if isNewDev:
        #     print("Discovered device", dev.addr)
        # elif isNewData:
        #     print("Received new data from", dev.addr)
        pass

scanner = Scanner().withDelegate(ScanDelegate())
devices = scanner.scan(10.0)



for dev in devices:
    if str(dev.addr).strip().upper() == '58:BF:25:3B:32:E2':
        perip = Peripheral(dev.addr)
        print("Device %s (%s), RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi))
        for (adtype, desc, value) in dev.getScanData():

            print("  %s = %s" % (desc, value))
        for char in perip.getCharacteristics():
            if char.uuid in arduinoCharsIdList:
                print("Characteristice : ", char.uuid)
            
    # else:
    #     print("Device %s (%s), RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi))



