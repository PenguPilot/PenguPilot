from dev.cdc import CDC_nRF


myDev=CDC_nRF('/dev/ttyACM0')


print myDev.enterBootloader()
