from time import sleep
from SX127x.LoRa import *
from SX127x.board_config import BOARD

BOARD.setup()

class LoRaRcvCont(LoRa):
    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)

    def start(self):
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        i=0
        while True:
            i=i+1
            print("\n=====cek terima data=====" ,i)
            sleep(5)
            rssi_value = self.get_rssi_value()
            status = self.get_modem_status()
            sys.stdout.flush()
            

    def on_rx_done(self):
        print("\nReceived: ")
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)
        #print(payload)
        print(bytes(payload).decode("utf-8",'ignore'))
        #self.set_mode(MODE.SLEEP)
        print("Data received, now send ACK.")
        #self.write_payload([0x0f, 65, 67, 75])
        rssi_value = self.get_rssi_value()
        print("RSSI=")
        print(rssi_value)
        self.set_mode(MODE.TX)
        print("done")
        sleep(5)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT) 

lora = LoRaRcvCont(verbose=True)
lora.set_mode(MODE.STDBY)
lora.set_freq(433.0)
print (lora.get_version())
print (lora.get_freq())

#  Medium Range  Defaults after init are 434.0MHz, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on 13 dBm

lora.set_pa_config(pa_select=1)

try:
    lora.start()
except KeyboardInterrupt:
    sys.stdout.flush()
    print("")
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    print("")
    lora.set_mode(MODE.SLEEP)
    BOARD.teardown()
