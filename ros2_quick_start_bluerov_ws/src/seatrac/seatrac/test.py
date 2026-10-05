import time
from seatrac import Seatrac
from loggingSystem import *

if __name__ == '__main__':
    now = time.time()
    ago = now

    x150 = Seatrac('/dev/ttyUSB0')

    x150.cidStatusCfgSet()
    x150.cidSettingsSet()

    log = LoggingSystem()

    while True:
        data = x150.update()
        log.addLine(data)

        now = time.time()
        if now - ago > 2.0:
            x150.cidPingSend(1,Seatrac.MSG_REQU)
            ago = now
