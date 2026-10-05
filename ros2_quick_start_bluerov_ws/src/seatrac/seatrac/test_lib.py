from seatracLib import *
import time

x150 = Seatrac('/dev/ttyUSB0',115200)

def cidSettingsGetCbk(settings):
    settings.setStatusOutputRate(0.0)
    settings.resetXcvrFlags()
    settings.enableUseAhrsForUsbl(False)
    settings.enableFixMsgs(True)
    #settings.enableUsblMsgs(True)
    print(settings)
    x150.cidSettingsSet(settings)

def cidSettingsSetCbk(result):
    if result == 0x0: #Seatrac.CST_OK:
        print('settings sent correctly')
    else:
        print('settings bad ' + hex(result))

if __name__ == '__main__':
    x150.setMsgCbk(Seatrac.CID_SETTINGS_GET,cidSettingsGetCbk)
    x150.setMsgCbk(Seatrac.CID_SETTINGS_SET,cidSettingsSetCbk)
    #
    x150.cidSettingsGet()
    now = time.time()
    ago = now

    while True:
        now = time.time()
        x150.update()
        if now - ago > 2.0:
            x150.cidPingSend(1,Seatrac.MSG_REQU)
            ago = now
            

