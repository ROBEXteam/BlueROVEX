#-*- coding: utf-8 -*-
from serial import Serial
import struct
import time
import numpy as np

class SeatracStruct(object):
    """ """
    keys = []
    fmt = ""

    def __setitem__(self,key,value):
        self.dict[key] = value

    def __getitem__(self,key):
        return self.dict[key]

    def __str__(self):
        s = ''
        for key in self.keys:
            s += key + ': ' + str(self.dict[key]) + '\n'
        s += '------------'
        return s #tr(self.dict)

    def __init__(self,data = None,timestamp = None):
        self.dict = {}
        self.timestamp = timestamp
        if data is not None:
            self.unpack(data)

    def unpack(self,data):
        tpl = struct.unpack(self.fmt,self.data)
        for key,value in zip(self.keys,tpl):
            self.dict[key] = value

    def unpackPart(self,data,fmt,start,keys):
        stop = start + struct.calcsize(fmt)
        values = struct.unpack(fmt,data[start:stop])
        self.dict.update({key: value for key,value in zip(keys,values)})
        return stop

    def pack(self):
        tpl =(self.dict[key] for key in self.keys)
        return struct.pack(self.fmt,*tpl)

class SeatracSettings(SeatracStruct):
    """ """
    STATUS_MODE_MANUAL = 0x0
    STATUS_MODE_1HZ = 0x1
    STATUS_MODE_2HZ5 = 0x02
    STATUS_MODE_5HZ = 0x3
    STATUS_MODE_10HZ = 0x4
    STATUS_MODE_25HZ = 0x5
    # bits used to set / unset message transmission on
    # serial line (xcvr_flags field)
    BIT_USBL_USE_AHRS  = 0
    BIT_XCVR_POSFLT_ENABLE = 1
    BIT_RESERVED = 2
    BIT_XCVR_TX_MSGCTRL_LSB = 3
    BIT_XCVR_TX_MSGCTRL_MSB = 4
    BIT_XCVR_USBL_MSGS = 5
    BIT_XCVR_FIX_MSGS = 6
    BIT_XCVR_DIAG_MSGS = 7
    #
    BIT_ENVIRONMENT = 0
    BIT_ATTITUDE = 1
    BIT_MAG_CAL = 2
    BIT_ACC_CAL = 3
    BIT_AHRS_RAW_DATA = 4
    BIT_AHRS_COMP_DATA = 5

    keys = [
        'status_flags','status_output','uart_main_baud','uart_aux_baud',
        'net_mac_addr','net_ip_addr','net_ip_subnet','net_ip_gateway',
        'net_ip_dns','net_tcp_port','env_flags','env_pressure_ofs',
        'env_salinity','env_vos','ahrs_flags',
        #'ahrs_cal',
        'acc_min_x','acc_min_y','acc_min_z','acc_max_x','acc_max_y','acc_max_z',
        'mag_valid','mag_hard_x','mag_hard_y','mag_hard_y','mag_soft_x','mag_soft_y',
        'mag_soft_z','mag_field','mag_error','gyro_offset_x','gyro_offset_y',
        'gyro_offset_z',
        'ahrs_yaw_ofs',
        'ahrs_pitch_ofs','ahrs_roll_ofs','xcvr_flags','xcvr_beacon_id',
        'xcvr_range_tmo','xcvr_resp_time','xcvr_yaw','xcvr_pitch',
        'xcvr_roll','xcvr_posflt_vel','xcvr_posflt_ang','xcvr_posflt_tmo'
    ]

    fmt = '<BBBB6B4B4B4B4BHBlHHB6hB8f3h3hBB5H3B'

    def unpack(self,data):
        print('datalen: ' + str(len(data)))
        tpl = struct.unpack(self.fmt,data)
        k = 0
        for key in self.keys:
            if key == 'net_mac_addr':
                self.dict[key] = tpl[k:k+6]
                k = k + 6
            elif key == 'net_ip_addr':
                self.dict[key] = tpl[k:k+4]
                k = k + 4
            elif key == 'net_ip_subnet':
                self.dict[key] = tpl[k:k+4]
                k = k + 4
            elif key == 'net_ip_gateway':
                self.dict[key] = tpl[k:k+4]
                k = k + 4
            elif key == 'net_ip_dns':
                self.dict[key] = tpl[k:k+4]
                k = k + 4
            else:
                self.dict[key] = tpl[k]
                k = k + 1

    def pack(self):
        tpl =()
        for key in self.keys:
            if key == 'net_mac_addr':
                tpl += self.dict[key]
            elif key == 'net_ip_addr':
                tpl += self.dict[key]
            elif key == 'net_ip_subnet':
                tpl += self.dict[key]
            elif key == 'net_ip_gateway':
                tpl += self.dict[key]
            elif key == 'net_ip_dns':
                tpl += self.dict[key]
            else:
                tpl += (self.dict[key],)
        #
        return struct.pack(self.fmt,*tpl)

    def setStatusOutputRate(self,rate):
        """ rate in [0.0,1.0,2.5,5.0,10.0,25.0] """
        choice = {
            0.0: self.STATUS_MODE_MANUAL,
            1.0: self.STATUS_MODE_1HZ,
            2.5: self.STATUS_MODE_2HZ5,
            5.0: self.STATUS_MODE_5HZ,
            10.0: self.STATUS_MODE_10HZ,
            25.0: self.STATUS_MODE_25HZ
        }

        try:
            self.dict['status_flags'] = choice[rate]
        except KeyError:
            print('bad rate for status output')
            return False
        return True

    def restetStatusOutputContents(self):
        self.dict['status_output'] = 0

    def setStatusOutputEnvironment(self):
        self.dict['status_output'] = (1 << self.BIT_ENVIRONMENT)

    def setStatusOutputAttitude(self):
        self.dict['status_output'] = (1 << self.BIT_ATTITUDE)

    def setStatusOutputMagCal(self):
        self.dict['status_output'] = (1 << self.BIT_MAG_CAL)

    def setStatusOutputAccCal(self):
        self.dict['status_output'] = (1 << self.BIT_ACC_CAL)

    def setStatusOutputAhrsRawData(self):
        self.dict['status_output'] = (1 << self.BIT_AHRS_RAW_DATA)

    def setStatusOutputAhrsCompensatedData(self):
        self.dict['status_output'] = (1 << self.BIT_AHRS_COMP_DATA)

    def resetXcvrFlags(self):
        self.dict['xcvr_flags'] = 0

    def enableUseAhrsForUsbl(self,flag):
        if flag is True:
            self.dict['xcvr_flags'] |= (1 << self.BIT_USBL_USE_AHRS)
        else:
            self.dict['xcvr_flags'] &= ~(1 << self.BIT_USBL_USE_AHRS)
            
    def enableUsblMsgs(self,flag):
        if flag is True:
            self.dict['xcvr_flags'] |= (1 << self.BIT_XCVR_USBL_MSGS)
        else:
            self.dict['xcvr_flags'] &= ~(1 << self.BIT_XCVR_USBL_MSGS)

    def enableFixMsgs(self,flag):
        if flag is True:
            self.dict['xcvr_flags'] |= (1 << self.BIT_XCVR_FIX_MSGS)
        else:
            self.dict['xcvr_flags'] &= ~(1 << self.BIT_XCVR_FIX_MSGS)

class SeatracStatusOutput(SeatracStruct):
    def unpack(self,data):
        fmt = '<BQ'
        start = 0
        keys = ['statusBits','timestampMs']
        start = self.unpackPart(data,fmt,start,keys)
        statusBits = self.dict['statusBits']
        if statusBits & 0x1: # Bit 0: ENVIRONMENT
            fmt = '<HhllH'
            keys = ['envSupply','envTemp','envPressure','envDepth','envVos']
            start = self.unpackPart(data,fmt,start,keys)
        if statusBits & 0x2:    # Bit 1: ATTITUDE
            fmt = '<hhh'
            keys = ['yaw','pitch','roll']
            start = self.unpackPart(data,fmt,start,keys)
        if statusBits & 0x4:    # Bit 2: MAG_CAL
            fmt = '<BBLB'
            keys = ['magCalBuf','magCalValid','magCalAge','magCalFit']
            start = self.unpackPart(data,fmt,start,keys)
        if statusBits & 0x8:    # Bit 3: ACC_CAL
            fmt = '<6h'
            keys = ['accLimMinX','accLimMinY','accLimMinZ']
            keys += ['accLimMaxX','accLimMaxY','accLimMaxZ']
            start = self.unpackPart(data,fmt,start,keys)
        if statusBits & 0x10:   # Bit 4: AHRS_RAW_DATA
            fmt = '<9h'
            keys = ['ahrsRawAccX','ahrsRawAccY','ahrsRawAccZ']
            keys += ['ahrsRawMagX','ahrsRawMagY','ahrsRawMagZ']
            keys += ['ahrsRawGyroX','ahrsRawGyroY','ahrsRawGyroZ']
            start = self.unpackPart(data,fmt,start,keys)
        if statusBits & 0x20:   # Bit 5: AHRS_COMP_DATA
            fmt = '<9f'
            keys = ['ahrsCompAccX','ahrsCompAccY','ahrsCompAccZ']
            keys += ['ahrsCompMagX','ahrsCompMagY','ahrsCompMagZ']
            keys += ['ahrsCompGyroX','ahrsCompGyroY','ahrsCompGyroZ']
            start = self.unpackPart(data,fmt,start,keys)

class SeatracXcvrUsbl(SeatracStruct):

    def unpack(self,data):
        start = 0
        fmt = '<ffHfHH'
        keys = [
            'xcorSigPeak','xcorThreshold','xcorCrossPoint','xcorCrossMag',
            'xcorDetect','xcorLength'
        ]
        start = self.unpackPart(data,fmt,start,keys)
        #
        n = self.dict['xcorLength']
        fmt = '<' + str(n) + 'f'
        start = self.unpackPart(data,fmt,start,keys)
        stop += struct.calcsize(fmt)
        xcorData = np.array(struct.unpack(fmt,data[start:stop]))
        self.dict['xcorData'] = xcorData
        start = stop
        #
        keys = ['channels']
        fmt = '<B'
        start = self.unpackPart(data,fmt,start,keys)
        n = self.dict['channels']
        #
        fmt = '<' + str(n) + 'h'
        keys = ['channelRssi' + str(k) for k in range(n)]
        start = self.unpackPart(data,fmt,start,keys)
        #
        fmt = '<B'
        keys = ['baselines']
        start = self.unpackPart(data,fmt,start,keys)
        n = self.dict['baselines']
        #
        fmt = '<' + str(n) + 'f'
        keys = ['phaseAngle' + str(k) for k in range(n)]
        start = self.unpackPart(data,fmt,start,keys)
        #
        fmt = '<hhfBB'
        keys = ['signalAzimuth','signalElevation','signalFitError','beaconDstId']
        keys += ['beaconSrcId']
        start = self.unpackPart(data,fmt,start,keys)

class SeatracAcoFix(SeatracStruct):
    # start of the structure
    # -> following members can change
    fmt = '<BBBBhhhHHh'
    keys = [
        'destId','srcId','flags','msgType','attitudeYaw','attitudePitch',
        'attitudeRoll','depthLocal','vos','rssi'
    ]

    def __str__(self):
        return 'Not implemented'

    def unpack(self,data):
        start = 0
        start = self.unpackPart(data,self.fmt,start,self.keys)
        flags = self.dict['flags']
        if flags & 0x01 :   # RANGE_VALID
            fmt = '<LlH'
            keys = ['rangeCount','rangeTime','rangeDist']
            start = self.unpackPart(data,fmt,start,keys)
        if flags & 0x02:    # USBL_VALID
            fmt = '<B'
            keys = ['usblChannels']
            start = self.unpackPart(data,fmt,start,keys)
            #
            channels = self.dict['usblChannels']
            fmt = '<' + str(channels) + 'h'
            keys = ['usblRssi' + str(k) for k in range(channels)]
            start = self.unpackPart(data,fmt,start,keys)
            #
            fmt = '<3h'
            keys = ['usblAzimuth','usblElevation','usblFitError']
            start = self.unpackPart(data,fmt,start,keys)
        if flags & 0x04:    # POSITION_VALID
            fmt = '<3h'
            keys = ['positionEasting','positionNorthing','positionDepth']
            start = self.unpackPart(data,fmt,start,keys)
        self.start = start

    def pack(self):
        pass

class SeatracNavQueryResp(SeatracAcoFix):

    def __str__(self):
        return 'Not implemented'

    def unpack(self,data):
        """
        BIT_NAV_QRY_DEPTH = 0
        BIT_NAV_QRY_SUPPLY = 1
        BIT_NAV_QRY_TEMP = 2
        BIT_NAV_QRY_ATTITUDE = 3
        BIT_NAV_QRY_DATA = 7
        """
        super().unpack(data)
        start = self.start
        #
        fmt = '<B'
        keys = ['queryFlags']
        start = self.unpackPart(data,fmt,start,keys)
        #
        query = self.dict['queryFlags']
        if query & Seatrac.BIT_NAV_QRY_DEPTH > 0:
            fmt = '<l'
            keys = ['remoteDepth']
            start = self.unpackPart(data,fmt,start,keys)
        if query & Seatrac.BIT_NAV_QRY_SUPPLY > 0:
            fmt = '<H'
            keys = ['remoteSupply']
            start = self.unpackPart(data,fmt,start,keys)
        if query & Seatrac.BIT_NAV_QRY_TEMP > 0:
            fmt = '<h'
            keys = ['remoteTemp']
            start = self.unpackPart(data,fmt,start,keys)
        if query & Seatrac.BIT_NAV_QRY_ATTITUDE > 0:
            fmt = '<hhh'
            keys = ['remoteYaw','remotePitch','remoteRoll']
            start = self.unpackPart(data,fmt,start,keys)
        if query & Seatrac.BIT_NAV_QRY_DATA > 0:
            fmt = '<B'
            keys = ['packetLen']
            start = self.unpackPart(data,fmt,start,keys)
            n = self.dict['packetLen']
            fmt = '<' + str(n) + 'sB'
            keys = ['packetData','localFlag']
            start = self.unpackPart(data,fmt,start,keys)

class Seatrac(object):
    """ Test class for Blueprint Seatrac USBL """
    # AMSGTYPE_E (Acoustic Message Type)
    MSG_OWAY = 0x0
    MSG_OWAYU = 0x01
    MSG_REQ = 0x02
    MSG_RESP = 0x03
    MSG_REQU = 0x04
    MSG_RESPU = 0x05
    MSG_REQX = 0x06
    MSG_RESP = 0x03
    MSG_RESPX = 0x07
    MSG_UNKNOWN = 0xFF

    # APAYLOAD_E Acoustic Payload Identifier
    PLOAD_PING = 0x0
    PLOAD_ECHO = 0x1
    PLOAD_NAV = 0x2
    PLOAD_DAT = 0x3
    PLOAD_DEX = 0x4

    # CID_E Command Identification Codes Enumeration
    CID_SYS_ALIVE = 0x01
    CID_SYS_INFO = 0x02
    CID_SYS_REBOOT = 0x03
    CID_SYS_ENGINEERING = 0x04
    # Firmware programming messages
    CID_PROG_INIT = 0x0D
    CID_PROG_BLOCK = 0x0E
    CID_PROG_UPDATE = 0x0F
    # Status Messages
    CID_STATUS = 0x10
    CID_STATUS_CFG_GET = 0x11
    CID_STATUS_CFG_SET = 0x12
    # Settings Messages
    CID_SETTINGS_GET = 0x15
    CID_SETTINGS_SET = 0x16
    CID_SETTINGS_LOAD = 0x17
    CID_SETTINGS_SAVE = 0x18
    CID_SETTINGS_RESET = 0x19
    # Calibration Messages
    CID_CAL_ACTION = 0x20
    CID_AHRS_CAL_GET = 0x21
    CID_AHRS_CAL_SET = 0x22
    # Acoustic Transceiver Messages
    CID_XCVR_ANALYSE = 0x30
    CID_XCVR_TX_MSG = 0x31
    CID_XCVR_RX_ERR = 0x32
    CID_XCVR_RX_MSG = 0x33
    CID_XCVR_RX_REQ = 0x34
    CID_XCVR_RX_RESP = 0x35
    CID_XCVR_RX_UNHANDLED = 0x37
    CID_XCVR_USBL = 0x38
    CID_XCVR_FIX = 0x39
    CID_XCVR_STATUS = 0x3A
    CID_XCVR_TX_MSGCTRL_SET = 0x3B
    # Ping Protocol Messages
    CID_PING_SEND = 0x40
    CID_PING_REQ = 0x41
    CID_PING_RESP = 0x42
    CID_PING_ERROR = 0x43
    # Echo Protocol Messages
    CID_ECHO_SEND = 0x48
    CID_ECHO_REQ = 0x49
    CID_ECHO_RESP = 0x4A
    CID_ECHO_ERROR = 0x4B
    # Nav Protocol Messages
    CID_NAV_QUERY_SEND = 0x50
    CID_NAV_QUERY_REQ = 0x51
    CID_NAV_QUERY_RESP = 0x52
    CID_NAV_ERROR = 0x53
    CID_NAV_QUEUE_SET= 0x58
    CID_NAV_QUEUE_CLR = 0x59
    CID_NAV_QUEUE_STATUS = 0x5A
    CID_NAV_STATUS_SEND = 0x5B
    CID_NAV_STATUS_RECEIVE = 0x5C
    # Dat Protocol Messages
    CID_DAT_SEND = 0x60
    CID_DAT_RECEIVE = 0x61
    CID_DAT_ERROR = 0x63
    CID_DAT_QUEUE_SET = 0x64
    CID_DAT_QUEUE_CLR = 0x65
    CID_DAT_QUEUE_STATUS = 0x66
    # CFG Protocol Messages
    CID_CFG_BEACON_GET = 0x80
    CID_CFG_BEACON_SET = 0x81
    CID_CFG_BEACON_RESP = 0x82

    # CST_E (Command Status Code)
        # General Status Codes
    CST_OK = 0x00
    CST_FAIL = 0x01
    CST_EEPROM_ERROR = 0x03
        # Command Processor Status Codes
    CST_CMD_PARAM_MISSING = 0x04
    CST_CMD_PARAM_INVALID = 0x05

    # NAV_QUERY_T (Nav Protocol Query Bit Mask)
    BIT_NAV_QRY_DEPTH = 0
    BIT_NAV_QRY_SUPPLY = 1
    BIT_NAV_QRY_TEMP = 2
    BIT_NAV_QRY_ATTITUDE = 3
    BIT_NAV_QRY_DATA = 7

    def __init__(self,serialName,baudrate,rosLogger = None):
        self.now = time.time()
        self.ago = self.now
        self.buf = b''
        self.serial = Serial(serialName,baudrate,timeout = 0.1)
        #
        self.rosLogger = rosLogger
        #
        self.msgHandler = {
            self.CID_SYS_ALIVE: None,
            self.CID_SYS_INFO: None,
            self.CID_SYS_REBOOT: None,
            self.CID_SYS_ENGINEERING: None,
            self.CID_PROG_INIT: None,
            self.CID_PROG_BLOCK: None,
            self.CID_PROG_UPDATE: None,
            self.CID_STATUS: self.statusOutputHandler,
            self.CID_STATUS_CFG_GET: None,
            self.CID_STATUS_CFG_SET: None,
            self.CID_SETTINGS_GET: self.settingsGetHandler,
            self.CID_SETTINGS_SET: self.settingsSetHandler,
            self.CID_SETTINGS_LOAD: None,
            self.CID_SETTINGS_SAVE: None,
            self.CID_SETTINGS_RESET: None,
            self.CID_CAL_ACTION: None,
            self.CID_AHRS_CAL_GET: None,
            self.CID_AHRS_CAL_SET: None,
            self.CID_XCVR_ANALYSE: None,
            self.CID_XCVR_TX_MSG: None,
            self.CID_XCVR_RX_ERR: None,
            self.CID_XCVR_RX_MSG: None,
            self.CID_XCVR_RX_REQ: None,
            self.CID_XCVR_RX_RESP: None,
            self.CID_XCVR_RX_UNHANDLED: None,
            self.CID_XCVR_USBL: self.cidXcvrUsblHandler,
            self.CID_XCVR_FIX: self.cidXcvrFixHandler,
            self.CID_XCVR_STATUS: None,
            self.CID_XCVR_TX_MSGCTRL_SET: None,
            self.CID_PING_SEND: None,
            self.CID_PING_REQ: None,
            self.CID_PING_RESP: None,
            self.CID_PING_ERROR: None,
            self.CID_ECHO_SEND: None,
            self.CID_ECHO_REQ: None,
            self.CID_ECHO_RESP: None,
            self.CID_ECHO_ERROR: None,
            self.CID_NAV_QUERY_SEND: None,
            self.CID_NAV_QUERY_REQ: None,
            self.CID_NAV_QUERY_RESP: self.cidNavQueryRespHandler,
            self.CID_NAV_ERROR: None,
            self.CID_NAV_QUEUE_SET: None,
            self.CID_NAV_QUEUE_CLR: None,
            self.CID_NAV_QUEUE_STATUS: None,
            self.CID_NAV_STATUS_SEND: None,
            self.CID_NAV_STATUS_RECEIVE: None,
            self.CID_DAT_SEND: None,
            self.CID_DAT_RECEIVE: None,
            self.CID_DAT_ERROR: None,
            self.CID_DAT_QUEUE_SET: None,
            self.CID_DAT_QUEUE_CLR: None,
            self.CID_DAT_QUEUE_STATUS: None,
            self.CID_CFG_BEACON_GET: None,
            self.CID_CFG_BEACON_SET: None,
            self.CID_CFG_BEACON_RESP: None
        }
        self.msgCbk = {}

    def setMsgCbk(self,msgId,cbk):
        self.msgCbk[msgId] = cbk

    def nextAnswer(self,data):
        """ """
        start = data.find(b'$')
        if start < 0:
            # no answer start, buffer can be returned empty
            return (b'',b'')
        stop = data.find(b'\r\n',start)
        if stop < 0:
            # no answer end, data may be the start of a message
            return (b'',data)
        return (data[start + 1:stop],data[stop + 2:])

    def update(self):
        self.buf += self.serial.read(1024)
        self.now = time.time()
        while True:
            msg,self.buf = self.nextAnswer(self.buf)
            if msg == b'':
                break
            #
            try:
                data = bytes.fromhex(msg[2:-4].decode('ascii'))
            except ValueError:
                print('value error: ' + msg.decode('ascii'))
            msgId = int(msg[0:2],16)
            print('msgId: ' + hex(msgId) + '  ' + str(len(msg)))
            #
            msgHandler = None
            try:
                msgHandler = self.msgHandler[msgId]
            except KeyError:
                print('no msg handler for msgId: ' + hex(msgId))
            if msgHandler is None:
                print('msg handler not coded for ' + hex(msgId))
                print(data)
            else:
                seatracStruct = msgHandler(data,self.now)
            cbk = None
            try:
                cbk =self.msgCbk[msgId]
            except KeyError:
                pass
            if cbk is not None:
                cbk(seatracStruct)

    def settingsGetHandler(self,data,timestamp):
        return SeatracSettings(data = data,timestamp = timestamp) 

    def settingsSetHandler(self,data,timestamp):
        return data[0]

    def statusOutputHandler(self,data,timestamp):
        statusOutput = SeatracStatusOutput(data = data,timestamp = timestamp)
        if self.rosLogger is not None:
            self.rosLogger.info(str(statusOutput.dict))
        return statusOutput

    def cidXcvrFixHandler(self,data,timestamp):
        acofix = SeatracAcoFix(data = data,timestamp = timestamp)
        return acofix

    def cidXcvrUsblHandler(self,data,timestamp):
        return SeatracXcvrUsbl(data = data,timestamp = timestamp)

    def cidNavQueryRespHandler(self,data,timestamp):
        navQueryResp = SeatracNavQueryResp(data,timestamp)
        if self.rosLogger is not None:
            self.rosLogger.info('cidNavQueryRespHandler')
            self.rosLogger.info('queryFlags: ' + str(navQueryResp['queryFlags']))
            self.rosLogger.info(str(navQueryResp.dict))
            #self.rosLogger.info('remoteDepth: ' + str(navQueryResp['remoteDepth']))
            #self.rosLogger.info('remoteSupply: ' + str(navQueryResp['remoteSupply']))
            #self.rosLogger.info('remoteYaw: ' + str(navQueryResp['remoteYaw']))
            #self.rosLogger.info('remotePitch: ' + str(navQueryResp['remotePitch']))
            #self.rosLogger.info('remoteRoll: ' + str(navQueryResp['remoteRoll']))
        return navQueryResp

    def crc16(self,data):
        """ """
        poly = 0xA001
        crc = 0
        for octet in data:
            b = octet
            for bit in range(8):
                if (b & 0x01) ^ (crc & 0x01) > 0:
                    crc >>= 1
                    crc ^= poly
                else:
                    crc >>= 1
                b >>= 1
        return crc

    def buildCmd(self,fmt,tpl):
        """ """
        data = struct.pack(fmt,*tpl)
        data += struct.pack('<H',self.crc16(data))
        data = '#' + data.hex() + '\r\n'
        return bytes(data.upper().encode('ascii'))

    def buildCmdFromData(self,msgId,data):
        """ """
        msgCode = msgId.to_bytes(1,'little')
        data = msgCode + data
        data += struct.pack('<H',self.crc16(data))
        data = '#' + data.hex() + '\r\n'
        return bytes(data.upper().encode('ascii'))
        
    def cidPingSend(self,beaconId,msgType):
        """
        beaconId: id of the beacon to ping
            0: BEACON_ALL   broadcast to all
        msgType: AMSGTYPE_E page 35 devlopper guide
        """
        fmt = '<BBB'
        cmd = self.buildCmd(fmt,(self.CID_PING_SEND,beaconId,msgType))
        self.serial.write(cmd)

    def cidStatusCfgSet(self):      # page 86
        """ """
        fmt = '<BBB'
        msgId = self.CID_STATUS_CFG_SET
        statusOutput = 0x03     # environment and attitude
        statusMode = 0x02 # rate 2.5 Hz #0x3        # rate 5 Hz
        statusOutput = 0
        statusMode = 0
        cmd = self.buildCmd(fmt,(msgId,statusOutput,statusMode))
        self.serial.write(cmd)

    def cidSettingsGet(self):
        fmt = '<B'
        cmd = self.buildCmd(fmt,(self.CID_SETTINGS_GET,))
        self.serial.write(cmd)
         
    def cidSettingsSet(self,settings):      # CID_SETTINGS_SET p88
        """
        SETTINGS_T page 60
        not finished... not tested
        """
        data = settings.pack()
        cmd = self.buildCmdFromData(self.CID_SETTINGS_SET,data)
        self.serial.write(cmd)

    def cidXcvrTxMsgctrlSet(self):
        """
        CID_XCVR_TX_MSGCTRL_SET p109
        """
        fmt = '<BB'
        # page 64
        # Bit 0  USBL_USE_AHRS
        # Bit 1  XCVR_POSFLT_ENABLE
        # Bit 2  RESERVED
        # Bit 4:3  XCVR_TX_MSGCTRL
        # Bit 5  XCVR_USBL_MSGS
        # Bit 6  XCVR_POSFLT_ENABLE
        # Bit 7  XCVR_DIAG_MSGS
        #
        xcvrFlags = 0x1 + (0x1 << 5)
        cmd = self.buildCmd(fmt,(0x3b,xcvrFlags))
        self.serial.write(cmd)

    def cidNavQuerySend(self,beaconId,flags = {},data = None):
        """
        beaconId: id of the beacon to ping
        flags: dictionnary with keys:
            'depth': True/False
            'voltage': True/False   (power supply voltage)
            'temperature': True/false
            'attitude': True/false
            'data': True/false
        data: data to be sent if flags['data'] == True (no more than 29 bytes)
         """
        flagKeys = ['depth','voltage','temperature','attitude','data']
        bits = [
            self.BIT_NAV_QRY_DEPTH,self.BIT_NAV_QRY_SUPPLY,
            self.BIT_NAV_QRY_TEMP,self.BIT_NAV_QRY_ATTITUDE,
            self.BIT_NAV_QRY_DATA
        ]
        query = 0
        value = False
        for flag,bit in zip(flagKeys,bits):
            try:
                value = flags[flag] 
            except KeyError:
                continue
            if value is True:
                query += (1 << bit)
        #
        dataLen = 0
        if query & (1 << self.BIT_NAV_QRY_DATA):
            if data is None:
                query &= ~(1 << self.BIT_NAV_QRY_DATA)
            else:
                dataLen = len(data)
                if dataLen > 29:
                    data = data[:29]
                    dataLen = 29
        #
        fmt = '<BBBB'
        if dataLen > 0:
            fmt += str(dataLen) + 's'
            cmd = self.buildCmd(fmt,(self.CID_NAV_QUERY_SEND,beaconId,query,dataLen,data))
        else:
            cmd = self.buildCmd(fmt,(self.CID_NAV_QUERY_SEND,beaconId,query,dataLen))
        #
        self.serial.write(cmd)

