#!/usr/bin/env python
# -*- coding: utf-8 -*-
# source: https://stackoverflow.com/questions/28867795/reading-i2c-data-from-gps2
import time
import json
import smbus
import logging

BUS = None
address = 0x42
gpsReadInterval = 0.0
TOTALmsg=1
OKmsg=1

# GUIDE
# http://ava.upuaut.net/?p=768

GPSDAT = {
    'strType': None,
    'fixTime': None,
    'lat': None,
    'latDir': None,
    'lon': None,
    'lonDir': None,
    'fixQual': None,
    'numSat': None,
    'horDil': None,
    'alt': None,
    'altUnit': None,
    'galt': None,
    'galtUnit': None,
    'DPGS_updt': None,
    'DPGS_ID': None
}

def connectBus():
    global BUS
    BUS = smbus.SMBus(1)

def parseResponse(gpsLine):

    global TOTALmsg
    global OKmsg
    global lastLocation
    global starttime
    gpsChars = ''.join(chr(c) for c in gpsLine)
    gpsCharsbin = ''.join(str(bin(c)) for c in gpsLine)
    
    if "*" not in gpsChars or gpsChars.endswith('*'):
        return False

    gpsStr, chkSum = gpsChars.split('*')
    gpsComponents = gpsStr.split(',')
    gpsStart = gpsComponents[0][1:]
    if (gpsStart == "GNGGA"):
        chkVal = 0
        TOTALmsg += 1
        for ch in gpsStr[1:]: # Remove the $
            chkVal ^= ord(ch)
        if (chkVal == int(chkSum, 16)):
            OKmsg +=1
            for i, k in enumerate(
                ['strType', 'fixTime',
                'lat', 'latDir', 'lon', 'lonDir',
                'fixQual', 'numSat', 'horDil',
                'alt', 'altUnit', 'galt', 'galtUnit',
                'DPGS_updt', 'DPGS_ID']):
                GPSDAT[k] = gpsComponents[i]
#            print json.dumps(GPSDAT, indent=2)
            print gpsComponents

def readGPS():
    c = None
    response = []
    try:
        while True: # Newline, or bad char.
            c = BUS.read_byte(address)
            if c == 255:
                return False
            elif c == 10:
                break
            else:
                response.append(c)
        parseResponse(response)
    except IOError:
        time.sleep(0.5)
        connectBus()
    except Exception, e:
        print e

connectBus()
starttime = time.time()
while time.time()-starttime < 200000000:
    readGPS()
    time.sleep(gpsReadInterval)

currenttime = time.time() 
print("ratio: ", 1.0*OKmsg/TOTALmsg)
print("freq:  ", 1.0*TOTALmsg/(currenttime-starttime))
print("freqok:", 1.0*OKmsg/(currenttime-starttime))
print()
