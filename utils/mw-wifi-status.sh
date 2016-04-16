#!/bin/sh

while [ 1 ]; do
        sleep 1;
        RSSI=`wpa_cli signal_poll | sed -n 's/AVG_RSSI=//p'`
	#RSSI=$((0-$RSSI))
	mwcli -b 1 -i 75 -d $RSSI 0
done
