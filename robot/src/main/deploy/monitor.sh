#!/usr/bin/env bash

MON_FILE=/tmp/robot_monitor.txt
rm -f $MON_FILE
while [ 1 ]; do
    if [ -e "$MON_FILE" ]; then
        while [ 1 ]; do
            CURR=$(head -1 $MON_FILE)
            if [ ! -z $LAST ]; then
                if [ "$CURR" = "$LAST" ]; then

                   FRC_PID=$(ps -ef | grep java | grep -v grep | grep -v bash | xargs echo | cut -d' '  -f1)

                   echo "$CURR == $LAST --  killing $FRC_PID"

                   kill -9 $FRC_PID
                   find /tmp/ -name 'hs_err_pid*' -exec cp "{}" ~ \;
                   rm -f $MON_FILE
                   exit 0
                fi
            fi
            LAST=$CURR
            sleep 5
        done
    fi
    sleep 5
done
