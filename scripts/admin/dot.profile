# commands to be run on login

/usr/local/sbin/stopme 5

if [ $? != 0 ]; then
    echo "Starting interactive shell"
else
    echo "Continuing with default"

    # for now try to setup ad-hoc wifi network here
    ./setup-ad-hoc.sh

    systemctl restart dhcpd.service

    ./setup-gpsd.sh

    #/usr/local/bin/ugear --remote-link on
    /usr/local/bin/uglink --serial /dev/ttyO2 > /dev/null 2>&1
fi
