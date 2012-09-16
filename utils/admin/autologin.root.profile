# commands to be run on login

echo "Press any key to start an interactive shell"

/home/root/bin/pressanykey 5

if [ $? != 0 ]; then
    echo "\nStarting interactive shell"
else
    echo "\nContinuing with default"
    /home/root/ugear --remote-link on
fi
