echo $PASSWD | sudo -S chmod 777 /sys/devices/pwm-fan/target_pwm
echo 70 > /sys/devices/pwm-fan/target_pwm