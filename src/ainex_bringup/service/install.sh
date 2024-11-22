#!/bin/bash
sudo cp *.service /etc/systemd/system/
cd /etc/systemd/system/
sudo systemctl enable clear_log.service expand_rootfs.service start_app_node.service voltage_detect.service
sudo systemctl daemon-reload
echo "finish "
