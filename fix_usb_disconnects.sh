#!/bin/bash
# Fix USB Disconnection Issues for ODrive Controllers on Raspberry Pi
# Run this script once to apply permanent fixes

echo "🔧 Fixing USB disconnection issues for ODrive controllers..."

# 1. Disable USB autosuspend (main cause of disconnections)
echo "📌 Disabling USB autosuspend..."
sudo bash -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend'

# Make USB autosuspend disable permanent
echo "📌 Making USB autosuspend disable permanent..."
if ! grep -q "usbcore.autosuspend=-1" /boot/cmdline.txt; then
    sudo sed -i 's/$/ usbcore.autosuspend=-1/' /boot/cmdline.txt
    echo "   ✅ Added usbcore.autosuspend=-1 to /boot/cmdline.txt"
else
    echo "   ✅ USB autosuspend already disabled in boot config"
fi

# 2. Increase USB power (ODrives can be power-hungry)
echo "📌 Increasing USB power limits..."
if ! grep -q "max_usb_current=1" /boot/config.txt; then
    echo "max_usb_current=1" | sudo tee -a /boot/config.txt
    echo "   ✅ Enabled max USB current in /boot/config.txt"
else
    echo "   ✅ Max USB current already enabled"
fi

# 3. Disable USB selective suspend for all devices
echo "📌 Disabling USB selective suspend..."
sudo bash -c 'for i in /sys/bus/usb/devices/*/power/autosuspend_delay_ms; do echo -1 > "$i" 2>/dev/null || true; done'
sudo bash -c 'for i in /sys/bus/usb/devices/*/power/control; do echo on > "$i" 2>/dev/null || true; done'

# 4. Create udev rules to prevent ODrive power management
echo "📌 Creating udev rules for ODrive controllers..."
sudo tee /etc/udev/rules.d/99-odrive-usb.rules > /dev/null << 'EOF'
# Disable power management for ODrive controllers
# ODrive uses VID 1209 (pid.codes) with various PIDs
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{power/control}="on"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{power/autosuspend}="-1"

# Also apply to CDC ACM devices (ODrive Native USB interface)
ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTR{../power/control}="on"
ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTR{../power/autosuspend}="-1"
EOF

echo "   ✅ Created udev rules for ODrive power management"

# 5. Reload udev rules
echo "📌 Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# 6. Set USB buffer sizes (helps with communication stability)
echo "📌 Optimizing USB buffer sizes..."
if ! grep -q "dwc_otg.fiq_fix_enable=1" /boot/cmdline.txt; then
    sudo sed -i 's/$/ dwc_otg.fiq_fix_enable=1/' /boot/cmdline.txt
    echo "   ✅ Enabled USB FIQ fix"
fi

# 7. Add user to dialout group (for USB serial access)
echo "📌 Ensuring user has USB access..."
sudo usermod -a -G dialout $USER
echo "   ✅ Added $USER to dialout group"

# 8. Create systemd service to apply USB fixes on boot
echo "📌 Creating systemd service for USB fixes..."
sudo tee /etc/systemd/system/fix-usb-power.service > /dev/null << 'EOF'
[Unit]
Description=Fix USB Power Management for ODrive Controllers
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend'
ExecStart=/bin/bash -c 'for i in /sys/bus/usb/devices/*/power/autosuspend_delay_ms; do echo -1 > "$i" 2>/dev/null || true; done'
ExecStart=/bin/bash -c 'for i in /sys/bus/usb/devices/*/power/control; do echo on > "$i" 2>/dev/null || true; done'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable fix-usb-power.service
echo "   ✅ Created and enabled USB power fix service"

echo ""
echo "🎉 USB fixes applied successfully!"
echo ""
echo "📋 SUMMARY OF CHANGES:"
echo "   • Disabled USB autosuspend globally"
echo "   • Increased USB power limits"
echo "   • Created udev rules for ODrive power management"
echo "   • Optimized USB buffer settings"
echo "   • Created systemd service for persistent fixes"
echo ""
echo "⚠️  REBOOT REQUIRED: Some changes require a reboot to take effect"
echo "💡 Run: sudo reboot"
echo ""
echo "🔍 TROUBLESHOOTING:"
echo "   • Use high-quality, short USB cables (< 1m)"
echo "   • Ensure adequate power supply (5V 3A+ recommended)"
echo "   • Check 'lsusb' before/after reboot to verify ODrives stay connected"
echo "   • Run 'dmesg | grep -i usb' to check for USB errors" 