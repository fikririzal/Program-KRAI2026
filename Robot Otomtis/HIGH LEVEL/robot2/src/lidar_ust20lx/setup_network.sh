#!/bin/bash
# Setup IP statis eth1 untuk Hokuyo UST-20LX (192.168.0.100)

CON_NAME="lidar-ust20lx"
IFACE="eth1"
IP="192.168.0.50/24"

if nmcli con show "$CON_NAME" &>/dev/null; then
    echo "[lidar] Koneksi '$CON_NAME' sudah ada, skip."
else
    echo "[lidar] Membuat koneksi '$CON_NAME' di $IFACE..."
    sudo nmcli con add type ethernet ifname "$IFACE" con-name "$CON_NAME" \
        ipv4.method manual ipv4.addresses "$IP" \
        ipv6.method ignore connection.autoconnect yes
fi

if ! nmcli con show --active "$CON_NAME" &>/dev/null; then
    echo "[lidar] Mengaktifkan koneksi '$CON_NAME'..."
    sudo nmcli con up "$CON_NAME"
else
    echo "[lidar] Koneksi '$CON_NAME' sudah aktif."
fi

echo "[lidar] IP eth1: $(ip -4 addr show $IFACE | grep inet | awk '{print $2}')"
