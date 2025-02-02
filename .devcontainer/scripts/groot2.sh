#!/usr/bin/env bash
#-------------------------------------------------------------------------------------------------------------
# This scripts install Groot2 for amd64 and Groot2 + Qemu for AArch64 (AppImage)

sudo apt install -y fuse
curl https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-x86_64.AppImage -o Groot2
chmod +x Groot2 && mv Groot2 /usr/local/bin/Groot2

# Setup desktop file
cat << EOF > /usr/share/applications/groot2.desktop
[Desktop Entry]
Type=Application
Name=Groot2
Exec=/usr/local/bin/Groot2
Icon=groot_icon
Comment=Graphical Editor for BehaviorTrees
Terminal=false
Categories=Development;
Name[en_US]=Groot2
EOF

if [ "$TARGETARCH" = "arm64" ]; then
    # Dependencies for Box64
    sudo apt install -y qemu qemu-user qemu-user-static binfmt-support libc6

    # Box64 ppa
    wget https://ryanfortner.github.io/box64-debs/box64.list -O /etc/apt/sources.list.d/box64.list
    wget -qO- https://ryanfortner.github.io/box64-debs/KEY.gpg | gpg --dearmor -o /etc/apt/trusted.gpg.d/box64-debs-archive-keyring.gpg 
    apt update && apt install box64 -y
fi
