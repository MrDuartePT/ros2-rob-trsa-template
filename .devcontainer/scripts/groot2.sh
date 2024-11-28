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
    # Install from source Box64/Box86
    sudo apt install -y qemu qemu-user qemu-user-static binfmt-support libc6

    # Box86
    git clone https://github.com/ptitSeb/box86
    cd box86
    mkdir build; cd build; cmake .. -DRPI4=1 -DCMAKE_BUILD_TYPE=RelWithDebInfo
    make -j$(nproc)
    make install

    # Box64
    git clone https://github.com/ptitSeb/box64
    cd box64
    mkdir build; cd build; cmake .. -D ARM_DYNAREC=ON -D CMAKE_BUILD_TYPE=RelWithDebInfo
    make -j$(nproc)
    make install
fi
