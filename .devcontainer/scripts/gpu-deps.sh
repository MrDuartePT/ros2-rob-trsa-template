#!/usr/bin/env bash
# Common Depedencies
apt-get update && apt-get install -y ocl-icd-libopencl1 clinfo
mkdir -p /etc/OpenCL/vendors

# Intel GPU/NPU depedencies (478MB)
add-apt-repository -y ppa:kobuk-team/intel-graphics
apt-get update && apt-get install -y intel-opencl-icd intel-media-va-driver-non-free mesa-va-drivers \
    mesa-vdpau-drivers mesa-vulkan-drivers va-driver-all \
    libze-intel-gpu1 libze1 intel-metrics-discovery intel-opencl-icd intel-gsc intel-ocloc clinfo
echo "libintelocl.so" > /etc/OpenCL/vendors/intel.icd

# AMD GPU/NPU depedencies (661MB)
curl -fsSL https://repo.radeon.com/rocm/rocm.gpg.key | gpg --dearmor -o /usr/share/keyrings/rocm.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/rocm.gpg] https://repo.radeon.com/rocm/apt/6.4/ $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/rocm.list && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/rocm.gpg] https://repo.radeon.com/amdgpu/6.4/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/amdgpu.list;

apt-get update && apt-get update && apt-get install -y rocm-opencl-runtime rocm-language-runtime
echo "libamdocl64.so" > /etc/OpenCL/vendors/amdocl64.icd

# NVIDIA depedencies (CUDA 12.9)
## This all based from nvidia docker images gitlab repo (https://gitlab.com/nvidia/container-images/cuda/)
curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc | gpg --dearmor -o /usr/share/keyrings/jetson-ota-public.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/jetson-ota-public.gpg] https://repo.download.nvidia.com/jetson/x86_64/$(. /etc/os-release && echo $UBUNTU_CODENAME) r36.4 main" > /etc/apt/sources.list.d/jetson.list
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin && sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
curl -fsSLO https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && dpkg -i cuda-keyring_1.1-1_all.deb

# Minimal Runtime (4205 MB)
apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-12-9 \
    libcudnn9-cuda-12 \
    cuda-nvtx-12-9 \
    libcusparse-12-9 \
    libnpp-12-9 \
    libcublas-12-9 \
    libnccl2

echo "/usr/local/cuda/lib64" >> /etc/ld.so.conf.d/nvidia.conf
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> /etc/profile.d/cuda.sh
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> /etc/profile.d/cuda.sh
chmod +x /etc/profile.d/cuda.sh

apt-mark hold libcublas-dev-12-9 libnccl-dev libcudnn9-dev-cuda-12

echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd