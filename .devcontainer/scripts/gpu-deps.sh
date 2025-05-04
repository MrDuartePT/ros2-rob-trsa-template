#!/usr/bin/env bash
# Common Depedencies
apt-get update && apt-get install -y ocl-icd-libopencl1 clinfo
mkdir -p /etc/OpenCL/vendors


# Intel GPU/NPU depedencies
curl -fsSL https://repositories.intel.com/gpu/intel-graphics.key | gpg --dearmor -o /usr/share/keyrings/intel-graphics.gpg && \
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/intel-graphics.gpg] https://repositories.intel.com/gpu/ubuntu jammy/lts/2350 unified" > /etc/apt/sources.list.d/intel-gpu-jammy.list

apt-get update && apt-get install -y intel-opencl-icd intel-media-va-driver-non-free mesa-va-drivers \
    mesa-vdpau-drivers mesa-vulkan-drivers va-driver-all level-zero intel-level-zero-gpu
echo "libintelocl.so" > /etc/OpenCL/vendors/intel.icd

# AMD GPU/NPU depedencies
curl -fsSL https://repo.radeon.com/rocm/rocm.gpg.key | gpg --dearmor -o /usr/share/keyrings/rocm.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/rocm.gpg] https://repo.radeon.com/rocm/apt/6.4/ $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/rocm.list && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/rocm.gpg] https://repo.radeon.com/amdgpu/6.4/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/amdgpu.list;

apt-get update && apt-get install -y --no-install-recommends rocm-opencl-runtime libnuma1 libtinfo6 libnvvpi3
echo "libamdocl64.so" > /etc/OpenCL/vendors/amdocl64.icd

# NVIDIA depedencies (CUDA 12.8)
## This all based from nvidia docker images gitlab repo (https://gitlab.com/nvidia/container-images/cuda/)
curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc | gpg --dearmor -o /usr/share/keyrings/jetson-ota-public.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/jetson-ota-public.gpg] https://repo.download.nvidia.com/jetson/x86_64/$(. /etc/os-release && echo $UBUNTU_CODENAME) r36.4 main" > /etc/apt/sources.list.d/jetson.list
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin && sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
curl -fsSLO https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && dpkg -i cuda-keyring_1.1-1_all.deb

# Runtime + nvcc compiler
apt-get update && apt-get install -y \
    cuda-compiler-12-8 \
    cuda-cudart-dev-12-8 \
    cuda-command-line-tools-12-8 \
    cuda-minimal-build-12-8 \
    cuda-compat-12-8 \
    cuda-libraries-dev-12-8 \
    cuda-nvml-dev-12-8 \
    cuda-nvprof-12-8 \
    libnpp-dev-12-8 \
    cuda-nvtx-12-8 \
    libcusparse-12-8 \
    libcublas-dev-12-8 \
    libnccl-dev \
    libcudnn9-cuda-12 \
    libcudnn9-dev-cuda-12 \
    #cuda-nsight-compute-12-8 (This 8gb in size install locally if needed)

echo "/usr/local/cuda/lib64" >> /etc/ld.so.conf.d/nvidia.conf
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> /etc/profile.d/cuda.sh
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> /etc/profile.d/cuda.sh
chmod +x /etc/profile.d/cuda.sh

apt-mark hold libcublas-dev-12-8 libnccl-dev libcudnn9-dev-cuda-12

echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd