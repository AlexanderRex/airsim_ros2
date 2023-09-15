# Airsim Pack

## How to Use It

### Cloning and Building Unreal Engine 4.27 on Linux

## Prerequisites:
- Git
- C++ Compiler (e.g., GCC)
- Development tools (build-essential, clang, etc.)


## Steps:

### 1. Clone the repository:
```bash
git clone https://github.com/EpicGames/UnrealEngine.git -b 4.27
```

### 2. Install and build UnrealEngine:
```bash
cd UnrealEngine
./Setup.sh
./GenerateProjectFiles.sh
make
```

## Build Airsim:
```bash
git clone https://github.com/Microsoft/AirSim.git
cd AirSim
./setup.sh
./build.sh
```
