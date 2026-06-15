# Build AR22_LCU firmware with the ARM GNU toolchain (no STM32CubeIDE required).
# Produces build/AR22_LCU_v1_1.elf/.hex/.bin/.map
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path "$PSScriptRoot\..").Path

# --- Toolchain ---
$gccDir = "C:\Program Files (x86)\Arm\GNU Toolchain mingw-w64-i686-arm-none-eabi\bin"
$gcc  = Join-Path $gccDir "arm-none-eabi-gcc.exe"
$size = Join-Path $gccDir "arm-none-eabi-size.exe"
$oc   = Join-Path $gccDir "arm-none-eabi-objcopy.exe"
if (-not (Test-Path $gcc)) { $gcc = "arm-none-eabi-gcc"; $size = "arm-none-eabi-size"; $oc = "arm-none-eabi-objcopy" }

New-Item -ItemType Directory -Force "$root\build" | Out-Null

# --- Sources ---
$srcs = @()
$srcs += Get-ChildItem "$root\Core\Src" -Filter *.c | ForEach-Object FullName
$srcs += Get-ChildItem "$root\Drivers\STM32F4xx_HAL_Driver\Src" -Filter *.c | ForEach-Object FullName
$srcs += Get-ChildItem "$root\MyLib" -Recurse -Filter *.c | ForEach-Object FullName
$startup = "$root\Core\Startup\startup_stm32f446vetx.s"

# --- Flags (match .cproject) ---
$cpu  = @("-mcpu=cortex-m4","-mfpu=fpv4-sp-d16","-mfloat-abi=hard","-mthumb")
$defs = @("-DUSE_HAL_DRIVER","-DSTM32F446xx","-DDEBUG",
          "-D_WIZCHIP_=W5500")   # WIZnet ioLibrary chip select (default would be W6300, unbundled)
$inc  = @(
  "-I$root\Core\Inc",
  "-I$root\Drivers\STM32F4xx_HAL_Driver\Inc",
  "-I$root\Drivers\STM32F4xx_HAL_Driver\Inc\Legacy",
  "-I$root\Drivers\CMSIS\Device\ST\STM32F4xx\Include",
  "-I$root\Drivers\CMSIS\Include",
  "-I$root\MyLib",
  "-I$root\MyLib\SSD1322_OLED_lib_NB",
  "-I$root\MyLib\MOVING_AVERAGE",
  "-I$root\MyLib\EEPROM_I2C",
  # W5500 Ethernet (data-logging TCP server): net_app + WIZnet ioLibrary headers
  "-I$root\MyLib\W5500",
  "-I$root\MyLib\W5500\ioLibrary",
  "-I$root\MyLib\W5500\ioLibrary\W5500"
)
$cflags = $cpu + $defs + $inc + @("-O0","-g3","-ffunction-sections","-fdata-sections","-std=gnu11","-Wall")
$lflags = $cpu + @(
  "-T$root\STM32F446VETX_FLASH.ld",
  "--specs=nano.specs","--specs=nosys.specs",
  "-u","_printf_float",
  "-Wl,--gc-sections",
  "-Wl,--no-warn-rwx-segments",
  "-Wl,-Map=$root\build\AR22_LCU_v1_1.map",
  "-lc","-lm"
)

$out = "$root\build\AR22_LCU_v1_1.elf"
$allargs = $cflags + @("-x","assembler-with-cpp",$startup,"-x","none") + $srcs + @("-o",$out) + $lflags

Write-Host "Building $($srcs.Count) source files..."
& $gcc @allargs
if ($LASTEXITCODE -ne 0) { throw "Build failed (gcc exit $LASTEXITCODE)" }

& $oc -O ihex   $out "$root\build\AR22_LCU_v1_1.hex"
& $oc -O binary $out "$root\build\AR22_LCU_v1_1.bin"
& $size $out
Write-Host "Build OK -> build\AR22_LCU_v1_1.hex"
