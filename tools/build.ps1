param(
    [ValidateSet('Debug', 'Release')][string]$Configuration = 'Debug',
    [string]$Toolchain = $(if ($env:OS -eq 'Windows_NT') { "$env:USERPROFILE/.eide/tools/gcc_arm/bin" } else { '/usr/bin' }),
    [string]$Output = ''
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
Push-Location $root
try {
    $suffix = if ($env:OS -eq 'Windows_NT') { '.exe' } else { '' }
    $gcc = Join-Path $Toolchain "arm-none-eabi-gcc$suffix"
    $objcopy = Join-Path $Toolchain "arm-none-eabi-objcopy$suffix"
    if (!(Test-Path -LiteralPath $gcc)) { throw "Arm GNU GCC not found: $gcc" }
    if (!$Output) { $Output = "build/verify-$($Configuration.ToLower())" }
    New-Item -ItemType Directory -Force -Path $Output | Out-Null
    $flags = @('-mcpu=cortex-m3','-mthumb','-mfloat-abi=soft','-Wall','-Wextra','-Werror','-g',
        '-ffunction-sections','-fdata-sections','--specs=nano.specs','--specs=nosys.specs',
        '-DSTM32F103xB','-DUSE_FULL_LL_DRIVER')
    $flags += if ($Configuration -eq 'Debug') { @('-Og','-DDEBUG') } else { @('-Os','-DNDEBUG') }
    $includes = @('Core/Inc','Drivers/STM32F1xx_HAL_Driver/Inc','Drivers/STM32F1xx_HAL_Driver/Inc/Legacy',
        'Drivers/CMSIS/Device/ST/STM32F1xx/Include','Drivers/CMSIS/Include','App/Inc','App/RTOS/include','App/RTOS/portable/GCC/ARM_CM3')
    $flags += @($includes | ForEach-Object { '-I' + $_ })
    $sources = @(rg --files App Core Drivers -g '*.c' -g '*.s')
    if ($LASTEXITCODE -ne 0) { throw 'Cannot enumerate source files' }
    $objects = @()
    $log = [System.Collections.Generic.List[string]]::new()
    foreach ($source in $sources) {
        $obj = Join-Path $Output ($source + '.o')
        New-Item -ItemType Directory -Force -Path (Split-Path $obj) | Out-Null
        $language = if ($source.EndsWith('.s')) { @('-x','assembler-with-cpp') } else { @('-x','c','-std=c11') }
        # Vendor HAL/LL has intentionally unused parameters on this MCU variant.
        $compileFlags = @($flags)
        if ($source.StartsWith('Drivers')) { $compileFlags += '-Wno-unused-parameter' }
        $messages = & $gcc @compileFlags @language '-c' $source '-o' $obj 2>&1
        $code = $LASTEXITCODE
        foreach ($message in $messages) { $log.Add("$message") }
        if ($code -ne 0) { $log | Set-Content "$Output/build.log"; $messages | Out-Host; throw "Compilation failed: $source" }
        $objects += $obj
    }
    $elf = "$Output/4-20-to-Modbus.elf"
    $messages = & $gcc '-mcpu=cortex-m3' '-mthumb' '-mfloat-abi=soft' '-T' 'STM32F103C8TX_FLASH.ld' `
        '-Wl,--gc-sections' '-Wl,--print-memory-usage' "-Wl,-Map=$Output/4-20-to-Modbus.map" `
        '--specs=nano.specs' '--specs=nosys.specs' '-o' $elf @objects 2>&1
    $code = $LASTEXITCODE
    foreach ($message in $messages) { $log.Add("$message") }
    $log | Set-Content "$Output/build.log"
    $messages | Out-Host
    if ($code -ne 0) { throw 'Link failed' }
    & $objcopy '-O' 'ihex' $elf "$Output/4-20-to-Modbus.hex"
    if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
    "Built $Configuration ($($sources.Count) sources): $elf"
} finally { Pop-Location }
