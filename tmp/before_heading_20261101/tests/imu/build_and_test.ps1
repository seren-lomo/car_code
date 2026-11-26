param(
    [string]$ClangDir = 'E:\APP\Matlab25\toolbox\sldrt\clang\win64',
    [string]$PythonExe = 'C:\Users\Administrator\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
)
$ErrorActionPreference='Stop'
Push-Location (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
try {
    foreach ($unit in @(@('imu/imu.c','imu'),@('tests/imu/harness.c','harness'))) {
        & (Join-Path $ClangDir 'clang.exe') -c -std=c99 -ffreestanding -fno-stack-protector -include tests/imu/mock.h -ICore/Inc -Iimu $unit[0] -o "tests/imu/$($unit[1]).obj"
        if ($LASTEXITCODE -ne 0) { throw 'IMU host compilation failed' }
    }
    & (Join-Path $ClangDir 'lld.exe') -flavor link /dll /noentry /nodefaultlib /export:IMU_Task /export:IMU_GetData /export:IMU_ZeroYaw /export:imu_debug /export:imu_data /out:tests/imu/imu_test.dll tests/imu/imu.obj tests/imu/harness.obj
    if ($LASTEXITCODE -ne 0) { throw 'IMU host link failed' }
    & $PythonExe tests/imu/run_tests.py
    if ($LASTEXITCODE -ne 0) { throw 'IMU tests failed' }
} finally { Pop-Location }
