param(
    [string]$ClangDir = 'E:\APP\Matlab25\toolbox\sldrt\clang\win64',
    [string]$PythonExe = 'C:\Users\Administrator\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
)
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Push-Location $taskRoot
try {
    & (Join-Path $ClangDir 'clang.exe') -c -std=c99 -DARM_VISION_TEST_MODE=0 -ffreestanding -fno-stack-protector -include tests/host/mock.h -ICore/Inc -Ican_zdt Core/Src/chassis_debug.c -o tests/host/chassis_test.obj
    if ($LASTEXITCODE -ne 0) { throw 'Chassis source compilation failed' }
    & (Join-Path $ClangDir 'clang.exe') -c -std=c99 -DARM_VISION_TEST_MODE=0 -ffreestanding -fno-stack-protector -include tests/host/mock.h -ICore/Inc -Ican_zdt Core/Src/chassis_motion.c -o tests/host/chassis_motion.obj
    if ($LASTEXITCODE -ne 0) { throw 'Motion source compilation failed' }
    & (Join-Path $ClangDir 'clang.exe') -c -std=c99 -DARM_VISION_TEST_MODE=0 -ffreestanding -fno-stack-protector -include tests/host/mock.h -ICore/Inc tests/host/harness.c -o tests/host/harness.obj
    if ($LASTEXITCODE -ne 0) { throw 'Harness compilation failed' }
    & (Join-Path $ClangDir 'lld.exe') -flavor link /dll /noentry /nodefaultlib /out:tests/host/chassis_test.dll tests/host/chassis_test.obj tests/host/chassis_motion.obj tests/host/harness.obj
    if ($LASTEXITCODE -ne 0) { throw 'Host DLL link failed' }
    & $PythonExe tests/host/run_tests.py
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
} finally {
    Pop-Location
}
