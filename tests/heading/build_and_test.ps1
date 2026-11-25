param(
    [string]$ClangDir = 'E:\APP\Matlab25\toolbox\sldrt\clang\win64',
    [string]$PythonExe = 'C:\Users\Administrator\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
)
$ErrorActionPreference='Stop'
Push-Location (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
try {
    $sources=@('Core/Src/chassis_heading.c','Core/Src/chassis_path.c','Core/Src/route.c','Core/Src/chassis_motion.c','Core/Src/chassis_debug.c','imu/imu.c','tests/heading/harness.c')
    $objects=@()
    foreach($source in $sources) {
        $obj='tests/heading/'+[IO.Path]::GetFileNameWithoutExtension($source)+'.obj'
        & (Join-Path $ClangDir 'clang.exe') -c -std=c99 -DARM_VISION_TEST_MODE=0 -ffreestanding -fno-stack-protector -include tests/heading/mock.h -ICore/Inc -Ican_zdt -Iimu $source -o $obj
        if($LASTEXITCODE -ne 0) { throw "Heading host compilation failed: $source" }
        $objects+=$obj
    }
    & (Join-Path $ClangDir 'lld.exe') -flavor link /dll /noentry /nodefaultlib /out:tests/heading/heading_test.dll $objects
    if($LASTEXITCODE -ne 0) { throw 'Heading host link failed' }
    & $PythonExe tests/heading/run_tests.py
    if($LASTEXITCODE -ne 0) { throw 'Heading tests failed' }
    & $PythonExe tests/heading/run_continuous.py
    if($LASTEXITCODE -ne 0) { throw 'Continuous route tests failed' }
} finally { Pop-Location }
