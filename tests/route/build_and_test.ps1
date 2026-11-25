param(
    [string]$ClangDir = 'E:\APP\Matlab25\toolbox\sldrt\clang\win64',
    [string]$PythonExe = 'C:\Users\Administrator\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
)
$ErrorActionPreference='Stop'
Push-Location (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
try {
    $objects=@()
    foreach($source in @('Core/Src/route.c','Core/Src/route_uart.c','Core/Src/route_keys.c','tests/route/harness.c')) {
        $obj='tests/route/'+[IO.Path]::GetFileNameWithoutExtension($source)+'.obj'
        & (Join-Path $ClangDir 'clang.exe') -c -std=c99 -DARM_VISION_TEST_MODE=0 -Wall -Wextra -Werror -ffreestanding -fno-stack-protector -include tests/route/mock.h -ICore/Inc -Iimu $source -o $obj
        if($LASTEXITCODE -ne 0) { throw "Route host compilation failed: $source" }
        $objects+=$obj
    }
    & (Join-Path $ClangDir 'lld.exe') -flavor link /dll /noentry /nodefaultlib /out:tests/route/route_test.dll $objects
    if($LASTEXITCODE -ne 0) { throw 'Route host link failed' }
    & $PythonExe tests/route/run_tests.py
    if($LASTEXITCODE -ne 0) { throw 'Route tests failed' }
} finally { Pop-Location }
