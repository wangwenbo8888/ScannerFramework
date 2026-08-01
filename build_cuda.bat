@echo off
:: build_cuda.bat — CUDA 构建包装脚本
:: 解决 CMake 3.31 + VS2022 + CUDA 12.6 的两个兼容问题:
::   1. vcxproj 注入 /MP /W4 /arch:SSE2 到 nvcc（nvcc 不识别）
::   2. Qt RCC 无法处理中文文件名
::
:: 用法: build_cuda.bat [Release|Debug]

setlocal enabledelayedexpansion
set "SRC=E:\workfold\framework"
set "BUILD=%SRC%\build"
set "CONFIG=%1"
if "%CONFIG%"=="" set CONFIG=Release

set "MSBUILD=C:\Program Files\Microsoft Visual Studio\2022\Enterprise\MSBuild\Current\Bin\MSBuild.exe"
set "RCC=C:\Qt\Qt5.15.2\5.15.2\msvc2019_64\bin\rcc.exe"
set "OP_VCXPROJ=%BUILD%\modules\09_operatorlib\mod_operatorlib.vcxproj"
set "APP_VCXPROJ=%BUILD%\app\scan_demo.vcxproj"

echo === Step 1: CMake Configure ===
cmake -S "%SRC%" -B "%BUILD%" -DENABLE_CUDA=ON --no-warn-unused-cli
if errorlevel 1 exit /b 1

echo === Step 2: Generate RCC output manually ===
set "RCC_DIR=%BUILD%\app\scan_demo_autogen\EWIEGA46WW"
if not exist "%RCC_DIR%" mkdir "%RCC_DIR%"
"%RCC%" -name resources "%SRC%\app\resources.qrc" -o "%RCC_DIR%\qrc_resources.cpp"
if errorlevel 1 (
    echo RCC failed
    exit /b 1
)

echo === Step 3: Patch vcxproj files ===
powershell -Command ^
  "$f1='%OP_VCXPROJ%'; $c=[IO.File]::ReadAllText($f1); $c=$c -replace ' /W4 /MP /arch:SSE2',''; [IO.File]::WriteAllText($f1,$c);" ^
  "$f2='%APP_VCXPROJ%'; $c=[IO.File]::ReadAllText($f2); $c=$c -replace '(?s)(<CustomBuild Include=\"[^\"]*qrc_resources[^\"]*\">.*?</CustomBuild>)','<!-- RCC skipped -->'; [IO.File]::WriteAllText($f2,$c);"

echo === Step 4: Build mod_operatorlib (CUDA) ===
"%MSBUILD%" "%OP_VCXPROJ%" /p:Configuration=%CONFIG% /p:Platform=x64 /m
if errorlevel 1 (
    echo mod_operatorlib build FAILED
    exit /b 1
)

echo === Step 5: Build scan_demo ===
"%MSBUILD%" "%APP_VCXPROJ%" /p:Configuration=%CONFIG% /p:Platform=x64 /m
if errorlevel 1 (
    echo scan_demo build FAILED
    exit /b 1
)

echo.
echo === BUILD SUCCESS ===
echo Output: %BUILD%\bin\%CONFIG%\scan_demo.exe
endlocal
