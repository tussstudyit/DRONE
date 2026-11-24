@echo off
chcp 65001 >nul
echo ========================================================
echo   Đang biên dịch project DRONE OpenGL (C++11)...
echo ========================================================

set GXX="C:\Program Files (x86)\Dev-Cpp\MinGW64\bin\g++.exe"
if not exist %GXX% (
    set GXX=g++
)

%GXX% -std=c++11 -O2 main.cpp -o main.exe -Iinclude -L"C:\Program Files (x86)\Dev-Cpp\MinGW64\lib" -lfreeglut -lopengl32 -lglu32

if %ERRORLEVEL% EQU 0 (
    echo [THÀNH CÔNG] Đã tạo file main.exe thành công!
) else (
    echo [LỖI] Quá trình biên dịch thất bại!
)
