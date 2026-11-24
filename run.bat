@echo off
if not exist main.exe (
    call build.bat
)
if exist main.exe (
    start main.exe
)
