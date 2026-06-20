@echo off

echo path_now=%cd%
rem 文件替换(存在则替换)
rem ====================================
copy /Y QMI8658.c             ..\Hiwonder\Peripherals\
copy /Y QMI8658.h             ..\Hiwonder\Peripherals\
copy /Y QMI8658reg.h          ..\Hiwonder\Peripherals\
copy /Y stm32f4xx_it.c        ..\Core\Src\
copy /Y syscall.c             ..\Hiwonder\Misc\
copy /Y usbh_hiwonder_hid.c   ..\Hiwonder\USB_HOST\
copy /Y imu_porting.c         ..\Hiwonder\Portings\
copy /Y packet_porting.c      ..\Hiwonder\Portings\
copy /Y packet_reports.h      ..\Hiwonder\Portings\
copy /Y chassis_porting.c     ..\Hiwonder\Portings\
copy /Y ackermann_chassis.c   ..\Hiwonder\Chassis\
copy /Y ackermann_chassis.h   ..\Hiwonder\Chassis\
copy /Y chassis.h             ..\Hiwonder\Chassis\
rem ====================================

rem 调用脚本,该脚本自动修复每次cubeMX重新生成工程后freertos编译源文件丢失的bug
rem =====================================
cd ..
echo path_now=%cd%
call armclang.bat
rem =====================================

echo Script is finished, press any key to exit!

rem 等待用户按下任意键
pause