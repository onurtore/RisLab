@ECHO off
cls

:BEGIN
SET /a turnCount = 3
SET /a perm = 1

SET /p userId= Enter user id: 
SET /p step= Start from step (1--%turnCount%): 
echo Starting from step %step%


if not %step%==1 GOTO START
"instructions\pilot1\instructionsB.ppsx"

:START
@ cd "C:\Documents and Settings\generic\Desktop\server\deney kodu\MazeGame\Release"
echo.
echo START THE EXPERIMENT
pause

if %perm%==1 echo.
GOTO P1
echo No such permutation is defined, please enter again
GOTO BEGIN


:P1
if %step% == 2 GOTO P1S2
@if %step% == 3 GOTO P1S3


echo.
echo.
echo TURN 1 of %turnCount%
pause

@"C:\Program Files\SensAble\PHANTOM Device Drivers\PHANToM Test.exe"
MazeGame %userId% 1 1 %perm%

set /a step=%step%+1

:P1S2
echo.
echo.
echo TURN 2 of %turnCount%
pause

@"C:\Program Files\SensAble\PHANTOM Device Drivers\PHANToM Test.exe"
MazeGame %userId% 1 2 %perm%


set /a step=%step%+1


:P1S3
echo.
echo.
echo TURN 3 of %turnCount%
pause

@"C:\Program Files\SensAble\PHANTOM Device Drivers\PHANToM Test.exe"
MazeGame %userId% 1 3 %perm%



set /a step=%step%+1


:END
echo.
echo THANK YOU FOR YOUR PARTICIPATION
pause
