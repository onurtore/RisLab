@ECHO off
cls

SET /p step=Start from step (1--4): 
echo Starting from step %step% 
SET /a turnCount=4


IF NOT %step%==1 GOTO START
"instructions\pilot1\instructionsG.ppsx"

:START
@ cd "C:\Documents and Settings\generic\Desktop\ConnectingClient2\Release"
echo.
echo START THE EXPERIMENT
pause


:LOOP
set /a turnCount = 5
IF %step%==%turnCount% GOTO END
echo.
echo.
set /a turnCount = 4
echo TURN %step% of %turnCount%
pause
@"C:\Program Files\SensAble\PHANTOM Device Drivers\PHANToM Test.exe"
echo PLEASE WAIT FOR THE EXPERIMENTER, DO NOT PRESS ANY KEY
@pause > nul
Client 

set /a step=%step%+1
GOTO LOOP

:END

echo.
echo THANK YOU FOR YOUR PARTICIPATION
pause