chcp 1251 
set robosd=I:\SOURCETREE\robosd++\content

set prev=%CD%

cd /D "%~dp0"

if not EXIST reference (
	MD   reference 
)

IF EXIST ".\reference\robosd-ref"  RMDIR ".\reference\robosd-ref"
MKLINK /J ".\reference\robosd-ref"  "%robosd%"\source  

cd %prev%
