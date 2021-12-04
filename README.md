# Rtmaps_Autopilot_Project

Please use carla-0.9.11-py3.7

****************************************************************************************************
Before opening AMI_Project.rtd, open CarlaUE4.exe with low quality level 

Change the directory in command window to the file where your Carla is installed for example :
C:\Users\XXX XXX\Desktop\CARLA_0.9.11\WindowsNoEditor

in Windows use :
CarlaUE4.exe -quality-level=Low
or in Linux use :
./CarlaUE4.sh -quality-level=Low

****************************************************************************************************
In /python_script
/Rtmaps_Control_AMI.py

Change line 13 the installation path of your Carla for example :
CARLA_PYTHON_DIRECTORY = "C:/Users/XXX XXX/Desktop/CARLA_0.9.11/WindowsNoEditor/PythonAPI"

In /python_script
/Location_Record_AMI.py
Change line 15 the path where the reference trajectory needs to be recorded :
C:/Users/XXX XXX/Desktop/RtMapsGit/Rtmaps_Autopilot_Project/AMI_Project/data/record_data.xlsx

****************************************************************************************************
To configure your python-bridge
Use command	where python	in your command window to find your python 3.7 installation path for example :
C:\Users\XXX XXX\AppData\Local\Programs\Python\Python37\python.exe

Change all the	 \   to	  /   and remove the "python.exe" at the end. That's what you should have :
C:/Users/XXX XXX/AppData/Local/Programs/Python/Python37/

Copy this path, then go to your Rtmaps python-bridge package file, normally the path to the file is :
C:\Program Files\Intempora\RTMaps 4\packages\rtmaps_python_bridge

Create a file named	rtmaps_python_bridge.conf
Paste the path in it and now it's time to run Rtmaps.

****************************************************************************************************


