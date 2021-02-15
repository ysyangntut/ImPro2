rem 
rem Option 1: Run ImProConsole2.exe with the following inputs: 
rem undistonline 
rem 1
rem Undistort_Fxy_Cxy_P12_K123456_ExternalFrameBfPeaks_Peak00000.xml
rem f
rem originalFiles.txt 
rem f
rem undistortFiles.txt
rem 800 450
rem 10
rem 
rem Option 2: Run ImProConsole2.exe with arguments
rem ImProConsole2 -func=undistonline -calib=Undistort_Fxy_Cxy_P12_K123456_ExternalFrameBfPeaks_Peak00000.xml -src=originalFiles.txt -dst=undistortFiles.txt -imshow_w=800 -imshow_h=450 -imshow_t=10
rem 

rem For help, run ImProConsole2.exe -func=undistonline -help 

ImProConsole2 -func=undistonline -calib=Undistort_Fxy_Cxy_P12_K123456_ExternalFrameBfPeaks_Peak00000.xml -src=originalFiles.txt -dst=undistortFiles.txt -imshow_w=800 -imshow_h=450 -imshow_t=10

