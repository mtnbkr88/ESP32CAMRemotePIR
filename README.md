# ESP32-CAM_Remote_PIR

ESP32-CAM Remote PIR Solar Powered Picture/Video Emailer

04/28/2022 Ed Williams 

This ESP32-CAM Remote PIR can work independent or in conjunction with the ESP32-CAM Video 
Recorder I also posted in github. This ESP32-CAM Remote PIR has the following capabilities:
  - Wake from PIR and do the following:
    - Optionally send a remote trigger to an ESP32-CAM Video Recorder with the following options:
      - Take a picture.
      - Take a picture and email it.
      - Make a video.
      - Make a video and send an email with name of video.   
    - Optionally email up to 6 pictures or a 10 second AVI video.
    - Go to deep sleep.
  - Wake from a time trigger and do the following:
    - Optionally email one picture.
    - Run a webserver for 10 minutes with the following options:
      - Refresh image.
      - Start/Stop a video stream
      - Email a picture or a 10 second video.
      - Access settings.
  - Firmware updates Over-The-Air.
  - No SD card required.
  - Solar powered using a single 18650 battery per the attached schematic.
  - Battery voltage indicator is connected to GPIO12.
  - PIR motion sensor is connected to GPIO13.
  - Pictures are JPG and videos are AVI.
  - Uses a simple email utility I enhanced to send buffers or files which embedded in this
    sketch and is also now found on github. 

See ECRP_Schematic.jpg for a picture of the wiring.

Used pinMode(GPIO_NUM_13, INPUT_PULLDOWN) in the setup to pull the pin low. Now when 
motion is detected, GPIO13 will be pulled high, otherwise it will be low.

ECRP_Main.jpg shows the main screen for the ESP32-CAM Remote PIR web site.

ECRP_Settings.jpg shows the settings page.

I 3D printed an enclosure and bracket for holding the solar panel. ECRP_Enclosure.jpg 
shows the completed project.

The Default Partition Scheme must be used for compiling so OTA can work.
