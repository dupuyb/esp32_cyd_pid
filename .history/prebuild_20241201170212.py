################################
# LVGL config loader
################################
# Put this file on the root of your PlatformIO LVGL project
# and add the next line on your env board in the platformio.ini file.
#
# extra_scripts = pre:prebuild.py
#
# The user_conf.h file should be placed in the root lib folder
################################

import os.path
import shutil
from platformio import util
from SCons.Script import DefaultEnvironment

try:
    import configparser
except ImportError:
    import ConfigParser as configparser

# get platformio environment variables
env = DefaultEnvironment()
flavor = env.get("PIOENV")

# copy my config for TFT_eSPI Lib
output_path =  ".pio/libdeps/" + flavor + "/TFT_eSPI/"
os.makedirs(output_path, 0o755, True)
shutil.copy("include/User_Setup.h", output_path)

# copy my config for lvgl Lib
output_path =  ".pio/libdeps/" + flavor +"/lvgl/"
os.makedirs(output_path, 0o755, True)
shutil.copy("include/lv_conf.h", output_path)