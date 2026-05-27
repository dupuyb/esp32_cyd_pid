################################
# LVGL config loader
# @hpsaturn 2022
################################
# Put this file on the root of your PlatformIO LVGL project
# and add the next line on your env board in the platformio.ini file.
#
# extra_scripts = pre:prebuild.py
#
# The lv_conf.h file should be placed in the root lib folder
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

# copy my config for lvgl Lib
output_path =  ".pio/libdeps/" + flavor 
os.makedirs(output_path, 0o755, True)
shutil.copy("lib/lv_conf.h", output_path)

# change lv_gif.c/h
lv_gif_path = ".pio/libdeps/esp32-s3-devkitc-1/lvgl/src/extra/libs/gif/"
os.makedirs(lv_gif_path, 0o755, True)
shutil.copy("lib/lv_gif.c", lv_gif_path)
shutil.copy("lib/lv_gif.h", lv_gif_path)