#!/usr/bin/python
from os import path
import sys
import click
from os.path import join, realpath
Import('env')

click.secho("Robus network build configuration:", underline=True)
# Find the appropriate hal
find_HAL = False
envdefs = env['CPPDEFINES'].copy()
for item in envdefs:
    if isinstance(item, tuple) and item[0] == "LUOSHAL":
        find_HAL = True
        if (path.exists("HAL/" + item[1])):
            click.secho(
                "\t* %s HAL selected for Robus network." % item[1], fg="green")
            click.secho("")
            if (path.exists("HAL/" + item[1] + "/hal_script.py")):
                # This is an extra script dedicated to this HAL, run it
                hal_script_path = realpath("HAL/" + item[1] + "/hal_script.py")
                env.SConscript(hal_script_path, exports="env")

        else:
            click.secho("\t* %s Robus network HAL not found" %
                        item[1], fg="red")
            click.secho("")

        env.Append(CPPPATH=[realpath("HAL/" + item[1])])
        env.Append(CPPPATH=[realpath("inc/")])
        env.Append(CPPPATH=[realpath(".")])
        env.Append(
            SRC_FILTER=["+<*.c>, +<../HAL/%s/*.c>, ++<../HAL/%s/*.cpp>" % (item[1], item[1])])
