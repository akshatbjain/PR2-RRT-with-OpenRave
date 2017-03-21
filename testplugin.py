#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/rrtplugin')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    rrtmodule = RaveCreateModule(env,'rrtmodule')
    print rrtmodule.SendCommand('help')
finally:
    RaveDestroy()
