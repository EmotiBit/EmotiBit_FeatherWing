Import("env")
import os

#print(env.Dump()) # uncomment this to print all variables in pio env
firmware_basename = os.path.basename(os.getcwd())
env.Replace(PROGNAME=firmware_basename + ".%s" % env.GetProjectOption("firmware_name_board_name"))