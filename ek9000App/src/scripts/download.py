#!/usr/bin/env python

# Autogeneration of CoE params via config file
import os
import sys
import urllib
import urllib.request
import zipfile

cfgfiles = "https://download.beckhoff.com/download/configuration-files/io/ethercat/xml-device-description/Beckhoff_EtherCAT_XML.zip"

extractablefiles = [
    "Beckhoff EL1xxx.xml",
    "Beckhoff EL2xxx.xml",
    "Beckhoff EL30xx.xml",
    "Beckhoff EL31xx.xml",
    "Beckhoff EL32xx.xml",
    "Beckhoff EL33xx.xml",
    "Beckhoff EL34xx.xml",
    "Beckhoff EL37xx.xml",
    "Beckhoff EL3xxx-0030.xml",
    "Beckhoff EL3xxx.xml",
    "Beckhoff EL4xxx.xml",
    "Beckhoff EL5xxx.xml",
    "Beckhoff EL7xxx.xml",
    "Beckhoff EL72xx.xml",
    "Beckhoff EL73xx.xml",
]

if os.path.exists("BhcConfigFiles/"):
    print("BhcConfigFiles already exists; delete it to redownload.")
    sys.exit(0)


if not os.path.exists("BhcConfigFiles.zip"):
    print("BhcConfigFiles.zip doesn't exist, downloading...")
    urllib.request.urlretrieve(cfgfiles, "./BhcConfigFiles.zip")

print("Extracting configs...")
with zipfile.ZipFile("BhcConfigFiles.zip", "r") as _zipfile:
    os.mkdir("BhcConfigFiles/")
    _zipfile.extractall(path="BhcConfigFiles/", members=extractablefiles)
os.remove("./BhcConfigFiles.zip")
if os.path.exists("./Beckhoff AX5xxx"):
    os.remove("./Beckhoff AX5xxx")
