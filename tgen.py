#!/usr/bin/python3
import os, sys, io, math
import xml.etree.ElementTree as etree

def write_comment(fs, comment):
	fs.write("//===============================================//\n")
	fs.write("// " + comment + "\n")
	fs.write("//===============================================//\n")

def do_stuff(file, outfile):
	root = etree.parse(file).getroot()
	with open(file, "r") as fs:
		# find all devices
		did = False
		devices = root.findall("./Descriptions/Devices/")
		for dev in devices:
			name = dev.find("Type").text
			rxpdo = dev.findall("RxPdo")
			txpdo = (dev.findall("TxPdo"))
			for pdo in txpdo:
				print("Name: " + pdo.find("Name").text)
				entries = pdo.findall("Entry")
				total = 0
				for entry in entries:
					bitlen = entry.find("BitLen").text
					print("Bitlen: " + bitlen)
					total = total + int(bitlen)
				print("Total RxPdo (bits): " + str(total))
				print("Total RxPdo (bytes): " + str(math.ceil(total/8)))
				print("\n")
			

do_stuff("el30xx.xml", "e")
