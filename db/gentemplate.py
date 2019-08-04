#
# Horrible script to auto generate templates and sub files
#
def make_bi(name, i, dtyp):
	num = 1
	subs = open(name + ".substitutions", "w+")
	with open(name + ".template", "w+") as fs:			
		fs.write("#\n# " + name + " template.\n#\n\n")
		subs.write("file " + name + ".template\n{ \tpattern \n\t{\n")
		subs.write("\t\tTERMINAL,\t\n")
		while num <= i:
			fs.write("record(bi,\"$(TERMINAL):" + str(num) +"\")\n{\n")
			fs.write("\tfield(DESC,  \"$(DESC" + str(num) + ")\")\n")
			fs.write("\tfield(DTYP,  \"" + dtyp + "\")\n")
			fs.write("\tfield(ZNAM,  \"$(ZNAM" + str(num) + ")\")\n")
			fs.write("\tfield(ONAM,  \"$(ONAM" + str(num) + ")\")\n")
			fs.write("\tfield(ZSV,   \"$(ZSV" + str(num) + ")\")\n")
			fs.write("\tfield(OSV,   \"$(OSV" + str(num) + ")\")\n")
			fs.write("\tfield(COSV,  \"$(COSV" + str(num) + ")\")\n")
			fs.write("}\n\n")			
			subs.write("\t\tDESC" + str(num) + ",\t")
			subs.write("ZNAM" + str(num) + ",\t")
			subs.write("ONAM" + str(num) + ",\t")
			subs.write("ZSV" + str(num) + ",\t")
			subs.write("OSV" + str(num) + ",\t")
			if not num == i:
				subs.write("COSV" + str(num) + ",\t")
			else:
				subs.write("COSV" + str(num) + "\t")
			subs.write("\n")
			num = num + 1
		subs.write("\t}\n\t{\n\t}\n}\n")
	subs.flush()
	subs.close()
			
def make_bo(name, i, dtyp):
	num = 1
	subs = open(name + ".substitutions", "w+")
	with open(name + ".template", "w+") as fs:			
		fs.write("#\n# " + name + " template.\n#\n\n")
		subs.write("file " + name + ".template\n{ \tpattern \n\t{\n")
		subs.write("\t\tTERMINAL,\t\n")
		while num <= i:
			fs.write("record(bo,\"$(TERMINAL):" + str(num) +"\")\n{\n")
			fs.write("\tfield(DESC,  \"$(DESC" + str(num) + ")\")\n")
			fs.write("\tfield(DTYP,  \"" + dtyp + "\")\n")
			fs.write("\tfield(ZNAM,  \"$(ZNAM" + str(num) + ")\")\n")
			fs.write("\tfield(ONAM,  \"$(ONAM" + str(num) + ")\")\n")
			fs.write("\tfield(PINI,  \"$(PINI" + str(num) + ")\")\n")
			fs.write("}\n\n")	
			subs.write("\t\tDESC" + str(num) + ",\t")
			subs.write("ZNAM" + str(num) + ",\t")
			subs.write("ONAM" + str(num) + ",\t")
			if not num == i:
				subs.write("PINI" + str(num) + ",\t")
			else:
				subs.write("PINI" + str(num) + "\t")
			num = num + 1
			subs.write("\n")
		subs.write("\t}\n\t{\n\t}\n}\n")
	subs.flush()
	subs.close()
	
	
	
def make_ai(name, i, dtyp):
	num = 1
	subs = open(name + ".substitutions", "w+")
	with open(name + ".template", "w+") as fs:			
		fs.write("#\n# " + name + " template.\n#\n\n")
		subs.write("file " + name + ".template\n{ \tpattern \n\t{\n")
		subs.write("\t\tTERMINAL,\t\n")
		while num <= i:
			fs.write("record(bo,\"$(TERMINAL):" + str(num) +"\")\n{\n")
			fs.write("\tfield(DESC,  \"$(DESC" + str(num) + ")\")\n")
			fs.write("\tfield(DTYP,  \"" + dtyp + "\")\n")
			fs.write("\tfield(SCAN,  \"$(SCAN" + str(num) + ")\")\n")
			fs.write("\tfield(ADEL,  \"$(ADEL" + str(num) + ")\")\n")
			fs.write("\tfield(EGUF,  \"$(EGUF" + str(num) + ")\")\n")
			fs.write("\tfield(EGUL,  \"$(EGUL" + str(num) + ")\")\n")
			fs.write("\tfield(HIGH,  \"$(HIGH" + str(num) + ")\")\n")
			fs.write("\tfield(HIHI,  \"$(HIHI" + str(num) + ")\")\n")
			fs.write("\tfield(HOPR,  \"$(HOPR" + str(num) + ")\")\n")
			fs.write("\tfield(HYST,  \"$(HYST" + str(num) + ")\")\n")
			fs.write("\tfield(LOLO,  \"$(LOLO" + str(num) + ")\")\n")
			fs.write("\tfield(LOPR,  \"$(LOPR" + str(num) + ")\")\n")
			fs.write("\tfield(LOW,   \"$(LOW" + str(num) + ")\")\n")
			fs.write("\tfield(MDEL,  \"$(MDEL" + str(num) + ")\")\n")
			fs.write("\tfield(SMOO,  \"$(SMOO" + str(num) + ")\")\n")
			fs.write("\tfield(UNIT,  \"$(UNIT" + str(num) + ")\")\n")
			fs.write("\tfield(LINR,  \"LINEAR\")\n")
			fs.write("}\n\n")
			subs.write("\t\tDESC" + str(num) + ",\t")
			subs.write("SCAN" + str(num) + ",\t")
			subs.write("ADEL" + str(num) + ",\t")
			subs.write("EGUF" + str(num) + ",\t")
			subs.write("EGUL" + str(num) + ",\t")
			subs.write("HIGH" + str(num) + ",\t")
			subs.write("HIHI" + str(num) + ",\t")
			subs.write("HOPR" + str(num) + ",\t")
			subs.write("HYST" + str(num) + ",\t")
			subs.write("LOLO" + str(num) + ",\t")
			subs.write("LOPR" + str(num) + ",\t")
			subs.write("LOW" + str(num) + ",\t")
			subs.write("MDEL" + str(num) + ",\t")
			subs.write("SMOO" + str(num) + ",\t")
			if not num == i:
				subs.write("UNIT" + str(num) + ",\t")
			else:
				subs.write("UNIT" + str(num) + "\t")
			num = num + 1
			subs.write("\n")
		subs.write("\t}\n\t{\n\t}\n}\n")
	subs.flush()
	subs.close()
			
def make_ao(name, i, dtyp):
	num = 1
	subs = open(name + ".substitutions", "w+")
	with open(name + ".template", "w+") as fs:			
		fs.write("#\n# " + name + " template.\n#\n\n")
		subs.write("file " + name + ".template\n{ \tpattern \n\t{\n")
		subs.write("\t\tTERMINAL,\t\n")
		while num <= i:
			fs.write("record(bo,\"$(TERMINAL):" + str(num) +"\")\n{\n")
			fs.write("\tfield(DESC,  \"$(DESC" + str(num) + ")\")\n")
			fs.write("\tfield(DTYP,  \"" + dtyp + "\")\n")
			fs.write("\tfield(SCAN,  \"$(SCAN" + str(num) + ")\")\n")
			fs.write("\tfield(ADEL,  \"$(ADEL" + str(num) + ")\")\n")
			fs.write("\tfield(EGUF,  \"$(EGUF" + str(num) + ")\")\n")
			fs.write("\tfield(EGUL,  \"$(EGUL" + str(num) + ")\")\n")
			fs.write("\tfield(DRVH,  \"$(DRVH" + str(num) + ")\")\n")
			fs.write("\tfield(DRVL,  \"$(DRVL" + str(num) + ")\")\n")
			fs.write("\tfield(HOPR,  \"$(HOPR" + str(num) + ")\")\n")
			fs.write("\tfield(LOPR,  \"$(LOPR" + str(num) + ")\")\n")
			fs.write("\tfield(LINR,  \"LINEAR\")\n")
			fs.write("\tfield(PINI,  \"$(PINI" + str(num) + ")\")\n")
			fs.write("}\n\n")
			subs.write("\t\tDESC" + str(num) + ",\t")
			subs.write("SCAN" + str(num) + ",\t")
			subs.write("ADEL" + str(num) + ",\t")
			subs.write("EGUF" + str(num) + ",\t")
			subs.write("EGUL" + str(num) + ",\t")
			subs.write("DRVH" + str(num) + ",\t")
			subs.write("DRVL" + str(num) + ",\t")
			subs.write("HOPR" + str(num) + ",\t")
			subs.write("LOPR" + str(num) + ",\t")
			subs.write("LOLO" + str(num) + ",\t")
			subs.write("LOPR" + str(num) + ",\t")
			if not num == i:
				subs.write("PINI" + str(num) + ",\t")
			else:
				subs.write("PINI" + str(num) + "\t")
			num = num + 1
			subs.write("\n")
		subs.write("\t}\n\t{\n\t}\n}\n")
	subs.flush()
	subs.close()

			
make_bo("EL2001", 1, "EL20XX")
make_bo("EL2002", 2, "EL20XX")
make_bo("EL2004", 4, "EL20XX")
make_bo("EL2008", 8, "EL20XX")
make_bi("EL1124", 4, "EL10XX")
make_ao("EL4001", 1, "EL40XX")
make_ao("EL4002", 2, "EL40XX")
make_ao("EL4004", 4, "EL40XX")
make_ai("EL3064", 4, "EL30XX")
make_ai("EL3054", 4, "EL30XX")
make_ai("EL3052", 4, "EL30XX")
make_ai("EL3062", 4, "EL30XX")
