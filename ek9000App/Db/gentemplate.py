#!/usr/bin/python3
#
# This file is part of the EK9000 device support module. It is subject to 
# the license terms in the LICENSE.txt file found in the top-level directory 
# of this distribution and at: 
#    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
# No part of the EK9000 device support module, including this file, may be 
# copied, modified, propagated, or distributed except according to the terms 
# contained in the LICENSE.txt file.

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
            fs.write("\tfield(DESC,  \"\")\n")
            fs.write("\tfield(DTYP,  \"" + dtyp + "\")\n")
            fs.write("\tfield(ZNAM,  \"low\")\n")
            fs.write("\tfield(ONAM,  \"high\")\n")
            fs.write("\tfield(SCAN,  \".1 second\")\n")
            fs.write("\tfield(PINI,  \"YES\")\n")
            fs.write("}\n\n")			
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
            fs.write("\tfield(DESC,  \"\")\n")
            fs.write("\tfield(DTYP,  \"" + dtyp + "\")\n")
            fs.write("\tfield(ZNAM,  \"low\")\n")
            fs.write("\tfield(ONAM,  \"high\")\n")
            fs.write("\tfield(PINI,  \"YES\")\n")
            fs.write("}\n\n")	
            #subs.write("\t\tDESC" + str(num) + ",\t")
            #subs.write("ZNAM" + str(num) + ",\t")
            #subs.write("ONAM" + str(num) + ",\t")
            #if not num == i:subs.write("PINI" + str(num) + ",\t")
            #else:
            #	subs.write("PINI" + str(num) + "\t")
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
            fs.write("record(ai,\"$(TERMINAL):" + str(num) +"\")\n{\n")
            fs.write("\tfield(DESC,  \"\")\n")
            fs.write("\tfield(DTYP,  \"" + dtyp + "\")\n")
            fs.write("\tfield(SCAN,  \".1 second\")\n")
            fs.write("\tfield(EGU,   \"Volts\")\n")
            fs.write("\tfield(LINR,  \"LINEAR\")\n")
            fs.write("\tfield(PINI,  \"YES\")\n")
            fs.write("}\n\n")
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
            fs.write("record(ao,\"$(TERMINAL):" + str(num) +"\")\n{\n")
            fs.write("\tfield(DESC,  \"\")\n")
            fs.write("\tfield(DTYP,  \"" + dtyp + "\")\n")
            fs.write("\tfield(SCAN,  \".1 second\")\n")
            fs.write("\tfield(LINR,  \"LINEAR\")\n")
            fs.write("\tfield(PINI,  \"YES\")\n")
            fs.write("}\n\n")
            num = num + 1
            subs.write("\n")
            subs.write("\t}\n\t{\n\t}\n}\n")
    subs.flush()
    subs.close()


# EL1XXX stuff 
make_bi("EL1001", 1, "EL10XX")
make_bi("EL1002", 2, "EL10XX")
make_bi("EL1004", 4, "EL10XX")
make_bi("EL1008", 8, "EL10XX")
make_bi("EL1012", 2, "EL10XX")
make_bi("EL1014", 4, "EL10XX")
make_bi("EL1018", 8, "EL10XX")
make_bi("EL1024", 4, "EL10XX")
make_bi("EL1034", 4, "EL10XX")
make_bi("EL1084", 4, "EL10XX")
make_bi("EL1088", 8, "EL10XX")
make_bi("EL1094", 4, "EL10XX")
make_bi("EL1098", 8, "EL10XX")
make_bi("EL1104", 4, "EL10XX")
make_bi("EL1114", 4, "EL10XX")
# Documentation does NOT say if these are supported by the ek9000
make_bi("EL1124", 4, "EL10XX")
make_bi("EL1134", 4, "EL10XX")
make_bi("EL1144", 4, "EL10XX")
# these are OK
make_bi("EL1202", 2, "EL10XX")
make_bi("EL1382", 2, "EL10XX")
make_bi("EL1702", 2, "EL10XX")
make_bi("EL1712", 2, "EL10XX")
make_bi("EL1722", 2, "EL10XX")
make_bi("EL1804", 4, "EL10XX")
make_bi("EL1808", 8, "EL10XX")
make_bi("EL1809", 16, "EL10XX")
make_bi("EL1814", 4, "EL10XX")
make_bi("EL1819", 16, "EL10XX")
make_bi("EL1184", 4, "EL10XX")


# EL2XXX stuff
make_bo("EL2001", 1, "EL20XX")
make_bo("EL2002", 2, "EL20XX")
make_bo("EL2004", 4, "EL20XX")
make_bo("EL2008", 8, "EL20XX")
make_bo("EL2022", 2, "EL20XX")
make_bo("EL2024", 4, "EL20XX")
make_bo("EL2042", 2, "EL20XX")
make_bo("EL2084", 4, "EL20XX")
make_bo("EL2088", 8, "EL20XX")
make_bo("EL2124", 4, "EL20XX")

# EL4XXX stuff
make_ao("EL4001", 1, "EL40XX")
make_ao("EL4002", 2, "EL40XX")
make_ao("EL4004", 4, "EL40XX")
make_ao("EL4008", 8, "EL40XX")
make_ao("EL4031", 1, "EL40XX")
make_ao("EL4032", 2, "EL40XX")
make_ao("EL4034", 4, "EL40XX")
make_ao("EL4038", 8, "EL40XX")
make_ao("EL4011", 1, "EL40XX")
make_ao("EL4012", 2, "EL40XX")
make_ao("EL4014", 4, "EL40XX")
make_ao("EL4018", 8, "EL40XX")
make_ao("EL4021", 1, "EL40XX")
make_ao("EL4022", 2, "EL40XX")
make_ao("EL4024", 4, "EL40XX")
make_ao("EL4028", 8, "EL40XX")
# Uncomment once we finish EL41XX support
#make_ao("EL4102", 2, "EL41XX")
#make_ao("EL4104", 4, "EL41XX")
#make_ao("EL4132", 2, "EL41XX")
#make_ao("EL4134", 4, "EL41XX")
#make_ao("EL4112", 2, "EL41XX")
#make_ao("EL4114", 4, "EL41XX")
#make_ao("EL4122", 2, "EL41XX")
#make_ao("EL4124", 4, "EL41XX")


# EL3XXX stuff
make_ai("EL3001", 1, "EL30XX")
make_ai("EL3002", 2, "EL30XX")
make_ai("EL3004", 4, "EL30XX")
make_ai("EL3008", 8, "EL30XX")
make_ai("EL3012", 2, "EL31XX")
make_ai("EL3014", 4, "EL30XX")
make_ai("EL3021", 1, "EL30XX")
make_ai("EL3022", 2, "EL30XX")
make_ai("EL3024", 4, "EL30XX")
make_ai("EL3041", 1, "EL30XX")
make_ai("EL3042", 2, "EL30XX")
make_ai("EL3044", 4, "EL30XX")
make_ai("EL3048", 8, "EL30XX")
make_ai("EL3051", 1, "EL30XX")
make_ai("EL3052", 2, "EL30XX")
make_ai("EL3054", 4, "EL30XX")
make_ai("EL3058", 8, "EL30XX")
make_ai("EL3061", 1, "EL30XX")
make_ai("EL3062", 2, "EL30XX")
make_ai("EL3064", 4, "EL30XX")
make_ai("EL3068", 8, "EL30XX")
make_ai("EL3101", 1, "EL31XX")
make_ai("EL3102", 2, "EL31XX")
make_ai("EL3104", 4, "EL31XX")
make_ai("EL3111", 1, "EL31XX")
make_ai("EL3112", 2, "EL31XX")
make_ai("EL3114", 4, "EL31XX")
make_ai("EL3121", 1, "EL31XX")
make_ai("EL3122", 2, "EL31XX")
make_ai("EL3124", 4, "EL31XX")
make_ai("EL3141", 1, "EL31XX")
make_ai("EL3142", 2, "EL31XX")
make_ai("EL3144", 4, "EL31XX")
make_ai("EL3151", 1, "EL31XX")
make_ai("EL3152", 2, "EL31XX")
make_ai("EL3154", 4, "EL31XX")
make_ai("EL3161", 1, "EL31XX")
make_ai("EL3162", 2, "EL31XX")
make_ai("EL3164", 4, "EL31XX")
make_ai("EL3174", 4, "EL31XX")
make_ai("EL3202", 2, "EL30XX")
make_ai("EL3201", 2, "EL30XX")
make_ai("EL3204", 4, "EL30XX")
make_ai("EL3208", 8, "EL30XX")

