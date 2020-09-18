import os, sys, io

#------------------------------------------------#
# A C header/source file
#------------------------------------------------#
class Header:
    name = ""
    structs = list()
    arrs = list()
    variables = list()
    defines = dict()
    in_struct = False
    standard = 0
    conditionals = list()

    class Standard:
        CPP98=0
        CPP03=1
        CPP11=2
        CPP14=3
        CPP17=4
        C89=5
        C99=6
        C11=7

    class StructAttribute: 
        Packed = 0
        

    def __init__(self, name: str, std=Standard.CPP11):
        self.name = name
        self.structs = list()
        self.arrs = list()
        self.variables = list()
        self.defines = dict()
        self.in_struct = False
        self.conditionals = list()
        if not name.endswith(".h"):
            name += ".h"
        self.fs = open(name, "w+")

    def __del__(self):
        self.fs.flush()
        self.fs.close()

    def set_standard(self, std: int):
        self.standard = std

    def get_standard(self) -> int:
        return self.standard

    def include_std(self, header):
        self.fs.write("#include <" + header + ">\n")
    
    def include(self, header: str):
        self.fs.write("#include \"" + header + "\"\n")

    def add_comment(self, text: str):
        self.fs.write("/* " + text + " */\n")

    def add_block_comment(self, text: str):
        lines = text.split("\n")
        self.fs.write("\n//=========================================================//\n")
        for line in lines:
            self.fs.write("// " + line + "\n")
        self.fs.write("//=========================================================//\n")

    def add_define(self, name, val):
        self.fs.write("#define\t" + name + "\t" + val + "\n")

    def begin_conditional_block(self, cond: str):
        self.fs.write("#if " + cond + "\n")
        self.conditionals.append(cond)

    def add_conditional_elif(self, cond: str):
        self.fs.write("#elif " + cond + "\n")

    def add_conditional_else(self, cond: str):
        self.fs.write("#else\n")

    def end_conditional_block(self, cond: str):
        self.fs.write("#endif /* " + str(self.conditionals.pop()) + " */\n")

    def add_array_variable(self, name: str, basetype: str, vars: list, const=False, static=False):
        if static:
            self.fs.write("static ")
        if const:
            self.fs.write("const ")
        if self.in_struct:
            self.fs.write("\t")
        self.fs.write(basetype + " " + name + "[" + str(len(vars)) + "] = {\n")
        for var in vars:
            self.fs.write("\t" + var + ",\n")
        self.fs.write("};\n")
    
    def add_variable(self, name: str, type: str, val: str = ""):
        if self.in_struct:
            self.fs.write("\t")
        if val.isspace() or val is "":
            self.fs.write(type + " " + name + ";\n")
        else:
            self.fs.write(type + " " + name + " = " + val + ";\n")

    def add_auto_variable(self, name: str, val: str):
        if self.in_struct:
            self.fs.write("\t")
        self.fs.write(type + " " + name + " = " + val + ";\n")

    def add_init_struct(self, struct: str, name: str, *vals, const=False, static=False):
        if static:
            self.fs.write("static ")
        if const:
            self.fs.write("const ")
        self.fs.write(struct + " " + name + " = {\n")
        for v in vals:
            self.fs.write("\t" + str(v) + ",\n")
        self.fs.write("};\n")

    def add_extern_variable(self, name: str, type: str):
        self.fs.write("extern " + type + " " + name + ";\n")

    def begin_struct(self, name: str, packed=False, align=1):
        self.in_struct = True
        self.fs.write("struct " + name + "\n{\n")

    def end_struct(self):
        self.in_struct = False
        self.fs.write("};\n")

    def add_typedef(self, name: str, basetype: str):
        self.fs.write("typedef " + basetype + " " + name + ";\n")

    def newline(self):
        self.fs.write("\n")

    def newlines(self, num = 3):
        for i in range(num):
            self.fs.write("\n")
