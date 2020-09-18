#
# Toolkit for working with EPICS stuff
#
import os, sys, io

# Custom record type
class Record:
	name = ""
	fields = list()
	class Field:
		name = ""
		type = "" 
		prompt = ""
		group = ""

class Override:
	name = ""
	val = ""

	def __init__(self, name: str, val: val):
		self.name = name
		self.val = val
	
class Template:
	name = ""
	record = ""
	overrides = list()
	fields = list()

	def __init__(self, name, record):
		self.name = name
		self.record = record
		self.overrides = list()

	def write(self, file: str, append: bool = False):
		if append:
			mode = "a"
		else:
			mode = "w+"
		with open(file, mode) as fs:
			fs.write("\nrecord(" + self.record + ",\"" + self.name + "\")\n{\n")
			# write constants
			for over in self.fields:
				fs.write("\tfield(" + over.name + ",\t\"" + over.val + "\")\n")
			# write overrides (macros)
			for over in self.overrides:
				fs.write("\tfield(" + over.name + ",\t\"$(" + over.val + ")\")\n")
			fs.write("}\n\n")

	def write_subs(self, file: str):
		with open(file, "w+") as fs:
			fs.write("gay")

	# adds the specified override with the specified val into the list
	# if one already exists, it will return False indicatinga failure
	def add_override(self, name: str, val: str) -> bool:
		for ov in self.overrides:
			if ov.name is name:
				return False
		self.overrides.append(Override(name, val))
		return True

	# returns if the specified override exists
	def has_override(self, name: str) -> bool:
		for ov in self.overrides:
			if ov.name is name:
				return True
		return False

	# Remove the override with the specified name
	# returns true if removed, false if otherwise
	def clear_override(self, name: str) -> bool:
		for ov in self.overrides:
			if ov.name is name:
				self.overrides.remove(ov)
				return True
		return False

	def set_field(self, name: str, val: str) -> bool:
		for fld in self.fields:
			if fld.name is name:
				return False
		self.fields.append(Override(name, val))
		return True

	def clear_field(self, name: str, val: str) -> bool:
		for fld in self.fields:
			if fld.name is name:
				self.fields.remove(fld)
				return True
		return False 

# Write a substitution file with the list of templates, optionally appending to an existing file
# this will only create ONE sub entry since Templates is a LIST of all templates in the given file
# path is the filepath to the template file
def write_subs(file: str, path: str, templates: list, append: bool = False):
	if append:
		mode = "a"
	else:
		mode = "w+"
	with open(file, mode) as fs:
		fs.write("\nfile \"" + path + "\"\n")
		fs.write("{\n\tpattern\n\t{\n")
		for temp in templates:
			fs.write("\t\t")
			for over in temp.overrides:
				fs.write(over.val + ",\t")
			fs.write("\n")
		fs.write("\t}\n\t{\n")
		for temp in templates:
			fs.write("\t\t")
			for over in temp.overrides:
				fs.write("\"\",\t")
			fs.write("\n")
		fs.write("\t}\n}\n")


# Write a list of templates to a file
def write_templates(file: str, templates: list):
	for temp in templates:
		temp.write(file, True)