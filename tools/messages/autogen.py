#!/usr/bin/python3

import argparse
import os
import re

from props import getNode
import props_json

parser = argparse.ArgumentParser(description='autogen messages code.')
parser.add_argument('--input', required=True, help='message definition file')
parser.add_argument('--prefix', help='optional namespace prefix')
args = parser.parse_args()

if not os.path.isfile(args.input):
    print("Specified input file not found:", args.input)
    quit()

root = getNode("/", True)
props_json.load(args.input, root)
# root.pretty_print()

if not root.hasChild("messages"):
    print("No message definition found in:", args.input)
    quit()

type_code = { "double": 'd', "float": 'f',
              "uint32_t": 'L', "int32_t": 'l',
              "uint16_t": 'H', "int16_t": 'h',
              "uint8_t": 'B', "int8_t": 'b',
              "bool": 'B'
}

reserved_fields = [ 'id', 'len', '_buf', '_i', '_pack_string' ]

basename, ext = os.path.splitext(args.input)

if args.prefix:
    base = args.prefix
else:
    base = "message"

def field_name_helper(f):
    name = f.getString("name")
    # test for array form: ident[size]
    parts = re.split('([\w-]+)\[(\d+)\]', name)
    if len(parts) == 4:
        name = parts[1]
        index = int(parts[2])
    else:
        index = None
    return (name, index)
    
def gen_cpp_header():
    result = []

    result.append("#pragma once")
    result.append("")
    result.append("#include <stdint.h>  // uint8_t, et. al.")
    result.append("#include <string.h>  // memcpy()")
    result.append("")
    result.append("static inline int32_t intround(float f) {")
    result.append("    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));")
    result.append("}")
    result.append("")
    result.append("static inline uint32_t uintround(float f) {")
    result.append("    return (int32_t)(f + 0.5);")
    result.append("}")
    result.append("")

    # generate messaging constants
    result.append("// Message id constants")
    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        result.append("const uint8_t %s_%s_id = %s;" % (base, m.getString("name"), m.getString("id")))
    result.append("")
    
    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        print("Processing:", m.getString("name"))
        # quick checks
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            name = f.getString("name")
            if name in reserved_fields:
                print("Error: %s is a reserved field name." % name)
                print("Aborting.")
                quit()

        # generate public c message struct
        result.append("// Message: %s (id: %d)" % (m.getString("name"), m.getInt("id")))
        result.append("struct %s_%s_t {" % (base, m.getString("name")))
        result.append("    // public fields")
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            result.append("    %s %s;" % (f.getString("type"), f.getString("name")))
        result.append("")

        # generate private c packed struct
        result.append("    // internal structure for packing")
        result.append("    #pragma pack(push, 1)")
        result.append("    struct {")
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            if f.hasChild("pack_type"):
                ptype = f.getString("pack_type")
            else:
                ptype = f.getString("type")
            result.append("        %s %s;" % (ptype, f.getString("name")))
        result.append("    } _buf;")
        result.append("    #pragma pack(pop)")
        result.append("")

        # generate built in constants
        result.append("    static const uint8_t id = %s;" % m.getString("id"))
        result.append("    static const uint16_t len = sizeof(_buf);")
        result.append("")
        
        # generate pack code
        result.append("    uint8_t *pack() {")
        for j in range(m.getLen("fields")):
            line = "        ";
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if index:
                line += "for (int _i=0; _i<%d; _i++) " % index
            line += "_buf.%s" % name
            if index:
                line += "[_i]"
            line += " = "
            #line += "_buf.%s = " % f.getString("name")
            if f.hasChild("pack_type"):
                ptype = f.getString("pack_type")
                if ptype[0] == "i":
                    line += "intround("
                else:
                    line += "uintround("
                line += "%s" % name
                if index:
                    line += "[_i]"
                if f.hasChild("pack_scale"):
                    line += " * %s" % f.getString("pack_scale")
                line += ")"
            else:
                line += "%s" % name
                if index:
                    line += "[_i]"
            line += ";"
            result.append(line)
        result.append("        return (uint8_t *)(&_buf);")
        result.append("    }")
        result.append("")

        # generate unpack code
        result.append("    void unpack(uint8_t *message) {")
        result.append("        memcpy(&_buf, message, len);")
        for j in range(m.getLen("fields")):
            line = "        ";
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if index:
                line += "for (int _i=0; _i<%d; _i++) " % index
            line += name
            if index:
                line += "[_i]"
            line += " = "
            if f.hasChild("pack_scale"):
                line += "_buf.%s" % name
                if index:
                    line += "[_i]"
                line += " / (float)%s" % f.getString("pack_scale")
                ptype = f.getString("pack_type")
            else:
                line += "_buf.%s" % name
                if index:
                    line += "[_i]"
            line += ";"
            result.append(line)
        result.append("    }")
        result.append("};")
        result.append("")
    return result

def gen_python_module():
    result = []

    result.append("import struct")
    result.append("")

    # generate message id constants
    result.append("# Message id constants")
    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        result.append("%s_id = %s" % (m.getString("name"), m.getString("id")))
    result.append("")
    
    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        print("Processing:", m.getString("name"))
        # generate python pack string and sanity check
        pack_string = "<"       # little endian byte order
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if f.hasChild("pack_type"):
                pack_code = type_code[f.getString("pack_type")]
            else:
                pack_code = type_code[f.getString("type")]
            if index:
                pack_string += pack_code * index
            else:
                pack_string += pack_code
            if name in reserved_fields:
                print("Error: %s is a reserved field name." % name)
                print("Aborting.")
                quit()

        # generate public message class
        result.append("# Message: %s" % m.getString("name"))
        result.append("# Id: %d" % m.getInt("id"))
        result.append("class %s():" % (m.getString("name")))
        result.append("    id = %s" % m.getString("id"))
        result.append("    _pack_string = \"%s\"" % pack_string)
        result.append("")
        result.append("    def __init__(self, msg=None):")
        result.append("        # public fields")
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            t = f.getString("type")
            line = "        self.%s = " % name
            if index:
                line += "["
            if t == "double" or t == "float":
                line += "0.0"
            elif "int" in t:
                line += "0"
            elif t == "bool":
                line += "False"
            elif t == "string":
                line += "\"\""
            else:
                line += "None"
            if index:
                line += "] * %d" % index
            result.append(line)
        result.append("        # unpack if requested")
        result.append("        if msg: self.unpack(msg)")
        result.append("")

        # generate pack code
        result.append("    def pack(self):")
        result.append("        msg = struct.pack(self._pack_string,")
        count = m.getLen("fields")
        for j in range(count):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if index:
                for k in range(index):
                    line = "                          "
                    if f.hasChild("pack_scale"):
                        line += "int(round(self.%s[%d] * %s))" % (name, k, f.getString("pack_scale"))
                    else:
                        line += "self.%s[%d]" % (name, k)
                    if j < count - 1 or k < index - 1:
                        line += ","
                    else:
                        line += ")"
                    result.append(line)
            else:
                line = "                          "
                if f.hasChild("pack_scale"):
                    line += "int(round(self.%s * %s))" % (name, f.getString("pack_scale"))
                else:
                    line += "self.%s" % f.getString("name")
                if j < count - 1:
                    line += ","
                else:
                    line += ")"
                result.append(line)
                
        result.append("        return msg")
        result.append("")

        # generate unpack code
        result.append("    def unpack(self, msg):")
        for j in range(count):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if index:
                for k in range(index):
                    if j == 0 and k == 0:
                        line = "        ("
                    else:
                        line = "         "
                    line += "self.%s[%d]" % (name, k)
                    if j < count - 1 or k < index - 1:
                        line += ","
                    else:
                        if count == 1:
                            line += ","
                        line += ") = struct.unpack(self._pack_string, msg)"
                    result.append(line)
            else:
                if j == 0:
                    line = "        ("
                else:
                    line = "         "
                line += "self.%s" % name
                if j < count - 1:
                    line += ","
                else:
                    if count == 1:
                        line += ","
                    line += ") = struct.unpack(self._pack_string, msg)"
                result.append(line)
        for j in range(count):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if index:
                for k in range(index):
                    if f.hasChild("pack_scale"):
                        line = "        self.%s[%d] /= %s" % (name, k, f.getString("pack_scale"))
                        result.append(line)
            else:
                if f.hasChild("pack_scale"):
                    line = "        self.%s /= %s" % (name, f.getString("pack_scale"))
                    result.append(line)
        result.append("")

    return result

do_cpp = True
if do_cpp:
    print("Generating C++ header:")
    code = gen_cpp_header()
    f = open(basename + ".h", "w")
    for line in code:
        f.write(line + "\n")
    f.close()

do_python = True
if do_python:
    print("Generating Python3 code:")
    code = gen_python_module()
    f = open(basename + ".py", "w")
    for line in code:
        f.write(line + "\n")
    f.close()
