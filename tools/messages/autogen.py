#!/usr/bin/python3

import argparse
import os

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

type_size = { "double": 8, "float": 4,
              "uint32_t": 4, "int32_t": 4,
              "uint16_t": 2, "int16_t": 2,
              "uint8_t": 1, "int8_t": 1
}

type_code = { "double": 'd', "float": 'f',
              "uint32_t": 'L', "int32_t": 'l',
              "uint16_t": 'H', "int16_t": 'h',
              "uint8_t": 'B', "int8_t": 'b'
}

if args.prefix:
    base = args.prefix
else:
    base = "message"

def gen_cpp_header():
    max_size = 0
    result = []

    result.append("#pragma once")
    result.append("#pragma pack(push, 1)")
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

    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        print("Processing:", m.getString("name"))
        # compute sizes and detect if an extra pack structure is needed
        struct_size = 0
        pack_size = 0
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            struct_size += type_size[f.getString("type")]
            if f.hasChild("pack_type"):
                pack_size += type_size[f.getString("pack_type")]
            else:
                pack_size += type_size[f.getString("type")]
        compaction = (not struct_size == pack_size)
        if pack_size > max_size:
            max_size = pack_size;

        # generate public c message struct
        result.append("// Message: %s" % m.getString("name"))
        result.append("// Id: %d" % m.getInt("id"))
        result.append("// Struct size: %d" % struct_size)
        result.append("// Packed message size: %d" % pack_size)
        result.append("struct %s_%s_t {" % (base, m.getString("name")))
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            result.append("    %s %s;" % (f.getString("type"), f.getString("name")))

        result.append("")
        result.append("    const uint8_t id = %s;" % m.getString("id"))
        result.append("    const uint8_t len = %d;" % pack_size)
        result.append("")
        
        if compaction:
            # generate private c packed struct
            result.append("    // internal structure for packing")
            result.append("    struct {")
            for j in range(m.getLen("fields")):
                f = m.getChild("fields[%d]" % j)
                if f.hasChild("pack_type"):
                    ptype = f.getString("pack_type")
                else:
                    ptype = f.getString("type")
                result.append("        %s %s;" % (ptype, f.getString("name")))
            result.append("    } buf;")
            result.append("")

        # generate pack code
        result.append("    uint8_t *pack() {")
        if compaction:
            for j in range(m.getLen("fields")):
                line = "        ";
                f = m.getChild("fields[%d]" % j)
                line += "buf.%s = " % f.getString("name")
                if f.hasChild("pack_type"):
                    ptype = f.getString("pack_type")
                    if ptype[0] == "i":
                        line += "intround("
                    else:
                        line += "uintround("
                    line += "%s" % f.getString("name")
                    if f.hasChild("pack_scale"):
                        line += " * %s" % f.getString("pack_scale")
                    line += ")"
                else:
                    line += "%s" % f.getString("name")
                line += ";"
                result.append(line)
            result.append("        return (uint8_t *)(&buf);")
        else:
            result.append("        return (uint8_t *)this;")
        result.append("    }")
        result.append("")

        # generate unpack code
        result.append("    void unpack(uint8_t *message) {")
        if compaction:
            result.append("        memcpy(&buf, message, %d);" % struct_size)
            for j in range(m.getLen("fields")):
                line = "        ";
                f = m.getChild("fields[%d]" % j)
                line += f.getString("name")
                line += " = "
                if f.hasChild("pack_scale"):
                    line += "buf.%s / (float)%s" % (f.getString("name"), f.getString("pack_scale"))
                    ptype = f.getString("pack_type")
                else:
                    line += "buf.%s" % f.getString("name")
                line += ";"
                result.append(line)
        else:
            result.append("        memcpy(this, message, %d);" % struct_size)
        result.append("    }")
        result.append("};")
        result.append("")

    # generate messaging constants
    result.append("// Message id constants")
    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        result.append("const uint8_t %s_id_%s = %s;" % (base, m.getString("name"), m.getString("id")))
    result.append("const uint16_t %s_max_size = %d;" % (base, max_size))
    result.append("")
    

    result.append("")
    result.append("#pragma pack(pop)")
    return result

def gen_python_module():
    result = []

    result.append("import struct")
    result.append("")

    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        print("Processing:", m.getString("name"))
        # compute sizes and generate python pack string
        pack_size = 0
        pack_string = "<"       # little endian byte order
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            if f.hasChild("pack_type"):
                pack_size += type_size[f.getString("pack_type")]
                pack_code = type_code[f.getString("pack_type")]
            else:
                pack_size += type_size[f.getString("type")]
                pack_code = type_code[f.getString("type")]
            pack_string += pack_code

        # generate public c message struct
        result.append("# Message: %s" % m.getString("name"))
        result.append("# Id: %d" % m.getInt("id"))
        result.append("# Packed message size: %d" % pack_size)
        result.append("class %s():" % (m.getString("name")))
        result.append("    __id = %s" % m.getString("id"))
        result.append("    __pack_string = \"%s\"" % pack_string)
        result.append("")
        result.append("    def __init__(self, msg=None):")
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            result.append("        self.%s = None" % f.getString("name"))
        result.append("        if msg:")
        result.append("            self.unpack(msg)")
        result.append("")

        # generate pack code
        result.append("    def pack(self):")
        result.append("        msg = struct.pack(self.__pack_string,")
        count = m.getLen("fields")
        for j in range(count):
            f = m.getChild("fields[%d]" % j)
            line = "                          "
            if f.hasChild("pack_scale"):
                line += "int(round(self.%s * %s))" % (f.getString("name"), f.getString("pack_scale"))
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
            if j == 0:
                line = "        ("
            else:
                line = "         "
            line += "self.%s" % f.getString("name")
            if j < count - 1:
                line += ","
            else:
                if count == 1:
                    line += ","
                line += ") = struct.unpack(self.__pack_string, msg)"
            result.append(line)
        for j in range(count):
            f = m.getChild("fields[%d]" % j)
            if f.hasChild("pack_scale"):
                line = "        self.%s /= %s" % (f.getString("name"), f.getString("pack_scale"))
                result.append(line)
        if False:
            line += " = "
            if f.hasChild("pack_scale"):
                line += "p->%s / (float)%s" % (f.getString("name"), f.getString("pack_scale"))
                ptype = f.getString("pack_type")
            else:
                line += "p->%s" % f.getString("name")
            line += ";"
            result.append(line)

        result.append("")

    # generate message id constants
    result.append("# Message id constants")
    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        result.append("%s_id = %s" % (m.getString("name"), m.getString("id")))
    result.append("")
    
    return result

do_cpp = True
if do_cpp:
    code = gen_cpp_header()
    print()
    print("Generated C++ code:")
    print()
    for line in code:
        print(line)

    f = open("messages.h", "w")
    for line in code:
        f.write(line + "\n")
    f.close()

do_python = True
if do_python:
    code = gen_python_module()
    print()
    print("Generated Python3 code:")
    print()
    for line in code:
        print(line)

    f = open("messages.py", "w")
    for line in code:
        f.write(line + "\n")
    f.close()
