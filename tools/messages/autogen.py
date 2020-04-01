#!/usr/bin/python3

import argparse
import os
import re

from props import getNode
import props_json

parser = argparse.ArgumentParser(description='autogen messages code.')
parser.add_argument('input', help='message definition file')
parser.add_argument('--namespace', default="message", help='optional namespace (for C++)')
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
              "uint64_t": "Q", "int64_t": "q",
              "uint32_t": 'L', "int32_t": 'l',
              "uint16_t": 'H', "int16_t": 'h',
              "uint8_t": 'B', "int8_t": 'b',
              "bool": 'B', "string": 'B'
}

reserved_names = [ 'id', 'len', 'payload', '_buf', '_i', '_pack_string',
                   'pack', 'unpack' ]
reserved_names += list(type_code.keys())

basename, ext = os.path.splitext(args.input)

# assign id numbers to message names
id_dict = {}
next_id = 10
for i in range(root.getLen("messages")):
    m = root.getChild("messages[%d]" % i)
    if m.hasChild("id"):
        next_id = m.getInt("id")
    id_dict[m.getString("name")] = next_id
    next_id += 1
    
def field_name_helper(f):
    name = f.getString("name")
    # test for array form: ident[size]
    parts = re.split('([\w]+)\[(\w+)\]', name)
    # print("parts:", parts)
    if len(parts) == 4:
        name = parts[1]
        index = parts[2]
    else:
        index = None
    return (name, index)
    
def gen_cpp_header():
    result = []

    # test if any messages use "string"
    has_dynamic_string = False
    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        # quick checks
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            name = f.getString("name")
            if f.getString("type") == "string":
                has_dynamic_string = True
                
    result.append("#pragma once")
    result.append("")
    result.append("#include <stdint.h>  // uint8_t, et. al.")
    result.append("#include <string.h>  // memcpy()")
    result.append("")
    if has_dynamic_string:
        result.append("#include <string>")
        result.append("using std::string;")
        result.append("")
    result.append("namespace %s {" % args.namespace)
    result.append("")
    result.append("static inline int32_t intround(float f) {")
    result.append("    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));")
    result.append("}")
    result.append("")
    result.append("static inline uint32_t uintround(float f) {")
    result.append("    return (int32_t)(f + 0.5);")
    result.append("}")
    result.append("")

    # generate message id constants (and quick checks)
    result.append("// Message id constants")
    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        id = id_dict[m.getString("name")]
        result.append("const uint8_t %s_id = %d;" % (m.getString("name"), id))
        # quick checks
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            name = f.getString("name")
            if name in reserved_names:
                print("Error: '%s' is reserved and cannot be used as a field name." % name)
                print("Aborting.")
                quit()
    result.append("")

    result.append("// max of one byte used to store message len")
    result.append("static const uint8_t message_max_len = 255;")
    result.append("");
    
    if root.getLen("constants"):
        result.append("// Constants")
        for i in range(root.getLen("constants")):
            m = root.getChild("constants[%d]" % i)
            line = "static const %s %s = %s;" % (m.getString("type"), m.getString("name"), m.getString("value"))
            if m.hasChild("desc") != "":
                line += "  // %s" % m.getString("desc")
            result.append(line)
        result.append("")

    enum_dict = {}
    if root.getLen("enums"):
        result.append("// Enums")
        for i in range(root.getLen("enums")):
            m = root.getChild("enums[%d]" % i)
            enum_dict[m.getString("name")] = 1
            result.append("enum class %s {" % m.getString("name"))
            for j in range(m.getLen("identifiers")):
                f = m.getChild("identifiers[%d]" % j)
                line = "    %s = %d" % (f.getString("name"), j)
                if j < m.getLen("identifiers") - 1:
                    line += ","
                if f.hasChild("desc"):
                    line += "  // %s" % f.getString("desc")
                result.append(line)
            result.append("};")
        result.append("")
    
    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        print("Processing:", m.getString("name"))
        # quick checks
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            name = f.getString("name")
            if name in reserved_names:
                print("Error: '%s' is reserved and cannot be used as a field name." % name)
                print("Aborting.")
                quit()

        # generate public c message struct
        id = id_dict[m.getString("name")]
        result.append("// Message: %s (id: %d)" % (m.getString("name"), id))
        result.append("struct %s_t {" % (m.getString("name")))
        result.append("    // public fields")
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            line = "    %s %s" % (f.getString("type"), f.getString("name"))
            if f.hasChild("default"):
                line += " = %s" % f.getString("default")
            line += ";"
            result.append(line)
        result.append("")

        # generate private c packed struct
        result.append("    // internal structure for packing")
        result.append("    uint8_t payload[message_max_len];")
        result.append("    #pragma pack(push, 1)")
        result.append("    struct _compact_t {")
        count = m.getLen("fields")
        for j in range(count):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if f.hasChild("pack_type"):
                ptype = f.getString("pack_type")
            elif f.getString("type") == "string":
                ptype = "uint8_t"
            elif f.getString("type") in enum_dict:
                ptype = "uint8_t"
            else:
                ptype = f.getString("type")
            line = "        %s %s" % (ptype, name)
            if f.getString("type") == "string":
                line += "_len"
            if index:
                line += "[%s]" % index
            line += ";"
            result.append(line)
        result.append("    };")
        result.append("    #pragma pack(pop)")
        result.append("")

        # generate built in constants
        result.append("    // public info fields")
        id = id_dict[m.getString("name")]
        result.append("    static const uint8_t id = %s;" % id)
        result.append("    int len = 0;")
        result.append("")
        
        # generate pack code
        result.append("    bool pack() {")
        result.append("        len = sizeof(_compact_t);")
        
        # it's c, so we have to add some attempt at a size sanity check
        result.append("        // size sanity check")
        result.append("        int size = len;")
        for j in range(count):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if index:
                if f.getString("type") == "string":
                    result.append("        for (int _i=0; _i<%s; _i++) size += %s[_i].length();" % (index, name))
            else:
                if f.getString("type") == "string":
                    result.append("        size += %s.length();" % name)
        result.append("        if ( size > message_max_len ) {")
        result.append("            return false;")
        result.append("        }")

        if count > 0:
            # copy values
            result.append("        // copy values")
            result.append("        _compact_t *_buf = (_compact_t *)payload;")
        for j in range(count):
            line = "        ";
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if index:
                line += "for (int _i=0; _i<%s; _i++) " % index
            line += "_buf->%s" % name
            if f.getString("type") == "string":
                line += "_len"
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
                if f.getString("type") in enum_dict:
                    line += "(uint8_t)"
                line += "%s" % name
                if index:
                    line += "[_i]"
                if f.getString("type") == "string":
                    line += ".length()"
            line += ";"
            result.append(line)
        # append string data if needed
        for j in range(count):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if index:
                if f.getString("type") == "string":
                    result.append("        for (int _i=0; _i<%s; _i++) {" % index)
                    result.append("            memcpy(&(payload[len]), %s[_i].c_str(), %s[_i].length());" % (name, name))
                    result.append("            len += %s[_i].length();" % name)
                    result.append("        }")
            else:
                if f.getString("type") == "string":
                    result.append("        memcpy(&(payload[len]), %s.c_str(), %s.length());" % (name, name))
                    result.append("        len += %s.length();" % name)
        result.append("        return true;")
        result.append("    }")
        result.append("")

        # generate unpack code
        result.append("    bool unpack(uint8_t *external_message, int message_size) {")
        result.append("        if ( message_size > message_max_len ) {")
        result.append("            return false;")
        result.append("        }")
        result.append("        memcpy(payload, external_message, message_size);")
        if count > 0:
            result.append("        _compact_t *_buf = (_compact_t *)payload;");
        result.append("        len = sizeof(_compact_t);")
        for j in range(count):
            line = "        ";
            f = m.getChild("fields[%d]" % j)
            if f.getString("type") != "string":
                (name, index) = field_name_helper(f)
                if index:
                    line += "for (int _i=0; _i<%s; _i++) " % index
                line += name
                if index:
                    line += "[_i]"
                line += " = "
                if f.hasChild("pack_scale"):
                    line += "_buf->%s" % name
                    if index:
                        line += "[_i]"
                    line += " / (float)%s" % f.getString("pack_scale")
                    ptype = f.getString("pack_type")
                else:
                    if f.getString("type") in enum_dict:
                        line += "(%s)" % f.getString("type")                    
                    line += "_buf->%s" % name
                    if index:
                        line += "[_i]"
                line += ";"
                result.append(line)
        # unpack string data if needed
        for j in range(count):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if index:
                if f.getString("type") == "string":
                    result.append("        for (int _i=0; _i<%s; _i++) {" % index)
                    result.append("            %s[_i] = string((char *)&(payload[len]), _buf->%s_len[_i]);" % (name, name))
                    result.append("            len += _buf->%s_len[_i];" % name)
                    result.append("        }")
            else:
                if f.getString("type") == "string":
                    result.append("        %s = string((char *)&(payload[len]), _buf->%s_len);" % (name, name))
                    result.append("        len += _buf->%s_len;" % name)
        result.append("        return true;")
        result.append("    }")
        result.append("};")
        result.append("")
    result.append("} // namespace %s" % args.namespace)
    return result

def gen_python_module():
    result = []

    result.append("import struct")
    result.append("")

    # generate message id constants
    result.append("# Message id constants")
    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        id = id_dict[m.getString("name")]
        result.append("%s_id = %s" % (m.getString("name"), id))
    result.append("")

    constants_dict = {}
    if root.getLen("constants"):
        result.append("# Constants")
        for i in range(root.getLen("constants")):
            m = root.getChild("constants[%d]" % i)
            constants_dict[m.getString("name")] = m.getInt("value")
            line = "%s = %s" % (m.getString("name"), m.getString("value"))
            if m.hasChild("desc") != "":
                line += "  # %s" % m.getString("desc")
            result.append(line)
        result.append("")
    
    enum_dict = {}
    if root.getLen("enums"):
        result.append("# Enums")
        for i in range(root.getLen("enums")):
            m = root.getChild("enums[%d]" % i)
            enum_dict[m.getString("name")] = 1
            for j in range(m.getLen("identifiers")):
                f = m.getChild("identifiers[%d]" % j)
                line = "%s_%s = %d" % (m.getString("name"), f.getString("name"), j)
                if f.hasChild("desc"):
                    line += "  # %s" % f.getChild("desc")
                result.append(line)
        result.append("")
    
    for i in range(root.getLen("messages")):
        m = root.getChild("messages[%d]" % i)
        print("Processing:", m.getString("name"))
        # generate python pack string and sanity check
        pack_string = "<"       # little endian byte order
        has_dynamic_string = False
        for j in range(m.getLen("fields")):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if f.hasChild("pack_type"):
                pack_code = type_code[f.getString("pack_type")]
            elif f.getString("type") in enum_dict:
                pack_code = "B"
            else:
                pack_code = type_code[f.getString("type")]
            if index:
                if index in constants_dict:
                    pack_string += pack_code * constants_dict[index]
                else:
                    pack_string += pack_code * int(index)
            else:
                pack_string += pack_code
            if name in reserved_names:
                print("Error: '%s' is reserved and cannot be used as a field name." % name)
                print("Aborting.")
                quit()

        # generate public message class
        id = id_dict[m.getString("name")]
        result.append("# Message: %s" % m.getString("name"))
        result.append("# Id: %d" % id)
        result.append("class %s():" % (m.getString("name")))
        result.append("    id = %s" % id)
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
            if f.hasChild("default"):
                line += f.getString("default")
            elif t == "double" or t == "float":
                line += "0.0"
            elif "int" in t:
                line += "0"
            elif t in enum_dict:
                line += "0"
            elif t == "bool":
                line += "False"
            elif t == "string":
                line += "\"\""
                has_dynamic_string = True
            else:
                line += "None"
            if index:
                line += "] * %s" % index
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
                if index in constants_dict:
                    index = int(constants_dict[index])
                else:
                    index = int(index)
                for k in range(index):
                    line = "                          "
                    if f.hasChild("pack_scale"):
                        line += "int(round(self.%s[%d] * %s))" % (name, k, f.getString("pack_scale"))
                    elif f.getString("type") == "string":
                        line += "len(self.%s[%d])" % (name, k)
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
                elif f.getString("type") == "string":
                    line += "len(self.%s)" % (name)
                else:
                    line += "self.%s" % f.getString("name")
                if j < count - 1:
                    line += ","
                else:
                    line += ")"
                result.append(line)
        # append string data if needed
        for j in range(count):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if index:
                if index in constants_dict:
                    index = int(constants_dict[index])
                else:
                    index = int(index)
                for k in range(index):
                    if f.getString("type") == "string":
                        result.append("        msg += str.encode(self.%s[%d])" % (name, k))
            else:
                if f.getString("type") == "string":
                    result.append("        msg += str.encode(self.%s)" % name)
                        
        result.append("        return msg")
        result.append("")

        # generate unpack code
        result.append("    def unpack(self, msg):")
        if has_dynamic_string:
            result.append("        base_len = struct.calcsize(self._pack_string)")
            result.append("        extra = msg[base_len:]")
            result.append("        msg = msg[:base_len]")
            for j in range(count):
                f = m.getChild("fields[%d]" % j)
                (name, index) = field_name_helper(f)
                if index:
                    result.append("        self.%s_len = [0] * %s" % (name, index))
        for j in range(count):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if index:
                if index in constants_dict:
                    index = int(constants_dict[index])
                else:
                    index = int(index)
                for k in range(index):
                    if j == 0 and k == 0:
                        line = "        ("
                    else:
                        line = "         "
                    line += "self.%s" % name
                    if f.getString("type") == "string":
                        line += "_len"
                    line += "[%d]" % k
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
                if f.getString("type") == "string":
                    line += "_len"
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
                if index in constants_dict:
                    index = int(constants_dict[index])
                else:
                    index = int(index)
                for k in range(index):
                    if f.hasChild("pack_scale"):
                        line = "        self.%s[%d] /= %s" % (name, k, f.getString("pack_scale"))
                        result.append(line)
            else:
                if f.hasChild("pack_scale"):
                    line = "        self.%s /= %s" % (name, f.getString("pack_scale"))
                    result.append(line)
        # unpack string data if needed
        for j in range(count):
            f = m.getChild("fields[%d]" % j)
            (name, index) = field_name_helper(f)
            if index:
                if index in constants_dict:
                    index = int(constants_dict[index])
                else:
                    index = int(index)
                for k in range(index):
                    if f.getString("type") == "string":
                        result.append("        self.%s[%d] = extra[:self.%s_len[%d]].decode()" % (name, k, name, k))
                        result.append("        extra = extra[self.%s_len[%d]:]" % (name, k))
            else:
                if f.getString("type") == "string":
                    result.append("        self.%s = extra[:self.%s_len].decode()" % (name, name))
                    result.append("        extra = extra[self.%s_len:]" % name)
        result.append("")

    return result

if True:
    print("Generating C++ header:")
    code = gen_cpp_header()
    f = open(basename + ".h", "w")
    for line in code:
        f.write(line + "\n")
    f.close()

if True:
    print("Generating Python3 code:")
    code = gen_python_module()
    f = open(basename + ".py", "w")
    for line in code:
        f.write(line + "\n")
    f.close()
