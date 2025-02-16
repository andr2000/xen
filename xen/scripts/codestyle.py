#!/usr/bin/env python3

import glob
import os
import re
import sys
from xen_analysis.settings import xen_dir, repo_dir
from xen_analysis import utils
from xen_analysis import exclusion_file_list
from xen_analysis.exclusion_file_list import ExclusionFileListError

# The Xen codestyle states that labels needs to be indented by at least one
# blank, but clang-format doesn't have an option for that and if it encounters
# a label indented by blank characters that are less than its indent
# configuration, it removes the indentation.
# So this action is meant as post step and checks every label syntax match and
# it adds one blank before the label
def action_fix_label_indent(filename, file_lines):
    label_rgx = re.compile('^[a-zA-Z_][a-zA-Z0-9_]*\s*:.*$')

    for i in range(0, len(file_lines)):
        if label_rgx.match(file_lines[i]):
            file_lines[i] = ' ' + file_lines[i]

    return file_lines


# clang-format most of the time breaks the content of asm(...) instructions,
# so with this function, we protect all the asm sections using the special
# in code comments that tells clang-format to don't touch the block.
# asm(...) instruction could be also inside macros, so in that case we protect
# the entire macro that is enclosing the instruction, in the un-protect stage
# however, we need to do clang-format's job at least on the tab-space conversion
# and to put the backslash on the right side.
def action_protect_asm(filename, file_lines, protect):
    opening_asm = False
    cf_off_comment = '/* clang-format off */'
    cf_on_comment = '/* clang-format on */'

    config = filename[1]["protect"]

    if protect:
        # Look for closing parenthesis with semicolon ');'
        closing_asm_rgx_rule = lambda cl_stx: rf'^.*{re.escape(cl_stx)}.*$'
        # Look for opening asm syntax
        opening_asm_rgx_rule = lambda op_stx: rf'^\s*{op_stx}.*$'
        macro_start_rgx_rule = r'^\s?#\s?define.*\\$'
        opening_asm_find = lambda op_stx: rf'({op_stx})'
        opening_asm_replace = cf_off_comment + r'\1'
        opening_def_find = r'#\s?define'
        opening_def_replace = f'{cf_off_comment}#define'
        closing_asm_find    = lambda cl_stx: re.escape(cl_stx)
        closing_asm_replace = lambda cl_stx: cl_stx + cf_on_comment
        closing_def_find    = '\n'
        closing_def_replace = cf_on_comment + '\n'
    else:
        # Look for closing parenthesis with semicolon ');' and with the
        # special clang-format comment
        closing_asm_rgx_rule = lambda cl_stx: \
            rf'^.*{re.escape(cl_stx)}.*{re.escape(cf_on_comment)}.*$'
        # Look for opening asm syntax preceded by the special clang-format
        # comment, the comment is optional to generalise the algorithm to
        # un-protect asm outside and inside macros. The case outside is easy
        # because we will find '/* clang-format off */asm', instead the case
        # inside is more tricky and we are going to find only 'asm' and then
        # go backwards until we find '/* clang-format off */#define'
        opening_asm_rgx_rule = lambda op_stx: \
            rf'^\s*({re.escape(cf_off_comment)})?{op_stx}.*$'
        # Look for the define just before the asm invocation, here we look for
        # '/* clang-format off */#define' or '#define', this is to handle a rare
        # corner case where an asm invocation is inside a macro, but was not
        # protected in the 'protect stage', because it was on the same line
        # of the define and was not ending with backslash but it was exceeding
        # line width so clang-format formatted anyway.
        # It's safe because we won't change code that has no clang-format
        # comments, but at least the tool won't complain
        macro_start_rgx_rule = \
            rf'^\s?(?:{re.escape(cf_off_comment)})?#\s?define.*\\$'
        opening_asm_find = \
            lambda op_stx: rf'({re.escape(cf_off_comment)}({op_stx}))'
        opening_asm_replace = r'\2'
        opening_def_find = rf'(?:{re.escape(cf_off_comment)})?#\s?define'
        opening_def_replace = '#define'
        closing_asm_find = lambda cl_stx: \
            rf'{re.escape(cl_stx)}.*{re.escape(cf_on_comment)}'
        closing_asm_replace = lambda cl_stx: cl_stx
        closing_def_find    = cf_on_comment + '\n'
        closing_def_replace = '\n'

    closing_asm_rgx = None
    macro_start_rgx = re.compile(macro_start_rgx_rule)
    regx_open = ""
    regx_close = ""

    i = 0
    i_max = len(file_lines)
    macro_begin = -1
    while i < i_max:
        opening_match = False
        # Keep track of the last define we found
        if macro_start_rgx.match(file_lines[i]):
            macro_begin = i
        # Try to find in the current line the asm syntax opening, but don't
        # touch asm syntax inside macros that usually have the line ended by
        # a backslash, this is because insert stuff in them will have issues
        # when later clang-format will format the macro itself
        if (not opening_asm):
            for elem in config:
                regx_open = elem["syntax_opening"]
                if re.match(opening_asm_rgx_rule(regx_open), file_lines[i]):
                    regx_close = elem["syntax_closing"]
                    closing_asm_rgx = \
                        re.compile(closing_asm_rgx_rule(regx_close))
                    opening_match = True
                    break

        if (not opening_asm) and opening_match:
            line_before = (i > 0) and file_lines[i-1].endswith('\\\n')
            if file_lines[i].endswith('\\\n') or line_before:
                # This should never happen, but if it does, it means an
                # unexpected syntax is found in the file
                if macro_begin < 0:
                    raise Exception("Begin of macro not found in {}\n"
                                    "The asm invocation is on line {}"
                                    .format(filename[0], i))

                # asm invocation inside macro, need to protect the entire macro,
                # macro_begin should point to this define
                file_lines[macro_begin] = re.sub(opening_def_find,
                                                 opening_def_replace,
                                                 file_lines[macro_begin])

                # now go to the end of the macro, it's easy as the first line
                # without a backslash will be the end
                for j in range(i, len(file_lines)):
                    if not file_lines[j].endswith('\\\n'):
                        file_lines[j] = \
                            file_lines[j].replace(closing_def_find,
                                                  closing_def_replace, 1)
                        # Advance i index, as j is the last line checked
                        i = j
                        break

                # In the un-protect stage, we need to do clang-format's job,
                # so convert tabs to 4 spaces and for macros put the backslash
                # at the end of the line
                if not protect:
                    for j in range(macro_begin, i+1):
                        # Tab replacement
                        file_lines[j] = file_lines[j].replace('\t', '    ')
                        # backslash indentation
                        line_len = len(file_lines[j])
                        # 81 counting also the newline character
                        if file_lines[j].endswith('\\\n') and line_len < 81:
                            spaces = ' ' * (81 - line_len)
                            file_lines[j] = \
                                file_lines[j].replace('\\\n', spaces + '\\\n',
                                                      1)

                macro_begin = -1
            else:
                # asm invocation is not inside a macro, protect the asm syntax
                opening_asm = True
                file_lines[i] = re.sub(opening_asm_find(regx_open),
                                       opening_asm_replace, file_lines[i])

        # Try to find in the current line the asm closing syntax, only if a
        # previous line already found an opening asm syntax
        if opening_asm and closing_asm_rgx.match(file_lines[i]):
            opening_asm = False
            file_lines[i] = re.sub(closing_asm_find(regx_close),
                                   closing_asm_replace(regx_close),
                                   file_lines[i])

        i += 1

    # This should never happen, but if it does, it means an unexpected syntax
    # is found in the file
    if opening_asm:
        raise Exception("Unbalanced asm parenthesis in {}".format(filename[0]))

    return file_lines


# This function runs a set of actions on the passed file name, the actions needs
# to have a specific interface: action(filename, file_lines), filename will be
# the name of the file where the script is working, file_lines will be the
# content of the file expressed as an array of lines in string format.
# The action function must return this array back to the caller and every
# possible modifications done on that
def stage(filename, actions):

    if len(actions) == 0:
        return

    try:
        with open(filename[0], "rt") as infile:
            file_lines = infile.readlines()
    except OSError as e:
        raise Exception("Issue with reading file {}: {}".format(filename[0], e))

    for task in actions:
        file_lines = task(filename, file_lines)

    try:
        with open(filename[0], "wt") as outfile:
            outfile.writelines(file_lines)
    except OSError as e:
        raise Exception("Issue writing file {}: {}".format(filename[0], e))


def main(argv):
    # Setup actions for pre-stage and post-stage
#    pre_stage_asm_protect = \
#        lambda file, file_lines: action_protect_asm(file, file_lines, True)
#    post_stage_asm_unprotect = \
#        lambda file, file_lines: action_protect_asm(file, file_lines, False)
#    pre_stage_actions = [pre_stage_asm_protect]
#    post_stage_actions = [post_stage_asm_unprotect, action_fix_label_indent]

    len_args = len(argv)
    if len_args > 0:
        c_files = []
        for i in range(0, len_args):
            if not argv[i].startswith('/'):
                # Assume a relative path to Xen
                abs_path = os.getcwd() + "/" + argv[i]
                check_path = [abs_path]
                if '*' in abs_path:
                    check_path = glob.glob(abs_path)
                for path in check_path:
                    if os.path.exists(path):
                        c_files.append(path)
                    else:
                        raise Exception("Malformed path {} solved from {}"
                                        .format(path, argv[i]))
    else:
        # Find all files with .c and .h extension
        c_files = utils.recursive_find_file('drivers', r'.*\.(?:c|h)$')

#    try:
#        exclusion_file = \
#                "{}/docs/misra/exclude-list.json".format(repo_dir)
#        exclusion_list = \
#                exclusion_file_list.load_exclusion_file_list(exclusion_file,
#                                                             "codestyle")
#    except ExclusionFileListError as e:
#        print("ERROR: Issue with reading file {}: {}".format(exclusion_file, e))
#        sys.exit(1)

    # Transform the lists of absolute path in the second element of the
    # exclusion_list into a plain list of absolute path to be exluded
    exclusion_list = []
    excluded_c_files = [(j, i[2]) for i in exclusion_list for j in i[1]]

    file_to_format = []
    default_config = {
        "syntax_opening": "(?:asm|__asm__)(?:\s(?:volatile|__volatile__))?\s?\(",
        "syntax_closing": ");"
    }
    for file in c_files:
        add_to_list = True
        cfg = {"protect": [default_config]}
        for excl in excluded_c_files:
            if excl[0] in file:
                if excl[1] is not None and "protect" in excl[1]:
                    excl[1]["protect"].append(default_config)
                    cfg = excl[1]
                else:
                    add_to_list = False
                break
        if add_to_list:
            file_to_format.append((file, cfg))
        elif len_args > 0:
            print("ERROR: file {} is excluded!".format(file))
            sys.exit(1)

    for file_entry in file_to_format:
        try:
            #stage(file_entry, pre_stage_actions)
            utils.invoke_command("/mnt/storage/projects/clang/llvm-project/build/bin/clang-format -i {}".format(file_entry[0]),
                                 False, Exception,
                                 "Error occured invoking: {}\n")
            #stage(file_entry, post_stage_actions)
        except Exception as e:
            print("ERROR: {}\n".format(e))


if __name__ == "__main__":
    main(sys.argv[1:])
