#
# Defines MC_VARIANT_SLUG for every build, so that `hwinfo` can name the board's variant without
# each of the ~90 variant files having to declare it.
#
# The slug is the variant's own directory name -- "rak4631", "heltec_v4_r8" -- which is the name
# the tree already uses for a board, is stable, and is short enough for a 160-byte CLI reply. The
# environment name is not used: it carries the firmware role as well ("RAK_4631_repeater"), and
# the longest of them would not fit the reply budget.
#
# Resolved by finding which variants/<slug>/platformio.ini declares this environment. An
# environment that no variant file declares -- one defined in platformio.local.ini, say -- simply
# gets no macro, and HardwareInfo reports "unknown" rather than a guess.
#
import glob
import os
import re

Import("env")

pioenv = env["PIOENV"]
declaration = re.compile(r"^\s*\[env:%s\]\s*$" % re.escape(pioenv), re.MULTILINE)

slug = None
pattern = os.path.join(env["PROJECT_DIR"], "variants", "*", "platformio.ini")
for ini_path in sorted(glob.glob(pattern)):
    try:
        with open(ini_path, "r", encoding="utf-8", errors="replace") as ini:
            contents = ini.read()
    except OSError:
        continue
    if declaration.search(contents):
        slug = os.path.basename(os.path.dirname(ini_path))
        break

if slug:
    env.Append(CPPDEFINES=[("MC_VARIANT_SLUG", env.StringifyMacro(slug))])
