#!/usr/bin/env python3
"""Patch the lift plugin to fix the segfault caused by modifying ECM during iteration."""
import sys

filepath = sys.argv[1]

with open(filepath, "r") as f:
    content = f.read()

# The bug: enableComponent is called inside Each() loop, corrupting the iterator
old_code = "          enableComponent<components::AxisAlignedBox>(ecm, entity, false);\n        }\n        return true;\n      });\n  }"

new_code = "          entities_to_disable.push_back(entity);\n        }\n        return true;\n      });\n    for (const auto& entity : entities_to_disable)\n    {\n      enableComponent<components::AxisAlignedBox>(ecm, entity, false);\n    }\n  }"

# Also need to add the vector declaration at the start of the function
old_func_start = "  void read_aabbs(EntityComponentManager& ecm)\n  {\n    ecm.Each"
new_func_start = "  void read_aabbs(EntityComponentManager& ecm)\n  {\n    std::vector<Entity> entities_to_disable;\n    ecm.Each"

if old_code not in content:
    print("ERROR: Could not find the code to patch (enableComponent inside Each)")
    sys.exit(1)

if old_func_start not in content:
    print("ERROR: Could not find function start to patch")
    sys.exit(1)

content = content.replace(old_code, new_code)
content = content.replace(old_func_start, new_func_start)

with open(filepath, "w") as f:
    f.write(content)

print("PATCHED successfully")
