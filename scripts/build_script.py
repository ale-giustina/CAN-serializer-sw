Import("env")

# The bare-metal builder only passes -mcpu; the G473 has a single-precision FPU.
# Same flags as the CubeMX Makefile.
FPU_FLAGS = ["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]

env.Append(
    ASFLAGS=FPU_FLAGS,
    ASPPFLAGS=FPU_FLAGS,
    CCFLAGS=FPU_FLAGS,
    LINKFLAGS=FPU_FLAGS + ["--specs=nano.specs"],
)

# Strict warnings only for our code, not for the ST drivers.
STRICT_WARNINGS = [
    "-Wall",
    "-Wextra",
    "-Wpedantic",
    "-Werror",
]


def apply_core_warnings(env, node):
    # Replace backslashes to ensure compatibility (Windows = bad)
    path = node.get_path().replace("\\", "/")

    if "Core/" in path:
        return env.Object(
            node,
            CCFLAGS=env.get("CCFLAGS", []) + STRICT_WARNINGS
        )

    return node


env.AddBuildMiddleware(apply_core_warnings)
