Import("env")

# Принудительно добавляем флаги FPU для всех файлов
env.Append(
    CCFLAGS=["-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"],
    LINKFLAGS=["-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"],
    LDFLAGS=["--specs=nano.specs", "-u _scanf_float", "-u", "_printf_float"]
)