set(srcs "main.c"
        "board.c"
        "dht.c"
        "max1704x.c"
        "i2cdev.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  ".")
