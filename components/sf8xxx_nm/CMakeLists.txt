set(srcs "sf8xxx_nm.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver)
