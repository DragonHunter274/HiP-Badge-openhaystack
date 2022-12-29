
set(srcs "blinken.c" "ws2812.c" "control.c" "openhaystack_main.c")
set(reqs "")

if(CONFIG_BLINKEN_BADGE)
    list(APPEND srcs "hipbadge.c")
endif()
if(CONFIG_BLINKEN_RAINBOW)
    list(APPEND srcs "rainbow.c")
endif()
if(CONFIG_BLINKEN_EYES)
    list(APPEND srcs "eyes.c")
endif()
if(CONFIG_BLINKEN_GAS)
    list(APPEND srcs "gassens.c")
endif()

idf_component_register(SRCS "${srcs}"
                       REQUIRES "${reqs}" INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
