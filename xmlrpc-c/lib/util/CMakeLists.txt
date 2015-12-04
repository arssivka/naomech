## -*- cmake -*-

set(util_SOURCES
        casprintf.c
        cmdline_parser.c
        cmdline_parser_cpp.cpp
        getoptx.c
        getoptx.h
        stripcaseeq.c
        string_parser.c
        )

add_library(util STATIC ${util_SOURCES})
