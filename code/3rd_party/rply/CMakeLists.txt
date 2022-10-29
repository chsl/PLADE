cmake_minimum_required(VERSION 3.12)

project(rply)

add_library(rply STATIC rply.c rply.h)

set_target_properties(rply PROPERTIES FOLDER "3rd_party")

if (MSVC)
    target_compile_definitions(rply PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _CRT_SECURE_NO_DEPRECATE
        )
endif()