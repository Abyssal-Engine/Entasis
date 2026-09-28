include(CMakeFindDependencyMacro)
if(NOT WIN32)
    find_dependency(Threads)
endif()

get_filename_component(_ENTASIS_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(_ENTASIS_INCLUDE "${_ENTASIS_PREFIX}/include")
set(_ENTASIS_LIB "${_ENTASIS_PREFIX}/lib")
if(WIN32)
    set(_ENTASIS_RUNTIME "${_ENTASIS_PREFIX}/bin/entasis.dll")
    set(_ENTASIS_COOKING "${_ENTASIS_PREFIX}/bin/entasis_cooking.dll")
    set(_ENTASIS_RUNTIME_STATIC "${_ENTASIS_LIB}/entasis_static.lib")
    set(_ENTASIS_COOKING_STATIC "${_ENTASIS_LIB}/entasis_cooking_static.lib")
    set(_ENTASIS_SYSTEM "")
else()
    set(_ENTASIS_RUNTIME "${_ENTASIS_LIB}/libentasis.so")
    set(_ENTASIS_COOKING "${_ENTASIS_LIB}/libentasis_cooking.so")
    set(_ENTASIS_RUNTIME_STATIC "${_ENTASIS_LIB}/libentasis.a")
    set(_ENTASIS_COOKING_STATIC "${_ENTASIS_LIB}/libentasis_cooking.a")
    set(_ENTASIS_SYSTEM "Threads::Threads;${CMAKE_DL_LIBS};m")
endif()

if(NOT TARGET Entasis::runtime)
    add_library(Entasis::runtime SHARED IMPORTED)
    set_target_properties(Entasis::runtime PROPERTIES
        IMPORTED_LOCATION "${_ENTASIS_RUNTIME}"
        INTERFACE_INCLUDE_DIRECTORIES "${_ENTASIS_INCLUDE}"
        INTERFACE_LINK_LIBRARIES "${_ENTASIS_SYSTEM}")
    if(WIN32)
        set_target_properties(Entasis::runtime PROPERTIES
            IMPORTED_IMPLIB "${_ENTASIS_LIB}/entasis.lib"
            INTERFACE_COMPILE_DEFINITIONS ENTASIS_USE_SHARED)
    endif()
endif()

if(NOT TARGET Entasis::runtime_static)
    add_library(Entasis::runtime_static STATIC IMPORTED)
    set_target_properties(Entasis::runtime_static PROPERTIES
        IMPORTED_LOCATION "${_ENTASIS_RUNTIME_STATIC}"
        INTERFACE_INCLUDE_DIRECTORIES "${_ENTASIS_INCLUDE}"
        INTERFACE_LINK_LIBRARIES "${_ENTASIS_SYSTEM}")
endif()

if(NOT TARGET Entasis::cooking)
    add_library(Entasis::cooking SHARED IMPORTED)
    set_target_properties(Entasis::cooking PROPERTIES
        IMPORTED_LOCATION "${_ENTASIS_COOKING}"
        INTERFACE_INCLUDE_DIRECTORIES "${_ENTASIS_INCLUDE}"
        INTERFACE_LINK_LIBRARIES "Entasis::runtime;${_ENTASIS_SYSTEM}")
    if(WIN32)
        set_target_properties(Entasis::cooking PROPERTIES
            IMPORTED_IMPLIB "${_ENTASIS_LIB}/entasis_cooking.lib"
            INTERFACE_COMPILE_DEFINITIONS ENTASIS_USE_SHARED)
    endif()
endif()

if(NOT TARGET Entasis::cooking_static)
    add_library(Entasis::cooking_static STATIC IMPORTED)
    set_target_properties(Entasis::cooking_static PROPERTIES
        IMPORTED_LOCATION "${_ENTASIS_COOKING_STATIC}"
        INTERFACE_INCLUDE_DIRECTORIES "${_ENTASIS_INCLUDE}"
        INTERFACE_LINK_LIBRARIES "Entasis::runtime_static;${_ENTASIS_SYSTEM}")
    if(WIN32)
        # separate Odin archives contain overlapping private support-package symbols
        set_target_properties(Entasis::cooking_static PROPERTIES
            INTERFACE_LINK_OPTIONS "/FORCE:MULTIPLE;/IGNORE:4006")
    else()
        set_target_properties(Entasis::cooking_static PROPERTIES
            INTERFACE_LINK_OPTIONS "LINKER:--allow-multiple-definition")
    endif()
endif()

unset(_ENTASIS_PREFIX)
unset(_ENTASIS_INCLUDE)
unset(_ENTASIS_LIB)
unset(_ENTASIS_RUNTIME)
unset(_ENTASIS_COOKING)
unset(_ENTASIS_RUNTIME_STATIC)
unset(_ENTASIS_COOKING_STATIC)
unset(_ENTASIS_SYSTEM)
