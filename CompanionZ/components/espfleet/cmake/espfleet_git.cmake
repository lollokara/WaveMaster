# Compiles the project's git identity into the ESPFleet agent, so a running
# device can tell the hub exactly which commit it is executing.
#
# This is the ESP-IDF equivalent of scripts/espfleet_build.py (PlatformIO).

function(espfleet_inject_git_info target)
    idf_build_get_property(project_dir PROJECT_DIR)

    find_package(Git QUIET)
    if(NOT GIT_FOUND)
        message(STATUS "espfleet: git not found, commit metadata will be empty")
        return()
    endif()

    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --show-toplevel
        WORKING_DIRECTORY "${project_dir}"
        OUTPUT_VARIABLE git_toplevel
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE git_status
    )
    if(NOT git_status EQUAL 0)
        message(STATUS "espfleet: ${project_dir} is not a git repository")
        return()
    endif()

    # Re-run CMake when the checked-out commit changes.
    if(EXISTS "${git_toplevel}/.git/HEAD")
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${git_toplevel}/.git/HEAD")
    endif()

    macro(_espfleet_git out_var)
        execute_process(
            COMMAND ${GIT_EXECUTABLE} ${ARGN}
            WORKING_DIRECTORY "${project_dir}"
            OUTPUT_VARIABLE ${out_var}
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
    endmacro()

    _espfleet_git(ESPFLEET_COMMIT rev-parse HEAD)
    _espfleet_git(ESPFLEET_BRANCH rev-parse --abbrev-ref HEAD)
    _espfleet_git(ESPFLEET_DESCRIBE describe --tags --always --dirty)
    _espfleet_git(ESPFLEET_PORCELAIN status --porcelain)
    _espfleet_git(ESPFLEET_REMOTES remote)

    set(ESPFLEET_DIRTY 0)
    if(NOT ESPFLEET_PORCELAIN STREQUAL "")
        set(ESPFLEET_DIRTY 1)
    endif()

    # Prefer origin, then a Gogs-looking remote, then the first one present.
    string(REPLACE "\n" ";" _remote_list "${ESPFLEET_REMOTES}")
    set(_remote_url "")
    foreach(_candidate origin gogs ${_remote_list})
        if(_candidate IN_LIST _remote_list AND _remote_url STREQUAL "")
            _espfleet_git(_remote_url remote get-url ${_candidate})
        endif()
    endforeach()

    set(ESPFLEET_SLUG "${ESPFLEET_PROJECT_SLUG}")  # explicit override wins
    # VENDORED PATCH (WaveMaster): only a Gogs-style remote gives a slug the hub knows; a GitHub origin
    # would register the device under a project that does not exist on the hub.
    if(_remote_url MATCHES "github\\.com")
        set(_remote_url "")
    endif()

    if(ESPFLEET_SLUG STREQUAL "" AND NOT _remote_url STREQUAL "")
        string(REGEX REPLACE "\\.git$" "" _remote_url "${_remote_url}")
        if(_remote_url MATCHES "[:/]([^/:]+)/([^/]+)$")
            # Gogs stores repositories under lowercased directory names.
            string(TOLOWER "${CMAKE_MATCH_1}/${CMAKE_MATCH_2}" ESPFLEET_SLUG)
        endif()
    endif()

    string(TIMESTAMP ESPFLEET_BUILD_TIME "%Y-%m-%dT%H:%M:%S" UTC)

    target_compile_definitions(${target} PRIVATE
        ESPFLEET_GIT_COMMIT="${ESPFLEET_COMMIT}"
        ESPFLEET_GIT_BRANCH="${ESPFLEET_BRANCH}"
        ESPFLEET_GIT_DIRTY=${ESPFLEET_DIRTY}
        ESPFLEET_BUILD_TIME="${ESPFLEET_BUILD_TIME}Z"
    )
    if(NOT ESPFLEET_SLUG STREQUAL "")
        target_compile_definitions(${target} PRIVATE ESPFLEET_PROJECT="${ESPFLEET_SLUG}")
    endif()
    if(NOT ESPFLEET_DESCRIBE STREQUAL "")
        target_compile_definitions(${target} PRIVATE ESPFLEET_VERSION="${ESPFLEET_DESCRIBE}")
    endif()

    message(STATUS "espfleet: version=${ESPFLEET_DESCRIBE} commit=${ESPFLEET_COMMIT} "
                   "branch=${ESPFLEET_BRANCH} dirty=${ESPFLEET_DIRTY} project=${ESPFLEET_SLUG}")
endfunction()
