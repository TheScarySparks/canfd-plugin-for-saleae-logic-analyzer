include(FetchContent)

# Use the C++11 standard
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_STANDARD_REQUIRED YES)

if (NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY OR NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/)
endif()

# Fetch the Analyzer SDK if the target does not already exist.
if(NOT TARGET Saleae::AnalyzerSDK)
    FetchContent_Declare(
        analyzersdk
        GIT_REPOSITORY https://github.com/saleae/AnalyzerSDK.git
        GIT_TAG        master
        GIT_SHALLOW    True
        GIT_PROGRESS   True
    )

    FetchContent_GetProperties(analyzersdk)

    if(NOT analyzersdk_POPULATED)
        FetchContent_Populate(analyzersdk)
        include(${analyzersdk_SOURCE_DIR}/AnalyzerSDKConfig.cmake)

        if(APPLE OR WIN32)
            get_target_property(analyzersdk_lib_location Saleae::AnalyzerSDK IMPORTED_LOCATION)
            if(CMAKE_LIBRARY_OUTPUT_DIRECTORY)
                file(COPY ${analyzersdk_lib_location} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
            else()
                message(WARNING "Please define CMAKE_RUNTIME_OUTPUT_DIRECTORY and CMAKE_LIBRARY_OUTPUT_DIRECTORY if you want unit tests to locate ${analyzersdk_lib_location}")
            endif()
        endif()

    endif()
endif()

function(add_analyzer_plugin TARGET)
    set(options )
    set(single_value_args )
    set(multi_value_args SOURCES)
    cmake_parse_arguments( _p "${options}" "${single_value_args}" "${multi_value_args}" ${ARGN} )


    add_library(${TARGET} MODULE ${_p_SOURCES})
    target_link_libraries(${TARGET} PRIVATE Saleae::AnalyzerSDK)

    set(ANALYZER_DESTINATION "Analyzers")
    install(TARGETS ${TARGET} RUNTIME DESTINATION ${ANALYZER_DESTINATION}
                              LIBRARY DESTINATION ${ANALYZER_DESTINATION})

    set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${ANALYZER_DESTINATION}
                                               LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${ANALYZER_DESTINATION})

    # The initial FetchContent-time copy above only lands in
    # CMAKE_LIBRARY_OUTPUT_DIRECTORY (build/bin), which doesn't match this
    # target's actual per-config output folder (e.g. Analyzers/Release with
    # the Visual Studio multi-config generator). Without the SDK's runtime
    # library sitting somewhere Windows can find it, Logic 2 silently fails
    # to load the plugin -- no error, it just doesn't show up in the
    # analyzer list.
    #
    # It's copied into a `deps` subfolder rather than directly alongside
    # the plugin: Logic 2 scans its configured Custom Low Level Analyzers
    # directory and tries to load every .dll found there as a plugin
    # candidate, throwing a "Failed to Load Custom Analyzer ... Loaded
    # library does not have required functions" error for the SDK runtime
    # itself -- but only checks files directly in that directory, not
    # subfolders. Windows dependency resolution for the plugin DLL still
    # finds it one level down in `deps`, so this keeps both working: no
    # spurious error, plugin still loads. (Same fix already applied to the
    # sibling classic CAN 2.0B plugin.)
    if (APPLE OR WIN32)
        add_custom_command(TARGET ${TARGET} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory
                $<TARGET_FILE_DIR:${TARGET}>/deps
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:Saleae::AnalyzerSDK>
                $<TARGET_FILE_DIR:${TARGET}>/deps
            COMMENT "Copying Saleae AnalyzerSDK runtime into deps/ next to ${TARGET}"
        )
    endif()

    # Also drop a copy of the built plugin itself into the shared
    # "Analyser DLLs" folder (a sibling of this repo, alongside the other
    # Molinaro plugin builds) -- that's the single folder Logic 2's Custom
    # Low Level Analyzers setting actually points at, since Logic 2 only
    # supports one such directory. Without this, every rebuild would need
    # a manual copy there to actually take effect in Logic 2.
    if (WIN32)
        add_custom_command(TARGET ${TARGET} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${TARGET}>
                "${PROJECT_SOURCE_DIR}/../Analyser DLLs"
            COMMENT "Copying ${TARGET} into the shared Analyser DLLs folder"
        )
    endif()
endfunction()