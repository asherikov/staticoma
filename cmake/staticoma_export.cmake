function(staticoma_export)
    if(NOT CATKIN_PACKAGE_SHARE_DESTINATION)
        message(FATAL_ERROR "staticoma_export() must be called after catkin_package()")
    endif()

    foreach(EXPORTED_FILE ${ARGN})
        set(INPUT_FILE_PATH "${PROJECT_SOURCE_DIR}/${EXPORTED_FILE}")
        get_filename_component(FILENAME "${INPUT_FILE_PATH}" NAME)
        add_custom_target(
            staticoma_${NAME} ALL
            COMMAND ${CMAKE_COMMAND} -E copy "${INPUT_FILE_PATH}" "${PROJECT_SOURCE_DIR}/staticoma/"
            DEPENDS staticoma_global_target
            BYPRODUCTS "${PROJECT_BINARY_DIR}/staticoma/${NAME}"
        )
    endforeach()

    install(FILES ${ARGN} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/staticoma)
endfunction()
