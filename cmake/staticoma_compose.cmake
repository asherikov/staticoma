# parameters: <output_file> <input_file> ...
function(staticoma_compose OUTPUT_FILE)
    if(NOT CATKIN_PACKAGE_SHARE_DESTINATION)
        message(FATAL_ERROR "staticoma_compose() must be called after catkin_package()")
    endif()

    set(INPUT_FILES ${ARGN})
    list(LENGTH INPUT_FILES LIST_LENGTH)

    if (LIST_LENGTH LESS 4)
        message(FATAL_ERROR "staticoma_compose: at least two input files must be specified")
    endif()


    set(STATICOMA_BUILD_OUTPUT_DIR "${PROJECT_SOURCE_DIR}/staticoma")
    set(STATICOMA_ARRAY_MERGE_STRATEGY "overwrite")
    set(STATICOMA_STRICT OFF)


    list(LENGTH INPUT_FILES LIST_LENGTH)
    while (LIST_LENGTH GREATER 0)
        list(GET INPUT_FILES 0 COMPOSE_ARG)
        if ("${COMPOSE_ARG}" STREQUAL ARRAY_MERGE_STRATEGY)
            list(REMOVE_AT INPUT_FILES 0)

            list(GET INPUT_FILES 0 ARRAY_MERGE_STRATEGY)
            list(REMOVE_AT INPUT_FILES 0)

            set(STATICOMA_ARRAY_MERGE_STRATEGY "${ARRAY_MERGE_STRATEGY}")
        elseif("${COMPOSE_ARG}" STREQUAL "STRICT")
            list(REMOVE_AT INPUT_FILES 0)
            set(STATICOMA_STRICT ON)
        else()
            break()
        endif()
        list(LENGTH INPUT_FILES LIST_LENGTH)
    endwhile()


    set (COMMAND_DEPENDS "")
    set (COMMAND_INPUTS "")
    list(LENGTH INPUT_FILES LIST_LENGTH)
    while (LIST_LENGTH GREATER 0)
        list(GET INPUT_FILES 0 SOURCE_PACKAGE)
        list(REMOVE_AT INPUT_FILES 0)

        list(GET INPUT_FILES 0 SOURCE_FILE)
        list(REMOVE_AT INPUT_FILES 0)

        if ("${SOURCE_PACKAGE}" STREQUAL "${PROJECT_NAME}")
            if (TARGET "staticoma_${SOURCE_FILE}")
                list(APPEND COMMAND_DEPENDS "staticoma_${SOURCE_FILE}")
                list(APPEND COMMAND_INPUTS "${STATICOMA_BUILD_OUTPUT_DIR}/${SOURCE_FILE}")
            else()
                if (EXISTS "${PROJECT_SOURCE_DIR}/${SOURCE_FILE}")
                    list(APPEND COMMAND_INPUTS "${PROJECT_SOURCE_DIR}/${SOURCE_FILE}")
                else()
                    message(FATAL_ERROR "Could not find file '${SOURCE_FILE}' in the current package.")
                endif()
            endif()
        else()
            if(EXISTS "${${SOURCE_PACKAGE}_STATICOMA}/${SOURCE_FILE}")
                list(APPEND COMMAND_INPUTS "${${SOURCE_PACKAGE}_STATICOMA}/${SOURCE_FILE}")
            else()
                if(EXISTS "${${SOURCE_PACKAGE}_STATICOMA_BUILD}/${SOURCE_FILE}")
                    list(APPEND COMMAND_INPUTS "${${SOURCE_PACKAGE}_STATICOMA_BUILD}/${SOURCE_FILE}")
                else()
                    message(FATAL_ERROR "Could not find file '${SOURCE_FILE}' in package '${SOURCE_PACKAGE}'")
                endif()
            endif()
        endif()
        list(LENGTH INPUT_FILES LIST_LENGTH)
    endwhile()


    set(OUTPUT_FILE_PATH "${STATICOMA_BUILD_OUTPUT_DIR}/${OUTPUT_FILE}")


    list(GET COMMAND_INPUTS 0 FIRST_INPUT_FILE)
    list(REMOVE_AT COMMAND_INPUTS 0)

    # https://mikefarah.gitbook.io/yq/
    set(YQ_MERGE yq merge --inplace --overwrite --comments append --arrays ${STATICOMA_ARRAY_MERGE_STRATEGY})

    add_custom_target(
        staticoma_${OUTPUT_FILE} ALL
        COMMAND ${CMAKE_COMMAND} -E copy   ${FIRST_INPUT_FILE} ${OUTPUT_FILE_PATH}
        COMMAND ${YQ_MERGE} ${OUTPUT_FILE_PATH} ${COMMAND_INPUTS}
        DEPENDS staticoma_global_target ${COMMAND_DEPENDS}
        BYPRODUCTS ${OUTPUT_FILE_PATH}
    )

    if (STATICOMA_STRICT)
        set(OUTPUT_FILE_PATH_STRICT "${PROJECT_BINARY_DIR}/staticoma/${OUTPUT_FILE}.strict")

        # merge without automatic creation of missing entries
        add_custom_target(
            staticoma_strict_step1_${OUTPUT_FILE} ALL
            COMMAND ${CMAKE_COMMAND} -E copy    ${FIRST_INPUT_FILE} ${OUTPUT_FILE_PATH_STRICT}
            COMMAND ${YQ_MERGE} --autocreate=false ${OUTPUT_FILE_PATH_STRICT} ${COMMAND_INPUTS}
            DEPENDS staticoma_global_target ${COMMAND_DEPENDS}
            BYPRODUCTS ${OUTPUT_FILE_PATH_STRICT}
        )

        # merges with/without automatic entry creation must be identical
        add_custom_target(
            staticoma_strict_step2_${OUTPUT_FILE} ALL
            COMMAND yq compare --printMode p ${OUTPUT_FILE_PATH_STRICT} ${OUTPUT_FILE_PATH} '**'
            DEPENDS staticoma_global_target staticoma_${OUTPUT_FILE} staticoma_strict_step1_${OUTPUT_FILE}
        )
    endif()

    install(FILES ${OUTPUT_FILE_PATH} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/staticoma)
endfunction()
