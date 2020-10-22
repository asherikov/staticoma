function(staticoma)
    if(CATKIN_PACKAGE_SHARE_DESTINATION)
        message(FATAL_ERROR "staticoma() must be called before catkin_package()")
    endif()


    add_custom_target(
        staticoma_global_target ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory "${PROJECT_SOURCE_DIR}/staticoma/"
    )

    file(
        WRITE
        "${PROJECT_SOURCE_DIR}/cmake/staticoma_${PROJECT_NAME}.cmake.in"
"set(${PROJECT_NAME}_STATICOMA \"${CMAKE_INSTALL_PREFIX}/@CATKIN_PACKAGE_SHARE_DESTINATION@/staticoma\" CACHE STRING \"\" FORCE)
set(${PROJECT_NAME}_STATICOMA_BUILD \"${PROJECT_SOURCE_DIR}/staticoma\" CACHE STRING \"\" FORCE)"
    )


    list(APPEND ${PROJECT_NAME}_CFG_EXTRAS "staticoma_${PROJECT_NAME}.cmake")
    set(${PROJECT_NAME}_CFG_EXTRAS "${${PROJECT_NAME}_CFG_EXTRAS}" PARENT_SCOPE)
endfunction()
