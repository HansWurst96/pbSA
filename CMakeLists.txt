cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
set(CMAKE_BUILD_TYPE_INIT Debug)

project(SimulationFramework)
if(DEFINED CMAKE_BUILD_TYPE)
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release MinSizeRel RelWithDebInfo)
endif()

set(GLAD_API gl=3.3 CACHE INTERNAL "")
set(GLAD_EXPORT ON CACHE INTERNAL "")
set(GLAD_EXTENSIONS GL_EXT_texture_compression_s3tc,GL_EXT_texture_sRGB,GL_EXT_texture_filter_anisotropic CACHE INTERNAL "") # https://www.khronos.org/opengl/wiki/Ubiquitous_Extension
set(GLAD_GENERATOR c CACHE INTERNAL "")
set(GLAD_INSTALL OFF CACHE INTERNAL "")
set(GLAD_NO_LOADER ON CACHE INTERNAL "")
set(GLAD_PROFILE core CACHE INTERNAL "")
set(GLAD_SPEC gl CACHE INTERNAL "")
add_subdirectory(glad)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_executable(${PROJECT_NAME} "")
set_directory_properties(
	PROPERTIES
	VS_STARTUP_PROJECT ${PROJECT_NAME}
)

target_sources(
	${PROJECT_NAME}
	PRIVATE
	main.cpp
	OpenGLWidget.cpp OpenGLWidget.hpp
	OpenGLRenderer.hpp
	GLMainWindow.cpp GLMainWindow.hpp GLMainWindow.ui
	ExampleRenderer.cpp ExampleRenderer.hpp
	shaders.qrc
	shaders/icosahedron.vert shaders/icosahedron.frag
	shaders/skybox.vert shaders/skybox.frag
	icon.qrc
	textures.qrc
)

target_link_libraries(
	${PROJECT_NAME}
	PRIVATE
	glad
)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

set(_QT_COMPONENTS Core Gui Widgets)
if(WIN32)
	set_property(TARGET ${PROJECT_NAME} PROPERTY WIN32_EXECUTABLE ON)

	target_sources(
		${PROJECT_NAME}
		PRIVATE
		icon.rc
	)

	file(GLOB _ICU_DLLS ${Qt5_DIR}/../../../bin/icu*[0-9].dll)
	set(_DEPENDENCIES)
	foreach(_ICU_DLL ${_ICU_DLLS})
		string(REPLACE ".dll" "$<$<CONFIG:Debug>:d>.dll" _ICU_DLL ${_ICU_DLL})
		list(APPEND _DEPENDENCIES ${_ICU_DLL})
	endforeach()
	foreach(_COMP ${_QT_COMPONENTS})
		list(APPEND _DEPENDENCIES $<TARGET_FILE:Qt5::${_COMP}>)
	endforeach()

	add_custom_command(
		TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS
		"-DSRC=${_DEPENDENCIES}"
		"-DDST=$<TARGET_FILE_DIR:${PROJECT_NAME}>"
		-P "${PROJECT_SOURCE_DIR}/InstallFile.cmake"
	)
	add_custom_command(
		TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS
		"-DSRC=${Qt5_DIR}/../../../plugins/platforms/qwindows$<$<CONFIG:Debug>:d>.dll"
		"-DDST=$<TARGET_FILE_DIR:${PROJECT_NAME}>/platforms"
		-P "${PROJECT_SOURCE_DIR}/InstallFile.cmake"
	)
	add_custom_command(
		TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} ARGS
		"-DSRC=${Qt5_DIR}/../../../plugins/imageformats/qico$<$<CONFIG:Debug>:d>.dll;${Qt5_DIR}/../../../plugins/imageformats/qjpeg$<$<CONFIG:Debug>:d>.dll"
		"-DDST=$<TARGET_FILE_DIR:${PROJECT_NAME}>/imageformats"
		-P "${PROJECT_SOURCE_DIR}/InstallFile.cmake"
	)
endif()

find_package(Qt5 5.4 REQUIRED COMPONENTS ${_QT_COMPONENTS})
foreach(_COMP ${_QT_COMPONENTS})
	target_link_libraries(
		${PROJECT_NAME}
		PRIVATE
		Qt5::${_COMP}
	)
endforeach()

set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)
set_property(GLOBAL PROPERTY AUTOGEN_SOURCE_GROUP "Generated Files")

target_include_directories(
	${PROJECT_NAME}
	PRIVATE
	${PROJECT_BINARY_DIR}
	${PROJECT_SOURCE_DIR}
	eigen
	eigen/unsupported
)

source_group("Form Files" REGULAR_EXPRESSION "\\.ui$")
source_group("Resource Files" REGULAR_EXPRESSION "\\.q?rc$")
source_group("Shader Files" REGULAR_EXPRESSION "\\.(vert|frag)$")
get_target_property(_SOURCES ${PROJECT_NAME} SOURCES)
foreach(_SOURCE ${_SOURCES})
	if(_SOURCE MATCHES "\\.(ui|qrc)$")
		if(_SOURCE MATCHES "\\.ui$")
			qt5_wrap_ui(_GENERATED ${_SOURCE})
		elseif(_SOURCE MATCHES "\\.qrc$")
			qt5_add_resources(_GENERATED ${_SOURCE})
		endif()
		target_sources(${PROJECT_NAME} PRIVATE ${_GENERATED})
		source_group("Generated Files" FILES ${_GENERATED})
	endif()
endforeach()
