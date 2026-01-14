include(GNUInstallDirs)

set(BOX3D_SOURCE_FILES

	aabb.cpp
	aabb.h
	algorithm.h
	arena_allocator.c
	arena_allocator.h
	array.h
	bitset.cpp
	bitset.h
	block_allocator.cpp
	block_allocator.h
	body.cpp
	body.h
	broad_phase.cpp
	broad_phase.h
	capsule.cpp
	compound.c
	compound.h
	constants.h
	constraint_graph.cpp
	constraint_graph.h
	contact.cpp
	contact.h
	contact_solver.cpp
	contact_solver.h
	convex_manifold.cpp
	core.cpp
	core.h
	ctz.h
	distance.cpp
	distance_joint.cpp
	dynamic_tree.cpp
	heap.cpp
	heap.h
	height_field.cpp
	hull.cpp
	id_pool.cpp
	id_pool.h
	island.cpp
	island.h
	joint.cpp
	joint.h
	math_functions.cpp
	math_internal.h
	mesh.cpp
	mesh_contact.cpp
	motor_joint.cpp
	mover.cpp
	parallel_joint.cpp
	physics_world.cpp
	physics_world.h
	platform.h
	pool.h
	pool_allocator.c
	pool_allocator.h
	prismatic_joint.cpp
	qhull.cpp
	qhull.h
	qsort.h
	revolute_joint.cpp
	sat.cpp
	sat.h
	sensor.cpp
	sensor.h
	shape.cpp
	shape.h
	simd.cpp
	simd.h
	solver.cpp
	solver.h
	solver_set.cpp
	solver_set.h
	sphere.cpp
	spherical_joint.cpp
	stack_array.h
	table.cpp
	table.h
	timer.c
	toi.cpp
	triangle_manifold.cpp
	types.cpp
	verstable.h
	weld_joint.cpp
	wheel_joint.cpp

	box3d.natvis
)

set(BOX3D_INCLUDE_FILES
	../include/box3d/base.h
	../include/box3d/box3d.h
	../include/box3d/collision.h
	../include/box3d/id.h
	../include/box3d/math_functions.h
	../include/box3d/types.h
)

# Hide internal functions
# https://gcc.gnu.org/wiki/Visibility
set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

add_library(box3d STATIC ${BOX3D_SOURCE_FILES} ${BOX3D_INCLUDE_FILES})

# target_link_libraries(box3d PRIVATE eastl)

target_include_directories(box3d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(CMAKE_DEBUG_POSTFIX "d")

# Box3D uses C++20
set_target_properties(box3d PROPERTIES
	CXX_STANDARD 20
	CXX_STANDARD_REQUIRED YES
	CXX_EXTENSIONS NO
	C_STANDARD 17
    C_STANDARD_REQUIRED YES
	# Need extensions for clock_gettime, etc
    C_EXTENSIONS YES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
	DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
)

if (BOX3D_COMPILE_WARNING_AS_ERROR)
	set_target_properties(box3d PROPERTIES COMPILE_WARNING_AS_ERROR ON)
endif()

if (BOX3D_PROFILE)
	target_compile_definitions(box3d PRIVATE BOX3D_PROFILE)

	FetchContent_Declare(
		tracy
		GIT_REPOSITORY https://github.com/wolfpld/tracy.git
		GIT_TAG v0.11.1
		GIT_SHALLOW TRUE
		GIT_PROGRESS TRUE
	)
	FetchContent_MakeAvailable(tracy)

	target_link_libraries(box3d PUBLIC TracyClient)
endif()

if(BOX3D_VALIDATE)
	message(STATUS "Box3D validation ON")	
	target_compile_definitions(box3d PRIVATE BOX3D_VALIDATE)
endif()

if (BOX3D_DISABLE_SIMD)
	message(STATUS "Box3D SIMD disabled")	
	target_compile_definitions(box3d PRIVATE BOX3D_DISABLE_SIMD)
endif()

if (MSVC)
	message(STATUS "Box3D on MSVC")	
	if (BUILD_SHARED_LIBS)
		# this is needed by DLL users to import Box3D symbols
		target_compile_definitions(box3d INTERFACE BOX3D_DLL)
	endif()

	# Visual Studio won't load the natvis unless it is in the project
	target_sources(box3d PRIVATE box3d.natvis)

	# Enable asserts in release with debug info
	target_compile_definitions(box3d PUBLIC "$<$<CONFIG:RELWITHDEBINFO>:B3_ENABLE_ASSERT>")

	# Warnings Wall is problematic in mixed C/C++, so using W4
	target_compile_options(box3d PRIVATE /W4)
	# target_compile_options(box3d PRIVATE /wd4820 /wd5045 /wd4061 /wd4711 /wd4514 /wd4365 /wd5219 /wd5039)
	target_compile_options(box3d PRIVATE /wd4820 /wd5045 /wd4061 /wd4711)
	# 4710 - warn about inline functions that are not inlined
	target_compile_options(box3d PRIVATE /wd4710 )

	if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
		target_compile_options(box3d PRIVATE -Wmissing-prototypes)
	endif()

elseif (MINGW)
	message(STATUS "Box3D on MinGW")
elseif (APPLE)
	message(STATUS "Box3D on Apple")
	target_compile_options(box3d PRIVATE -Wmissing-prototypes -Wall -Wextra -pedantic)
elseif (EMSCRIPTEN)
	message(STATUS "Box3D on Emscripten")
	target_compile_options(box3d PRIVATE -msimd128 -msse2)
elseif (UNIX)
	message(STATUS "Box3D using Unix")
	target_compile_options(box3d PRIVATE -Wmissing-prototypes -Wall -Wextra -pedantic -Wno-unused-value)
	if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
		# raspberry pi
		# -mfpu=neon
		# target_compile_options(box3d PRIVATE)
	else()
	endif()
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${BOX3D_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${BOX3D_INCLUDE_FILES})

add_library(box3d::box3d ALIAS box3d)

install(
  TARGETS box3d
  EXPORT box3dConfig
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  EXPORT box3dConfig
  NAMESPACE box2d::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box3d"
)

install(
  FILES ${BOX3D_API_FILES}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/box3d
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/box3dConfigVersion.cmake"
  VERSION ${BOX3D_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/box3dConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box3d"
)
