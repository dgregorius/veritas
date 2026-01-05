
-- Create Veritas solution
workspace "veritas"

	-- Global settings
	startproject "veritas_testlab"
	
	location "build"
	configurations { "debug", "release" }
	language "C++"
	cppdialect "C++20"
	architecture "x64"
	floatingpoint "Default"
	vectorextensions "AVX2"
	warnings "Extra"
	systemversion( "latest" )
	defines { "_SCL_SECURE_NO_WARNINGS", "_CRT_SECURE_NO_WARNINGS", "_SILENCE_CXX20_IS_POD_DEPRECATION_WARNING" }
	flags { "NoManifest", "MultiProcessorCompile" }
	characterset ( "MBCS" )
	editandcontinue "Off"
	
	disablewarnings 
	{ 
		"4100", -- Unused formal parameter.
		"4201", -- Nameless struct/union. 
		"4324", -- Structure was padded due to alignment specifier
		"4838"  -- 'Unsigned int' to 'int' requires a narrowing conversion (VMATH)
	}

	-- Filters
	filter "configurations:debug"
		objdir "%{prj.location}/obj/debug"
		targetdir "%{prj.location}/out/debug"
		defines { "DEBUG" }
		symbols "On"

	filter "configurations:instrumented"
		objdir "%{prj.location}/obj/instrumented"
		targetdir "%{prj.location}/out/instrumented"
		fatalwarnings { "All" }
		defines { "NDEBUG", "TRACY_ENABLE" }
		optimize "Speed"
			
	filter "configurations:release"
		objdir "%{prj.location}/obj/release"
		targetdir "%{prj.location}/out/release"
		fatalwarnings { "All" }
		defines { "NDEBUG" }
		optimize "Speed"

	-- Externals
	group "externals"

		-- Add EnkiTS 
			project "enkits"
			location "build/externals/enkits"
			kind "StaticLib"
			files { "externals/enkits/**" }	

		-- Add GLAD 
			project "glad"
			location "build/externals/glad"
			kind "StaticLib"
			files { "externals/glad/**" }	

		-- Add IMGUI 
		project "imgui"
			kind "StaticLib"
			location "build/externals/imgui"
			files { "externals/imgui/**" }
			includedirs { "externals/glad", "externals/glfw3/include" }

	-- Plugins
	group "plugins"

		-- Add Veritas Box3d
		project "veritas_box3d"
			kind "SharedLib"
			location "build/plugins/veritas_box3d"
			files { "plugins/veritas_box3d/**" }
			includedirs { "source", "externals/box3d/include" }
			libdirs { "externals/box3d/lib" }
			links { "veritas" }
			filter "configurations:Debug"
				links { "box3dd" }
			filter "configurations:Release"
				links { "box3d" }
   			filter {}
			
			postbuildcommands { "{COPY} %{cfg.targetdir}/veritas_box3d.dll %{_MAIN_SCRIPT_DIR}/bin/plugins/box3d" }
			
		-- Add Veritas Jolt
		project "veritas_jolt"
			kind "SharedLib"
			location "build/plugins/veritas_jolt"
			files { "plugins/veritas_jolt/**" }
			includedirs { "source", "externals/jolt/include" }
			libdirs { "externals/jolt/lib" }
			links { "jolt", "veritas" }	
			
			postbuildcommands { "{COPY} %{cfg.targetdir}/veritas_jolt.dll %{_MAIN_SCRIPT_DIR}/bin/plugins/jolt" }

		-- Add Veritas PhysX
		project "veritas_physx"
			kind "SharedLib"
			location "build/plugins/veritas_physx"
			files { "plugins/veritas_physx/**" }
			includedirs { "source", "externals/physx/include" }
			libdirs { "externals/physx/lib" }
			links { "physx_64", "physxcommon_64", "physxcooking_64", "physxfoundation_64", "veritas" }	

			postbuildcommands { "{COPY} %{cfg.targetdir}/veritas_physx.dll %{_MAIN_SCRIPT_DIR}/bin/plugins/physx" }

	-- Source
	group "source"

		-- Add Veritas
		project "veritas"
			kind "StaticLib"
			location "build/source/veritas/"
			files { "source/veritas/**" }
			includedirs { "source" }

		-- Add Veritas TestLab
		project "veritas_testlab"
			kind "ConsoleApp"
			location "build/source/veritas_testlab"
			files { "source/veritas_testlab/**" }
			includedirs { "source", "externals/glad", "externals/glfw3/include", "externals/imgui" }
			libdirs { "externals/glfw3/lib" }
			links { "glad", "glfw3", "imgui", "veritas" }	
			
			postbuildcommands { "{COPY} %{cfg.targetdir}/veritas_testlab.exe %{_MAIN_SCRIPT_DIR}/bin" }
			debugcommand ( "bin/veritas_testlab.exe" )
			debugdir "bin"