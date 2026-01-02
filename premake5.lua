
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
			
	-- Source
	group "source"

		-- Add Veritas
		project "veritas"
			kind "StaticLib"
			location "build/source/veritas/"
			files { "source/veritas/**" }
			includedirs { "source" }

		-- Add Veritas Box3d
			project "veritas_box3d"
				kind "StaticLib"
				location "build/source/veritas_box3d"
				files { "source/veritas_box3d/**" }
				includedirs { "source" }

			-- Add Veritas Jolt
			project "veritas_jolt"
				kind "StaticLib"
				location "build/source/veritas_jolt"
				files { "source/veritas_jolt/**" }
				includedirs { "source" }

			-- Add Veritas PhysX
			project "veritas_physx"
				kind "StaticLib"
				location "build/source/veritas_physx"
				files { "source/veritas_physx/**" }
				includedirs { "source" }

		-- Add Veritas TestLab
		project "veritas_testlab"
			kind "ConsoleApp"
			location "build/source/veritas_testlab"
			files { "source/veritas_testlab/**" }
			includedirs { "source", "externals/glad", "externals/glfw3/include", "externals/imgui" }
			links { "glad", "glfw3", "imgui" }	
			libdirs { "externals/glfw3/lib" }
			
			postbuildcommands { "{COPY} %{cfg.targetdir}/veritas_testlab.exe %{_MAIN_SCRIPT_DIR}/bin" }
			debugcommand ( "bin/veritas_testlab.exe" )
			debugdir "bin"

		