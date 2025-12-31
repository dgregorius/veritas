
-- Create Ragnarok solution
workspace "physicslab"

	-- Global settings
	startproject "testlab"
	
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

		-- Add TestLab
		project "testlab"
			kind "ConsoleApp"
			location "build/source/"
			files { "source/**" }
			includedirs { "source", "externals/box3d/include", "externals/glad", "externals/glm/include", "externals/glfw3/include", "externals/imgui", "externals/jolt/include", "externals/physx/include" }
			links { "box3d", "glad", "glfw3", "jolt", "imgui", "physx_64", "physxcommon_64", "physxcooking_64", "physxfoundation_64" }	
			libdirs { "externals/box3d/lib", "externals/glfw3/lib", "externals/jolt/lib" ,"externals/physx/lib" }
			
			postbuildcommands { "{COPY} %{cfg.targetdir}/testlab.exe %{_MAIN_SCRIPT_DIR}/bin" }
			debugcommand ( "bin/testlab.exe" )
			debugdir "bin"