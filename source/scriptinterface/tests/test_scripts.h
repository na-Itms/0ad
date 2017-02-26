/* Copyright (C) 2017 Wildfire Games.
 * This file is part of 0 A.D.
 *
 * 0 A.D. is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * 0 A.D. is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with 0 A.D.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "lib/self_test.h"

#include "ps/XML/Xeromyces.h"
#include "ps/Filesystem.h"

#include "simulation2/system/Component.h"

class TestScripts : public CxxTest::TestSuite
{
public:
	void setUp()
	{
		/**
		 * TODO
		 * Currently this only tests the public mod, mounting the mod mod first as a dependency.
		 * Then we test a handful of hard-coded paths.
		 *
		 * Ideally, each mod should define paths for each part of the script testing. For instance,
		 * mods could have tests.json files alongside mod.json.
		 * For each mod with a tests.json file, we would then set up a VFS with only the tested mod
		 * and its dependencies and run the scripted tests located in the provided paths.
		 */

		g_VFS = CreateVfs(20 * MiB);
		g_VFS->Mount(L"", DataDir()/"mods"/"mod", VFS_MOUNT_MUST_EXIST);
		g_VFS->Mount(L"", DataDir()/"mods"/"public", VFS_MOUNT_MUST_EXIST, 1); // ignore directory-not-found errors
		CXeromyces::Startup();
	}

	void tearDown()
	{
		CXeromyces::Terminate();
		g_VFS.reset();
	}

	static bool check_setups(const char* testName, const VfsPaths& pathnames)
	{
		for (const VfsPath& pathname : pathnames)
		{
			if (VfsFileExists(pathname))
				continue;
			debug_printf("WARNING: Skipping %s tests (can't find %s)\n", testName, pathname.string8().c_str());
			return false;
		}
		return true;
	}

	static void load_script(ScriptInterface& scriptInterface, const VfsPath& pathname)
	{
		TSM_ASSERT(L"Running script "+pathname.string(), scriptInterface.LoadScriptFile(pathname));
	}

	static void Script_LoadGlobalScript(ScriptInterface::CxPrivate* pCxPrivate, const VfsPath& pathname)
	{
		TS_ASSERT(pCxPrivate->pScriptInterface->LoadGlobalScriptFile(VfsPath(L"globalscripts") / pathname));
	}
	static void Script_LoadSimulationHelperScript(ScriptInterface::CxPrivate* pCxPrivate, const VfsPath& pathname)
	{
		TS_ASSERT(pCxPrivate->pScriptInterface->LoadScriptFile(VfsPath(L"simulation/helpers") / pathname));
	}
	static void Script_LoadSimulationComponentScript(ScriptInterface::CxPrivate* pCxPrivate, const VfsPath& pathname)
	{
		TS_ASSERT(pCxPrivate->pScriptInterface->LoadScriptFile(VfsPath(L"simulation/components") / pathname));
	}

	void test_globalscripts()
	{
		const char* name = "globalscripts";
		VfsPaths setups = {
			L"globalscripts/tests/setup.js"
		};
		VfsPath folder = L"globalscripts/tests/";

		if (!check_setups(name, setups))
			return;

		VfsPaths paths;
		TS_ASSERT_OK(vfs::GetPathnames(g_VFS, folder, L"test_*.js", paths));
		std::reverse(paths.begin(), paths.end());

		for (const VfsPath& path : paths)
		{
			// The script interface is barebones, so global scripts are tested in an general environment.
			ScriptInterface scriptInterface("Engine", "globalscript tests", g_ScriptRuntime);

			ScriptTestSetup(scriptInterface);
			scriptInterface.RegisterFunction<void, VfsPath, Script_LoadGlobalScript>("LoadGlobalScript");

			for (const VfsPath& setup : setups)
				load_script(scriptInterface, setup);

			load_script(scriptInterface, path);
		}
	}

	void test_simulation()
	{
		const char* name = "simulation";
		VfsPaths setups = {
			L"simulation/tests/setup.js"
		};
		VfsPath folder = L"simulation/tests/";

		if (!check_setups(name, setups))
			return;

		VfsPaths paths;
		TS_ASSERT_OK(vfs::GetPathnames(g_VFS, folder, L"test_*.js", paths));

		for (const VfsPath& path : paths)
		{
			CSimContext context;
			CComponentManager componentManager(context, g_ScriptRuntime, true);

			ScriptTestSetup(componentManager.GetScriptInterface());
			componentManager.GetScriptInterface().RegisterFunction<void, VfsPath, Script_LoadSimulationHelperScript>("LoadHelperScript");

			componentManager.LoadComponentTypes();

			for (const VfsPath& setup : setups)
				load_script(componentManager.GetScriptInterface(), setup);

			load_script(componentManager.GetScriptInterface(), path);
		}
	}

	void test_simulation_components()
	{
		const char* name = "simulation components";
		VfsPaths setups = {
			L"simulation/tests/setup.js",
			L"simulation/components/tests/setup.js"
		};
		VfsPath folder = L"simulation/components/tests/";

		if (!check_setups(name, setups))
			return;

		VfsPaths paths;
		TS_ASSERT_OK(vfs::GetPathnames(g_VFS, folder, L"test_*.js", paths));

		for (const VfsPath& path : paths)
		{
			CSimContext context;
			CComponentManager componentManager(context, g_ScriptRuntime, true);

			ScriptTestSetup(componentManager.GetScriptInterface());
			componentManager.GetScriptInterface().RegisterFunction<void, VfsPath, Script_LoadSimulationHelperScript>("LoadHelperScript");
			componentManager.GetScriptInterface().RegisterFunction<void, VfsPath, Script_LoadSimulationComponentScript>("LoadComponentScript");

			componentManager.LoadComponentTypes();

			for (const VfsPath& setup : setups)
				load_script(componentManager.GetScriptInterface(), setup);

			load_script(componentManager.GetScriptInterface(), path);
		}
	}
};
