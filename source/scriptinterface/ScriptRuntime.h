/* Copyright (C) 2020 Wildfire Games.
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

#ifndef INCLUDED_SCRIPTRUNTIME
#define INCLUDED_SCRIPTRUNTIME

#include "ScriptTypes.h"
#include "ScriptExtraHeaders.h"

#include <sstream>

#define STACK_CHUNK_SIZE 8192

// Those are minimal defaults. The runtime for the main game is larger and GCs upon a larger growth.
#define DEFAULT_RUNTIME_SIZE 16 * 1024 * 1024
#define DEFAULT_HEAP_GROWTH_BYTES_GCTRIGGER 2 * 1024 * 1024

/**
 * Abstraction around a SpiderMonkey JSRuntime.
 * Each ScriptRuntime can be used to initialize several ScriptInterface
 * contexts which can then share data, but a single ScriptRuntime should
 * only be used on a single thread.
 *
 * (One means to share data between threads and runtimes is to create
 * a ScriptInterface::StructuredClone.)
 */

class ScriptRuntime
{
public:
	ScriptRuntime(shared_ptr<ScriptRuntime> parentRuntime, int runtimeSize, int heapGrowthBytesGCTrigger);
	~ScriptRuntime();

	/**
	 * Returns a runtime, which can used to initialise any number of
	 * ScriptInterfaces contexts. Values created in one context may be used
	 * in any other context from the same runtime (but not any other runtime).
	 * Each runtime should only ever be used on a single thread.
	 * @param parentRuntime Parent runtime from the parent thread, with which we share some thread-safe data
	 * @param runtimeSize Maximum size in bytes of the new runtime
	 * @param heapGrowthBytesGCTrigger Size in bytes of cumulated allocations after which a GC will be triggered
	 */
	static shared_ptr<ScriptRuntime> CreateRuntime(
		shared_ptr<ScriptRuntime> parentRuntime = shared_ptr<ScriptRuntime>(),
		int runtimeSize = DEFAULT_RUNTIME_SIZE,
		int heapGrowthBytesGCTrigger = DEFAULT_HEAP_GROWTH_BYTES_GCTRIGGER);

	/**
	 * MaybeIncrementalRuntimeGC tries to determine whether a runtime-wide garbage collection would free up enough memory to
	 * be worth the amount of time it would take. It does this with our own logic and NOT some predefined JSAPI logic because
	 * such functionality currently isn't available out of the box.
	 * It does incremental GC which means it will collect one slice each time it's called until the garbage collection is done.
	 * This can and should be called quite regularly. The delay parameter allows you to specify a minimum time since the last GC
	 * in seconds (the delay should be a fraction of a second in most cases though).
	 * It will only start a new incremental GC or another GC slice if this time is exceeded. The user of this function is
	 * responsible for ensuring that GC can run with a small enough delay to get done with the work.
	 */
	void MaybeIncrementalGC(double delay);
	void ShrinkingGC();

	void RegisterContext(JSContext* cx);
	void UnRegisterContext(JSContext* cx);

	JSRuntime* m_rt;

private:

	void PrepareContextsForIncrementalGC();

	std::list<JSContext*> m_Contexts;

	int m_RuntimeSize;
	int m_HeapGrowthBytesGCTrigger;
	int m_LastGCBytes;
	double m_LastGCCheck;
};

#endif // INCLUDED_SCRIPTRUNTIME
