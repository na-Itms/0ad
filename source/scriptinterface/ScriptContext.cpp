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

#include "precompiled.h"

#include "ScriptContext.h"

#include "ps/GameSetup/Config.h"
#include "ps/Profile.h"
#include "scriptinterface/ScriptEngine.h"
#include "scriptinterface/ScriptInterface.h"


void GCSliceCallbackHook(JSContext* UNUSED(cx), JS::GCProgress progress, const JS::GCDescription& UNUSED(desc))
{
	/*
	 * During non-incremental GC, the GC is bracketed by JSGC_CYCLE_BEGIN/END
	 * callbacks. During an incremental GC, the sequence of callbacks is as
	 * follows:
	 *   JSGC_CYCLE_BEGIN, JSGC_SLICE_END  (first slice)
	 *   JSGC_SLICE_BEGIN, JSGC_SLICE_END  (second slice)
	 *   ...
	 *   JSGC_SLICE_BEGIN, JSGC_CYCLE_END  (last slice)
	*/


	if (progress == JS::GC_SLICE_BEGIN)
	{
		if (CProfileManager::IsInitialised() && ThreadUtil::IsMainThread())
			g_Profiler.Start("GCSlice");
		g_Profiler2.RecordRegionEnter("GCSlice");
	}
	else if (progress == JS::GC_SLICE_END)
	{
		if (CProfileManager::IsInitialised() && ThreadUtil::IsMainThread())
			g_Profiler.Stop();
    	g_Profiler2.RecordRegionLeave();
	}
	else if (progress == JS::GC_CYCLE_BEGIN)
	{
		if (CProfileManager::IsInitialised() && ThreadUtil::IsMainThread())
			g_Profiler.Start("GCSlice");
		g_Profiler2.RecordRegionEnter("GCSlice");
	}
	else if (progress == JS::GC_CYCLE_END)
	{
		if (CProfileManager::IsInitialised() && ThreadUtil::IsMainThread())
			g_Profiler.Stop();
    	g_Profiler2.RecordRegionLeave();
	}

	// The following code can be used to print some information aobut garbage collection
	// Search for "Nonincremental reason" if there are problems running GC incrementally.
	#if 0
	if (progress == JS::GCProgress::GC_CYCLE_BEGIN)
		printf("starting cycle ===========================================\n");

	const char16_t* str = desc.formatMessage(cx);
	int len = 0;

	for(int i = 0; i < 10000; i++)
	{
		len++;
		if(!str[i])
			break;
	}

	wchar_t outstring[len];

	for(int i = 0; i < len; i++)
	{
		outstring[i] = (wchar_t)str[i];
	}

	printf("---------------------------------------\n: %ls \n---------------------------------------\n", outstring);
	#endif
}

shared_ptr<ScriptContext> ScriptContext::CreateContext(shared_ptr<ScriptContext> parentContext, int contextSize, int heapGrowthBytesGCTrigger)
{
	return shared_ptr<ScriptContext>(new ScriptContext(parentContext, contextSize, heapGrowthBytesGCTrigger));
}

namespace {

void ErrorReporter(JSContext* cx, const char* message, JSErrorReport* report)
{
	JSAutoRequest rq(cx);

	std::stringstream msg;
	bool isWarning = JSREPORT_IS_WARNING(report->flags);
	msg << (isWarning ? "JavaScript warning: " : "JavaScript error: ");
	if (report->filename)
	{
		msg << report->filename;
		msg << " line " << report->lineno << "\n";
	}

	msg << message;

	// If there is an exception, then print its stack trace
	JS::RootedValue excn(cx);
	if (JS_GetPendingException(cx, &excn) && excn.isObject())
	{
		JS::RootedValue stackVal(cx);
		JS::RootedObject excnObj(cx, &excn.toObject());
		JS_GetProperty(cx, excnObj, "stack", &stackVal);

		std::string stackText;
		ScriptInterface::FromJSVal(cx, stackVal, stackText);

		std::istringstream stream(stackText);
		for (std::string line; std::getline(stream, line);)
			msg << "\n  " << line;
	}

	if (isWarning)
		LOGWARNING("%s", msg.str().c_str());
	else
		LOGERROR("%s", msg.str().c_str());

	// When running under Valgrind, print more information in the error message
//	VALGRIND_PRINTF_BACKTRACE("->");
}

} // anonymous namespace

ScriptContext::ScriptContext(shared_ptr<ScriptContext> parentContext, int contextSize, int heapGrowthBytesGCTrigger):
	m_LastGCBytes(0),
	m_LastGCCheck(0.0f),
	m_HeapGrowthBytesGCTrigger(heapGrowthBytesGCTrigger),
	m_ContextSize(contextSize)
{
	ENSURE(ScriptEngine::IsInitialised() && "The ScriptEngine must be initialized before constructing any ScriptContexts!");

	JSContext* parentJSContext = parentContext ? parentContext->m_cx : nullptr;
	m_cx = JS_NewContext(contextSize, JS::DefaultNurseryBytes, parentJSContext);
	ENSURE(m_cx); // TODO: error handling

	ENSURE(JS::InitSelfHostedCode(m_cx));

	JS::SetGCSliceCallback(m_cx, GCSliceCallbackHook);

	JS_SetGCParameter(m_cx, JSGC_MAX_MALLOC_BYTES, m_ContextSize);
	JS_SetGCParameter(m_cx, JSGC_MAX_BYTES, m_ContextSize);
	JS_SetGCParameter(m_cx, JSGC_MODE, JSGC_MODE_INCREMENTAL);

	// The whole heap-growth mechanism seems to work only for non-incremental GCs.
	// We disable it to make it more clear if full GCs happen triggered by this JSAPI internal mechanism.
	JS_SetGCParameter(m_cx, JSGC_DYNAMIC_HEAP_GROWTH, false);

	JS_SetOffthreadIonCompilationEnabled(m_cx, true);

	// For GC debugging:
	// JS_SetGCZeal(m_cx, 2, JS_DEFAULT_ZEAL_FREQ);

	JS_SetContextPrivate(m_cx, nullptr);

	JS_SetErrorReporter(m_cx, ErrorReporter);

	JS_SetGlobalJitCompilerOption(m_cx, JSJITCOMPILER_ION_ENABLE, 1);
	JS_SetGlobalJitCompilerOption(m_cx, JSJITCOMPILER_BASELINE_ENABLE, 1);

	JS::ContextOptionsRef(m_cx)
		.setExtraWarnings(true)
		.setWerror(false)
		.setStrictMode(true);

	ScriptEngine::GetSingleton().RegisterContext(m_cx);
}

ScriptContext::~ScriptContext()
{
	ENSURE(ScriptEngine::IsInitialised() && "The ScriptEngine must be active (initialized and not yet shut down) when destroying a ScriptContext!");

	JS_DestroyContext(m_cx);
	ScriptEngine::GetSingleton().UnRegisterContext(m_cx);
}

void ScriptContext::RegisterCompartment(JSCompartment* cmpt)
{
	ENSURE(cmpt);
	m_Compartments.push_back(cmpt);
}

void ScriptContext::UnRegisterCompartment(JSCompartment* cmpt)
{
	m_Compartments.remove(cmpt);
}

#define GC_DEBUG_PRINT 0
void ScriptContext::MaybeIncrementalGC(double delay)
{
	PROFILE2("MaybeIncrementalGC");

	if (JS::IsIncrementalGCEnabled(m_cx))
	{
		// The idea is to get the heap size after a completed GC and trigger the next GC when the heap size has
		// reached m_LastGCBytes + X.
		// In practice it doesn't quite work like that. When the incremental marking is completed, the sweeping kicks in.
		// The sweeping actually frees memory and it does this in a background thread (if JS_USE_HELPER_THREADS is set).
		// While the sweeping is happening we already run scripts again and produce new garbage.

		const int GCSliceTimeBudget = 30; // Milliseconds an incremental slice is allowed to run

		// Have a minimum time in seconds to wait between GC slices and before starting a new GC to distribute the GC
		// load and to hopefully make it unnoticeable for the player. This value should be high enough to distribute
		// the load well enough and low enough to make sure we don't run out of memory before we can start with the
		// sweeping.
		if (timer_Time() - m_LastGCCheck < delay)
			return;

		m_LastGCCheck = timer_Time();

		int gcBytes = JS_GetGCParameter(m_cx, JSGC_BYTES);

#if GC_DEBUG_PRINT
			std::cout << "gcBytes: " << gcBytes / 1024 << " KB" << std::endl;
#endif

		if (m_LastGCBytes > gcBytes || m_LastGCBytes == 0)
		{
#if GC_DEBUG_PRINT
			printf("Setting m_LastGCBytes: %d KB \n", gcBytes / 1024);
#endif
			m_LastGCBytes = gcBytes;
		}

		// Run an additional incremental GC slice if the currently running incremental GC isn't over yet
		// ... or
		// start a new incremental GC if the JS heap size has grown enough for a GC to make sense
		if (JS::IsIncrementalGCInProgress(m_cx) || (gcBytes - m_LastGCBytes > m_HeapGrowthBytesGCTrigger))
		{
#if GC_DEBUG_PRINT
			if (JS::IsIncrementalGCInProgress(m_cx))
				printf("An incremental GC cycle is in progress. \n");
			else
				printf("GC needed because JSGC_BYTES - m_LastGCBytes > m_HeapGrowthBytesGCTrigger \n"
					"    JSGC_BYTES: %d KB \n    m_LastGCBytes: %d KB \n    m_HeapGrowthBytesGCTrigger: %d KB \n",
					gcBytes / 1024,
					m_LastGCBytes / 1024,
					m_HeapGrowthBytesGCTrigger / 1024);
#endif

			// A hack to make sure we never exceed the context size because we can't collect the memory
			// fast enough.
			if (gcBytes > m_ContextSize / 2)
			{
				if (JS::IsIncrementalGCInProgress(m_cx))
				{
#if GC_DEBUG_PRINT
					printf("Finishing incremental GC because gcBytes > m_ContextSize / 2. \n");
#endif
					PrepareCompartmentsForIncrementalGC();
					JS::FinishIncrementalGC(m_cx, JS::gcreason::REFRESH_FRAME);
				}
				else
				{
					if (gcBytes > m_ContextSize * 0.75)
					{
						ShrinkingGC();
#if GC_DEBUG_PRINT
						printf("Running shrinking GC because gcBytes > m_ContextSize * 0.75. \n");
#endif
					}
					else
					{
#if GC_DEBUG_PRINT
						printf("Running full GC because gcBytes > m_ContextSize / 2. \n");
#endif
						JS_GC(m_cx);
					}
				}
			}
			else
			{
#if GC_DEBUG_PRINT
				if (!JS::IsIncrementalGCInProgress(m_cx))
					printf("Starting incremental GC \n");
				else
					printf("Running incremental GC slice \n");
#endif
				PrepareCompartmentsForIncrementalGC();
				if (!JS::IsIncrementalGCInProgress(m_cx))
					JS::StartIncrementalGC(m_cx, GC_NORMAL, JS::gcreason::REFRESH_FRAME, GCSliceTimeBudget);
				else
					JS::IncrementalGCSlice(m_cx, JS::gcreason::REFRESH_FRAME, GCSliceTimeBudget);
			}
			m_LastGCBytes = gcBytes;
		}
	}
}

void ScriptContext::ShrinkingGC()
{
	JS_SetGCParameter(m_cx, JSGC_MODE, JSGC_MODE_ZONE);
	JS::PrepareForFullGC(m_cx);
	JS::GCForReason(m_cx, GC_SHRINK, JS::gcreason::REFRESH_FRAME);
	JS_SetGCParameter(m_cx, JSGC_MODE, JSGC_MODE_INCREMENTAL);
}

void ScriptContext::PrepareCompartmentsForIncrementalGC() const
{
	for (JSCompartment* const& cmpt : m_Compartments)
		JS::PrepareZoneForGC(js::GetCompartmentZone(cmpt));
}
