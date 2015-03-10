/* Copyright (C) 2015 Wildfire Games.
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

#ifndef INCLUDED_CCMPPATHFINDER_COMMON
#define INCLUDED_CCMPPATHFINDER_COMMON

/**
 * @file
 * Declares CCmpPathfinder, whose implementation is split into multiple source files,
 * and provides common code needed for more than one of those files.
 * CCmpPathfinder includes two pathfinding algorithms (one tile-based, one vertex-based)
 * with some shared state and functionality, so the code is split into
 * CCmpPathfinder_Vertex.cpp, CCmpPathfinder_Tile.cpp and CCmpPathfinder.cpp
 */

#include "simulation2/system/Component.h"

#include "ICmpPathfinder.h"

#include "graphics/Overlay.h"
#include "graphics/Terrain.h"
#include "maths/MathUtil.h"
#include "ps/CLogger.h"
#include "renderer/Scene.h"
#include "simulation2/components/ICmpObstructionManager.h"
#include "simulation2/helpers/Geometry.h"
#include "simulation2/helpers/Grid.h"
#include "simulation2/helpers/HierarchicalPathfinder.h"

class PathfinderOverlay;
class SceneCollector;
struct PathfindTile;
struct PathfindTileJPS;
class JumpPointCache;

#ifdef NDEBUG
#define PATHFIND_DEBUG 0
#else
#define PATHFIND_DEBUG 1
#endif

#define PATHFIND_USE_JPS 1

typedef SparseGrid<PathfindTile> PathfindTileGrid;
typedef SparseGrid<PathfindTileJPS> PathfindTileJPSGrid;

/**
 * Represents the 2D coordinates of a tile.
 * The i/j components are packed into a single u32, since we usually use these
 * objects for equality comparisons and the VC2010 optimizer doesn't seem to automatically
 * compare two u16s in a single operation.
 * TODO: maybe VC2012 will?
 */
struct TileID
{
	TileID() { }

	TileID(u16 i, u16 j) : data((i << 16) | j) { }

	bool operator==(const TileID& b) const
	{
		return data == b.data;
	}

	/// Returns lexicographic order over (i,j)
	bool operator<(const TileID& b) const
	{
		return data < b.data;
	}

	u16 i() const { return data >> 16; }
	u16 j() const { return data & 0xFFFF; }

private:
	u32 data;
};

/**
 * Represents the cost of a path consisting of horizontal/vertical and
 * diagonal movements over a uniform-cost grid.
 * Maximum path length before overflow is about 45K steps.
 */
struct PathCost
{
	PathCost() : data(0) { }

	/// Construct from a number of horizontal/vertical and diagonal steps
	PathCost(u16 hv, u16 d)
		: data(hv*65536 + d*92682) // 2^16 * sqrt(2) == 92681.9
	{
	}

	/// Construct for horizontal/vertical movement of given number of steps
	static PathCost horizvert(u16 n)
	{
		return PathCost(n, 0);
	}

	/// Construct for diagonal movement of given number of steps
	static PathCost diag(u16 n)
	{
		return PathCost(0, n);
	}

	PathCost operator+(const PathCost& a) const
	{
		PathCost c;
		c.data = data + a.data;
		return c;
	}

	bool operator<=(const PathCost& b) const { return data <= b.data; }
	bool operator< (const PathCost& b) const { return data <  b.data; }
	bool operator>=(const PathCost& b) const { return data >= b.data; }
	bool operator> (const PathCost& b) const { return data >  b.data; }

	u32 ToInt()
	{
		return data;
	}

private:
	u32 data;
};



struct AsyncLongPathRequest
{
	u32 ticket;
	entity_pos_t x0;
	entity_pos_t z0;
	PathGoal goal;
	pass_class_t passClass;
	entity_id_t notify;
};

struct AsyncShortPathRequest
{
	u32 ticket;
	entity_pos_t x0;
	entity_pos_t z0;
	entity_pos_t r;
	entity_pos_t range;
	PathGoal goal;
	pass_class_t passClass;
	bool avoidMovingUnits;
	entity_id_t group;
	entity_id_t notify;
};

/**
 * Implementation of ICmpPathfinder
 */
class CCmpPathfinder : public ICmpPathfinder
{
public:
	static void ClassInit(CComponentManager& componentManager)
	{
		componentManager.SubscribeToMessageType(MT_Update);
		componentManager.SubscribeToMessageType(MT_RenderSubmit); // for debug overlays
		componentManager.SubscribeToMessageType(MT_TerrainChanged);
		componentManager.SubscribeToMessageType(MT_WaterChanged);
		componentManager.SubscribeToMessageType(MT_ObstructionMapShapeChanged);
		componentManager.SubscribeToMessageType(MT_TurnStart);
	}

	DEFAULT_COMPONENT_ALLOCATOR(Pathfinder)

	// Template state:

	std::map<std::string, pass_class_t> m_PassClassMasks;
	std::vector<PathfinderPassability> m_PassClasses;

	// Dynamic state:

	std::vector<AsyncLongPathRequest> m_AsyncLongPathRequests;
	std::vector<AsyncShortPathRequest> m_AsyncShortPathRequests;
	u32 m_NextAsyncTicket; // unique IDs for asynchronous path requests
	u16 m_SameTurnMovesCount; // current number of same turn moves we have processed this turn

	// Lazily-constructed dynamic state (not serialized):

	u16 m_MapSize; // tiles per side
	Grid<NavcellData>* m_Grid; // terrain/passability information
	Grid<NavcellData>* m_BaseGrid; // same as m_Grid, but only with terrain, to avoid some recomputations
	size_t m_ObstructionGridDirtyID; // dirty ID for ICmpObstructionManager::NeedUpdate
	bool m_TerrainDirty; // indicates if m_Grid has been updated since terrain changed
	
	std::map<pass_class_t, shared_ptr<JumpPointCache> > m_JumpPointCache; // for JPS pathfinder

	// Interface to the hierarchical pathfinder.
	HierarchicalPathfinder* m_PathfinderHier;
	void PathfinderHierRecompute()
	{
		m_PathfinderHier->Recompute(m_PassClassMasks, m_Grid);
	}
	void PathfinderHierRenderSubmit(SceneCollector& collector)
	{
		for (size_t i = 0; i < m_PathfinderHier->m_DebugOverlayLines.size(); ++i)
			collector.Submit(&m_PathfinderHier->m_DebugOverlayLines[i]);
	}

	void PathfinderJPSMakeDirty();

	// For responsiveness we will process some moves in the same turn they were generated in
	
	u16 m_MaxSameTurnMoves; // max number of moves that can be created and processed in the same turn

	// Debugging - output from last pathfind operation:

	PathfindTileGrid* m_DebugGrid;
	PathfindTileJPSGrid* m_DebugGridJPS;
	u32 m_DebugSteps;
	double m_DebugTime;
	PathGoal m_DebugGoal;
	WaypointPath* m_DebugPath;
	PathfinderOverlay* m_DebugOverlay;
	pass_class_t m_DebugPassClass;

	std::vector<SOverlayLine> m_DebugOverlayShortPathLines;

	static std::string GetSchema()
	{
		return "<a:component type='system'/><empty/>";
	}

	virtual void Init(const CParamNode& paramNode);

	virtual void Deinit();

	virtual void Serialize(ISerializer& serialize);

	virtual void Deserialize(const CParamNode& paramNode, IDeserializer& deserialize);

	virtual void HandleMessage(const CMessage& msg, bool global);

	virtual pass_class_t GetPassabilityClass(const std::string& name);

	virtual std::map<std::string, pass_class_t> GetPassabilityClasses();

	const PathfinderPassability* GetPassabilityFromMask(pass_class_t passClass);

	virtual const Grid<u16>& GetPassabilityGrid();

	virtual void ComputePath(entity_pos_t x0, entity_pos_t z0, const PathGoal& goal, pass_class_t passClass, WaypointPath& ret);

	virtual void ComputePathJPS(entity_pos_t x0, entity_pos_t z0, const PathGoal& goal, pass_class_t passClass, WaypointPath& ret);

	/**
	 * Same kind of interface as ICmpPathfinder::ComputePath, but works when the
	 * unit is starting on an impassable navcell. Returns a path heading directly
	 * to the nearest passable navcell, then the goal.
	 */
	void ComputePathOffImpassable(entity_pos_t x0, entity_pos_t z0, const PathGoal& origGoal, pass_class_t passClass, WaypointPath& path);

	/**
	 * Given a path with an arbitrary collection of waypoints, updates the
	 * waypoints to be nicer. (In particular, the current implementation
	 * ensures a consistent maximum distance between adjacent waypoints.
	 * Might be nice to improve it to smooth the paths, etc.)
	 */
	void NormalizePathWaypoints(WaypointPath& path);

	/**
	 * Given a path with an arbitrary collection of waypoints, updates the
	 * waypoints to be nicer. Calls "Testline" between waypoints
	 * so that bended paths can become straight if there's nothing in between
	 * (this happens because A* is 8-direction, and the map isn't actually a grid).
	 */
	void ImprovePathWaypoints(WaypointPath& path, pass_class_t passClass);

	virtual u32 ComputePathAsync(entity_pos_t x0, entity_pos_t z0, const PathGoal& goal, pass_class_t passClass, entity_id_t notify);

	virtual void ComputeShortPath(const IObstructionTestFilter& filter, entity_pos_t x0, entity_pos_t z0, entity_pos_t r, entity_pos_t range, const PathGoal& goal, pass_class_t passClass, WaypointPath& ret);

	virtual u32 ComputeShortPathAsync(entity_pos_t x0, entity_pos_t z0, entity_pos_t r, entity_pos_t range, const PathGoal& goal, pass_class_t passClass, bool avoidMovingUnits, entity_id_t controller, entity_id_t notify);

	virtual void SetDebugPath(entity_pos_t x0, entity_pos_t z0, const PathGoal& goal, pass_class_t passClass);

	virtual void ResetDebugPath();

	virtual void SetDebugOverlay(bool enabled);

	virtual void SetHierDebugOverlay(bool enabled)
	{
		m_PathfinderHier->SetDebugOverlay(enabled, &GetSimContext());
	}

	virtual void GetDebugData(u32& steps, double& time, Grid<u8>& grid);

	virtual void GetDebugDataJPS(u32& steps, double& time, Grid<u8>& grid);

	virtual CFixedVector2D GetNearestPointOnGoal(CFixedVector2D pos, const PathGoal& goal);

	virtual bool CheckMovement(const IObstructionTestFilter& filter, entity_pos_t x0, entity_pos_t z0, entity_pos_t x1, entity_pos_t z1, entity_pos_t r, pass_class_t passClass);

	virtual ICmpObstruction::EFoundationCheck CheckUnitPlacement(const IObstructionTestFilter& filter, entity_pos_t x, entity_pos_t z, entity_pos_t r, pass_class_t passClass, bool onlyCenterPoint);

	virtual ICmpObstruction::EFoundationCheck CheckBuildingPlacement(const IObstructionTestFilter& filter, entity_pos_t x, entity_pos_t z, entity_pos_t a, entity_pos_t w, entity_pos_t h, entity_id_t id, pass_class_t passClass);

	virtual ICmpObstruction::EFoundationCheck CheckBuildingPlacement(const IObstructionTestFilter& filter, entity_pos_t x, entity_pos_t z, entity_pos_t a, entity_pos_t w, entity_pos_t h, entity_id_t id, pass_class_t passClass, bool onlyCenterPoint);

	virtual void FinishAsyncRequests();

	void ProcessLongRequests(const std::vector<AsyncLongPathRequest>& longRequests);
	
	void ProcessShortRequests(const std::vector<AsyncShortPathRequest>& shortRequests);

	virtual void ProcessSameTurnMoves();

	/**
	 * Regenerates the grid based on the current obstruction list, if necessary
	 */
	virtual void UpdateGrid();

	Grid<u16> ComputeShoreGrid();

	void ComputeTerrainPassabilityGrid(const Grid<u16>& shoreGrid);

	void RenderSubmit(SceneCollector& collector);
};

#endif // INCLUDED_CCMPPATHFINDER_COMMON
