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
#include "simulation2/components/ICmpObstructionManager.h"
#include "simulation2/helpers/Geometry.h"
#include "simulation2/helpers/Grid.h"

class PathfinderOverlay;
class SceneCollector;
struct PathfindTile;
struct PathfindTileJPS;
class JumpPointCache;
class CCmpPathfinder_Hier;

#ifdef NDEBUG
#define PATHFIND_DEBUG 0
#else
#define PATHFIND_DEBUG 1
#endif

#define PATHFIND_USE_JPS 1

/*
 * For efficient pathfinding we want to try hard to minimise the per-tile search cost,
 * so we precompute the tile passability flags and movement costs for the various different
 * types of unit.
 * We also want to minimise memory usage (there can easily be 100K tiles so we don't want
 * to store many bytes for each).
 *
 * To handle passability efficiently, we have a small number of passability classes
 * (e.g. "infantry", "ship"). Each unit belongs to a single passability class, and
 * uses that for all its pathfinding.
 * Passability is determined by water depth, terrain slope, forestness, buildingness.
 * We need at least one bit per class per tile to represent passability.
 *
 * We use a separate bit to indicate building obstructions (instead of folding it into
 * the class passabilities) so that it can be ignored when doing the accurate short paths.
 * We use another bit to indicate tiles near obstructions that block construction,
 * for the AI to plan safe building spots.
 */
class PathfinderPassability
{
public:
	PathfinderPassability(ICmpPathfinder::pass_class_t mask, const CParamNode& node) :
		m_Mask(mask)
	{
		if (node.GetChild("MinWaterDepth").IsOk())
			m_MinDepth = node.GetChild("MinWaterDepth").ToFixed();
		else
			m_MinDepth = std::numeric_limits<fixed>::min();

		if (node.GetChild("MaxWaterDepth").IsOk())
			m_MaxDepth = node.GetChild("MaxWaterDepth").ToFixed();
		else
			m_MaxDepth = std::numeric_limits<fixed>::max();

		if (node.GetChild("MaxTerrainSlope").IsOk())
			m_MaxSlope = node.GetChild("MaxTerrainSlope").ToFixed();
		else
			m_MaxSlope = std::numeric_limits<fixed>::max();

		if (node.GetChild("MinShoreDistance").IsOk())
			m_MinShore = node.GetChild("MinShoreDistance").ToFixed();
		else
			m_MinShore = std::numeric_limits<fixed>::min();

		if (node.GetChild("MaxShoreDistance").IsOk())
			m_MaxShore = node.GetChild("MaxShoreDistance").ToFixed();
		else
			m_MaxShore = std::numeric_limits<fixed>::max();

		if (node.GetChild("Clearance").IsOk())
		{
			m_HasClearance = true;
			m_Clearance = node.GetChild("Clearance").ToFixed();

			if (!(m_Clearance % ICmpObstructionManager::NAVCELL_SIZE).IsZero())
			{
				// If clearance isn't an integer number of navcells then we'll
				// probably get weird behaviour when expanding the navcell grid
				// by clearance, vs expanding static obstructions by clearance
				LOGWARNING("Pathfinder passability class has clearance %f, should be multiple of %f",
					m_Clearance.ToFloat(), ICmpObstructionManager::NAVCELL_SIZE.ToFloat());
			}
		}
		else
		{
			m_HasClearance = false;
			m_Clearance = fixed::Zero();
		}
	}

	bool IsPassable(fixed waterdepth, fixed steepness, fixed shoredist)
	{
		return ((m_MinDepth <= waterdepth && waterdepth <= m_MaxDepth) && (steepness < m_MaxSlope) && (m_MinShore <= shoredist && shoredist <= m_MaxShore));
	}

	ICmpPathfinder::pass_class_t m_Mask;

	bool m_HasClearance; // whether static obstructions are impassable
	fixed m_Clearance; // min distance from static obstructions

private:
	fixed m_MinDepth;
	fixed m_MaxDepth;
	fixed m_MaxSlope;
	fixed m_MinShore;
	fixed m_MaxShore;
};

typedef u16 NavcellData; // 1 bit per passability class (up to PASS_CLASS_BITS)
static const int PASS_CLASS_BITS = 16;
#define IS_PASSABLE(item, classmask) (((item) & (classmask)) == 0)
#define PASS_CLASS_MASK_FROM_INDEX(id) ((pass_class_t)(1u << id))

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
	ICmpPathfinder::pass_class_t passClass;
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
	ICmpPathfinder::pass_class_t passClass;
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
	// (This is hidden behind proxy methods to keep the code
	// slightly better encapsulated.)
	CCmpPathfinder_Hier* m_PathfinderHier;
	void PathfinderHierInit();
	void PathfinderHierDeinit();
	void PathfinderHierReload();
	void PathfinderHierRenderSubmit(SceneCollector& collector);
	bool PathfinderHierMakeGoalReachable(u16 i0, u16 j0, PathGoal& goal, pass_class_t passClass);
	void PathfinderHierFindNearestPassableNavcell(u16& i, u16& j, pass_class_t passClass);

	void PathfinderJPSMakeDirty();

	// For responsiveness we will process some moves in the same turn they were generated in
	
	u16 m_MaxSameTurnMoves; // max number of moves that can be created and processed in the same turn

	// Debugging - output from last pathfind operation:

	PathfindTileGrid* m_DebugGrid;
	PathfindTileJPSGrid* m_DebugGridJPS;
	u32 m_DebugSteps;
	double m_DebugTime;
	PathGoal m_DebugGoal;
	Path* m_DebugPath;
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

	virtual void ComputePath(entity_pos_t x0, entity_pos_t z0, const PathGoal& goal, pass_class_t passClass, Path& ret);

	virtual void ComputePathJPS(entity_pos_t x0, entity_pos_t z0, const PathGoal& goal, pass_class_t passClass, Path& ret);

	/**
	 * Same kind of interface as ICmpPathfinder::ComputePath, but works when the
	 * unit is starting on an impassable navcell. Returns a path heading directly
	 * to the nearest passable navcell, then the goal.
	 */
	void ComputePathOffImpassable(entity_pos_t x0, entity_pos_t z0, const PathGoal& origGoal, pass_class_t passClass, Path& path);

	/**
	 * Given a path with an arbitrary collection of waypoints, updates the
	 * waypoints to be nicer. (In particular, the current implementation
	 * ensures a consistent maximum distance between adjacent waypoints.
	 * Might be nice to improve it to smooth the paths, etc.)
	 */
	void NormalizePathWaypoints(Path& path);

	/**
	 * Given a path with an arbitrary collection of waypoints, updates the
	 * waypoints to be nicer. Calls "Testline" between waypoints
	 * so that bended paths can become straight if there's nothing in between
	 * (this happens because A* is 8-direction, and the map isn't actually a grid).
	 */
	void ImprovePathWaypoints(Path& path, pass_class_t passClass);

	virtual u32 ComputePathAsync(entity_pos_t x0, entity_pos_t z0, const PathGoal& goal, pass_class_t passClass, entity_id_t notify);

	virtual void ComputeShortPath(const IObstructionTestFilter& filter, entity_pos_t x0, entity_pos_t z0, entity_pos_t r, entity_pos_t range, const PathGoal& goal, pass_class_t passClass, Path& ret);

	virtual u32 ComputeShortPathAsync(entity_pos_t x0, entity_pos_t z0, entity_pos_t r, entity_pos_t range, const PathGoal& goal, pass_class_t passClass, bool avoidMovingUnits, entity_id_t controller, entity_id_t notify);

	virtual void SetDebugPath(entity_pos_t x0, entity_pos_t z0, const PathGoal& goal, pass_class_t passClass);

	virtual void ResetDebugPath();

	virtual void SetDebugOverlay(bool enabled);

	virtual void SetHierDebugOverlay(bool enabled);

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
	 * Compute the navcell indexes on the grid nearest to a given point
	 */
	void NearestNavcell(entity_pos_t x, entity_pos_t z, u16& i, u16& j)
	{
		i = (u16)clamp((x / ICmpObstructionManager::NAVCELL_SIZE).ToInt_RoundToNegInfinity(), 0, m_MapSize*ICmpObstructionManager::NAVCELLS_PER_TILE - 1);
		j = (u16)clamp((z / ICmpObstructionManager::NAVCELL_SIZE).ToInt_RoundToNegInfinity(), 0, m_MapSize*ICmpObstructionManager::NAVCELLS_PER_TILE - 1);
	}

	/**
	 * Returns the position of the center of the given tile
	 */
	static void TileCenter(u16 i, u16 j, entity_pos_t& x, entity_pos_t& z)
	{
		cassert(TERRAIN_TILE_SIZE % 2 == 0);
		x = entity_pos_t::FromInt(i*(int)TERRAIN_TILE_SIZE + (int)TERRAIN_TILE_SIZE/2);
		z = entity_pos_t::FromInt(j*(int)TERRAIN_TILE_SIZE + (int)TERRAIN_TILE_SIZE/2);
	}

	static void NavcellCenter(u16 i, u16 j, entity_pos_t& x, entity_pos_t& z)
	{
		x = entity_pos_t::FromInt(i*2 + 1).Multiply(ICmpObstructionManager::NAVCELL_SIZE / 2);
		z = entity_pos_t::FromInt(j*2 + 1).Multiply(ICmpObstructionManager::NAVCELL_SIZE / 2);
	}

	/**
	 * Regenerates the grid based on the current obstruction list, if necessary
	 */
	virtual void UpdateGrid();

	Grid<u16> ComputeShoreGrid();

	void ComputeTerrainPassabilityGrid(const Grid<u16>& shoreGrid);

	void RenderSubmit(SceneCollector& collector);
};

#endif // INCLUDED_CCMPPATHFINDER_COMMON
