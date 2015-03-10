/* Copyright (C) 2012 Wildfire Games.
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

/**
 * @file
 * Tile-based algorithm for CCmpPathfinder.
 * This is a fairly naive algorithm and could probably be improved substantially
 * (hopefully without needing to change the interface much).
 */

#include "precompiled.h"

#include "CCmpPathfinder_Common.h"

#include "ps/Profile.h"
#include "renderer/TerrainOverlay.h"
#include "simulation2/helpers/PriorityQueue.h"

#define PATHFIND_STATS 1

typedef PriorityQueueHeap<TileID, PathCost> PriorityQueue;

/**
 * Tile data for A* computation.
 * (We store an array of one of these per terrain navcell, so it ought to be optimized for size)
 */
struct PathfindTile
{
public:
	enum {
		STATUS_UNEXPLORED = 0,
		STATUS_OPEN = 1,
		STATUS_CLOSED = 2
	};

	bool IsUnexplored() { return status == STATUS_UNEXPLORED; }
	bool IsOpen() { return status == STATUS_OPEN; }
	bool IsClosed() { return status == STATUS_CLOSED; }
	void SetStatusOpen() { status = STATUS_OPEN; }
	void SetStatusClosed() { status = STATUS_CLOSED; }

	// Get pi,pj coords of predecessor to this tile on best path, given i,j coords of this tile
	u16 GetPredI(u16 i) { return (u16)(i + dpi); }
	u16 GetPredJ(u16 j) { return (u16)(j + dpj); }
	// Set the pi,pj coords of predecessor, given i,j coords of this tile
	void SetPred(u16 pi, u16 pj, u16 i, u16 j)
	{
#if PATHFIND_DEBUG
		// predecessor must be adjacent
		ENSURE(pi-i == -1 || pi-i == 0 || pi-i == 1);
		ENSURE(pj-j == -1 || pj-j == 0 || pj-j == 1);
#endif
		dpi = (i8)((int)pi - (int)i);
		dpj = (i8)((int)pj - (int)j);
	}

	PathCost GetCost() const { return g; }
	void SetCost(PathCost cost) { g = cost; }

private:
	i8 dpi, dpj; // values are in {-1, 0, 1}, pointing to an adjacent tile
	u8 status; // this only needs 2 bits
	PathCost g; // cost to reach this tile

public:
#if PATHFIND_DEBUG
	u32 GetStep() { return step; }
	void SetStep(u32 s) { step = s; }
private:
	u32 step; // step at which this tile was last processed (for debug rendering)
#else
	u32 GetStep() { return 0; }
	void SetStep(u32) { }
#endif

};

/**
 * Terrain overlay for pathfinder debugging.
 * Renders a representation of the most recent pathfinding operation.
 */
class PathfinderOverlay : public TerrainTextureOverlay
{
public:
	CCmpPathfinder& m_Pathfinder;

	PathfinderOverlay(CCmpPathfinder& pathfinder) :
		TerrainTextureOverlay(Pathfinding::NAVCELLS_PER_TILE), m_Pathfinder(pathfinder)
	{
	}

	virtual void BuildTextureRGBA(u8* data, size_t w, size_t h)
	{
		// Ensure m_Pathfinder.m_Grid is up-to-date
		m_Pathfinder.UpdateGrid();

		// Grab the debug data for the most recently generated path
		u32 steps;
		double time;
		Grid<u8> debugGrid;
		if (m_Pathfinder.m_DebugGridJPS)
			m_Pathfinder.GetDebugDataJPS(steps, time, debugGrid);
		else if (m_Pathfinder.m_DebugGrid)
			m_Pathfinder.GetDebugData(steps, time, debugGrid);

		// Render navcell passability
		u8* p = data;
		for (size_t j = 0; j < h; ++j)
		{
			for (size_t i = 0; i < w; ++i)
			{
				SColor4ub color(0, 0, 0, 0);
				if (!IS_PASSABLE(m_Pathfinder.m_Grid->get((int)i, (int)j), m_Pathfinder.m_DebugPassClass))
					color = SColor4ub(255, 0, 0, 127);

				if (debugGrid.m_W && debugGrid.m_H)
				{
					u8 n = debugGrid.get((int)i, (int)j);

					if (n == 1)
						color = SColor4ub(255, 255, 0, 127);
					else if (n == 2)
						color = SColor4ub(0, 255, 0, 127);

					if (m_Pathfinder.m_DebugGoal.NavcellContainsGoal(i, j))
						color = SColor4ub(0, 0, 255, 127);
				}

				*p++ = color.R;
				*p++ = color.G;
				*p++ = color.B;
				*p++ = color.A;
			}
		}

		// Render the most recently generated path
		if (m_Pathfinder.m_DebugPath && !m_Pathfinder.m_DebugPath->m_Waypoints.empty())
		{
			std::vector<Waypoint>& waypoints = m_Pathfinder.m_DebugPath->m_Waypoints;
			u16 ip = 0, jp = 0;
			for (size_t k = 0; k < waypoints.size(); ++k)
			{
				u16 i, j;
				Pathfinding::NearestNavcell(waypoints[k].x, waypoints[k].z, i, j, m_Pathfinder.m_MapSize, m_Pathfinder.m_MapSize);
				if (k == 0)
				{
					ip = i;
					jp = j;
				}
				else
				{
					bool firstCell = true;
					do
					{
						if (data[(jp*w + ip)*4+3] == 0)
						{
							data[(jp*w + ip)*4+0] = 0xFF;
							data[(jp*w + ip)*4+1] = 0xFF;
							data[(jp*w + ip)*4+2] = 0xFF;
							data[(jp*w + ip)*4+3] = firstCell ? 0xA0 : 0x60;
						}
						ip = ip < i ? ip+1 : ip > i ? ip-1 : ip;
						jp = jp < j ? jp+1 : jp > j ? jp-1 : jp;
						firstCell = false;
					}
					while (ip != i || jp != j);
				}
			}
		}
	}
};

void CCmpPathfinder::SetDebugOverlay(bool enabled)
{
	if (enabled && !m_DebugOverlay)
	{
		m_DebugOverlay = new PathfinderOverlay(*this);
	}
	else if (!enabled && m_DebugOverlay)
	{
		delete m_DebugOverlay;
		m_DebugOverlay = NULL;
	}
}

void CCmpPathfinder::SetDebugPath(entity_pos_t x0, entity_pos_t z0, const PathGoal& goal, pass_class_t passClass)
{
	if (!m_DebugOverlay)
		return;

	SAFE_DELETE(m_DebugGrid);
	delete m_DebugPath;
	m_DebugPath = new WaypointPath();
#if PATHFIND_USE_JPS
	ComputePathJPS(x0, z0, goal, passClass, *m_DebugPath);
#else
	ComputePath(x0, z0, goal, passClass, *m_DebugPath);
#endif
	m_DebugPassClass = passClass;
}

void CCmpPathfinder::ResetDebugPath()
{
	SAFE_DELETE(m_DebugGrid);
	SAFE_DELETE(m_DebugPath);
}


//////////////////////////////////////////////////////////

struct PathfinderState
{
	u32 steps; // number of algorithm iterations

	u16 iGoal, jGoal; // goal tile

	pass_class_t passClass;

	PriorityQueue open;
	// (there's no explicit closed list; it's encoded in PathfindTile)

	PathfindTileGrid* tiles;
	Grid<NavcellData>* terrain;

	PathCost hBest; // heuristic of closest discovered tile to goal
	u16 iBest, jBest; // closest tile

#if PATHFIND_STATS
	// Performance debug counters
	size_t numProcessed;
	size_t numImproveOpen;
	size_t numImproveClosed;
	size_t numAddToOpen;
	size_t sumOpenSize;
#endif
};

// Calculate heuristic cost from tile i,j to goal
// (This ought to be an underestimate for correctness)
static PathCost CalculateHeuristic(int i, int j, int iGoal, int jGoal)
{
	int di = abs(i - iGoal);
	int dj = abs(j - jGoal);
	int diag = std::min(di, dj);
	return PathCost(di-diag + dj-diag, diag);
}

// Do the A* processing for a neighbour tile i,j.
static void ProcessNeighbour(int pi, int pj, int i, int j, PathCost pg, PathfinderState& state)
{
#if PATHFIND_STATS
	state.numProcessed++;
#endif

	PathfindTile& n = state.tiles->get(i, j);

	if (n.IsClosed())
		return;

	// Reject impassable tiles
	if (!IS_PASSABLE(state.terrain->get(i, j), state.passClass))
		return;
	// Also, diagonal moves are only allowed if the adjacent tiles
	// are also unobstructed
	if (pi != i && pj != j)
	{
		if (!IS_PASSABLE(state.terrain->get(pi, j), state.passClass))
			return;
		if (!IS_PASSABLE(state.terrain->get(i, pj), state.passClass))
			return;
	}

	PathCost dg;
	if (pi == i || pj == j)
		dg = PathCost::horizvert(1);
	else
		dg = PathCost::diag(1);

	PathCost g = pg + dg; // cost to this tile = cost to predecessor + delta from predecessor

	PathCost h = CalculateHeuristic(i, j, state.iGoal, state.jGoal);

	// If this is a new tile, compute the heuristic distance
	if (n.IsUnexplored())
	{
		// Remember the best tile we've seen so far, in case we never actually reach the target
		if (h < state.hBest)
		{
			state.hBest = h;
			state.iBest = i;
			state.jBest = j;
		}
	}
	else
	{
		// If we've already seen this tile, and the new path to this tile does not have a
		// better cost, then stop now
		if (g >= n.GetCost())
			return;

		// Otherwise, we have a better path.

		// If we've already added this tile to the open list:
		if (n.IsOpen())
		{
			// This is a better path, so replace the old one with the new cost/parent
			PathCost gprev = n.GetCost();
			n.SetCost(g);
			n.SetPred(pi, pj, i, j);
			n.SetStep(state.steps);
			state.open.promote(TileID(i, j), gprev + h, g + h);
#if PATHFIND_STATS
			state.numImproveOpen++;
#endif
			return;
		}

#if PATHFIND_STATS
		// If we've already found the 'best' path to this tile:
		if (n.IsClosed())
		{
			// This is a better path (possible when we use inadmissible heuristics), so reopen it
			// by falling through
			state.numImproveClosed++;
		}
#endif
	}

	// Add it to the open list:
	n.SetStatusOpen();
	n.SetCost(g);
	n.SetPred(pi, pj, i, j);
	n.SetStep(state.steps);
	PriorityQueue::Item t = { TileID(i, j), g + h };
	state.open.push(t);
#if PATHFIND_STATS
	state.numAddToOpen++;
#endif
}

void CCmpPathfinder::ComputePath(entity_pos_t x0, entity_pos_t z0, const PathGoal& origGoal, pass_class_t passClass, WaypointPath& path)
{
	UpdateGrid();

	PROFILE3("ComputePath");
	TIMER(L"ComputePath");
	double time = timer_Time();

	PathfinderState state = { 0 };

	// Convert the start/end coordinates to tile indexes
	u16 i0, j0;
	Pathfinding::NearestNavcell(x0, z0, i0, j0, m_MapSize, m_MapSize);

	// To be consistent with the JPS pathfinder (which requires passable source navcell),
	// and to let us guarantee the goal is reachable from the source, we switch to
	// the escape-from-impassability mode if currently on an impassable navcell
	if (!IS_PASSABLE(m_Grid->get(i0, j0), passClass))
	{
		ComputePathOffImpassable(x0, z0, origGoal,passClass, path);
		return;
	}

	// Adjust the goal so that it's reachable from the source navcell
	PathGoal goal = origGoal;
	m_PathfinderHier->MakeGoalReachable(i0, j0, goal, passClass);

	// If we're already at the goal tile, then move directly to the exact goal coordinates
	// XXX: this seems bogus for non-point goals, it should be the point on the current cell nearest the goal
	if (goal.NavcellContainsGoal(i0, j0))
	{
		Waypoint w = { goal.x, goal.z };
		path.m_Waypoints.push_back(w);
		return;
	}

	// Store the navcell at the goal center, for A* heuristics
	Pathfinding::NearestNavcell(goal.x, goal.z, state.iGoal, state.jGoal, m_MapSize, m_MapSize);

	state.passClass = passClass;

	state.steps = 0;

	state.tiles = new PathfindTileGrid(m_Grid->m_W, m_Grid->m_H);
	state.terrain = m_Grid;

	state.iBest = i0;
	state.jBest = j0;
	state.hBest = CalculateHeuristic(i0, j0, state.iGoal, state.jGoal);

	PriorityQueue::Item start = { TileID(i0, j0), PathCost() };
	state.open.push(start);
	state.tiles->get(i0, j0).SetStatusOpen();
	state.tiles->get(i0, j0).SetPred(i0, j0, i0, j0);
	state.tiles->get(i0, j0).SetCost(PathCost());

	while (1)
	{
		++state.steps;

		// If we ran out of tiles to examine, give up
		if (state.open.empty())
			break;

#if PATHFIND_STATS
		state.sumOpenSize += state.open.size();
#endif

		// Move best tile from open to closed
		PriorityQueue::Item curr = state.open.pop();
		u16 i = curr.id.i();
		u16 j = curr.id.j();
		state.tiles->get(i, j).SetStatusClosed();

		// If we've reached the destination, stop
		if (goal.NavcellContainsGoal(i, j))
		{
			state.iBest = i;
			state.jBest = j;
			state.hBest = PathCost();
			break;
		}

		PathCost g = state.tiles->get(i, j).GetCost();

		// Try all 8 neighbors
		ProcessNeighbour(i, j, i-1, j-1, g, state);
		ProcessNeighbour(i, j, i+1, j-1, g, state);
		ProcessNeighbour(i, j, i-1, j+1, g, state);
		ProcessNeighbour(i, j, i+1, j+1, g, state);
		ProcessNeighbour(i, j, i-1, j, g, state);
		ProcessNeighbour(i, j, i+1, j, g, state);
		ProcessNeighbour(i, j, i, j-1, g, state);
		ProcessNeighbour(i, j, i, j+1, g, state);
	}

	// Reconstruct the path (in reverse)
	u16 ip = state.iBest, jp = state.jBest;
	while (ip != i0 || jp != j0)
	{
		PathfindTile& n = state.tiles->get(ip, jp);
		entity_pos_t x, z;
		Pathfinding::NavcellCenter(ip, jp, x, z);
		Waypoint w = { x, z };
		path.m_Waypoints.push_back(w);

		// Follow the predecessor link
		ip = n.GetPredI(ip);
		jp = n.GetPredJ(jp);
	}

	NormalizePathWaypoints(path);

	// Save this grid for debug display
	m_DebugTime = timer_Time() - time;
	delete m_DebugGrid;
	m_DebugGrid = state.tiles;
	m_DebugSteps = state.steps;
	m_DebugGoal = goal;

	PROFILE2_ATTR("from: (%d, %d)", i0, j0);
	PROFILE2_ATTR("to: (%d, %d)", state.iGoal, state.jGoal);
	PROFILE2_ATTR("reached: (%d, %d)", state.iBest, state.jBest);
	PROFILE2_ATTR("steps: %u", state.steps);

#if PATHFIND_STATS
	debug_printf("PATHFINDER: steps=%d avgo=%d proc=%d impc=%d impo=%d addo=%d\n", state.steps, state.sumOpenSize/state.steps, state.numProcessed, state.numImproveClosed, state.numImproveOpen, state.numAddToOpen);
#endif
}

void CCmpPathfinder::GetDebugData(u32& steps, double& time, Grid<u8>& grid)
{
	steps = m_DebugSteps;
	time = m_DebugTime;

	if (!m_DebugGrid)
		return;

	grid = Grid<u8>(m_DebugGrid->m_W, m_DebugGrid->m_H);
	for (u16 j = 0; j < grid.m_H; ++j)
	{
		for (u16 i = 0; i < grid.m_W; ++i)
		{
			PathfindTile t = m_DebugGrid->get(i, j);
			grid.set(i, j, (t.IsOpen() ? 1 : 0) | (t.IsClosed() ? 2 : 0));
		}
	}
}
