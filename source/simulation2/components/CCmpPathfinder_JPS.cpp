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

#include "precompiled.h"

#include "CCmpPathfinder_Common.h"

#include "lib/bits.h"
#include "lib/sysdep/rtl.h"
#include "ps/Profile.h"
#include "renderer/TerrainOverlay.h"
#include "simulation2/helpers/PriorityQueue.h"

#define PATHFIND_STATS 0

#define USE_JUMPPOINT_CACHE 1

#define ACCEPT_DIAGONAL_GAPS 0

typedef PriorityQueueHeap<TileID, PathCost> PriorityQueue;

/**
 * Jump point cache.
 * 
 * The JPS algorithm wants to efficiently either find the first jump point
 * in some direction from some cell (not counting the cell itself),
 * if it is reachable without crossing any impassable cells;
 * or know that there is no such reachable jump point.
 * The jump point is always on a passable cell.
 * We cache that data to allow fast lookups, which helps performance
 * significantly (especially on sparse maps).
 * Recalculation might be expensive but the underlying passability data
 * changes relatively rarely.
 * 
 * To allow the algorithm to detect goal cells, we want to treat them as
 * jump points too. (That means the algorithm will push those cells onto
 * its open queue, and will eventually pop a goal cell and realise it's done.)
 * (Goals might be circles/squares/etc, not just a single cell.)
 * But the goal generally changes for every path request, so we can't cache
 * it like the normal jump points.
 * Instead, if there's no jump point from some cell then we'll cache the
 * first impassable cell as an 'obstruction jump point'
 * (with a flag to distinguish from a real jump point), and then the caller
 * can test whether the goal includes a cell that's closer than the first
 * (obstruction or real) jump point,
 * and treat the goal cell as a jump point in that case.
 * 
 * We only ever need to find the jump point relative to a passable cell;
 * the cache is allowed to return bogus values for impassable cells.
 */
class JumpPointCache
{
	/**
	 * Simple space-inefficient row storage.
	 */
	struct RowRaw
	{
		std::vector<u16> data;

		size_t GetMemoryUsage() const
		{
			return data.capacity() * sizeof(u16);
		}

		RowRaw(int length)
		{
			data.resize(length);
		}

		/**
		 * Set cells x0 <= x < x1 to have jump point x1.
		 */
		void SetRange(int x0, int x1, bool obstruction)
		{
			ENSURE(0 <= x0 && x0 <= x1 && x1 < (int)data.size());
			for (int x = x0; x < x1; ++x)
				data[x] = (x1 << 1) | (obstruction ? 1 : 0);
		}

		/**
		 * Returns the coordinate of the next jump point xp (where x < xp),
		 * and whether it's an obstruction point or jump point.
		 */
		void Get(int x, int& xp, bool& obstruction)
		{
			ENSURE(0 <= x && x < (int)data.size());
			xp = data[x] >> 1;
			obstruction = data[x] & 1;
		}

		void Finish() { }
	};

	struct RowTree
	{
		/**
		 * Represents an interval [u15 x0, u16 x1)
		 * with a boolean obstruction flag,
		 * packed into a single u32.
		 */
		struct Interval
		{
			Interval() : data(0) { }

			Interval(int x0, int x1, bool obstruction)
			{
				ENSURE(0 <= x0 && x0 < 0x8000);
				ENSURE(0 <= x1 && x1 < 0x10000);
				data = ((u32)x0 << 17) | (u32)(obstruction ? 0x10000 : 0) | (u32)x1;
			}

			int x0() { return data >> 17; }
			int x1() { return data & 0xFFFF; }
			bool obstruction() { return (data & 0x10000) != 0; }

			u32 data;
		};

		std::vector<Interval> data;

		size_t GetMemoryUsage() const
		{
			return data.capacity() * sizeof(Interval);
		}

		RowTree(int UNUSED(length))
		{
		}

		void SetRange(int x0, int x1, bool obstruction)
		{
			ENSURE(0 <= x0 && x0 <= x1);
			Interval iv(x0, x1, obstruction);
			data.push_back(iv);
		}

		/**
		 * Recursive helper function for Finish().
		 * Given two ranges [x0, pivot) and [pivot, x1) in the sorted array 'data',
		 * the pivot element is added onto the binary tree (stored flattened in an
		 * array), and then each range is split into two sub-ranges with a pivot in
		 * the middle (to ensure the tree remains balanced) and ConstructTree recurses.
		 */
		void ConstructTree(std::vector<Interval>& tree, size_t x0, size_t pivot, size_t x1, size_t idx_tree)
		{
			ENSURE(x0 < data.size());
			ENSURE(x1 <= data.size());
			ENSURE(x0 <= pivot);
			ENSURE(pivot < x1);
			ENSURE(idx_tree < tree.size());

			tree[idx_tree] = data[pivot];

			if (x0 < pivot)
				ConstructTree(tree, x0, (x0 + pivot) / 2, pivot, (idx_tree << 1) + 1);
			if (pivot+1 < x1)
				ConstructTree(tree, pivot+1, (pivot + x1) / 2, x1, (idx_tree << 1) + 2);
		}

		void Finish()
		{
			// Convert the sorted interval list into a balanced binary tree

			std::vector<Interval> tree;

			if (!data.empty())
			{
				size_t depth = ceil_log2(data.size() + 1);
				tree.resize((1 << depth) - 1);
				ConstructTree(tree, 0, data.size() / 2, data.size(), 0);
			}

			data.swap(tree);
		}

		void Get(int x, int& xp, bool& obstruction)
		{
			// Search the binary tree for an interval which contains x
			int i = 0;
			while (true)
			{
				ENSURE(i < (int)data.size());
				Interval interval = data[i];
				if (x < interval.x0())
					i = (i << 1) + 1;
				else if (x >= interval.x1())
					i = (i << 1) + 2;
				else
				{
					ENSURE(interval.x0() <= x && x < interval.x1());
					xp = interval.x1();
					obstruction = interval.obstruction();
					return;
				}
			}
		}
	};

	// Pick one of the row implementations
	typedef RowRaw Row;

public:
	int m_Width;
	int m_Height;
	std::vector<Row> m_JumpPointsRight;
	std::vector<Row> m_JumpPointsLeft;
	std::vector<Row> m_JumpPointsUp;
	std::vector<Row> m_JumpPointsDown;

	/**
	 * Compute the cached obstruction/jump points for each cell,
	 * in a single direction. By default the code assumes the rightwards
	 * (+i) direction; set 'transpose' to switch to upwards (+j),
	 * and/or set 'mirror' to reverse the direction.
	 */
	void ComputeRows(std::vector<Row>& rows,
		const Grid<NavcellData>& terrain, ICmpPathfinder::pass_class_t passClass,
		bool transpose, bool mirror)
	{
		int w = terrain.m_W;
		int h = terrain.m_H;

		if (transpose)
			std::swap(w, h);

		// Check the terrain passability, adjusted for transpose/mirror
#define TERRAIN_IS_PASSABLE(i, j) \
	IS_PASSABLE( \
		mirror \
		? (transpose ? terrain.get((j), w-1-(i)) : terrain.get(w-1-(i), (j))) \
		: (transpose ? terrain.get((j), (i)) : terrain.get((i), (j))) \
	, passClass)

		rows.reserve(h);
		for (int j = 0; j < h; ++j)
			rows.push_back(Row(w));

		for (int j = 1; j < h - 1; ++j)
		{
			// Find the first passable cell.
			// Then, find the next jump/obstruction point after that cell,
			// and store that point for the passable range up to that cell,
			// then repeat.

			int i = 0;
			while (i < w)
			{
				// Restart the 'while' loop until we reach a passable cell
				if (!TERRAIN_IS_PASSABLE(i, j))
				{
					++i;
					continue;
				}

				// i is now a passable cell; find the next jump/obstruction point.
				// (We assume the map is surrounded by impassable cells, so we don't
				// need to explicitly check for world bounds here.)

				int i0 = i;
				while (true)
				{
					++i;

					// Check if we hit an obstructed tile
					if (!TERRAIN_IS_PASSABLE(i, j))
					{
						rows[j].SetRange(i0, i, true);
						break;
					}

					// Check if we reached a jump point
#if ACCEPT_DIAGONAL_GAPS
					if ((!TERRAIN_IS_PASSABLE(i, j-1) && TERRAIN_IS_PASSABLE(i+1, j-1)) ||
						(!TERRAIN_IS_PASSABLE(i, j+1) && TERRAIN_IS_PASSABLE(i+1, j+1)))
#else
					if ((!TERRAIN_IS_PASSABLE(i-1, j-1) && TERRAIN_IS_PASSABLE(i, j-1)) ||
						(!TERRAIN_IS_PASSABLE(i-1, j+1) && TERRAIN_IS_PASSABLE(i, j+1)))
#endif
					{
						rows[j].SetRange(i0, i, false);
						break;
					}
				}
			}

			rows[j].Finish();
		}
	}

	void reset(const Grid<NavcellData>* terrain, ICmpPathfinder::pass_class_t passClass)
	{
		PROFILE3("JumpPointCache reset");
		TIMER(L"JumpPointCache reset");

		m_Width = terrain->m_W;
		m_Height = terrain->m_H;

		ComputeRows(m_JumpPointsRight, *terrain, passClass, false, false);
		ComputeRows(m_JumpPointsLeft, *terrain, passClass, false, true);
		ComputeRows(m_JumpPointsUp, *terrain, passClass, true, false);
		ComputeRows(m_JumpPointsDown, *terrain, passClass, true, true);
	}

	size_t GetMemoryUsage() const
	{
		size_t bytes = 0;
		for (int i = 0; i < m_Width; ++i)
		{
			bytes += m_JumpPointsUp[i].GetMemoryUsage();
			bytes += m_JumpPointsDown[i].GetMemoryUsage();
		}
		for (int j = 0; j < m_Height; ++j)
		{
			bytes += m_JumpPointsRight[j].GetMemoryUsage();
			bytes += m_JumpPointsLeft[j].GetMemoryUsage();
		}
		return bytes;
	}

	/**
	 * Returns the next jump point (or goal point) to explore,
	 * at (ip, j) where i < ip.
	 * Returns i if there is no such point.
	 */
	int GetJumpPointRight(int i, int j, const PathGoal& goal)
	{
		int ip;
		bool obstruction;
		m_JumpPointsRight[j].Get(i, ip, obstruction);
		// Adjust ip to be a goal cell, if there is one closer than the jump point;
		// and then return the new ip if there is a goal,
		// or the old ip if there is a (non-obstruction) jump point
		if (goal.NavcellRectContainsGoal(i+1, j, ip-1, j, &ip, NULL) || !obstruction)
			return ip;
		return i;
	}

	int GetJumpPointLeft(int i, int j, const PathGoal& goal)
	{
		int mip; // mirrored value, because m_JumpPointsLeft is generated from a mirrored map
		bool obstruction;
		m_JumpPointsLeft[j].Get(m_Width-1 - i, mip, obstruction);
		int ip = m_Width-1 - mip;
		if (goal.NavcellRectContainsGoal(i-1, j, ip+1, j, &ip, NULL) || !obstruction)
			return ip;
		return i;
	}

	int GetJumpPointUp(int i, int j, const PathGoal& goal)
	{
		int jp;
		bool obstruction;
		m_JumpPointsUp[i].Get(j, jp, obstruction);
		if (goal.NavcellRectContainsGoal(i, j+1, i, jp-1, NULL, &jp) || !obstruction)
			return jp;
		return j;
	}

	int GetJumpPointDown(int i, int j, const PathGoal& goal)
	{
		int mjp; // mirrored value
		bool obstruction;
		m_JumpPointsDown[i].Get(m_Height-1 - j, mjp, obstruction);
		int jp = m_Height-1 - mjp;
		if (goal.NavcellRectContainsGoal(i, j-1, i, jp+1, NULL, &jp) || !obstruction)
			return jp;
		return j;
	}
};

/**
 * Tile data for A* computation.
 * (We store an array of one of these per terrain tile, so it ought to be optimised for size)
 */
struct PathfindTileJPS
{
public:
	enum {
		STATUS_UNEXPLORED = 0,
		STATUS_OPEN = 1,
		STATUS_CLOSED = 2
	};

	bool IsUnexplored() { return GetStatus() == STATUS_UNEXPLORED; }
	bool IsOpen() { return GetStatus() == STATUS_OPEN; }
	bool IsClosed() { return GetStatus() == STATUS_CLOSED; }
	void SetStatusOpen() { SetStatus(STATUS_OPEN); }
	void SetStatusClosed() { SetStatus(STATUS_CLOSED); }

	// Get pi,pj coords of predecessor to this tile on best path, given i,j coords of this tile
	int GetPredI(int i) { return i + GetPredDI(); }
	int GetPredJ(int j) { return j + GetPredDJ(); }

	PathCost GetCost() const { return g; }
	void SetCost(PathCost cost) { g = cost; }

private:
	PathCost g; // cost to reach this tile
	u32 data; // 2-bit status; 15-bit PredI; 15-bit PredJ; packed for storage efficiency

public:
	u8 GetStatus() const
	{
		return data & 3;
	}

	void SetStatus(u8 s)
	{
		ASSERT(s < 4);
		data &= ~3;
		data |= (s & 3);
	}

	int GetPredDI() const
	{
		return (i32)data >> 17;
	}

	int GetPredDJ() const
	{
		return ((i32)data << 15) >> 17;
	}

	// Set the pi,pj coords of predecessor, given i,j coords of this tile
	void SetPred(int pi, int pj, int i, int j)
	{
		int di = pi - i;
		int dj = pj - j;
		ASSERT(-16384 <= di && di < 16384);
		ASSERT(-16384 <= dj && dj < 16384);
		data &= 3;
		data |= (((u32)di & 0x7FFF) << 17) | (((u32)dj & 0x7FFF) << 2);
	}
};

//////////////////////////////////////////////////////////

struct PathfinderStateJPS
{
	u32 steps; // number of algorithm iterations

	PathGoal goal;

	u16 iGoal, jGoal; // goal tile

	ICmpPathfinder::pass_class_t passClass;

	PriorityQueue open;
	// (there's no explicit closed list; it's encoded in PathfindTile)

	PathfindTileJPSGrid* tiles;
	Grid<NavcellData>* terrain;

	PathCost hBest; // heuristic of closest discovered tile to goal
	u16 iBest, jBest; // closest tile

	JumpPointCache* jpc;

#if PATHFIND_STATS
	// Performance debug counters
	size_t numProcessed;
	size_t numImproveOpen;
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
// TODO: it'd be nice to not duplicate so much code with CCmpPathfinder_Tile.cpp
static void ProcessNeighbour(int pi, int pj, int i, int j, PathCost pg, PathfinderStateJPS& state)
{
#if PATHFIND_STATS
	state.numProcessed++;
#endif

	// Reject impassable tiles
	if (!IS_PASSABLE(state.terrain->get(i, j), state.passClass))
		return;

	PathfindTileJPS& n = state.tiles->get(i, j);

	if (n.IsClosed())
		return;

	PathCost dg;
	if (pi == i)
		dg = PathCost::horizvert(abs(pj - j));
	else if (pj == j)
		dg = PathCost::horizvert(abs(pi - i));
	else
	{
		ASSERT(abs((int)pi - (int)i) == abs((int)pj - (int)j)); // must be 45 degrees
		dg = PathCost::diag(abs((int)pi - (int)i));
	}

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
			state.open.promote(TileID(i, j), gprev + h, g + h);
#if PATHFIND_STATS
			state.numImproveOpen++;
#endif
			return;
		}
	}

	// Add it to the open list:
	n.SetStatusOpen();
	n.SetCost(g);
	n.SetPred(pi, pj, i, j);
	PriorityQueue::Item t = { TileID(i, j), g + h };
	state.open.push(t);
#if PATHFIND_STATS
	state.numAddToOpen++;
#endif
}

#define PASSABLE(i, j) IS_PASSABLE(state.terrain->get(i, j), state.passClass)

/*
 * In the JPS algorithm, after a tile is taken off the open queue,
 * we don't process every adjacent neighbour (as in standard A*).
 * Instead we only move in a subset of directions (depending on the
 * direction from the predecessor); and instead of moving by a single
 * cell, we move up to the next jump point in that direction.
 * The AddJumped... functions do this by calling ProcessNeighbour
 * on the jump point (if any) in a certain direction.
 * The HasJumped... functions return whether there is any jump point
 * in that direction.
 */

#if USE_JUMPPOINT_CACHE

// Use the jump-point cache to find the jump points:

static void AddJumpedHoriz(int i, int j, int di, PathCost g, PathfinderStateJPS& state)
{
	int jump;
	if (di > 0)
		jump = state.jpc->GetJumpPointRight(i, j, state.goal);
	else
		jump = state.jpc->GetJumpPointLeft(i, j, state.goal);

	if (jump != i)
		ProcessNeighbour(i, j, jump, j, g, state);
}

static bool HasJumpedHoriz(int i, int j, int di, PathfinderStateJPS& state)
{
	int jump;
	if (di > 0)
		jump = state.jpc->GetJumpPointRight(i, j, state.goal);
	else
		jump = state.jpc->GetJumpPointLeft(i, j, state.goal);

	return (jump != i);
}

static void AddJumpedVert(int i, int j, int dj, PathCost g, PathfinderStateJPS& state)
{
	int jump;
	if (dj > 0)
		jump = state.jpc->GetJumpPointUp(i, j, state.goal);
	else
		jump = state.jpc->GetJumpPointDown(i, j, state.goal);

	if (jump != j)
		ProcessNeighbour(i, j, i, jump, g, state);
}

static bool HasJumpedVert(int i, int j, int dj, PathfinderStateJPS& state)
{
	int jump;
	if (dj > 0)
		jump = state.jpc->GetJumpPointUp(i, j, state.goal);
	else
		jump = state.jpc->GetJumpPointDown(i, j, state.goal);

	return (jump != j);
}

#else // USE_JUMPPOINT_CACHE

// Find the jump points by scanning along the map:

static void AddJumpedHoriz(int i, int j, int di, PathCost g, PathfinderStateJPS& state)
{
	ASSERT(di == 1 || di == -1);
	int ni = i + di;
	while (true)
	{
		if (!PASSABLE(ni, j))
			break;

		if ((ni == state.iGoal && j == state.jGoal) || // XXX
#if ACCEPT_DIAGONAL_GAPS
			(!PASSABLE(ni, j-1) && PASSABLE(ni+di, j-1)) ||
			(!PASSABLE(ni, j+1) && PASSABLE(ni+di, j+1)))
#else
			(!PASSABLE(ni-di, j-1) && PASSABLE(ni, j-1)) ||
			(!PASSABLE(ni-di, j+1) && PASSABLE(ni, j+1)))
#endif
		{
			ProcessNeighbour(i, j, ni, j, g, state);
			break;
		}

		ni += di;
	}
}

static bool HasJumpedHoriz(int i, int j, int di, PathfinderStateJPS& state)
{
	ASSERT(di == 1 || di == -1);
	int ni = i + di;
	while (true)
	{
		if (!PASSABLE(ni, j))
			return false;

		if ((ni == state.iGoal && j == state.jGoal) || // XXX
#if ACCEPT_DIAGONAL_GAPS
			(!PASSABLE(ni, j-1) && PASSABLE(ni+di, j-1)) ||
			(!PASSABLE(ni, j+1) && PASSABLE(ni+di, j+1)))
#else
			(!PASSABLE(ni-di, j-1) && PASSABLE(ni, j-1)) ||
			(!PASSABLE(ni-di, j+1) && PASSABLE(ni, j+1)))
#endif
		{
			return true;
		}

		ni += di;
	}
}

static void AddJumpedVert(int i, int j, int dj, PathCost g, PathfinderStateJPS& state)
{
	ASSERT(dj == 1 || dj == -1);
	int nj = j + dj;
	while (true)
	{
		if (!PASSABLE(i, nj))
			break;

		if ((i == state.iGoal && nj == state.jGoal) ||
#if ACCEPT_DIAGONAL_GAPS
			(!PASSABLE(i-1, nj) && PASSABLE(i-1, nj+dj)) ||
			(!PASSABLE(i+1, nj) && PASSABLE(i+1, nj+dj)))
#else
			(!PASSABLE(i-1, nj-dj) && PASSABLE(i-1, nj)) ||
			(!PASSABLE(i+1, nj-dj) && PASSABLE(i+1, nj)))
#endif
		{
			ProcessNeighbour(i, j, i, nj, g, state);
			break;
		}

		nj += dj;
	}
}

static bool HasJumpedVert(int i, int j, int dj, PathfinderStateJPS& state)
{
	ASSERT(dj == 1 || dj == -1);
	int nj = j + dj;
	while (true)
	{
		if (!PASSABLE(i, nj))
			return false;

		if ((i == state.iGoal && nj == state.jGoal) ||
#if ACCEPT_DIAGONAL_GAPS
			(!PASSABLE(i-1, nj) && PASSABLE(i-1, nj+dj)) ||
			(!PASSABLE(i+1, nj) && PASSABLE(i+1, nj+dj)))
#else
			(!PASSABLE(i-1, nj-dj) && PASSABLE(i-1, nj)) ||
			(!PASSABLE(i+1, nj-dj) && PASSABLE(i+1, nj)))
#endif
		{
			return true;
		}

		nj += dj;
	}
}

#endif // USE_JUMPPOINT_CACHE


/*
 * We don't cache diagonal jump points - they're usually so frequent that
 * a linear search is about as cheap and avoids the setup cost and memory cost.
 */
static void AddJumpedDiag(int i, int j, int di, int dj, PathCost g, PathfinderStateJPS& state)
{
// 	ProcessNeighbour(i, j, i + di, j + dj, g, state);
// 	return;

	ASSERT(di == 1 || di == -1);
	ASSERT(dj == 1 || dj == -1);

	int ni = i + di;
	int nj = j + dj;
	while (true)
	{
		// Stop if we hit an obstructed cell
		if (!PASSABLE(ni, nj))
			return;

		// Stop if moving onto this cell caused us to
		// touch the corner of an obstructed cell
#if !ACCEPT_DIAGONAL_GAPS
		if (!PASSABLE(ni - di, nj) || !PASSABLE(ni, nj - dj))
			return;
#endif

		// Process this cell if it's at the goal
		if (state.goal.NavcellContainsGoal(ni, nj))
		{
			ProcessNeighbour(i, j, ni, nj, g, state);
			return;
		}

#if ACCEPT_DIAGONAL_GAPS
		if ((!PASSABLE(ni - di, nj) && PASSABLE(ni - di, nj + dj)) ||
			(!PASSABLE(ni, nj - dj) && PASSABLE(ni + di, nj - dj)))
		{
			ProcessNeighbour(i, j, ni, nj, g, state);
			return;
		}
#endif

		if (HasJumpedHoriz(ni, nj, di, state) || HasJumpedVert(ni, nj, dj, state))
		{
			ProcessNeighbour(i, j, ni, nj, g, state);
			return;
		}

		ni += di;
		nj += dj;
	}
}


void CCmpPathfinder::PathfinderJPSMakeDirty()
{
	m_JumpPointCache.clear();
}

void CCmpPathfinder::ComputePathJPS(entity_pos_t x0, entity_pos_t z0, const PathGoal& origGoal, pass_class_t passClass, Path& path)
{
	UpdateGrid();

	PathfinderStateJPS state = { 0 };

	state.jpc = m_JumpPointCache[passClass].get();
#if USE_JUMPPOINT_CACHE
	if (!state.jpc)
	{
		state.jpc = new JumpPointCache;
		state.jpc->reset(m_Grid, passClass);
		debug_printf(L"PATHFINDER: JPC memory: %d kB\n", (int)state.jpc->GetMemoryUsage() / 1024);
		m_JumpPointCache[passClass] = shared_ptr<JumpPointCache>(state.jpc);
	}
#endif

	PROFILE3("ComputePathJPS");
	//TIMER(L"ComputePathJPS");
	double time = timer_Time();

	// Convert the start coordinates to tile indexes
	u16 i0, j0;
	NearestNavcell(x0, z0, i0, j0);

	if (!IS_PASSABLE(m_Grid->get(i0, j0), passClass))
	{
		// The JPS pathfinder requires units to be on passable tiles
		// (otherwise it might crash), so handle the supposedly-invalid
		// state specially
		ComputePathOffImpassable(i0, j0, passClass, path);
		return;
	}

	state.goal = origGoal;
	PathfinderHierMakeGoalReachable(i0, j0, state.goal, passClass);

	// If we're already at the goal tile, then move directly to the exact goal coordinates
	// XXX: this seems bogus for non-point goals, it should be the point on the current cell nearest the goal
	if (state.goal.NavcellContainsGoal(i0, j0))
	{
		Waypoint w = { state.goal.x, state.goal.z };
		path.m_Waypoints.push_back(w);
		return;
	}

	NearestNavcell(state.goal.x, state.goal.z, state.iGoal, state.jGoal);

	state.passClass = passClass;

	state.steps = 0;

	state.tiles = new PathfindTileJPSGrid(m_Grid->m_W, m_Grid->m_H);
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
		if (state.goal.NavcellContainsGoal(i, j))
		{
			state.iBest = i;
			state.jBest = j;
			state.hBest = PathCost();
			break;
		}

		PathfindTileJPS tile = state.tiles->get(i, j);
		PathCost g = tile.GetCost();

		// Get the direction of the predecessor tile from this tile
		int dpi = tile.GetPredDI();
		int dpj = tile.GetPredDJ();
		dpi = (dpi < 0 ? -1 : dpi > 0 ? 1 : 0);
		dpj = (dpj < 0 ? -1 : dpj > 0 ? 1 : 0);

		if (dpi != 0 && dpj == 0)
		{
			// Moving horizontally from predecessor
#if ACCEPT_DIAGONAL_GAPS
			if (!IS_PASSABLE(state.terrain->get(i, j-1), state.passClass))
				AddJumpedDiag(i, j, -dpi, -1, g, state);
			if (!IS_PASSABLE(state.terrain->get(i, j+1), state.passClass))
				AddJumpedDiag(i, j, -dpi, +1, g, state);
#else
			if (!IS_PASSABLE(state.terrain->get(i + dpi, j-1), state.passClass))
			{
				AddJumpedDiag(i, j, -dpi, -1, g, state);
				AddJumpedVert(i, j, -1, g, state);
			}
			if (!IS_PASSABLE(state.terrain->get(i + dpi, j+1), state.passClass))
			{
				AddJumpedDiag(i, j, -dpi, +1, g, state);
				AddJumpedVert(i, j, +1, g, state);
			}
#endif
			AddJumpedHoriz(i, j, -dpi, g, state);
		}
		else if (dpi == 0 && dpj != 0)
		{
			// Moving vertically from predecessor
#if ACCEPT_DIAGONAL_GAPS
			if (!IS_PASSABLE(state.terrain->get(i-1, j), state.passClass))
				AddJumpedDiag(i, j, -1, -dpj, g, state);
			if (!IS_PASSABLE(state.terrain->get(i+1, j), state.passClass))
				AddJumpedDiag(i, j, +1, -dpj, g, state);
#else
			if (!IS_PASSABLE(state.terrain->get(i-1, j + dpj), state.passClass))
			{
				AddJumpedDiag(i, j, -1, -dpj, g, state);
				AddJumpedHoriz(i, j, -1, g, state);
			}
			if (!IS_PASSABLE(state.terrain->get(i+1, j + dpj), state.passClass))
			{
				AddJumpedDiag(i, j, +1, -dpj, g, state);
				AddJumpedHoriz(i, j, +1, g, state);
			}
#endif
			AddJumpedVert(i, j, -dpj, g, state);
		}
		else if (dpi != 0 && dpj != 0)
		{
			// Moving diagonally from predecessor
#if ACCEPT_DIAGONAL_GAPS
			if (!IS_PASSABLE(state.terrain->get(i + dpi, j), state.passClass))
				AddJumpedDiag(i, j, dpi, -dpj, g, state);
			if (!IS_PASSABLE(state.terrain->get(i, j + dpj), state.passClass))
				AddJumpedDiag(i, j, -dpi, dpj, g, state);
#endif
			AddJumpedHoriz(i, j, -dpi, g, state);
			AddJumpedVert(i, j, -dpj, g, state);
			AddJumpedDiag(i, j, -dpi, -dpj, g, state);
		}
		else
		{
			// No predecessor, i.e. the start tile
			// Start searching in every direction

			// XXX - check passability?

			bool passl = IS_PASSABLE(state.terrain->get(i-1, j), state.passClass);
			bool passr = IS_PASSABLE(state.terrain->get(i+1, j), state.passClass);
			bool passd = IS_PASSABLE(state.terrain->get(i, j-1), state.passClass);
			bool passu = IS_PASSABLE(state.terrain->get(i, j+1), state.passClass);

			if (passl && passd)
				ProcessNeighbour(i, j, i-1, j-1, g, state);
			if (passr && passd)
				ProcessNeighbour(i, j, i+1, j-1, g, state);
			if (passl && passu)
				ProcessNeighbour(i, j, i-1, j+1, g, state);
			if (passr && passu)
				ProcessNeighbour(i, j, i+1, j+1, g, state);
			if (passl)
				ProcessNeighbour(i, j, i-1, j, g, state);
			if (passr)
				ProcessNeighbour(i, j, i+1, j, g, state);
			if (passd)
				ProcessNeighbour(i, j, i, j-1, g, state);
			if (passu)
				ProcessNeighbour(i, j, i, j+1, g, state);
		}
	}

	// Reconstruct the path (in reverse)
	u16 ip = state.iBest, jp = state.jBest;
	while (ip != i0 || jp != j0)
	{
		PathfindTileJPS& n = state.tiles->get(ip, jp);
		entity_pos_t x, z;
		NavcellCenter(ip, jp, x, z);
		Waypoint w = { x, z };
		path.m_Waypoints.push_back(w);

		// Follow the predecessor link
		ip = n.GetPredI(ip);
		jp = n.GetPredJ(jp);
	}

	NormalizePathWaypoints(path);

	// Save this grid for debug display
	m_DebugTime = timer_Time() - time;
	delete m_DebugGridJPS;
	m_DebugGridJPS = state.tiles;
	m_DebugSteps = state.steps;
	m_DebugGoal = state.goal;

	PROFILE2_ATTR("from: (%d, %d)", i0, j0);
	PROFILE2_ATTR("to: (%d, %d)", state.iGoal, state.jGoal);
	PROFILE2_ATTR("reached: (%d, %d)", state.iBest, state.jBest);
	PROFILE2_ATTR("steps: %d", state.steps);

#if PATHFIND_STATS
	debug_printf(L"PATHFINDER: steps=%d avgo=%d proc=%d impo=%d addo=%d\n", state.steps, state.sumOpenSize/state.steps, state.numProcessed, state.numImproveOpen, state.numAddToOpen);
#endif
}

void CCmpPathfinder::GetDebugDataJPS(u32& steps, double& time, Grid<u8>& grid)
{
	steps = m_DebugSteps;
	time = m_DebugTime;

	if (!m_DebugGridJPS)
		return;

	u16 iGoal, jGoal;
	NearestNavcell(m_DebugGoal.x, m_DebugGoal.z, iGoal, jGoal);

	grid = Grid<u8>(m_DebugGridJPS->m_W, m_DebugGridJPS->m_H);
	for (u16 j = 0; j < grid.m_H; ++j)
	{
		for (u16 i = 0; i < grid.m_W; ++i)
		{
			if (i == iGoal && j == jGoal)
				continue;
			PathfindTileJPS t = m_DebugGridJPS->get(i, j);
			grid.set(i, j, (t.IsOpen() ? 1 : 0) | (t.IsClosed() ? 2 : 0));
		}
	}
}
