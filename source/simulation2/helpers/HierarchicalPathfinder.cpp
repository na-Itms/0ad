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

#include "precompiled.h"

#include "HierarchicalPathfinder.h"

#include "graphics/Overlay.h"
#include "ps/Profile.h"

#define PATHFINDER_HIER_PROFILE 1
#if PATHFINDER_HIER_PROFILE
	#include "lib/timer.h"
	TIMER_ADD_CLIENT(tc_MakeGoalReachable);
	TIMER_ADD_CLIENT(tc_InitRegions);
#else
	#define	TIMER_ACCRUE(a) ;
#endif

// Find the root ID of a region, used by InitRegions
inline u16 RootID(u16 x, std::vector<u16> v)
{
	// Just add a basic check for infinite loops
	int checkLoop = 0;
	while (v[x] < x)
	{
		++checkLoop;
		ENSURE(checkLoop < 1000 && "Long loop (probably infinite), unable to find an invariant point");
		x = v[x];
	}

	return x;
}

void HierarchicalPathfinder::Chunk::InitRegions(int ci, int cj, Grid<NavcellData>* grid, pass_class_t passClass)
{
	TIMER_ACCRUE(tc_InitRegions);
	ENSURE(ci < 256 && cj < 256); // avoid overflows
	m_ChunkI = ci;
	m_ChunkJ = cj;

	memset(m_Regions, 0, sizeof(m_Regions));

	int i0 = ci * CHUNK_SIZE;
	int j0 = cj * CHUNK_SIZE;
	int i1 = std::min(i0 + CHUNK_SIZE, (int)grid->m_W);
	int j1 = std::min(j0 + CHUNK_SIZE, (int)grid->m_H);

	// Efficiently flood-fill the m_Regions grid

	int regionID = 0;
	std::vector<u16> connect;

	u16* pCurrentID = NULL;
	u16 LeftID = 0;
	u16 DownID = 0;

	connect.reserve(32); // TODO: What's a sensible number?
	connect.push_back(0); // connect[0] = 0

	// Start by filling the grid with 0 for blocked,
	// and regionID for unblocked
	for (int j = j0; j < j1; ++j)
	{
		for (int i = i0; i < i1; ++i)
		{
			pCurrentID = &m_Regions[j-j0][i-i0];
			if (!IS_PASSABLE(grid->get(i, j), passClass))
			{
				*pCurrentID = 0;
				continue;
			}

			if (j > j0)
				DownID = m_Regions[j-1-j0][i-i0];

			if (i == i0)
				LeftID = 0;
			else
				LeftID = m_Regions[j-j0][i-1-i0];

			if (LeftID > 0)
			{
				*pCurrentID = LeftID;
				if (*pCurrentID != DownID && DownID > 0)
				{
					u16 id0 = RootID(DownID, connect);
					u16 id1 = RootID(LeftID, connect);

					if (id0 < id1)
						connect[id1] = id0;
					else if (id0 > id1)
						connect[id0] = id1;
				}
			}
			else if (DownID > 0)
				*pCurrentID = DownID;
			else
			{
				// New ID
				*pCurrentID = ++regionID;
				connect.push_back(regionID);
			}
		}
	}

	// Directly point the root ID
	m_NumRegions = 0;
	for (u16 i = regionID; i > 0; --i)
	{
		if (connect[i] == i)
			++m_NumRegions;
		else
			connect[i] = RootID(i,connect);
	}

	// Replace IDs by the root ID
	for (int i = 0; i < CHUNK_SIZE; ++i)
		for (int j = 0; j < CHUNK_SIZE; ++j)
			m_Regions[i][j] = connect[m_Regions[i][j]];
}

/**
 * Returns a RegionID for the given global navcell coords
 * (which must be inside this chunk);
 */
HierarchicalPathfinder::RegionID HierarchicalPathfinder::Chunk::Get(int i, int j)
{
	ENSURE(i < CHUNK_SIZE && j < CHUNK_SIZE);
	return RegionID(m_ChunkI, m_ChunkJ, m_Regions[j][i]);
}

/**
 * Return the global navcell coords that correspond roughly to the
 * center of the given region in this chunk.
 * (This is not guaranteed to be actually inside the region.)
 */
void HierarchicalPathfinder::Chunk::RegionCenter(u16 r, int& i_out, int& j_out) const
{
	// Find the mean of i,j coords of navcells in this region:

	u32 si = 0, sj = 0; // sum of i,j coords
	u32 n = 0; // number of navcells in region

	cassert(CHUNK_SIZE < 256); // conservative limit to ensure si and sj don't overflow

	for (int j = 0; j < CHUNK_SIZE; ++j)
	{
		for (int i = 0; i < CHUNK_SIZE; ++i)
		{
			if (m_Regions[j][i] == r)
			{
				si += i;
				sj += j;
				n += 1;
			}
		}
	}

	// Avoid divide-by-zero
	if (n == 0)
		n = 1;

	i_out = m_ChunkI * CHUNK_SIZE + si / n;
	j_out = m_ChunkJ * CHUNK_SIZE + sj / n;
}

/**
 * Returns whether any navcell in the given region is inside the goal.
 */
bool HierarchicalPathfinder::Chunk::RegionContainsGoal(u16 r, const PathGoal& goal) const
{
	// Inefficiently check every single navcell:
	for (u16 j = 0; j < CHUNK_SIZE; ++j)
	{
		for (u16 i = 0; i < CHUNK_SIZE; ++i)
		{
			if (m_Regions[j][i] == r)
			{
				if (goal.NavcellContainsGoal(m_ChunkI * CHUNK_SIZE + i, m_ChunkJ * CHUNK_SIZE + j))
					return true;
			}
		}
	}

	return false;
}

/**
 * Returns the global navcell coords, and the squared distance to the goal
 * navcell, of whichever navcell inside the given region is closest to
 * that goal.
 */
void HierarchicalPathfinder::Chunk::RegionNavcellNearest(u16 r, int iGoal, int jGoal, int& iBest, int& jBest, u32& dist2Best) const
{
	iBest = 0;
	jBest = 0;
	dist2Best = std::numeric_limits<u32>::max();

	for (int j = 0; j < CHUNK_SIZE; ++j)
	{
		for (int i = 0; i < CHUNK_SIZE; ++i)
		{
			if (m_Regions[j][i] == r)
			{
				u32 dist2 = (i + m_ChunkI*CHUNK_SIZE - iGoal)*(i + m_ChunkI*CHUNK_SIZE - iGoal)
				          + (j + m_ChunkJ*CHUNK_SIZE - jGoal)*(j + m_ChunkJ*CHUNK_SIZE - jGoal);

				if (dist2 < dist2Best)
				{
					iBest = i + m_ChunkI*CHUNK_SIZE;
					jBest = j + m_ChunkJ*CHUNK_SIZE;
					dist2Best = dist2;
				}
			}
		}
	}
}

HierarchicalPathfinder::HierarchicalPathfinder()
{
	m_DebugOverlay = NULL;
}

HierarchicalPathfinder::~HierarchicalPathfinder()
{
	SAFE_DELETE(m_DebugOverlay);
}

void HierarchicalPathfinder::SetDebugOverlay(bool enabled, const CSimContext* simContext)
{
	if (enabled && !m_DebugOverlay)
	{
		m_DebugOverlay = new HierarchicalOverlay(*this);
		m_DebugOverlayLines.clear();
		m_SimContext = simContext;
		AddDebugEdges(GetPassabilityClass("default"));
	}
	else if (!enabled && m_DebugOverlay)
	{
		SAFE_DELETE(m_DebugOverlay);
		m_DebugOverlayLines.clear();
		m_SimContext = NULL;
	}
}

void HierarchicalPathfinder::Recompute(const std::map<std::string, pass_class_t>& passClassMasks, Grid<NavcellData>* grid)
{
	PROFILE3("Hierarchical Recompute");

	m_PassClassMasks = passClassMasks;

	m_W = grid->m_W;
	m_H = grid->m_H;

	// Divide grid into chunks with round-to-positive-infinity
	m_ChunksW = (grid->m_W + CHUNK_SIZE - 1) / CHUNK_SIZE;
	m_ChunksH = (grid->m_H + CHUNK_SIZE - 1) / CHUNK_SIZE;

	ENSURE(m_ChunksW < 256 && m_ChunksH < 256); // else the u8 Chunk::m_ChunkI will overflow

	m_Chunks.clear();
	m_Edges.clear();

	for (auto& passClassMask : passClassMasks)
	{
		pass_class_t passClass = passClassMask.second;

		// Compute the regions within each chunk
		m_Chunks[passClass].resize(m_ChunksW*m_ChunksH);
		for (int cj = 0; cj < m_ChunksH; ++cj)
		{
			for (int ci = 0; ci < m_ChunksW; ++ci)
			{
				m_Chunks[passClass].at(cj*m_ChunksW + ci).InitRegions(ci, cj, grid, passClass);
			}
		}

		// Construct the search graph over the regions

		EdgesMap& edges = m_Edges[passClass];

		for (int cj = 0; cj < m_ChunksH; ++cj)
		{
			for (int ci = 0; ci < m_ChunksW; ++ci)
			{
				FindEdges(ci, cj, passClass, edges);
			}
		}
	}

	if (m_DebugOverlay)
	{
		PROFILE("debug overlay");
		m_DebugOverlayLines.clear();
		AddDebugEdges(GetPassabilityClass("default"));
	}
}

/**
 * Find edges between regions in this chunk and the adjacent below/left chunks.
 */
void HierarchicalPathfinder::FindEdges(u8 ci, u8 cj, pass_class_t passClass, EdgesMap& edges)
{
	std::vector<Chunk>& chunks = m_Chunks[passClass];

	Chunk& a = chunks.at(cj*m_ChunksW + ci);

	// For each edge between chunks, we loop over every adjacent pair of
	// navcells in the two chunks. If they are both in valid regions
	// (i.e. are passable navcells) then add a graph edge between those regions.
	// (We don't need to test for duplicates since EdgesMap already uses a
	// std::set which will drop duplicate entries.)

	if (ci > 0)
	{
		Chunk& b = chunks.at(cj*m_ChunksW + (ci-1));
		for (int j = 0; j < CHUNK_SIZE; ++j)
		{
			RegionID ra = a.Get(0, j);
			RegionID rb = b.Get(CHUNK_SIZE-1, j);
			if (ra.r && rb.r)
			{
				edges[ra].insert(rb);
				edges[rb].insert(ra);
			}
		}
	}

	if (cj > 0)
	{
		Chunk& b = chunks.at((cj-1)*m_ChunksW + ci);
		for (int i = 0; i < CHUNK_SIZE; ++i)
		{
			RegionID ra = a.Get(i, 0);
			RegionID rb = b.Get(i, CHUNK_SIZE-1);
			if (ra.r && rb.r)
			{
				edges[ra].insert(rb);
				edges[rb].insert(ra);
			}
		}
	}

}

/**
 * Debug visualisation of graph edges between regions.
 */
void HierarchicalPathfinder::AddDebugEdges(pass_class_t passClass)
{
	const EdgesMap& edges = m_Edges[passClass];
	const std::vector<Chunk>& chunks = m_Chunks[passClass];

	for (EdgesMap::const_iterator it = edges.begin(); it != edges.end(); ++it)
	{
		for (std::set<RegionID>::const_iterator rit = it->second.begin(); rit != it->second.end(); ++rit)
		{
			// Draw a line between the two regions' centers

			int i0, j0, i1, j1;
			chunks[it->first.cj * m_ChunksW + it->first.ci].RegionCenter(it->first.r, i0, j0);
			chunks[rit->cj * m_ChunksW + rit->ci].RegionCenter(rit->r, i1, j1);

			CFixedVector2D a, b;
			Pathfinding::NavcellCenter(i0, j0, a.X, a.Y);
			Pathfinding::NavcellCenter(i1, j1, b.X, b.Y);

			// Push the endpoints inwards a little to avoid overlaps
			CFixedVector2D d = b - a;
			d.Normalize(entity_pos_t::FromInt(1));
			a += d;
			b -= d;

			std::vector<float> xz;
			xz.push_back(a.X.ToFloat());
			xz.push_back(a.Y.ToFloat());
			xz.push_back(b.X.ToFloat());
			xz.push_back(b.Y.ToFloat());

			m_DebugOverlayLines.push_back(SOverlayLine());
			m_DebugOverlayLines.back().m_Color = CColor(1.0, 1.0, 1.0, 1.0);
			SimRender::ConstructLineOnGround(*m_SimContext, xz, m_DebugOverlayLines.back(), true);
		}
	}
}

HierarchicalPathfinder::RegionID HierarchicalPathfinder::Get(u16 i, u16 j, pass_class_t passClass)
{
	int ci = i / CHUNK_SIZE;
	int cj = j / CHUNK_SIZE;
	ENSURE(ci < m_ChunksW && cj < m_ChunksH);
	return m_Chunks[passClass][cj*m_ChunksW + ci].Get(i % CHUNK_SIZE, j % CHUNK_SIZE);
}

bool HierarchicalPathfinder::MakeGoalReachable(u16 i0, u16 j0, PathGoal& goal, pass_class_t passClass)
{
	TIMER_ACCRUE(tc_MakeGoalReachable);
	RegionID source = Get(i0, j0, passClass);

	// Find everywhere that's reachable
	std::set<RegionID> reachableRegions;
	FindReachableRegions(source, reachableRegions, passClass);

// 	debug_printf("\nReachable from (%d,%d): ", i0, j0);
// 	for (std::set<RegionID>::iterator it = reachableRegions.begin(); it != reachableRegions.end(); ++it)
// 		debug_printf("[%d,%d,%d], ", it->ci, it->cj, it->r);
// 	debug_printf("\n");

	// Check whether any reachable region contains the goal
	for (std::set<RegionID>::const_iterator it = reachableRegions.begin(); it != reachableRegions.end(); ++it)
	{
		// Skip region if its chunk doesn't contain the goal area
		entity_pos_t x0 = Pathfinding::NAVCELL_SIZE * (it->ci * CHUNK_SIZE);
		entity_pos_t z0 = Pathfinding::NAVCELL_SIZE * (it->cj * CHUNK_SIZE);
		entity_pos_t x1 = x0 + Pathfinding::NAVCELL_SIZE * CHUNK_SIZE;
		entity_pos_t z1 = z0 + Pathfinding::NAVCELL_SIZE * CHUNK_SIZE;
		if (!goal.RectContainsGoal(x0, z0, x1, z1))
			continue;

		// If the region contains the goal area, the goal is reachable
		// and we don't need to move it
		if (GetChunk(it->ci, it->cj, passClass).RegionContainsGoal(it->r, goal))
			return false;
	}

	// The goal area wasn't reachable,
	// so find the navcell that's nearest to the goal's center

	u16 iGoal, jGoal;
	Pathfinding::NearestNavcell(goal.x, goal.z, iGoal, jGoal, m_W, m_H);

	FindNearestNavcellInRegions(reachableRegions, iGoal, jGoal, passClass);

	// Construct a new point goal at the nearest reachable navcell
	PathGoal newGoal;
	newGoal.type = PathGoal::POINT;
	Pathfinding::NavcellCenter(iGoal, jGoal, newGoal.x, newGoal.z);
	goal = newGoal;

	return true;
}

void HierarchicalPathfinder::FindNearestPassableNavcell(u16& i, u16& j, pass_class_t passClass)
{
	std::set<RegionID> regions;
	FindPassableRegions(regions, passClass);
	FindNearestNavcellInRegions(regions, i, j, passClass);
}

void HierarchicalPathfinder::FindNearestNavcellInRegions(const std::set<RegionID>& regions, u16& iGoal, u16& jGoal, pass_class_t passClass)
{
	// Find the navcell in the given regions that's nearest to the goal navcell:
	// * For each region, record the (squared) minimal distance to the goal point
	// * Sort regions by that underestimated distance
	// * For each region, find the actual nearest navcell
	// * Stop when the underestimated distances are worse than the best real distance

	std::vector<std::pair<u32, RegionID> > regionDistEsts; // pair of (distance^2, region)

	for (std::set<RegionID>::const_iterator it = regions.begin(); it != regions.end(); ++it)
	{
		int i0 = it->ci * CHUNK_SIZE;
		int j0 = it->cj * CHUNK_SIZE;
		int i1 = i0 + CHUNK_SIZE - 1;
		int j1 = j0 + CHUNK_SIZE - 1;

		// Pick the point in the chunk nearest the goal
		int iNear = Clamp((int)iGoal, i0, i1);
		int jNear = Clamp((int)jGoal, j0, j1);

		int dist2 = (iNear - iGoal)*(iNear - iGoal)
		          + (jNear - jGoal)*(jNear - jGoal);

		regionDistEsts.push_back(std::make_pair(dist2, *it));
	}

	// Sort by increasing distance (tie-break on RegionID)
	std::sort(regionDistEsts.begin(), regionDistEsts.end());

	int iBest = iGoal;
	int jBest = jGoal;
	u32 dist2Best = std::numeric_limits<u32>::max();

	for (size_t n = 0; n < regionDistEsts.size(); ++n)
	{
		RegionID region = regionDistEsts[n].second;

		if (regionDistEsts[n].first >= dist2Best)
			break;

		int i, j;
		u32 dist2;
		GetChunk(region.ci, region.cj, passClass).RegionNavcellNearest(region.r, iGoal, jGoal, i, j, dist2);

		if (dist2 < dist2Best)
		{
			iBest = i;
			jBest = j;
			dist2Best = dist2;
		}
	}

	iGoal = iBest;
	jGoal = jBest;
}

void HierarchicalPathfinder::FindReachableRegions(RegionID from, std::set<RegionID>& reachable, pass_class_t passClass)
{
	// Flood-fill the region graph, starting at 'from',
	// collecting all the regions that are reachable via edges

	std::vector<RegionID> open;
	open.push_back(from);
	reachable.insert(from);

	while (!open.empty())
	{
		RegionID curr = open.back();
		open.pop_back();

		const std::set<RegionID>& neighbours = m_Edges[passClass][curr];
		for (std::set<RegionID>::const_iterator it = neighbours.begin(); it != neighbours.end(); ++it)
		{
			// Add to the reachable set; if this is the first time we added
			// it then also add it to the open list
			if (reachable.insert(*it).second)
				open.push_back(*it);
		}
	}
}

void HierarchicalPathfinder::FindPassableRegions(std::set<RegionID>& regions, pass_class_t passClass)
{
	// Construct a set of all regions of all chunks for this pass class

	const std::vector<Chunk>& chunks = m_Chunks[passClass];
	for (size_t c = 0; c < chunks.size(); ++c)
	{
		// region 0 is impassable tiles
		for (int r = 1; r <= chunks[c].m_NumRegions; ++r)
			regions.insert(RegionID(chunks[c].m_ChunkI, chunks[c].m_ChunkJ, r));
	}
}

Grid<u16> HierarchicalPathfinder::GetConnectivityGrid(pass_class_t passClass)
{
	Grid<u16> connectivityGrid(m_W, m_H);
	connectivityGrid.reset();

	u16 idx = 1;

	for (size_t j1 = 0; j1 < m_H; ++j1)
	{
		for (size_t i1 = 0; i1 < m_W; ++i1)
		{
			if (connectivityGrid.get(i1, j1) != 0)
				continue;

			RegionID region = Get(i1, j1, passClass);
			if (region.r == 0)
				continue;

			std::set<RegionID> reachable;
			FindReachableRegions(region, reachable, passClass);
			for (size_t j2 = 0; j2 < m_H; ++j2)
			{
				for (size_t i2 = 0; i2 < m_W; ++i2)
				{
					if (std::find(reachable.begin(), reachable.end(), Get(i2, j2, passClass)) != reachable.end())
						connectivityGrid.set(i1, j1, idx);
				}
			}
			++idx;
		}
	}

	return connectivityGrid;
}
