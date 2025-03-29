using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Retriangulator2D.Voronoi;

public class VoronoiRegion<T, Centroid> where T : IFloatingPointIeee754<T> where Centroid : IPoint2<T>
{
	/// <summary>
	/// The centroid of the Voronoi region.
	/// </summary>
	public readonly Centroid RegionCentroid;

	/// <summary>
	/// The regions adjacent to this one.
	/// </summary>
	private List<VoronoiRegion<T, Centroid>> _AdjacentRegions = new List<VoronoiRegion<T, Centroid>>();

	/// <summary>
	/// The regions adjacent to this one.
	/// </summary>
	public IEnumerable<VoronoiRegion<T, Centroid>> AdjacentRegions => _AdjacentRegions;

	internal VoronoiRegion(Centroid centroid)
	{
		RegionCentroid = centroid;
	}

	/// <summary>
	/// Adds an adjacent region to this one.
	/// </summary>
	/// <param name="adjacentRegion">The adjacent region to register.</param>
	internal void AddAdjacentRegion(VoronoiRegion<T, Centroid> adjacentRegion)
	{
		_AdjacentRegions.Add(adjacentRegion);
	}
}
