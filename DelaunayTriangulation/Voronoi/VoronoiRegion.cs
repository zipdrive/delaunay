using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace DelaunayTriangulation.Voronoi;

public class VoronoiRegion<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
	/// <summary>
	/// The centroid of the Voronoi region.
	/// </summary>
	public readonly Vertex Centroid;

	private List<VoronoiRegion<T, Vertex>> _AdjacentRegions = new List<VoronoiRegion<T, Vertex>>();

	public IEnumerable<VoronoiRegion<T, Vertex>> AdjacentRegions => _AdjacentRegions;

	internal VoronoiRegion(Vertex centroid)
	{
		Centroid = centroid;
	}

	/// <summary>
	/// Adds an adjacent region to this one.
	/// </summary>
	/// <param name="adjacentRegion">The adjacent region to register.</param>
	internal void AddAdjacentRegion(VoronoiRegion<T, Vertex> adjacentRegion)
	{
		_AdjacentRegions.Add(adjacentRegion);
	}
}
