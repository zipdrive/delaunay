using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Retriangulator2D.Voronoi;

/// <summary>
/// A Voronoi diagram.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
/// <typeparam name="Centroid">The data type for each region's centroid.</typeparam>
public class VoronoiDiagram<T, Centroid> where T : IFloatingPointIeee754<T> where Centroid : IVertex2<T>
{
	/// <summary>
	/// The vertices of the Voronoi diagram.
	/// Correspond to the circumcenters of the triangles in the Delaunay triangulation.
	/// </summary>
	protected HashSet<IVertex2<T>> _Vertices;

	/// <summary>
	/// The vertices of the Voronoi diagram.
	/// Correspond to the circumcenters of the triangles in the Delaunay triangulation.
	/// </summary>
	public IEnumerable<IVertex2<T>> Vertices => _Vertices;

	/// <summary>
	/// The edges of the Voronoi diagram. An edge may be a VoronoiBoundedEdge (if finite) or VoronoiUnboundedEdge (if infinite).
	/// Correspond to triangle adjacencies of the Delaunay triangulation.
	/// </summary>
	protected HashSet<VoronoiEdge<T, Centroid>> _Edges;

	/// <summary>
	/// The edges of the Voronoi diagram. An edge may be a VoronoiBoundedEdge (if finite) or VoronoiUnboundedEdge (if infinite).
	/// Correspond to triangle adjacencies of the Delaunay triangulation.
	/// </summary>
	public IEnumerable<VoronoiEdge<T, Centroid>> Edges => _Edges;

	/// <summary>
	/// The regions of the Voronoi diagram.
	/// The centroids of each region correspond to the vertices of the Delaunay triangulation.
	/// </summary>
	protected List<VoronoiRegion<T, Centroid>> _Regions;

	/// <summary>
	/// The regions of the Voronoi diagram.
	/// The centroids of each region correspond to the vertices of the Delaunay triangulation.
	/// </summary>
	public IEnumerable<VoronoiRegion<T, Centroid>> Regions => _Regions;

	/// <summary>
	/// Constructs the Voronoi diagram that is the dual to a Delaunay triangulation.
	/// </summary>
	/// <param name="mesh">A constructed Delaunay triangulation.</param>
	/// <returns>The Voronoi diagram that is the dual to that triangulation.</returns>
	public static VoronoiDiagram<T, Centroid> FromMesh(Mesh<T, Centroid> mesh)
	{
		// Construct regions corresponding to each vertex of the original graph
		Dictionary<Centroid, VoronoiRegion<T, Centroid>> regions = new Dictionary<Centroid, VoronoiRegion<T, Centroid>>(
			mesh.Vertices.Select(vertex => new KeyValuePair<Centroid, VoronoiRegion<T, Centroid>>(vertex, new VoronoiRegion<T, Centroid>(vertex)))
		);
		foreach (var pair in regions)
		{
			var edges = mesh.FindAllEdges(pair.Key);
			foreach (Edge<T, Centroid> edge in edges)
			{
				Centroid otherVertex = edge.Vertex1.Equals(pair.Key) ? edge.Vertex2 : edge.Vertex1;
				pair.Value.AddAdjacentRegion(regions[otherVertex]);
			}
		}

		// Construct edges from adjacencies of triangles
		HashSet<VoronoiEdge<T, Centroid>> voronoiEdges = new HashSet<VoronoiEdge<T, Centroid>>();
		foreach (Edge<T, Centroid> edge in mesh.Edges)
		{
			if (edge.Left != null)
			{
				if (edge.Right != null)
				{
					voronoiEdges.Add(new VoronoiBoundedEdge<T, Centroid>(edge.Left.CircumcircleCenter, edge.Right.CircumcircleCenter, regions[edge.Vertex2], regions[edge.Vertex1]));
				}
				else
				{
					VoronoiRegion<T, Centroid> left = regions[edge.Vertex2], right = regions[edge.Vertex1];
					Centroid vleft = left.RegionCentroid, vright = right.RegionCentroid;
					Vector2<T> edgeVector = new Vector2<T> { X = vleft.Y - vright.Y, Y = vright.X - vleft.X };
					voronoiEdges.Add(new VoronoiUnboundedEdge<T, Centroid>(edge.Left.CircumcircleCenter, edgeVector, left, right));
				}
			}
			else if (edge.Right != null)
			{
				VoronoiRegion<T, Centroid> left = regions[edge.Vertex1], right = regions[edge.Vertex2];
				Centroid vleft = left.RegionCentroid, vright = right.RegionCentroid;
				Vector2<T> edgeVector = new Vector2<T> { X = vleft.Y - vright.Y, Y = vright.X - vleft.X };
				voronoiEdges.Add(new VoronoiUnboundedEdge<T, Centroid>(edge.Right.CircumcircleCenter, edgeVector, left, right));
			}
		}

		// Construct vertices from circumcenters of triangles
		return new VoronoiDiagram<T, Centroid>(
			mesh.Triangles.Select(triangle => triangle.CircumcircleCenter),
			voronoiEdges,
			regions.Values
		);
	}

	internal VoronoiDiagram(IEnumerable<IVertex2<T>> vertices, HashSet<VoronoiEdge<T, Centroid>> edges, IEnumerable<VoronoiRegion<T, Centroid>> regions)
	{
		_Vertices = new HashSet<IVertex2<T>>(vertices);
		_Edges = edges;
		_Regions = new List<VoronoiRegion<T, Centroid>>(regions);
	}
}
