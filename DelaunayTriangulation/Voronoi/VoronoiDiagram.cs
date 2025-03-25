using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace DelaunayTriangulation.Voronoi;

/// <summary>
/// A Voronoi diagram.
/// </summary>
/// <typeparam name="T"></typeparam>
/// <typeparam name="Vertex"></typeparam>
public class VoronoiDiagram<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
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
	protected HashSet<VoronoiEdge<T, Vertex>> _Edges;

	/// <summary>
	/// The edges of the Voronoi diagram. An edge may be a VoronoiBoundedEdge (if finite) or VoronoiUnboundedEdge (if infinite).
	/// Correspond to triangle adjacencies of the Delaunay triangulation.
	/// </summary>
	public IEnumerable<VoronoiEdge<T, Vertex>> Edges => _Edges;

	/// <summary>
	/// The regions of the Voronoi diagram.
	/// The centroids of each region correspond to the vertices of the Delaunay triangulation.
	/// </summary>
	protected List<VoronoiRegion<T, Vertex>> _Regions;

	/// <summary>
	/// The regions of the Voronoi diagram.
	/// The centroids of each region correspond to the vertices of the Delaunay triangulation.
	/// </summary>
	public IEnumerable<VoronoiRegion<T, Vertex>> Regions => _Regions;

	/// <summary>
	/// Constructs the Voronoi diagram that is the dual to a Delaunay triangulation.
	/// </summary>
	/// <param name="mesh">A constructed Delaunay triangulation.</param>
	/// <returns>The Voronoi diagram that is the dual to that triangulation.</returns>
	public static VoronoiDiagram<T, Vertex> FromMesh(Mesh<T, Vertex> mesh)
	{
		// Construct regions corresponding to each vertex of the original graph
		Dictionary<Vertex, VoronoiRegion<T, Vertex>> regions = new Dictionary<Vertex, VoronoiRegion<T, Vertex>>(
			mesh.Vertices.Select(vertex => new KeyValuePair<Vertex, VoronoiRegion<T, Vertex>>(vertex, new VoronoiRegion<T, Vertex>(vertex)))
		);
		foreach (var pair in regions)
		{
			var edges = mesh.FindAllEdges(pair.Key);
			foreach (Edge<T, Vertex> edge in edges)
			{
				Vertex otherVertex = edge.Vertex1.Equals(pair.Key) ? edge.Vertex2 : edge.Vertex1;
				pair.Value.AddAdjacentRegion(regions[otherVertex]);
			}
		}

		// Construct edges from adjacencies of triangles
		HashSet<VoronoiEdge<T, Vertex>> voronoiEdges = new HashSet<VoronoiEdge<T, Vertex>>();
		foreach (Edge<T, Vertex> edge in mesh.Edges)
		{
			if (edge.Left != null)
			{
				if (edge.Right != null)
				{
					voronoiEdges.Add(new VoronoiBoundedEdge<T, Vertex>(edge.Left.CircumcircleCenter, edge.Right.CircumcircleCenter, regions[edge.Vertex2], regions[edge.Vertex1]));
				}
				else
				{
					voronoiEdges.Add(new VoronoiUnboundedEdge<T, Vertex>(edge.Left.CircumcircleCenter, regions[edge.Vertex2], regions[edge.Vertex1]));
				}
			}
			else if (edge.Right != null)
			{
				voronoiEdges.Add(new VoronoiUnboundedEdge<T, Vertex>(edge.Right.CircumcircleCenter, regions[edge.Vertex1], regions[edge.Vertex2]));
			}
		}

		// Construct vertices from circumcenters of triangles
		return new VoronoiDiagram<T, Vertex>
		{
			_Vertices = new HashSet<IVertex2<T>>(mesh.Triangles.Select(triangle => triangle.CircumcircleCenter)),
			_Edges = voronoiEdges,
			_Regions = new List<VoronoiRegion<T, Vertex>>(regions.Values)
		};
	}
}
