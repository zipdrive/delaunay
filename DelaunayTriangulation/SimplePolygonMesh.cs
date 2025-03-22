using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace DelaunayTriangulation;

/// <summary>
/// A Delaunay triangulation where each vertex is on the boundary of the mesh.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
/// <typeparam name="Vertex">The data type for each vertex.</typeparam>
public class SimplePolygonMesh<T, Vertex> : Mesh<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
	#region Initial construction

	/// <summary>
	/// Constructs a Delaunay triangulation of a simple polygon.
	/// </summary>
	/// <param name="vertices">The vertices of the polygon, ordered either clockwise or counter-clockwise.</param>
	/// <returns>The Delaunay triangulation of the polygon.</returns>
	public static SimplePolygonMesh<T, Vertex> Construct(IEnumerable<Vertex> vertices) => Construct(vertices, T.Epsilon * T.CreateChecked(4));

	/// <summary>
	/// Constructs a Delaunay triangulation of a simple polygon.
	/// </summary>
	/// <param name="vertices">The vertices of the polygon, ordered either clockwise or counter-clockwise.</param>
	/// <param name="numericTolerance">The largest number which is still considered 0.</param>
	/// <returns>The Delaunay triangulation of the polygon.</returns>
	public static SimplePolygonMesh<T, Vertex> Construct(IEnumerable<Vertex> vertices, T numericTolerance)
	{
		List<Vertex> vertexList = vertices as List<Vertex> ?? new List<Vertex>(vertices);
		if (vertexList.Count < 3)
		{
			// TODO
		}

		List<Edge<T, Vertex>> outerEdgeSequence = new List<Edge<T, Vertex>>();
		for (int k = 0; k < vertexList.Count; ++k)
		{
			Edge<T, Vertex> newEdge = new Edge<T, Vertex>(vertexList[k], vertexList[(k + 1) % vertexList.Count]);
			outerEdgeSequence.Add(newEdge);
		}
	}

	#endregion Initial construction
}
