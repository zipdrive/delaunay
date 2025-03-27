using System;
using System.Collections.Generic;
using System.ComponentModel.Design;
using System.Numerics;

namespace DelaunayTriangulation;

/// <summary>
/// A Delaunay triangulation where the boundary is the convex hull of the vertices.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
/// <typeparam name="Vertex">The data type for each vertex.</typeparam>
public class ConvexHullMesh<T, Vertex> : Mesh<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
	#region Properties

	/// <summary>
	/// The edges of the convex hull.
	/// </summary>
	private List<Edge<T, Vertex>> _ConvexHull = new List<Edge<T, Vertex>>();

	public override IEnumerable<Edge<T, Vertex>> Boundary => _ConvexHull;

    #endregion Properties

    #region Initial construction

    /// <summary>
    /// Uses the divide-and-conquer approach to divide the vertices into increasingly-small subsets, then merging the resulting triangulations into the final mesh.
    /// </summary>
    /// <param name="vertices">The vertices of the mesh.</param>
    /// <returns>A Delaunay triangulation of the vertices.</returns>
    public static ConvexHullMesh<T, Vertex> Construct(IEnumerable<Vertex> vertices) => Construct(vertices, T.Epsilon * T.CreateChecked(4));

	/// <summary>
	/// Uses the divide-and-conquer approach to divide the vertices into increasingly-small subsets, then merging the resulting triangulations into the final mesh.
	/// </summary>
	/// <param name="vertices">The vertices of the mesh.</param>
	/// <param name="numericTolerance">The largest number which is still considered 0.</param>
	/// <returns>A Delaunay triangulation of the vertices.</returns>
	public static ConvexHullMesh<T, Vertex> Construct(IEnumerable<Vertex> vertices, T numericTolerance) => _ConstructHelperX(vertices.ToArray(), numericTolerance);

	/// <summary>
	/// Recursive helper function to subdivide the graph into two regions, then merge the resulting triangulations.
	/// </summary>
	/// <param name="vertices">The vertices of the graph.</param>
	/// <returns>The Delaunay triangulation of those vertices.</returns>
	private static ConvexHullMesh<T, Vertex> _ConstructHelperX(Vertex[] vertices, T numericTolerance)
	{
		if (vertices.Length > 3)
		{
			Array.Sort(vertices, (lhs, rhs) => lhs.X.CompareTo(rhs.X));
			ConvexHullMesh<T, Vertex> mesh1 = _ConstructHelperY(vertices[..(vertices.Length / 2)], numericTolerance);
			ConvexHullMesh<T, Vertex> mesh2 = _ConstructHelperY(vertices[(vertices.Length / 2)..], numericTolerance);
			mesh1._Merge(mesh2);
			return mesh1;
		}
		else
		{
			return new ConvexHullMesh<T, Vertex>(vertices, numericTolerance);
		}
	}

	/// <summary>
	/// Recursive helper function to subdivide the graph into two regions, then merge the resulting triangulations.
	/// </summary>
	/// <param name="vertices">The vertices of the graph.</param>
	/// <returns>The Delaunay triangulation of those vertices.</returns>
	private static ConvexHullMesh<T, Vertex> _ConstructHelperY(Vertex[] vertices, T numericTolerance)
	{
		if (vertices.Length > 3)
		{
			Array.Sort(vertices, (lhs, rhs) => lhs.Y.CompareTo(rhs.Y));
			ConvexHullMesh<T, Vertex> mesh1 = _ConstructHelperY(vertices[..(vertices.Length / 2)], numericTolerance);
			ConvexHullMesh<T, Vertex> mesh2 = _ConstructHelperY(vertices[(vertices.Length / 2)..], numericTolerance);
			mesh1._Merge(mesh2);
			return mesh1;
		}
		else
		{
			return new ConvexHullMesh<T, Vertex>(vertices, numericTolerance);
		}
	}

	/// <summary>
	/// Constructs a mesh with a single vertex.
	/// </summary>
	/// <param name="vertex">The only vertex in the mesh.</param>
	internal ConvexHullMesh(Vertex vertex, T numericTolerance) : base(numericTolerance)
	{
		_Vertices = new Dictionary<Vertex, List<Edge<T, Vertex>>> { [vertex] = new List<Edge<T, Vertex>>() };
		_Edges = new HashSet<Edge<T, Vertex>>();
		_Triangles = new HashSet<Triangle<T, Vertex>>();
	}

	/// <summary>
	/// Constructs a Delaunay triangulation of the given vertices.
	/// </summary>
	/// <param name="vertices">The vertices of a graph.</param>
	/// <exception cref="ArgumentException">Thrown if more than 3 vertices are passed in to the constructor.</exception>
	internal ConvexHullMesh(IEnumerable<Vertex> vertices, T numericTolerance) : base(numericTolerance)
	{
		if (vertices.Count() > 3)
			throw new ArgumentException("More than 3 vertices passed to Mesh constructor.");

		// Initial vertices
		_Vertices = new Dictionary<Vertex, List<Edge<T, Vertex>>>();
		
		// Initial edges
		foreach (Vertex vertex1 in vertices)
		{
			foreach (Vertex vertex2 in vertices.TakeWhile(v => !v.Equals(vertex1)))
			{
				_AddEdge(new Edge<T, Vertex>(vertex1, vertex2));
				
				// Initial triangles
				foreach (Vertex vertex3 in vertices.TakeWhile(v => !v.Equals(vertex2)))
				{
					Triangle<T, Vertex> triangle = new Triangle<T, Vertex>(this, vertex1, vertex2, vertex3);
					_Triangles.Add(triangle);
					
					// Initial convex hull
					_ConvexHull.AddRange(triangle.Edges);
				}
			}
		}

	}

	#endregion Initial construction

	#region Convex hull

	/// <summary>
	/// Verifies the integrity of the convex hull. Throws an exception if there is a problem.
	/// </summary>
	private void _VerifyConvexHull()
	{
		if (_ConvexHull.Count == 0)
		{
			if (_Vertices.Count >= 3)
				throw new Exception("No convex hull despite having at least three vertices.");
			return;
		}

		for (int k = 0; k < _ConvexHull.Count; ++k)
		{
			Edge<T, Vertex> edge = _ConvexHull[k];
			
			Edge<T, Vertex> nextEdge = _ConvexHull[(k + 1) % _ConvexHull.Count];
			if (!edge.Vertex2.Equals(nextEdge.Vertex1))
				throw new Exception("Discontinuity in the convex hull."); // Discontinuity

			if (_Vertices.Keys.Any(vertex => edge.GetRighthandOffset(vertex) > NumericTolerance))
				throw new Exception("Not all vertices in the graph are on the lefthand side of the convex hull edges."); // Not all vertices in graph are on lefthand side
		}
	}

	/// <summary>
	/// Checks if a vertex is inside the convex hull of the mesh.
	/// </summary>
	/// <param name="vertex">A vertex to test.</param>
	/// <returns>+1 if the vertex is inside the convex hull.
	/// 0 if the vertex is on the convex hull.
	/// -1 if the vertex is outside the convex hull.</returns>
	public int IsInsideConvexHull(Vertex vertex)
	{
		foreach (Edge<T, Vertex> edge in _ConvexHull)
		{
			T offset = edge.GetRighthandOffset(vertex);
			if (offset > NumericTolerance)
				return -1;
			else if (offset >= -NumericTolerance)
			{
				Vector2<T> edgeVector = edge.Vector;
				T edgeVectorDot = edgeVector.Dot(Vector2<T>.VectorDifference(vertex, edge.Vertex1));
				if (edgeVectorDot >= -NumericTolerance && edgeVectorDot <= edgeVector.LengthSquared + NumericTolerance)
					return 0;
			}
		}
		return 1;
	}

	#endregion Convex hull

	#region Insertion

	/// <summary>
	/// Inserts a vertex which is known to exist strictly inside the convex hull.
	/// </summary>
	/// <param name="vertex">A vertex which is known to exist strictly inside the convex hull.</param>
	private void _AddInsideConvexHull(Vertex vertex)
	{
		#region Construct influence triangulation
		HashSet<Triangle<T, Vertex>> influenceTriangulation = new HashSet<Triangle<T, Vertex>>();
		
		// Iterate over all triangles to see which triangles contain the vertex in circumcenter
		foreach (Triangle<T, Vertex> triangle in _Triangles)
		{
			if (triangle.IsInsideCircumcircle(vertex))
			{
				HashSet<Triangle<T, Vertex>> suspects = new HashSet<Triangle<T, Vertex>> { triangle };
				HashSet<Triangle<T, Vertex>> clearedSuspects = new HashSet<Triangle<T, Vertex>>();
				while (suspects.Count > 0)
				{
					Triangle<T, Vertex> suspectedTriangle = suspects.First();
					suspects.Remove(suspectedTriangle);
					if (influenceTriangulation.Contains(suspectedTriangle) || clearedSuspects.Contains(suspectedTriangle))
						continue;

					if (suspectedTriangle.IsInsideCircumcircle(vertex))
					{
						influenceTriangulation.Add(suspectedTriangle);
						suspects.UnionWith(suspectedTriangle.AdjacentTriangles);
					}
					else
					{
						clearedSuspects.Add(suspectedTriangle);
					}
				}
				break;
			}
		}
		#endregion Construct influence triangulation

		// Remove all edges shared by TWO of the removed triangles (i.e. do not remove the edges on the border of the influence triangulation)
		_RemoveEdges(influenceTriangulation.SelectMany(t => t.Edges).Where(e => influenceTriangulation.Count(t => t.Edges.Contains(e)) > 1));

		// Add the new vertex, with triangles connecting it to each edge forming the convex hull of the influence triangulation
		List<Edge<T, Vertex>> influenceTriangulationConvexHull = new List<Edge<T, Vertex>>(_Edges.Where(e => influenceTriangulation.Any(t => t.Edges.Contains(e))));
		foreach (Edge<T, Vertex> influenceTriangulationConvexHullEdge in influenceTriangulationConvexHull)
		{
			_AddTriangle(new Triangle<T, Vertex>(this, influenceTriangulationConvexHullEdge, vertex));
		}
	}

	/// <summary>
	/// Inserts a vertex which is known to exist on the boundary of the convex hull.
	/// </summary>
	/// <param name="vertex">A vertex which is known to exist on the boundary of the convex hull.</param>
	private void _AddOnBoundaryOfConvexHull(Vertex vertex)
	{
		_AddInsideConvexHull(vertex);

		// Locate the convex hull edge(s) that should be removed
		int convexHullRemovedEdgeIndex = _ConvexHull.FindIndex(edge =>
		{
            // Test if vertex is on the line defined by the edge
            T righthandOffset = edge.GetRighthandOffset(vertex);
            if (righthandOffset >= -NumericTolerance && righthandOffset <= NumericTolerance)
            {
                // Test if vertex is contained in the line segment defined by the edge
                Vector2<T> edgeVector = edge.Vector;
                T dot = edgeVector.Dot(Vector2<T>.VectorDifference(edge.Vertex1, vertex));
                if (dot >= -NumericTolerance && dot <= edgeVector.LengthSquared)
                {
					return true;
                }
            }
			return false;
        });
        if (convexHullRemovedEdgeIndex < 0)
            throw new Exception("Expected a convex hull edge to be removed.");

        Edge<T, Vertex> convexHullRemovedEdge = _ConvexHull[convexHullRemovedEdgeIndex];
		_RemoveEdge(convexHullRemovedEdge);
		_ConvexHull.RemoveAt(convexHullRemovedEdgeIndex);

		// Insert the first edge
		Edge<T, Vertex> edge1 = FindOrCreateEdge(convexHullRemovedEdge.Vertex1, vertex);
		if (!edge1.Vertex2.Equals(vertex))
			edge1.Flip();
		_ConvexHull.Insert(convexHullRemovedEdgeIndex, edge1);

		// Insert the second edge
		Edge<T, Vertex> edge2 = FindOrCreateEdge(vertex, convexHullRemovedEdge.Vertex2);
		if (!edge2.Vertex1.Equals(vertex))
			edge2.Flip();
		_ConvexHull.Insert(convexHullRemovedEdgeIndex + 1, edge2);
	}

	/// <summary>
	/// Search for the edge on the convex hull of the combination of this mesh and another mesh,
	/// where Vertex1 is a vertex of this mesh and Vertex2 is a vertex of the other mesh.
	/// </summary>
	/// <param name="mesh">The other mesh.</param>
	/// <param name="edge">The edge to inspect.</param>
	/// <returns>An edge on the convex hull that bridges this mesh with the other mesh. Null if no such edge was found.</returns>
	private Edge<T, Vertex>? _FindBaseLREdgeHelper(ConvexHullMesh<T, Vertex> mesh, Edge<T, Vertex> edge, HashSet<Vertex> remainingVertices)
	{
        // Find the vertex with the largest righthand offset from this edge
        (Vertex, T)? bestCandidate = null;
        foreach (Vertex vertex in remainingVertices)
        {
            T offset = edge.GetRighthandOffset(vertex);
            if (offset > NumericTolerance)
            {
                if (bestCandidate == null || bestCandidate.Value.Item2 < offset)
                {
                    bestCandidate = (vertex, offset);
                }
            }
            else if (offset <= NumericTolerance && offset >= -NumericTolerance)
            {
                Vector2<T> edgeVector = edge.Vector;
                T edgeVectorDot = edgeVector.Dot(Vector2<T>.VectorDifference(edge.Vertex1, vertex));
                if (edgeVectorDot >= -NumericTolerance && edgeVectorDot < edgeVector.LengthSquared)
                {
                    if (bestCandidate == null || bestCandidate.Value.Item2 < offset)
                    {
                        bestCandidate = (vertex, offset);
                    }
                }
            }
        }

        if (bestCandidate != null)
        {
			Vertex bestVertex = bestCandidate.Value.Item1;
			remainingVertices.Remove(bestVertex);

            // Add two edges from each endpoint to the new vertex
            Edge <T, Vertex> edge1 = new Edge<T, Vertex>(edge.Vertex1, bestVertex);
            Edge<T, Vertex> edge2 = new Edge<T, Vertex>(bestVertex, edge.Vertex2);
            return _FindBaseLREdgeHelper(mesh, edge1, remainingVertices) ?? _FindBaseLREdgeHelper(mesh, edge2, remainingVertices);
        }
		else 
		{
			return _Vertices.ContainsKey(edge.Vertex1) && mesh._Vertices.ContainsKey(edge.Vertex2) ? edge : null;
		}
    }

    /// <summary>
    /// Search for the edge on the convex hull of the combination of this mesh and another mesh,
    /// where Vertex1 is a vertex of this mesh and Vertex2 is a vertex of the other mesh.
    /// </summary>
    /// <param name="mesh">The other mesh.</param>
    /// <returns>An edge on the convex hull that bridges this mesh with the other mesh. Null if no such edge was found.</returns>
    private Edge<T, Vertex> _FindBaseLREdge(ConvexHullMesh<T, Vertex> mesh)
	{
        // Construct set of initial vertices
        (Vertex, T) minA = (_Vertices.Keys.First(), _Vertices.Keys.First().X + _Vertices.Keys.First().Y);
        (Vertex, T) maxA = (_Vertices.Keys.First(), _Vertices.Keys.First().X + _Vertices.Keys.First().Y);
        (Vertex, T) minB = (_Vertices.Keys.First(), _Vertices.Keys.First().X - _Vertices.Keys.First().Y);
        (Vertex, T) maxB = (_Vertices.Keys.First(), _Vertices.Keys.First().X - _Vertices.Keys.First().Y);
        foreach (Vertex vertex in _Vertices.Keys.Skip(1).Concat(mesh._Vertices.Keys))
        {
            T a = vertex.X + vertex.Y;
            T b = vertex.X - vertex.Y;
            if (a < minA.Item2)
                minA = (vertex, a);
            if (a > maxA.Item2)
                maxA = (vertex, a);
            if (b < minB.Item2)
                minB = (vertex, b);
            if (b > maxB.Item2)
                maxB = (vertex, b);
        }
        List<Vertex> initialVertices = new List<Vertex>(new Vertex[] { maxA.Item1, minB.Item1, minA.Item1, maxB.Item1 }.Distinct());
        
        // Recursion to fill out convex hull
        if (initialVertices.Count > 1)
        {
			HashSet<Vertex> remainingVertices = new HashSet<Vertex>(_Vertices.Keys.Concat(mesh._Vertices.Keys));
			remainingVertices.ExceptWith(initialVertices);

            for (int k = 0; k < initialVertices.Count; ++k)
            {
                Vertex vertex1 = initialVertices[k];
                Vertex vertex2 = initialVertices[(k + 1) % initialVertices.Count];

                Edge<T, Vertex> edge = new Edge<T, Vertex>(vertex1, vertex2);
                Edge<T, Vertex>? result = _FindBaseLREdgeHelper(mesh, edge, remainingVertices);
				if (result != null)
					return result;
            }
        }

		_VerifyConvexHull();
		throw new Exception("Could not find a base LR edge to merge two meshes.");
    }

	/// <summary>
	/// Combines this mesh in-place with a mesh that has no overlap of the convex hull.
	/// </summary>
	/// <param name="mesh">The mesh to combine with.</param>
	private void _Merge(ConvexHullMesh<T, Vertex> mesh)
	{
        // Get the LL edges
        HashSet<Edge<T, Vertex>> edgesLL = new HashSet<Edge<T, Vertex>>(_Edges);

		// Create the first connecting edge between the two meshes
		Edge<T, Vertex> convexHullLREdge = _FindBaseLREdge(mesh);
		Edge<T, Vertex> connectingEdge = convexHullLREdge;
		Vertex l = connectingEdge.Vertex1;
		Vertex r = connectingEdge.Vertex2;

		// Stores the triangles that bridge the two meshes
		List<Triangle<T, Vertex>> trianglesLR = new List<Triangle<T, Vertex>>();

        // Then, iterate
        while (true)
		{
			#region Left candidate vertex
			SortedList<T, Vertex> potentialCandidateVerticesL = new SortedList<T, Vertex>();
			foreach (Edge<T, Vertex> edgeL in FindAllEdges(l))
			{
				T edgeAngleL = connectingEdge.GetAngularDifference(edgeL);
				potentialCandidateVerticesL.Add(edgeAngleL, edgeL.Vertex1.Equals(l) ? edgeL.Vertex2 : edgeL.Vertex1);
			}

			(Vertex, Triangle<T, Vertex>)? candidateL = null;
			while (potentialCandidateVerticesL.Count > 0)
			{
				if (potentialCandidateVerticesL.Keys[0] >= T.Pi - NumericTolerance)
				{
					break; // Angle of next potential candidate is greater than or equal to 180 degrees, so no candidate is chosen for left side
				}

				Vertex nextCandidateVertexL = potentialCandidateVerticesL.Values[0];
				potentialCandidateVerticesL.RemoveAt(0);

				Triangle<T, Vertex> triangle = new Triangle<T, Vertex>(this, connectingEdge, nextCandidateVertexL);
				if (potentialCandidateVerticesL.Count > 0)
				{
					if (triangle.IsInsideCircumcircle(potentialCandidateVerticesL.Values[0]))
					{
						Edge<T, Vertex> contradictedEdgeL = _RemoveEdge(l, nextCandidateVertexL);
						edgesLL.Remove(contradictedEdgeL);
					}
					else
					{
						candidateL = (nextCandidateVertexL, triangle);
						break;
					}
				}
				else candidateL = (nextCandidateVertexL, triangle);
			}
			#endregion Left candidate vertex

			#region Right candidate vertex
			SortedList<T, Vertex> potentialCandidateVerticesR = new SortedList<T, Vertex>();
			foreach (Edge<T, Vertex> edgeR in mesh._Edges)
			{
				if (edgeR.Vertex1.Equals(r) || edgeR.Vertex2.Equals(r))
				{
					T edgeAngleR = T.Tau - connectingEdge.GetAngularDifference(edgeR);
                    potentialCandidateVerticesR.Add(edgeAngleR, edgeR.Vertex1.Equals(r) ? edgeR.Vertex2 : edgeR.Vertex1);
				}
			}

			(Vertex, Triangle<T, Vertex>)? candidateR = null;
			while (potentialCandidateVerticesR.Count > 0)
			{
				if (potentialCandidateVerticesR.Keys[0] >= T.Pi - NumericTolerance)
				{
					break; // Angle of next potential candidate is greater than or equal to 180 degrees, so no candidate is chosen for left side
				}

				Vertex nextCandidateVertexR = potentialCandidateVerticesR.Values[0];
				potentialCandidateVerticesR.RemoveAt(0);

				Triangle<T, Vertex> triangle = new Triangle<T, Vertex>(mesh, connectingEdge, nextCandidateVertexR);
				if (potentialCandidateVerticesR.Count > 0)
				{
					if (triangle.IsInsideCircumcircle(potentialCandidateVerticesR.Values[0]))
					{
						mesh._RemoveEdge(r, nextCandidateVertexR);
					}
					else
					{
						candidateR = (nextCandidateVertexR, triangle);
						break;
					}
				}
				else candidateR = (nextCandidateVertexR, triangle);
			}
			#endregion Right candidate vertex

			#region Settle on a final candidate
			(Vertex, Triangle<T, Vertex>) candidate;
			if (candidateL != null && candidateR != null)
			{
				if (candidateL.Value.Item2.IsInsideCircumcircle(candidateR.Value.Item1))
				{
					candidate = candidateR.Value;
					r = candidateR.Value.Item1;
				}
				else //if (candidateR.Value.Item2.IsInsideCircumcircle(candidateL.Value.Item1))
				{
					// If both vertices are on the same circumcircle (within tolerance), we arbitrarily pick the left vertex as our candidate
					candidate = candidateL.Value;
					l = candidateL.Value.Item1;
				}
				//else throw new Exception("The circumcircles of both candidate vertices contain the other candidate.");
			}
			else if (candidateL != null)
			{
				candidate = candidateL.Value;
				l = candidateL.Value.Item1;
			}
			else if (candidateR != null)
			{
				candidate = candidateR.Value;
				r = candidateR.Value.Item1;
			}
			else
				break;

			trianglesLR.Add(candidate.Item2);

			connectingEdge = candidate.Item2.Edges.First(e => (e.Vertex1.Equals(l) && e.Vertex2.Equals(r)) || (e.Vertex1.Equals(r) && e.Vertex2.Equals(l)));
			if (!connectingEdge.Vertex1.Equals(r))
				connectingEdge.Flip();
			#endregion Settle on a final candidate
		}

		Edge<T, Vertex> convexHullRLEdge = connectingEdge;

		// Insert all vertices and all remaining edges/triangles from the other triangulation
		foreach (Triangle<T, Vertex> triangle in trianglesLR.Concat(mesh._Triangles))
			_AddTriangle(triangle);

        // Recalculate convex hull
        Dictionary<Vertex, Edge<T, Vertex>> convexHullLL = new Dictionary<Vertex, Edge<T, Vertex>>(_ConvexHull.Select(e => new KeyValuePair<Vertex, Edge<T, Vertex>>(e.Vertex1, e)));
        Dictionary<Vertex, Edge<T, Vertex>> convexHullRR = new Dictionary<Vertex, Edge<T, Vertex>>(mesh._ConvexHull.Select(e => new KeyValuePair<Vertex, Edge<T, Vertex>>(e.Vertex1, e)));

        _ConvexHull = new List<Edge<T, Vertex>> { convexHullRLEdge };

		Vertex lastConvexHullVertex = convexHullRLEdge.Vertex2;
		while (!lastConvexHullVertex.Equals(convexHullLREdge.Vertex1))
		{
			if (convexHullLL.Count > 0)
			{
				if (convexHullLL.TryGetValue(lastConvexHullVertex, out Edge<T, Vertex>? nextConvexHullEdge) && nextConvexHullEdge != null)
				{
					_ConvexHull.Add(nextConvexHullEdge);
					lastConvexHullVertex = nextConvexHullEdge.Vertex2;
				}
				else throw new Exception("Expected to find edge on the convex hull that does not exist.");
			}
			else
			{
				Edge<T, Vertex>? nextConvexHullEdge = edgesLL.FirstOrDefault(e => e.Vertex1.Equals(lastConvexHullVertex) || e.Vertex2.Equals(lastConvexHullVertex));
				if (nextConvexHullEdge == null)
					throw new Exception("Expected to find edge that does not exist.");
				else
				{
					if (nextConvexHullEdge.Vertex2.Equals(lastConvexHullVertex))
						nextConvexHullEdge.Flip();
					_ConvexHull.Add(nextConvexHullEdge);
					lastConvexHullVertex = nextConvexHullEdge.Vertex2;
				}
			}
		}

		_ConvexHull.Add(convexHullLREdge);

        lastConvexHullVertex = convexHullLREdge.Vertex2;
        while (!lastConvexHullVertex.Equals(convexHullRLEdge.Vertex1))
        {
			if (convexHullRR.Count > 0)
			{
				if (convexHullRR.TryGetValue(lastConvexHullVertex, out Edge<T, Vertex>? nextConvexHullEdge) && nextConvexHullEdge != null)
				{
					_ConvexHull.Add(nextConvexHullEdge);
					lastConvexHullVertex = nextConvexHullEdge.Vertex2;
				}
				else throw new Exception("Expected to find edge on the convex hull that does not exist.");
			}
			else
			{
                Edge<T, Vertex>? nextConvexHullEdge = mesh._Edges.FirstOrDefault(e => e.Vertex1.Equals(lastConvexHullVertex) || e.Vertex2.Equals(lastConvexHullVertex));
                if (nextConvexHullEdge == null)
                    throw new Exception("Expected to find edge that does not exist.");
                else
                {
                    if (nextConvexHullEdge.Vertex2.Equals(lastConvexHullVertex))
                        nextConvexHullEdge.Flip();
                    _ConvexHull.Add(nextConvexHullEdge);
                    lastConvexHullVertex = nextConvexHullEdge.Vertex2;
                }
            }
        }
	}

	/// <summary>
	/// Adds a single vertex to the mesh, re-updating the triangulation to compensate.
	/// </summary>
	/// <param name="vertex">The vertex to insert.</param>
	public void Add(Vertex vertex)
	{
		int region = IsInsideConvexHull(vertex);
		if (region > 0)
		{
			_AddInsideConvexHull(vertex);
		}
		else if (region < 0)
		{
			// Merge a mesh with a single vertex into this one
			_Merge(new ConvexHullMesh<T, Vertex>(vertex, NumericTolerance));
		}
		else
		{
			_AddOnBoundaryOfConvexHull(vertex);
		}
	}

	/// <summary>
	/// Adds multiple vertices to this mesh, re-updating the triangulation to compensate.
	/// </summary>
	/// <param name="vertices">The vertices to insert.</param>
	public void AddRange(IEnumerable<Vertex> vertices)
	{
		// Insert all of the vertices which are known to exist inside or on the boundary of the convex hull of this mesh
		List<Vertex> remainingVertices = new List<Vertex>(vertices);
        while (remainingVertices.Count > 0)
        {
			// Handle vertices inside the convex hull
            int index = 0;
			int numLeftToInspect = remainingVertices.Count;
			while (numLeftToInspect > 0)
			{
				Vertex vertex = remainingVertices[index];
				--numLeftToInspect;

				int region = IsInsideConvexHull(vertex);
				if (region > 0)
				{
					_AddInsideConvexHull(vertex);
					remainingVertices.RemoveAt(index);
				}
				else if (region == 0)
				{
					_AddOnBoundaryOfConvexHull(vertex);
					remainingVertices.RemoveAt(index);
					numLeftToInspect = remainingVertices.Count;
				}
				else ++index;

				if (index >= remainingVertices.Count)
					index = 0;
			}

			if (remainingVertices.Count > 0)
			{
				// Handle the vertices outside of the convex hull
				bool anyVerticesAddedToMesh = false;
				foreach (Edge<T, Vertex> convexHullEdge in _ConvexHull)
				{
					List<Vertex> partitionedVertices = new List<Vertex>();
					for (int i = remainingVertices.Count - 1; i >= 0; --i)
					{
						if (convexHullEdge.GetRighthandOffset(remainingVertices[i]) > NumericTolerance)
						{
							partitionedVertices.Add(remainingVertices[i]);
							remainingVertices.RemoveAt(i);
						}
					}

					if (partitionedVertices.Count > 0)
					{
						// Construct a new mesh from the remaining vertices, then merge into this mesh
						_Merge(Construct(partitionedVertices));
						anyVerticesAddedToMesh = true;
						break;
					}
				}

				if (!anyVerticesAddedToMesh)
				{
					_VerifyConvexHull();
					throw new Exception("Expected to find a vertex outside of the convex hull.");
				}
			}
		}
	}

	#endregion Insertion

	#region Removal

	/// <summary>
	/// Search for the edge on the convex hull of the combination of this mesh and another mesh,
	/// where Vertex1 is a vertex of this mesh and Vertex2 is a vertex of the other mesh.
	/// </summary>
	/// <param name="mesh">The other mesh.</param>
	/// <param name="edge">The edge to inspect.</param>
	/// <returns>An edge on the convex hull that bridges this mesh with the other mesh. Null if no such edge was found.</returns>
	private (Edge<T, Vertex>, Edge<T, Vertex>)? _FindPolygonOrientationRepresentativeEdgesHelper(Edge<T, Vertex> edge, HashSet<Vertex> remainingVertices, IEnumerable<Edge<T, Vertex>> allEdges)
	{
		// Find the vertex with the largest righthand offset from this edge
		(Vertex, T)? bestCandidate = null;
		foreach (Vertex vertex in remainingVertices)
		{
			T offset = edge.GetRighthandOffset(vertex);
			if (offset > NumericTolerance)
			{
				if (bestCandidate == null || bestCandidate.Value.Item2 < offset)
				{
					bestCandidate = (vertex, offset);
				}
			}
			else if (offset <= NumericTolerance && offset >= -NumericTolerance)
			{
				Vector2<T> edgeVector = edge.Vector;
				T edgeVectorDot = edgeVector.Dot(Vector2<T>.VectorDifference(edge.Vertex1, vertex));
				if (edgeVectorDot >= -NumericTolerance && edgeVectorDot < edgeVector.LengthSquared)
				{
					if (bestCandidate == null || bestCandidate.Value.Item2 < offset)
					{
						bestCandidate = (vertex, offset);
					}
				}
			}
		}

		if (bestCandidate != null)
		{
			Vertex bestVertex = bestCandidate.Value.Item1;
			remainingVertices.Remove(bestVertex);

			// Add two edges from each endpoint to the new vertex
			Edge<T, Vertex> edge1 = new Edge<T, Vertex>(edge.Vertex1, bestVertex);
			Edge<T, Vertex> edge2 = new Edge<T, Vertex>(bestVertex, edge.Vertex2);
			return _FindPolygonOrientationRepresentativeEdgesHelper(edge1, remainingVertices, allEdges) ?? 
				_FindPolygonOrientationRepresentativeEdgesHelper(edge2, remainingVertices, allEdges);
		}
		else
		{
			Vertex convexHullVertex = edge.Vertex1;
			IEnumerable<Edge<T, Vertex>> edgesAdjacentToConvexHullVertex = allEdges.Where(e => e.Vertices.Contains(convexHullVertex));
			if (edgesAdjacentToConvexHullVertex.Count() > 1)
			{
				Edge<T, Vertex> edge1 = edgesAdjacentToConvexHullVertex.First();
				Edge<T, Vertex> edge2 = edgesAdjacentToConvexHullVertex.Last();
				Triangle<T, Vertex> triangle = new Triangle<T, Vertex>(this, edge1, edge2);
				T orientation = triangle.Orientation;
				if (orientation > NumericTolerance)
				{
					if (edge1.Vertex1.Equals(convexHullVertex))
						edge1.Flip();
					if (edge2.Vertex2.Equals(convexHullVertex))
						edge2.Flip();
					return (edge1, edge2);
				}
				else if (orientation < -NumericTolerance)
				{
					if (edge2.Vertex1.Equals(convexHullVertex))
						edge2.Flip();
					if (edge1.Vertex2.Equals(convexHullVertex))
						edge1.Flip();
					return (edge2, edge1);
				}
			}
			return null;
		}
	}

	/// <summary>
	/// .
	/// </summary>
	/// <param name="mesh">The other mesh.</param>
	/// <returns>An edge on the convex hull that bridges this mesh with the other mesh. Null if no such edge was found.</returns>
	private (Edge<T, Vertex>, Edge<T, Vertex>) _FindPolygonOrientationRepresentativeEdges(IEnumerable<Edge<T, Vertex>> edges)
	{
		// Construct set of initial vertices
		IEnumerable<Vertex> vertices = edges.SelectMany(e => e.Vertices).Distinct();
		Vertex firstVertex = vertices.First();
		(Vertex, T) minA = (firstVertex, firstVertex.X + firstVertex.Y);
		(Vertex, T) maxA = (firstVertex, firstVertex.X + firstVertex.Y);
		(Vertex, T) minB = (firstVertex, firstVertex.X - firstVertex.Y);
		(Vertex, T) maxB = (firstVertex, firstVertex.X - firstVertex.Y);
		foreach (Vertex vertex in vertices.Skip(1).Concat(vertices))
		{
			T a = vertex.X + vertex.Y;
			T b = vertex.X - vertex.Y;
			if (a < minA.Item2)
				minA = (vertex, a);
			if (a > maxA.Item2)
				maxA = (vertex, a);
			if (b < minB.Item2)
				minB = (vertex, b);
			if (b > maxB.Item2)
				maxB = (vertex, b);
		}
		List<Vertex> initialVertices = new List<Vertex>(new Vertex[] { maxA.Item1, minB.Item1, minA.Item1, maxB.Item1 }.Distinct());

		// Recursion to fill out convex hull
		if (initialVertices.Count > 1)
		{
			HashSet<Vertex> remainingVertices = new HashSet<Vertex>(vertices);
			remainingVertices.ExceptWith(initialVertices);

			for (int k = 0; k < initialVertices.Count; ++k)
			{
				Vertex vertex1 = initialVertices[k];
				Vertex vertex2 = initialVertices[(k + 1) % initialVertices.Count];

				Edge<T, Vertex> edge = new Edge<T, Vertex>(vertex1, vertex2);
				(Edge<T, Vertex>, Edge<T, Vertex>)? result = _FindPolygonOrientationRepresentativeEdgesHelper(edge, remainingVertices, edges);
				if (result != null)
					return result.Value;
			}
		}

		// All vertices are co-linear
		// Find two arbitrarily joined edges, flip them so that edge1.Vertex2 == edge2.Vertex1
		Edge<T, Vertex> edge1 = edges.First();
		Edge<T, Vertex> edge2 = edges.First(e => !e.Equals(edge1) && e.Vertices.Intersect(edge1.Vertices).Any());
		if (edge1.Vertex1.Equals(edge2.Vertex1))
		{
			edge1.Flip();
		}
		else if (edge1.Vertex1.Equals(edge2.Vertex2))
		{
			edge1.Flip();
			edge2.Flip();
		}
		else if (edge1.Vertex2.Equals(edge2.Vertex2))
		{
			edge2.Flip();
		}
		return (edge1, edge2);
	}

	private void _RemoveRetriangulateSubpolygonHelper(Vertex removedVertex, List<Edge<T, Vertex>> outerEdges)
	{
        // Using algorithm https://doi.org/10.1145/304893.304969
        // Possible input from https://doi.org/10.1016/j.comgeo.2010.10.001 ??

        // Find possible ears
        PriorityQueue<Triangle<T, Vertex>, T> possibleEars = new PriorityQueue<Triangle<T, Vertex>, T>();
        for (int k = outerEdges.Count - 1; k >= 0; --k)
        {
            Triangle<T, Vertex> triangle = new Triangle<T, Vertex>(this, outerEdges[k], outerEdges[(k + 1) % outerEdges.Count]);
            T orientation = triangle.Orientation;
			if (orientation > NumericTolerance)
				possibleEars.Enqueue(triangle, -triangle.Power(removedVertex) / orientation);
        }

        while (possibleEars.Count > 0)
        {
            Triangle<T, Vertex> nextEar = possibleEars.Dequeue();
            _AddTriangle(nextEar);

            // Reconstruct the polygon
            int index1 = outerEdges.IndexOf(nextEar.Edge1);
            int index2 = outerEdges.IndexOf(nextEar.Edge2);
            int index3 = outerEdges.IndexOf(nextEar.Edge3);
            if (index3 < 0)
            {
                outerEdges.RemoveAt(index1);
                outerEdges.Insert(index1, nextEar.Edge3);
                outerEdges.RemoveAt(index2);
            }
            else if (index1 < 0)
            {
                outerEdges.RemoveAt(index2);
                outerEdges.Insert(index2, nextEar.Edge1);
                outerEdges.RemoveAt(index3);
            }
            else if (index2 < 0)
            {
                outerEdges.RemoveAt(index3);
                outerEdges.Insert(index3, nextEar.Edge2);
                outerEdges.RemoveAt(index1);
            }
            else
            {
                break;
            }

            // Reconstruct the priority queue
            possibleEars = new PriorityQueue<Triangle<T, Vertex>, T>();
            for (int k = outerEdges.Count - 1; k >= 0; --k)
            {
                Triangle<T, Vertex> triangle = new Triangle<T, Vertex>(this, outerEdges[k], outerEdges[(k + 1) % outerEdges.Count]);
                T orientation = triangle.Orientation;
                if (orientation > NumericTolerance)
                    possibleEars.Enqueue(triangle, -triangle.Power(removedVertex) / orientation);
            }
        }
    }

	/// <summary>
	/// Removes a single vertex from the mesh, re-calculating the triangulation to compensate.
	/// </summary>
	/// <param name="vertex">The vertex to remove.</param>
	public void Remove(Vertex vertex)
	{
		if (_Vertices.Count <= 1)
		{
			_Vertices.Remove(vertex);
			return;
		}

        // Remove the vertices in the clique, record the edges and triangles attached to those vertices
        HashSet<Edge<T, Vertex>> removedEdges = new HashSet<Edge<T, Vertex>>();
        HashSet<Triangle<T, Vertex>> removedTriangles = new HashSet<Triangle<T, Vertex>>();
        if (_Vertices.Remove(vertex, out List<Edge<T, Vertex>>? connectedEdges) && connectedEdges != null)
        {
            removedEdges.UnionWith(connectedEdges);
            foreach (Edge<T, Vertex> removedEdge in connectedEdges)
            {
                if (removedEdge.Left != null)
                    removedTriangles.Add(removedEdge.Left);
                if (removedEdge.Right != null)
                    removedTriangles.Add(removedEdge.Right);
            }
        }
        foreach (Edge<T, Vertex> removedEdge in removedEdges)
            _RemoveEdge(removedEdge);

        // Construct a polygon from the surrounding outer edges
        HashSet<Edge<T, Vertex>> unorderedOuterEdges = new HashSet<Edge<T, Vertex>>(removedTriangles.SelectMany(t => t.Edges.Where(e => !removedEdges.Contains(e))));
        int unorderedOuterEdgesCount = unorderedOuterEdges.Count();
        if (unorderedOuterEdgesCount == 0 || (unorderedOuterEdgesCount == 1 && _Edges.Count == 1))
        {
            // Not enough edges remain in the mesh for a convex hull
            _ConvexHull = new List<Edge<T, Vertex>>();
        }
        else if (unorderedOuterEdgesCount == 1)
        {
            // The clique used to be on the convex hull, and the only outer edge bridges the gap in the convex hull
            // Remove former edges from the convex hull, insert the new edge that should be on the convex hull
            Edge<T, Vertex> newConvexHullEdge = unorderedOuterEdges.First();
            int startIndex = _ConvexHull.FindIndex(removedEdges.Contains);
            int endIndex = _ConvexHull.FindLastIndex(removedEdges.Contains);
            if (vertex.Equals(_ConvexHull[startIndex].Vertex2))
            {
                if (newConvexHullEdge.Vertex1.Equals(_ConvexHull[startIndex].Vertex1))
                    newConvexHullEdge.Flip();
                _ConvexHull.RemoveRange(startIndex, endIndex - startIndex + 1);
                _ConvexHull.Insert(startIndex, newConvexHullEdge);
            }
            else
            {
                if (newConvexHullEdge.Vertex1.Equals(_ConvexHull[endIndex].Vertex1))
                    newConvexHullEdge.Flip();
                _ConvexHull.RemoveRange(endIndex, _ConvexHull.Count - endIndex);
                _ConvexHull.RemoveRange(0, startIndex + 1);
                _ConvexHull.Add(newConvexHullEdge);
            }
        }
        else
        {
			#region Construct surrounding polygon

			List<Edge<T, Vertex>> outerEdges; // Edges should be arranged counter-clockwise
			(Edge<T, Vertex>, Edge<T, Vertex>) startingEdges = _FindPolygonOrientationRepresentativeEdges(unorderedOuterEdges);

            // The two starting edges are arranged counter-clockwise
            outerEdges = new List<Edge<T, Vertex>> { startingEdges.Item1, startingEdges.Item2 };
			unorderedOuterEdges.ExceptWith(outerEdges);
			while (!outerEdges[outerEdges.Count - 1].Vertex2.Equals(outerEdges[0].Vertex1))
			{
				Edge<T, Vertex>? nextEdge = unorderedOuterEdges.FirstOrDefault(e => e.Vertices.Contains(outerEdges[outerEdges.Count - 1].Vertex2));
				if (nextEdge == null)
					break;
				unorderedOuterEdges.Remove(nextEdge);
				if (nextEdge.Vertex2.Equals(outerEdges[outerEdges.Count - 1].Vertex2))
					nextEdge.Flip();
				outerEdges.Add(nextEdge);
			}
			while (!outerEdges[outerEdges.Count - 1].Vertex2.Equals(outerEdges[0].Vertex1))
			{
				Edge<T, Vertex>? precedingEdge = unorderedOuterEdges.FirstOrDefault(e => e.Vertices.Contains(outerEdges[0].Vertex1));
				if (precedingEdge == null)
					break;
				unorderedOuterEdges.Remove(precedingEdge);
				if (precedingEdge.Vertex1.Equals(outerEdges[0].Vertex1))
					precedingEdge.Flip();
				outerEdges.Insert(0, precedingEdge);
			}

            (Vertex, Vertex)? convexHullGap = outerEdges[0].Vertex1.Equals(outerEdges[outerEdges.Count - 1].Vertex2) ? null : (outerEdges[outerEdges.Count - 1].Vertex2, outerEdges[0].Vertex1);
			#endregion Construct surrounding polygon

			if (convexHullGap != null)
			{
                // Removed vertex was on the convex hull

                #region Construct the new convex hull

                int startIndex = _ConvexHull.FindIndex(e => !_Edges.Contains(e) && e.Vertex2.Equals(vertex));
				int endIndex = _ConvexHull.FindIndex(e => !_Edges.Contains(e) && e.Vertex1.Equals(vertex));
				List<Edge<T, Vertex>> convexHullBridgingEdges = new List<Edge<T, Vertex>> { FindOrCreateEdge(_ConvexHull[startIndex].Vertex1, _ConvexHull[endIndex].Vertex2) };

				HashSet<Vertex> verticesNotOnConvexHull = new HashSet<Vertex>(outerEdges.Skip(1).Select(e => e.Vertex1));
				int index = 0;
				while (index < convexHullBridgingEdges.Count)
				{
					Vertex? vertexOnConvexHull = default(Vertex);
					foreach (Vertex vertexNotOnConvexHull in verticesNotOnConvexHull)
					{
						if (convexHullBridgingEdges[index].GetRighthandOffset(vertexNotOnConvexHull) > NumericTolerance)
						{
							vertexOnConvexHull = vertexNotOnConvexHull;
							break;
						}
					}

					if (!EqualityComparer<Vertex>.Default.Equals(vertexOnConvexHull, default(Vertex)))
					{
						verticesNotOnConvexHull.Remove(vertexOnConvexHull);
						convexHullBridgingEdges.Insert(index + 1, FindOrCreateEdge(convexHullBridgingEdges[index].Vertex1, vertexOnConvexHull));
						convexHullBridgingEdges.Insert(index + 2, FindOrCreateEdge(vertexOnConvexHull, convexHullBridgingEdges[index].Vertex2));
						convexHullBridgingEdges.RemoveAt(index);
					}
					else ++index;
				}

				if (startIndex < endIndex)
				{
					_ConvexHull.RemoveAt(endIndex);
					_ConvexHull.RemoveAt(startIndex);
                    _ConvexHull.InsertRange(startIndex, convexHullBridgingEdges);
                }
				else
				{
					_ConvexHull.RemoveAt(startIndex);
					_ConvexHull.RemoveAt(endIndex);
					_ConvexHull.InsertRange(endIndex, convexHullBridgingEdges);
				}

				#endregion Construct the new convex hull

				#region Retriangulate each sub-polygon

				List<Edge<T, Vertex>> subpolygon = new List<Edge<T, Vertex>>();
				foreach (Edge<T, Vertex> insideBoundaryEdge in outerEdges)
				{
					subpolygon.Add(insideBoundaryEdge);
					Vertex[] insideBoundaryEndpoints = new Vertex[] { subpolygon[0].Vertex1, insideBoundaryEdge.Vertex2 };

					for (int k = 0; k < convexHullBridgingEdges.Count; ++k)
					{
						Edge<T, Vertex> convexHullEdge = convexHullBridgingEdges[k];
						Vertex[] convexHullBoundaryEndpoints = new Vertex[] { convexHullBridgingEdges[0].Vertex1, convexHullEdge.Vertex2 };

                        if (insideBoundaryEndpoints.Intersect(convexHullBoundaryEndpoints).Count() == 2)
						{
							if (insideBoundaryEndpoints[0].Equals(convexHullBoundaryEndpoints[0]))
							{
								foreach (Edge<T, Vertex> subpolygonEdge in subpolygon)
									subpolygonEdge.Flip();
							}

							subpolygon.AddRange(convexHullBridgingEdges[0..(k+1)]);
							convexHullBridgingEdges.RemoveRange(k, convexHullBridgingEdges.Count - k);
							if (subpolygon.Count >= 3) // Skip triangulating "polygons" with 2 edges
								_RemoveRetriangulateSubpolygonHelper(vertex, subpolygon);
							subpolygon = new List<Edge<T, Vertex>>();
						}
					}
				}

                #endregion Retriangulate each sub-polygon
            }
            else
			{
				// Removed vertex was inside the convex hull
				_RemoveRetriangulateSubpolygonHelper(vertex, outerEdges);
			}
        }
    }

	/// <summary>
	/// Removes vertices from the mesh, re-calculating the triangulation to compensate.
	/// </summary>
	/// <param name="vertices">The vertices to remove.</param>
	public void RemoveRange(IEnumerable<Vertex> vertices)
	{
		foreach (Vertex vertex in vertices)
			Remove(vertex);
	}

	#endregion Removal

}
