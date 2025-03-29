using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Retriangulator2D;

/// <summary>
/// A vertex node storing a median vertex and the edges connected to that vertex, and divides a number of child vertices based on the coordinate of the median vertex.
/// </summary>
/// <typeparam name="T"></typeparam>
/// <typeparam name="Vertex"></typeparam>
internal abstract class VertexNode<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IPoint2<T>
{
	/// <summary>
	/// The median vertex.
	/// </summary>
	public Vertex Median;

	/// <summary>
	/// The edges connected to the median vertex.
	/// </summary>
	public List<Edge<T, Vertex>> ConnectedEdges;

	/// <summary>
	/// Locates the vertex node corresponding to the given vertex.
	/// </summary>
	/// <param name="vertex">A vertex.</param>
	/// <returns>The vertex node corresponding to the given vertex.</returns>
	public abstract VertexNode<T, Vertex>? FindVertexNode(Vertex vertex);

	/// <summary>
	/// Enumerates vertex nodes in approximate order from near to far from the given vertex.
	/// </summary>
	/// <param name="vertex"></param>
	/// <returns></returns>
	public abstract IEnumerable<VertexNode<T, Vertex>> EnumerateClosestVertices(Vertex vertex);

	/// <summary>
	/// Inserts a vertex into the tree.
	/// </summary>
	/// <param name="vertex">The vertex to add.</param>
	/// <param name="connectedEdges">The edges connected to that vertex.</param>
	public abstract void Add(Vertex vertex, IEnumerable<Edge<T, Vertex>>? connectedEdges = null);

	/// <summary>
	/// Removes a vertex from the tree.
	/// </summary>
	/// <param name="vertex">The vertex to remove.</param>
	public abstract VertexNode<T, Vertex>? Remove(Vertex vertex);
}

/// <summary>
/// A vertex node that divides child vertices at the x-axis median value.
/// </summary>
/// <typeparam name="T"></typeparam>
/// <typeparam name="Vertex"></typeparam>
internal class VertexXNode<T, Vertex> : VertexNode<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IPoint2<T>
{
	private VertexYNode<T, Vertex>? _Left;
	private VertexYNode<T, Vertex>? _Right;

	public VertexXNode(Vertex vertex, IEnumerable<Edge<T, Vertex>>? connectedEdges = null)
	{
		Median = vertex;
		ConnectedEdges = connectedEdges != null ? new List<Edge<T, Vertex>>(connectedEdges) : new List<Edge<T, Vertex>>();
	}

	public VertexXNode(Vertex[] vertices)
	{
		if (vertices.Length == 1)
		{
			Median = vertices[0];
		}
		else
		{
			Array.Sort(vertices, (lhs, rhs) =>
			{
				int compX = lhs.X.CompareTo(rhs.X);
				return compX == 0 ? lhs.Y.CompareTo(rhs.Y) : compX;
			});
			int medianIndex = vertices.Length / 2;
			Median = vertices[medianIndex];
			if (medianIndex > 0)
				_Left = new VertexYNode<T, Vertex>(vertices[..medianIndex]);
			_Right = new VertexYNode<T, Vertex>(vertices[(medianIndex + 1)..]);
		}
		ConnectedEdges = new List<Edge<T, Vertex>>();
	}

	public override VertexNode<T, Vertex>? FindVertexNode(Vertex vertex)
	{
		if (Median.Equals(vertex))
		{
			return this;
		}
		else if (vertex.X < Median.X)
		{
			return _Left?.FindVertexNode(vertex);
		}
		else if (vertex.X > Median.X)
		{
			return _Right?.FindVertexNode(vertex);
		}
		else if (vertex.Y < Median.Y)
		{
			return _Left?.FindVertexNode(vertex);
		}
		else if (vertex.Y > Median.Y)
		{
			return _Right?.FindVertexNode(vertex);
		}
		return null;
	}

	public override IEnumerable<VertexNode<T, Vertex>> EnumerateClosestVertices(Vertex vertex)
	{
		if (vertex.X < Median.X)
		{
			if (_Left != null)
				foreach (VertexNode<T, Vertex> node in _Left.EnumerateClosestVertices(vertex))
					yield return node;
			yield return this;
			if (_Right != null)
				foreach (VertexNode<T, Vertex> node in _Right.EnumerateClosestVertices(vertex))
					yield return node;
		}
		else if (vertex.X > Median.X)
		{
			if (_Right != null)
				foreach (VertexNode<T, Vertex> node in _Right.EnumerateClosestVertices(vertex))
					yield return node;
			yield return this;
			if (_Left != null)
				foreach (VertexNode<T, Vertex> node in _Left.EnumerateClosestVertices(vertex))
					yield return node;
		}
		else if (vertex.Y < Median.Y)
		{
			if (_Left != null)
				foreach (VertexNode<T, Vertex> node in _Left.EnumerateClosestVertices(vertex))
					yield return node;
			yield return this;
			if (_Right != null)
				foreach (VertexNode<T, Vertex> node in _Right.EnumerateClosestVertices(vertex))
					yield return node;
		}
		else if (vertex.Y > Median.Y)
		{
			if (_Right != null)
				foreach (VertexNode<T, Vertex> node in _Right.EnumerateClosestVertices(vertex))
					yield return node;
			yield return this;
			if (_Left != null)
				foreach (VertexNode<T, Vertex> node in _Left.EnumerateClosestVertices(vertex))
					yield return node;
		}
		else
		{
			yield return this;
			if (_Left != null)
				foreach (VertexNode<T, Vertex> node in _Left.EnumerateClosestVertices(vertex))
					yield return node;
			if (_Right != null)
				foreach (VertexNode<T, Vertex> node in _Right.EnumerateClosestVertices(vertex))
					yield return node;
		}
	}

	public override void Add(Vertex vertex, IEnumerable<Edge<T, Vertex>>? connectedEdges = null)
	{
		if (Median.Equals(vertex))
		{
			return;
		}
		else if (vertex.X < Median.X)
		{
			if (_Left == null)
				_Left = new VertexYNode<T, Vertex>(vertex, connectedEdges);
			else
				_Left.Add(vertex, connectedEdges);
		}
		else if (vertex.X > Median.X)
		{
			if (_Right == null)
				_Right = new VertexYNode<T, Vertex>(vertex, connectedEdges);
			else
				_Right.Add(vertex, connectedEdges);
		}
		else if (vertex.Y < Median.Y)
		{
			if (_Left == null)
				_Left = new VertexYNode<T, Vertex>(vertex, connectedEdges);
			else
				_Left.Add(vertex, connectedEdges);
		}
		else if (vertex.Y > Median.Y)
		{
			if (_Right == null)
				_Right = new VertexYNode<T, Vertex>(vertex, connectedEdges);
			else
				_Right.Add(vertex, connectedEdges);
		}
		throw new Exception("Vertex has same coordinates as a vertex already inserted into the mesh.");
	}

	public override VertexNode<T, Vertex>? Remove(Vertex vertex)
	{
		throw new NotImplementedException();
	}
}

/// <summary>
/// A vertex node that divides child vertices at the y-axis median value.
/// </summary>
/// <typeparam name="T"></typeparam>
/// <typeparam name="Vertex"></typeparam>
internal class VertexYNode<T, Vertex> : VertexNode<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IPoint2<T>
{
	private VertexXNode<T, Vertex>? _Left;
	private VertexXNode<T, Vertex>? _Right;

	public VertexYNode(Vertex vertex, IEnumerable<Edge<T, Vertex>>? connectedEdges = null)
	{
		Median = vertex;
		ConnectedEdges = connectedEdges != null ? new List<Edge<T, Vertex>>(connectedEdges) : new List<Edge<T, Vertex>>();
	}

	public VertexYNode(Vertex[] vertices)
	{
		if (vertices.Length == 1)
		{
			Median = vertices[0];
		}
		else
		{
			Array.Sort(vertices, (lhs, rhs) =>
			{
				int compY = lhs.Y.CompareTo(rhs.Y);
				return compY == 0 ? lhs.X.CompareTo(rhs.X) : compY;
			});
			int medianIndex = vertices.Length / 2;
			Median = vertices[medianIndex];
			if (medianIndex > 0)
				_Left = new VertexXNode<T, Vertex>(vertices[..medianIndex]);
			_Right = new VertexXNode<T, Vertex>(vertices[(medianIndex + 1)..]);
		}
		ConnectedEdges = new List<Edge<T, Vertex>>();
	}

	public override VertexNode<T, Vertex>? FindVertexNode(Vertex vertex)
	{
		if (Median.Equals(vertex))
		{
			return this;
		}
		else if (vertex.Y < Median.Y)
		{
			return _Left?.FindVertexNode(vertex);
		}
		else if (vertex.Y > Median.Y)
		{
			return _Right?.FindVertexNode(vertex);
		}
		else if (vertex.X < Median.X)
		{
			return _Left?.FindVertexNode(vertex);
		}
		else if (vertex.X > Median.X)
		{
			return _Right?.FindVertexNode(vertex);
		}
		return null;
	}

	public override IEnumerable<VertexNode<T, Vertex>> EnumerateClosestVertices(Vertex vertex)
	{
		if (vertex.Y < Median.Y)
		{
			if (_Left != null)
				foreach (VertexNode<T, Vertex> node in _Left.EnumerateClosestVertices(vertex))
					yield return node;
			yield return this;
			if (_Right != null)
				foreach (VertexNode<T, Vertex> node in _Right.EnumerateClosestVertices(vertex))
					yield return node;
		}
		else if (vertex.Y > Median.Y)
		{
			if (_Right != null)
				foreach (VertexNode<T, Vertex> node in _Right.EnumerateClosestVertices(vertex))
					yield return node;
			yield return this;
			if (_Left != null)
				foreach (VertexNode<T, Vertex> node in _Left.EnumerateClosestVertices(vertex))
					yield return node;
		}
		else if (vertex.X < Median.X)
		{
			if (_Left != null)
				foreach (VertexNode<T, Vertex> node in _Left.EnumerateClosestVertices(vertex))
					yield return node;
			yield return this;
			if (_Right != null)
				foreach (VertexNode<T, Vertex> node in _Right.EnumerateClosestVertices(vertex))
					yield return node;
		}
		else if (vertex.X > Median.X)
		{
			if (_Right != null)
				foreach (VertexNode<T, Vertex> node in _Right.EnumerateClosestVertices(vertex))
					yield return node;
			yield return this;
			if (_Left != null)
				foreach (VertexNode<T, Vertex> node in _Left.EnumerateClosestVertices(vertex))
					yield return node;
		}
		else
		{
			yield return this;
			if (_Left != null)
				foreach (VertexNode<T, Vertex> node in _Left.EnumerateClosestVertices(vertex))
					yield return node;
			if (_Right != null)
				foreach (VertexNode<T, Vertex> node in _Right.EnumerateClosestVertices(vertex))
					yield return node;
		}
	}

	public override void Add(Vertex vertex, IEnumerable<Edge<T, Vertex>>? connectedEdges = null)
	{
		if (Median.Equals(vertex))
		{
			return;
		}
		else if (vertex.Y < Median.Y)
		{
			if (_Left == null)
				_Left = new VertexXNode<T, Vertex>(vertex, connectedEdges);
			else
				_Left.Add(vertex, connectedEdges);
		}
		else if (vertex.Y > Median.Y)
		{
			if (_Right == null)
				_Right = new VertexXNode<T, Vertex>(vertex, connectedEdges);
			else
				_Right.Add(vertex, connectedEdges);
		}
		else if (vertex.X < Median.X)
		{
			if (_Left == null)
				_Left = new VertexXNode<T, Vertex>(vertex, connectedEdges);
			else
				_Left.Add(vertex, connectedEdges);
		}
		else if (vertex.X > Median.X)
		{
			if (_Right == null)
				_Right = new VertexXNode<T, Vertex>(vertex, connectedEdges);
			else
				_Right.Add(vertex, connectedEdges);
		}
		throw new Exception("Vertex has same coordinates as a vertex already inserted into the mesh.");
	}

	public override VertexNode<T, Vertex>? Remove(Vertex vertex)
	{
		throw new NotImplementedException();
	}
}