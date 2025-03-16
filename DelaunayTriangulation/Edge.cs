using System;
using System.Numerics;

namespace DelaunayTriangulation;

internal class Edge<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
    public required Vertex Vertex1;
    public required Vertex Vertex2;

    /// <summary>
    /// The vector parallel to the edge.
    /// </summary>
    public Vector2<T> Vector => new Vector2<T>
    {
        X = Vertex2.X - Vertex1.X,
        Y = Vertex2.Y - Vertex1.Y
    };

    /// <summary>
    /// The vector perpendicular to the edge.
    /// </summary>
    public Vector2<T> Normal => new Vector2<T>
    {
        X = Vertex2.Y - Vertex1.Y,
        Y = Vertex1.X - Vertex2.X
    };
}
