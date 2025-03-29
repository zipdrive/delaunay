using System;
using System.Numerics;

namespace Retriangulator2D;

/// <summary>
/// A point in 2-dimensional space.
/// </summary>
/// <typeparam name="T">The floating-point type, used for numeric calculation.
/// Use float for less memory, and double for greater precision.</typeparam>
public interface IPoint2<T> where T : IFloatingPointIeee754<T>
{
    /// <summary>
    /// The x-coordinate of the vertex.
    /// </summary>
    public T X { get; }

    /// <summary>
    /// The y-coordinate of the vertex.
    /// </summary>
    public T Y { get; }
}

/// <summary>
/// A simple vertex.
/// </summary>
/// <typeparam name="T">The floating-point type, used for numeric calculation.</typeparam>
internal class SimpleVertex2<T> : IPoint2<T> where T : IFloatingPointIeee754<T>
{
	public required T X { get; set; }

	public required T Y { get; set; }
}
