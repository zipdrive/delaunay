using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Retriangulator2D;

public class InvalidInterpolationException : Exception
{
	public InvalidInterpolationException(string? message) : base(message) { }
}
