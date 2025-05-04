using OxyPlot;
using OxyPlot.Series;
using OxyPlot.WindowsForms;
using Retriangulator2D;
using OxyPlot.Annotations;
using OxyPlot.Axes;
using System.Reflection;
using System.Text.Json;

namespace DelaunayTriangulationTestCase1
{
	public partial class Form1 : Form
	{
		PlotView plot1;

		public Form1()
		{
			#region Setup
			this.plot1 = new OxyPlot.WindowsForms.PlotView();
			this.SuspendLayout();
			// 
			// plot1
			// 
			this.plot1.Dock = System.Windows.Forms.DockStyle.Bottom;
			this.plot1.Location = new System.Drawing.Point(0, 0);
			this.plot1.Name = "plot1";
			this.plot1.PanCursor = System.Windows.Forms.Cursors.Hand;
			this.plot1.Size = new System.Drawing.Size(500, 500);
			this.plot1.TabIndex = 0;
			this.plot1.Text = "plot1";
			this.plot1.ZoomHorizontalCursor = System.Windows.Forms.Cursors.SizeWE;
			this.plot1.ZoomRectangleCursor = System.Windows.Forms.Cursors.SizeNWSE;
			this.plot1.ZoomVerticalCursor = System.Windows.Forms.Cursors.SizeNS;
			this.Controls.Add(this.plot1);

			InitializeComponent();
			var model = new PlotModel { Title = "Test" };

			// Display the edges onto the plot
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Left,
				Minimum = -0.1,
				Maximum = 1.1
			});
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Bottom,
				Minimum = -0.1,
				Maximum = 1.1
			});

			this.plot1.Model = model;
			#endregion Setup
		}

		/// <summary>
		/// Runs the test with randomized parameters.
		/// </summary>
		/// <param name="sender"></param>
		/// <param name="e"></param>
		private void ClickRunButton(object sender, EventArgs e)
		{
#if TESTCASE_MERGE_ALGORITHM
			RunMerge();
#endif

#if TESTCASE_INSERT_MANY_ALGORITHM
			RunInsert();
#endif

#if TESTCASE_REMOVE_MANY_ALGORITHM
			RunRemoveVertices();
#endif

#if TESTCASE_INTERPOLATE_ALGORITHM
			RunInterpolation();
#endif
		}


#if TESTCASE_MERGE_ALGORITHM

		/// <summary>
		/// Runs the merge test.
		/// </summary>
		private void RunMerge()
		{
			var model = this.plot1.Model;
			model.Annotations.Clear();

			// Randomly initialize some vertices
			const int NUM_VERTICES = 100;
			Random rand = new Random();
			List<Vertex2> vertices = new List<Vertex2>();
			for (int k = 0; k < NUM_VERTICES; ++k)
				vertices.Add(new Vertex2
				{
					X = rand.NextDouble(),
					Y = rand.NextDouble()
				});

			// Construct a mesh from them
			ConvexHullMesh<double, Vertex2> mesh = ConvexHullMesh<double, Vertex2>.Construct(vertices);

			#region Display
			// Display the edges onto the plot
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Left,
				Minimum = -0.1,
				Maximum = 1.1
			});
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Bottom,
				Minimum = -0.1,
				Maximum = 1.1
			});

			// Draw edges
			foreach (Edge<double, Vertex2> edge in ConvexHullMesh.Edges)
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}

			// Draw triangles
			foreach (Triangle<double, Vertex2> triangle in ConvexHullMesh.Triangles)
			{
				PolygonAnnotation triangleLines = new PolygonAnnotation
				{
					LineStyle = LineStyle.Dot,
					Stroke = OxyColor.FromRgb(0xFF, 0x00, 0x00),
					StrokeThickness = 2.0,
					Fill = OxyColor.FromArgb(0x10, 0xFF, 0x00, 0x00)
				};
				triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
				model.Annotations.Add(triangleLines);
			}

			// Draw vertices
			foreach (Vertex2 vertex in ConvexHullMesh.Vertices)
			{
				model.Annotations.Add(new PointAnnotation
				{
					X = vertex.X,
					Y = vertex.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0x00, 0x00, 0x00)
				});
			}
			#endregion Display
			
			this.plot1.Refresh();
		}

#endif

#if TESTCASE_INSERT_MANY_ALGORITHM

		/// <summary>
		/// Runs the insert test.
		/// </summary>
		private void RunInsertVertices()
		{
			var model = this.plot1.Model;
			model.Annotations.Clear();

			// Randomly initialize some vertices
			const int NUM_INITIAL_VERTICES = 20;
			Random rand = new Random();
			List<Vertex2> vertices = new List<Vertex2>();
			for (int k = 0; k < NUM_INITIAL_VERTICES; ++k)
				vertices.Add(new Vertex2
				{
					X = rand.NextDouble(),
					Y = rand.NextDouble()
				});


			// Construct a mesh from them
			ConvexHullMesh<double, Vertex2> mesh = ConvexHullMesh<double, Vertex2>.Construct(vertices);
			var initialEdges = new HashSet<Edge<double, Vertex2>>(mesh.Edges);
			var initialTriangles = new HashSet<Triangle<double, Vertex2>>(mesh.Triangles);

			// Add new vertices
			const int NUM_ADDITIONAL_VERTICES = 6;
			List<Vertex2> newVertices = new List<Vertex2>();
			for (int k = 0; k < NUM_ADDITIONAL_VERTICES; ++k)
			{
				newVertices.Add(new Vertex2
				{
					X = rand.NextDouble(),
					Y = rand.NextDouble()
				});
			}
			mesh.AddRange(newVertices.Take(NUM_ADDITIONAL_VERTICES / 2));
			mesh.AddRange(newVertices.Skip(NUM_ADDITIONAL_VERTICES / 2));

			var finalEdges = new HashSet<Edge<double, Vertex2>>(mesh.Edges);
			var finalTriangles = new HashSet<Triangle<double, Vertex2>>(mesh.Triangles);

			#region Display
			// Display the edges onto the plot
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Left,
				Minimum = -0.1,
				Maximum = 1.1
			});
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Bottom,
				Minimum = -0.1,
				Maximum = 1.1
			});

			// Draw edges
			foreach (Edge<double, Vertex2> edge in initialEdges.Intersect(finalEdges))
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					Color = OxyColor.FromRgb(0x00, 0x80, 0xFF),
					StrokeThickness = 3.0,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}
			foreach (Edge<double, Vertex2> edge in finalEdges.Except(initialEdges))
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					Color = OxyColor.FromRgb(0xFF, 0x80, 0x00),
					StrokeThickness = 3.0,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}

			// Draw triangles
			foreach (Triangle<double, Vertex2> triangle in initialTriangles.Intersect(finalTriangles))
			{
				PolygonAnnotation triangleLines = new PolygonAnnotation
				{
					LineStyle = LineStyle.Dot,
					Stroke = OxyColor.FromRgb(0x00, 0x00, 0xFF),
					StrokeThickness = 2.0,
					Fill = OxyColor.FromArgb(0x10, 0x00, 0x00, 0xFF)
				};
				triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
				model.Annotations.Add(triangleLines);
			}
			foreach (Triangle<double, Vertex2> triangle in finalTriangles.Except(initialTriangles))
			{
				PolygonAnnotation triangleLines = new PolygonAnnotation
				{
					LineStyle = LineStyle.Dot,
					Stroke = OxyColor.FromRgb(0xFF, 0x00, 0x00),
					StrokeThickness = 2.0,
					Fill = OxyColor.FromArgb(0x10, 0xFF, 0x00, 0x00)
				};
				triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
				model.Annotations.Add(triangleLines);
			}

			// Draw vertices
			foreach (Vertex2 vertex in vertices)
			{
				model.Annotations.Add(new PointAnnotation
				{
					X = vertex.X,
					Y = vertex.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0x00, 0x00, 0x00)
				});
			}
			foreach (Vertex2 vertex in newVertices)
			{
				model.Annotations.Add(new PointAnnotation
				{
					X = vertex.X,
					Y = vertex.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0x00, 0x80, 0x80)
				});
			}
			#endregion Display

			this.plot1.Refresh();
		}

#endif

#if TESTCASE_REMOVE_MANY_ALGORITHM

		List<Vertex2> InitialVertices;
		List<Vertex2> RemoveVertices;

		private void ClickSaveButton(object sender, EventArgs e)
		{
			try
			{
				string json = JsonSerializer.Serialize(new Dictionary<string, List<Vertex2>>
				{
					["initialVertices"] = InitialVertices,
					["removeVertices"] = RemoveVertices
				});
				using FileStream fstream = File.OpenWrite($"remove_test_{DateTime.UtcNow:yyyyMMddHHmmssfff}.json");
				using StreamWriter writer = new StreamWriter(fstream);
				writer.Write(json);
			}
			catch (Exception ex)
			{
				label1.Text = ex.Message;
			}
		}

		private void ClickLoadRunButton(object sender, EventArgs e)
		{
			if (openFileDialog1.ShowDialog() == DialogResult.OK)
			{
				List<Vertex2> initialVertices, removeVertices;
				try
				{
					using StreamReader reader = new StreamReader(openFileDialog1.FileName);
					string json = reader.ReadToEnd();
					var obj = JsonSerializer.Deserialize<Dictionary<string, List<Vertex2>>>(json);
					initialVertices = obj["initialVertices"];
					removeVertices = obj["removeVertices"];
				}
				catch (Exception ex)
				{
					label1.Text = ex.Message;
					return;
				}

				InitialVertices = initialVertices;
				RemoveVertices = new List<Vertex2>(initialVertices.Where(v1 => removeVertices.Any(v2 => v1.X == v2.X && v1.Y == v2.Y)));
				RunRemoveVertices(InitialVertices, RemoveVertices);
			}
		}

		private void ClickRerunButton(object sender, EventArgs e)
		{
			RunRemoveVertices(InitialVertices, RemoveVertices);
		}



		/// <summary>
		/// Runs the remove test.
		/// </summary>
		private void RunRemoveVertices()
		{
			// Randomly initialize some vertices
			const int NUM_INITIAL_VERTICES = 50;
			Random rand = new Random();
			InitialVertices = new List<Vertex2>();
			for (int k = 0; k < NUM_INITIAL_VERTICES; ++k)
				InitialVertices.Add(new Vertex2
				{
					X = rand.NextDouble(),
					Y = rand.NextDouble()
				});

			// Randomly select some vertices to remove
			const int NUM_REMOVED_VERTICES = 10;
			RemoveVertices = new List<Vertex2>(InitialVertices.Take(NUM_REMOVED_VERTICES));

			// Run the algorithm
			RunRemoveVertices(InitialVertices, RemoveVertices);
		}

		private void RunRemoveVertices(List<Vertex2> initialVertices, List<Vertex2> removeVertices)
		{
			var model = this.plot1.Model;
			model.Annotations.Clear();

			// Construct a mesh from the initial vertices
			ConvexHullMesh<double, Vertex2> mesh = ConvexHullMesh<double, Vertex2>.Construct(initialVertices);
			var initialEdges = new HashSet<Edge<double, Vertex2>>(mesh.Edges);
			var initialTriangles = new HashSet<Triangle<double, Vertex2>>(mesh.Triangles);

			// Add new vertices
			mesh.RemoveRange(removeVertices[..(removeVertices.Count / 2)]);
			mesh.RemoveRange(removeVertices[(removeVertices.Count / 2)..]);

			var finalEdges = new HashSet<Edge<double, Vertex2>>(mesh.Edges);
			var finalTriangles = new HashSet<Triangle<double, Vertex2>>(mesh.Triangles);

			#region Display

			// Draw edges
			foreach (Edge<double, Vertex2> edge in initialEdges.Intersect(finalEdges))
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					Color = OxyColor.FromRgb(0x00, 0x80, 0xFF),
					StrokeThickness = 3.0,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}
			foreach (Edge<double, Vertex2> edge in initialEdges.Except(finalEdges))
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					Color = OxyColor.FromRgb(0xFF, 0x00, 0x00),
					StrokeThickness = 3.0,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}
			foreach (Edge<double, Vertex2> edge in finalEdges.Except(initialEdges))
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					Color = OxyColor.FromRgb(0xFF, 0x80, 0x00),
					StrokeThickness = 3.0,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}
			foreach (Edge<double, Vertex2> edge in mesh.Boundary)
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					Color = OxyColor.FromArgb(0xFF, 0x00, 0x00, 0x00),
					StrokeThickness = 3.0,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}

			// Draw triangles
			foreach (Triangle<double, Vertex2> triangle in initialTriangles.Intersect(finalTriangles))
			{
				PolygonAnnotation triangleLines = new PolygonAnnotation
				{
					LineStyle = LineStyle.Dot,
					Stroke = OxyColor.FromRgb(0x00, 0x00, 0xFF),
					StrokeThickness = 2.0,
					Fill = OxyColor.FromArgb(0x10, 0x00, 0x00, 0xFF)
				};
				triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
				model.Annotations.Add(triangleLines);
			}
			foreach (Triangle<double, Vertex2> triangle in finalTriangles.Except(initialTriangles))
			{
				PolygonAnnotation triangleLines = new PolygonAnnotation
				{
					LineStyle = LineStyle.Dot,
					Stroke = OxyColor.FromRgb(0x00, 0xFF, 0x00),
					StrokeThickness = 2.0,
					Fill = OxyColor.FromArgb(0x10, 0x00, 0xFF, 0x00)
				};
				triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
				model.Annotations.Add(triangleLines);
			}

			// Draw vertices
			foreach (Vertex2 vertex in mesh.Vertices)
			{
				model.Annotations.Add(new PointAnnotation
				{
					X = vertex.X,
					Y = vertex.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0x00, 0x00, 0x00)
				});
			}

			foreach (Vertex2 vertex in removeVertices)
			{
				model.Annotations.Add(new PointAnnotation
				{
					X = vertex.X,
					Y = vertex.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0xFF, 0x00, 0x00)
				});
			}

			// Display circumcircle overlay
			/*
			foreach (var triangle in mesh.Triangles)
			{
				model.Annotations.Add(new EllipseAnnotation
				{
					X = triangle.CircumcircleCenter.X,
					Y = triangle.CircumcircleCenter.Y,
					Width = 2.0 * Math.Sqrt(triangle.CircumcircleRadiusSquared),
					Height = 2.0 * Math.Sqrt(triangle.CircumcircleRadiusSquared),
					Stroke = OxyColor.FromArgb(0x80, 0x00, 0xFF, 0x00),
					StrokeThickness = 1.0,
					Fill = OxyColor.FromArgb(0x10, 0x00, 0xFF, 0x00)
				});
				model.Annotations.Add(new PointAnnotation
				{
					X = triangle.CircumcircleCenter.X,
					Y = triangle.CircumcircleCenter.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0x00, 0xFF, 0x00)
				});
			}
			*/
			#endregion Display

			model.InvalidatePlot(true);
		}

#endif

#if TESTCASE_INTERPOLATE_ALGORITHM

		List<Vertex2> Vertices;
		List<Vertex2> Points;

		private void ClickSaveButton(object sender, EventArgs e)
		{
			try
			{
				string json = JsonSerializer.Serialize(new Dictionary<string, List<Vertex2>>
				{
					["vertices"] = Vertices,
					["points"] = Points
				});
				using FileStream fstream = File.OpenWrite($"interpolate_test_{DateTime.UtcNow:yyyyMMddHHmmssfff}.json");
				using StreamWriter writer = new StreamWriter(fstream);
				writer.Write(json);
			}
			catch (Exception ex)
			{
				label1.Text = ex.Message;
			}
		}

		private void ClickLoadRunButton(object sender, EventArgs e)
		{
			if (openFileDialog1.ShowDialog() == DialogResult.OK)
			{
				List<Vertex2> vertices, points;
				try
				{
					using StreamReader reader = new StreamReader(openFileDialog1.FileName);
					string json = reader.ReadToEnd();
					var obj = JsonSerializer.Deserialize<Dictionary<string, List<Vertex2>>>(json);
					vertices = obj["vertices"];
					points = obj["points"];
				}
				catch (Exception ex)
				{
					label1.Text = ex.Message;
					return;
				}

				Vertices = vertices;
				Points = points;
				RunInterpolation(Vertices, Points);
			}
		}

		private void ClickRerunButton(object sender, EventArgs e)
		{
			RunInterpolation(Vertices, Points);
		}



		/// <summary>
		/// Runs the merge test.
		/// </summary>
		private void RunInterpolation()
		{
			// Randomly initialize some vertices
			const int NUM_VERTICES = 50;
			const int NUM_POINTS = 5;
			Random rand = new Random();
			Vertices = new List<Vertex2>();
			for (int k = 0; k < NUM_VERTICES; ++k)
				Vertices.Add(new Vertex2
				{
					X = rand.NextDouble(),
					Y = rand.NextDouble()
				});

			// Randomly choose points to interpolate
			Points = new List<Vertex2>();
			for (int k = 0; k < NUM_POINTS; ++k)
				Points.Add(new Vertex2
				{
					X = rand.NextDouble(),
					Y = rand.NextDouble()
				});

			// Run the algorithm
			RunInterpolation(Vertices, Points);
		}

		private void RunInterpolation(List<Vertex2> vertices, List<Vertex2> points)
		{
			var model = this.plot1.Model;
			model.Annotations.Clear();

			// Construct a mesh from the initial vertices
			ConvexHullMesh<double, Vertex2> mesh = ConvexHullMesh<double, Vertex2>.Construct(vertices);
			var initialEdges = new HashSet<Edge<double, Vertex2>>(mesh.Edges);
			var initialTriangles = new HashSet<Triangle<double, Vertex2>>(mesh.Triangles);

			// Interpolate each point
			label2.Text = "";
			foreach (Vertex2 point in points)
			{
				try
				{
					var crds = mesh.GetBarycentricDecomposition(point);
					label2.Text += $"{point} => [{crds[0]}, {crds[1]}, {crds[2]}]\n";
				}
				catch (InvalidInterpolationException)
				{
					label2.Text += $"{point} => Outside of boundaries\n";
				}
			}

			#region Display

			// Draw edges
			foreach (Edge<double, Vertex2> edge in initialEdges)
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					Color = OxyColor.FromRgb(0x00, 0x80, 0xFF),
					StrokeThickness = 3.0,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}

			// Draw triangles
			foreach (Triangle<double, Vertex2> triangle in initialTriangles)
			{
				PolygonAnnotation triangleLines = new PolygonAnnotation
				{
					LineStyle = LineStyle.Dot,
					Stroke = OxyColor.FromRgb(0x00, 0x00, 0xFF),
					StrokeThickness = 2.0,
					Fill = OxyColor.FromArgb(0x10, 0x00, 0x00, 0xFF)
				};
				triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
				model.Annotations.Add(triangleLines);
			}

			// Draw vertices
			foreach (Vertex2 vertex in mesh.Vertices)
			{
				model.Annotations.Add(new PointAnnotation
				{
					X = vertex.X,
					Y = vertex.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0x00, 0x00, 0x00)
				});
			}

			// Draw interpolated points
			foreach (Vertex2 point in points)
			{
				model.Annotations.Add(new PointAnnotation
				{
					X = point.X,
					Y = point.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0xFF, 0x00, 0xFF)
				});
			}

			// Display circumcircle overlay
			/*
			foreach (var triangle in mesh.Triangles)
			{
				model.Annotations.Add(new EllipseAnnotation
				{
					X = triangle.CircumcircleCenter.X,
					Y = triangle.CircumcircleCenter.Y,
					Width = 2.0 * Math.Sqrt(triangle.CircumcircleRadiusSquared),
					Height = 2.0 * Math.Sqrt(triangle.CircumcircleRadiusSquared),
					Stroke = OxyColor.FromArgb(0x80, 0x00, 0xFF, 0x00),
					StrokeThickness = 1.0,
					Fill = OxyColor.FromArgb(0x10, 0x00, 0xFF, 0x00)
				});
				model.Annotations.Add(new PointAnnotation
				{
					X = triangle.CircumcircleCenter.X,
					Y = triangle.CircumcircleCenter.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0x00, 0xFF, 0x00)
				});
			}
			*/
			#endregion Display

			model.InvalidatePlot(true);
		}

#endif
	}
}
